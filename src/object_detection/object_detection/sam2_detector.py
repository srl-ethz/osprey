import os
import torch
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import tqdm
import cv2

# use bfloat16 for the entire notebook
torch.autocast(device_type="cuda", dtype=torch.bfloat16).__enter__()

if torch.cuda.get_device_properties(0).major >= 8:
    # turn on tfloat32 for Ampere GPUs (https://pytorch.org/docs/stable/notes/cuda.html#tensorfloat-32-tf32-on-ampere-devices)
    torch.backends.cuda.matmul.allow_tf32 = True
    torch.backends.cudnn.allow_tf32 = True

from sam2.build_sam import build_sam2_video_predictor


class Sam2Detector:
    def __init__(self, model_cfg, sam2_checkpoint, frame_dir):
        self.predictor = build_sam2_video_predictor(model_cfg, sam2_checkpoint)
        self.video_dir = frame_dir
        self.has_annotation = False
        self.frame_idx = 0

    def add_annotation(self, points, labels):
        self.inference_state = self.predictor.init_state(self.video_dir, offload_video_to_cpu=True, offload_state_to_cpu=True)
        _, out_obj_ids, out_mask_logits = self.predictor.add_new_points(
            inference_state=self.inference_state,
            # this should always be the first frame
            frame_idx=0,
            # random object id, doesn't really matter because we should only track one object
            obj_id=1,
            points=points,
            labels=labels,
        )

        self.has_annotation = True

        return out_obj_ids, out_mask_logits

    def run_init_batch(self):

        assert self.has_annotation, "Please add annotation before running tracking"

        video_segments = {}  # video_segments contains the per-frame segmentation results
        for out_frame_idx, out_obj_ids, out_mask_logits in self.predictor.propagate_in_video(self.inference_state):
            video_segments[out_frame_idx] = {
                out_obj_id: (out_mask_logits[i] > 0.0).cpu().numpy()
                for i, out_obj_id in enumerate(out_obj_ids)
            }
            self.frame_idx = out_frame_idx

            yield out_obj_ids, out_mask_logits

    def run_single_frame(self, frame):
        frame = self.prepare_frame(frame)

        # put frame on CPU if it's not there already
        frame = frame.cpu()

        # run the model
        frame_idx, obj_ids, out_mask_logits = self.predictor.run_single_frame(self.inference_state, frame, self.frame_idx + 1)
        self.frame_idx = frame_idx
        print(f'model processed frame index {frame_idx}')
        return obj_ids, out_mask_logits

    def show_mask(self, mask, ax, obj_id=None, random_color=False):
        # TODO: integrate with viz, global plt probably does not work well
        if random_color:
            color = np.concatenate([np.random.random(3), np.array([0.6])], axis=0)
        else:
            cmap = plt.get_cmap("tab10")
            cmap_idx = 0 if obj_id is None else obj_id
            color = np.array([*cmap(cmap_idx)[:3], 0.6])
        h, w = mask.shape[-2:]
        mask_image = mask.reshape(h, w, 1) * color.reshape(1, 1, -1)
        ax.imshow(mask_image)


    def show_points(self, coords, labels, ax, marker_size=200):
        # TODO: integrate with viz, global plt probably does not work well
        pos_points = coords[labels==1]
        neg_points = coords[labels==0]
        ax.scatter(pos_points[:, 0], pos_points[:, 1], color='green', marker='*', s=marker_size, edgecolor='white', linewidth=1.25)
        ax.scatter(neg_points[:, 0], neg_points[:, 1], color='red', marker='*', s=marker_size, edgecolor='white', linewidth=1.25)

    def prepare_frame(self, frame: np.ndarray, target_size=1024, mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]):
        print('frame shape before: ', frame.shape)
        
        frame = np.array(Image.fromarray(frame).resize((target_size, target_size)))

        if frame.dtype == np.uint8:
            frame = frame / 255.0
        else:
            raise ValueError("Frame should be in uint8 format")
        print(frame.shape)
        frame = torch.from_numpy(frame).permute(2, 0, 1)

        frame -= torch.tensor(mean)[:, None, None]
        frame /= torch.tensor(std)[:, None, None]
        # output shape should be (C, W, H)
        np_frame = np.array(frame.numpy().copy()).transpose(1,2,0)
        return frame
