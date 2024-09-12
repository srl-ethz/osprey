import open3d as o3d
import numpy as np
import cv2
import time
import math
import os
# from frame_transformations import get_transformation_matrix_about_arb_axis


class GraspCandidate:
    def __init__(self, file=None):
        self.pointcloud = o3d.geometry.PointCloud()
        if file is not None:
            self.pointcloud = o3d.io.read_point_cloud(file)
            self.pointcloud, idxs = self.pointcloud.remove_radius_outlier(nb_points=16, radius=0.05)


        self.bbox = None
        self.main_axis = None
        self.grasp_pcd = None

    def set_point_cloud_from_aligned_frames(self, frame, depth_frame, cam_intrinsics):
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_depth = o3d.geometry.Image(depth_frame)
        img_color = o3d.geometry.Image(frame)
        
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(img_color, img_depth, convert_rgb_to_intensity=False)
        intrinsics = o3d.camera.PinholeCameraIntrinsic(cam_intrinsics.width, cam_intrinsics.height, cam_intrinsics.fx, cam_intrinsics.fy, cam_intrinsics.cx, cam_intrinsics.cy)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsics)
        pcd.transform([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        
        # Save full PCD for illustration purposes
        
        pcd = pcd.voxel_down_sample(0.03)
        self.pointcloud.points = pcd.points
        self.pointcloud.colors = pcd.colors
        
        self.save_pcd('pcd/graphics/color_full_pcd.pcd')

    def set_point_cloud_from_aligned_masked_frames(self, frame, depth_frame, cam_intrinsics):
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # print(np.isnan(depth_frame).any())
        img_depth = o3d.geometry.Image(depth_frame)
        # img_depth = o3d.geometry.Image(depth_frame.tobytes(), depth_frame.shape[1], depth_frame.shape[0], 2)
        img_color = o3d.geometry.Image(frame)
        
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(img_color, img_depth, convert_rgb_to_intensity=False)
        intrinsics = o3d.camera.PinholeCameraIntrinsic(cam_intrinsics.width, cam_intrinsics.height, cam_intrinsics.fx, cam_intrinsics.fy, cam_intrinsics.cx, cam_intrinsics.cy)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsics)
        pcd.transform([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])


        # Save full PCD for illustration purposes
        # self.save_pcd('pcd/graphics/color_full_pcd.pcd')
        # print('saving pcd at creation')
        # o3d.io.write_point_cloud('pcd/graphics/color_masked_full_pcd.pcd', pcd)
        # ROI selection

        # Get points and colors
        points = np.asarray(pcd.points)


        colors = np.asarray(pcd.colors)

        # Filter out points where the color is exactly black - this will filter out all points that were masked before
        rows, _ = np.where(colors != [0,0,0])
        res_points = points[rows]
        res_colors = colors[rows]
        # Set the points and color of the point cloud to the masked point
        pcd.points = o3d.utility.Vector3dVector(res_points)
        pcd.colors = o3d.utility.Vector3dVector(res_colors)

        # Save masked PCD for illustration purposes
        self.save_pcd('pcd/graphics/color_masked_pcd.pcd')
        o3d.io.write_point_cloud('pcd/graphics/color_masked_pcd.pcd', pcd)

        # Downsampling to reduce computation time later on
        pcd = pcd.voxel_down_sample(0.01)

        # Update point cloud with calculated points
        self.pointcloud.points = pcd.points
        self.pointcloud.colors = pcd.colors

        # Remove radius outliers
        self.pointcloud, idxs = self.pointcloud.remove_radius_outlier(nb_points=16, radius=0.05)

        if len(self.pointcloud.points) == 0:
            raise ValueError(f'No points found in point cloud!')
        # self.display_inlier_outlier(self.pointcloud, idxs)


    def display_inlier_outlier(cloud, ind):
        inlier_cloud = cloud.select_by_index(ind)
        outlier_cloud = cloud.select_by_index(ind, invert=True)

        print("Showing outliers (red) and inliers (gray): ")
        outlier_cloud.paint_uniform_color([1, 0, 0])
        inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
        o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])



    def save_pcd(self, file):
        base_path = os.path.dirname(file)
        os.makedirs(base_path, exist_ok=True)
        o3d.io.write_point_cloud(file, self.pointcloud)

    def add_points_and_color_to_pcd(self, points, color):
        curr_points = np.asarray(self.pointcloud.points)
        curr_colors = np.asarray(self.pointcloud.colors)

        new_points = np.concatenate((curr_points, points), axis=0)
        new_colors = np.concatenate((curr_colors, [color for _ in range(len(points))]), axis=0)

        self.pointcloud.points = o3d.utility.Vector3dVector(new_points)
        self.pointcloud.colors = o3d.utility.Vector3dVector(new_colors)

    def visualise_pcd(self, file=None):
        if file is None:
            pcd = self.pointcloud
        else: 
            pcd = o3d.io.read_point_cloud(file)
        
        vis = o3d.visualization.Visualizer()
        vis.create_window('PCD', width=1280, height=720)
        
        # Add default coordinate frame
        # mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
        # vis.add_geometry(mesh)

        vis.add_geometry(pcd)
        while True:
            try:
                vis.poll_events()
                vis.update_renderer()
            except KeyboardInterrupt:
                vis.destroy_window()
                del vis

    def visualize_geometries(self, geometries):
        vis = o3d.visualization.Visualizer()
        vis.create_window('PCD', width=1280, height=720)
        
        # Add default coordinate frame
        # mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
        # vis.add_geometry(mesh)
        for g in geometries:
            vis.add_geometry(g)
        while True:
            try:
                vis.poll_events()
                vis.update_renderer()
            except KeyboardInterrupt:
                vis.destroy_window()
                del vis
        pass

    def find_centroid(self, add_to_pcd=False):
        mean, _ = self.pointcloud.compute_mean_and_covariance()
        return mean
    
    def find_grasping_point_from_top(self):
        try:
            bbox = o3d.geometry.OrientedBoundingBox.create_from_points(self.pointcloud.points, robust=True)
        except Exception:
            print('Something went wrong trying to find the bounding box')
            return None
        
        centroid = bbox.center
        extents = bbox.extents

        print(f'bbox center: {centroid}')
        print(f'bbox extents: {extents}')

        

    def find_largest_axis(self):
        try:
            bbox = o3d.geometry.OrientedBoundingBox.create_from_points(self.pointcloud.points, robust=True)
        except Exception:
            print('Something went wrong trying to find the bounding box')
            return None
        self.bbox = bbox
        box_points = np.asarray(bbox.get_box_points())
        center = bbox.center
        extents = bbox.extent
        
        # Get index of maximum in array to determine longest axis and create a unit vector in the direction of the longest axis
        max_idx = np.argmax(extents)
        main_unit_vec = np.zeros(3)
        main_unit_vec[max_idx] = 1


        # These are the indices for the other unit vectors perpendicular to the one above
        idx_1 = (max_idx + 1) % 3
        idx_2 = (max_idx + 2) % 3
        
        unit_vec_plane_1 = np.zeros(3)
        unit_vec_plane_2 = np.zeros(3)
        unit_vec_plane_1[idx_1] = 1
        unit_vec_plane_2[idx_2] = 1

        
        # Now, apply the transformation to rotate the base frame vectors to the bounding box frame
        main_axis = np.dot(bbox.R, main_unit_vec)
        axis_plane_1 = np.dot(bbox.R, unit_vec_plane_1)
        axis_plane_2 = np.dot(bbox.R, unit_vec_plane_2)

        # Use for visualisation          
        axis_points = [np.add(center, np.dot(main_axis, 0.1*k)) for k in range(8)]
        # axis_points_1 = [np.add(center, np.dot(axis_plane_1, 0.1*k)) for k in range(8)]
        # axis_points_2 = [np.add(center, np.dot(axis_plane_2, 0.1*k)) for k in range(8)]
        self.add_points_and_color_to_pcd(axis_points, (0,255,0))
        # self.add_points_and_color_to_pcd(axis_points_1, (0,255,0))
        # self.add_points_and_color_to_pcd(axis_points_2, (0,255,0))

        self.main_axis = main_axis

        return [[main_axis, extents[max_idx]], [axis_plane_1, extents[idx_1]], [axis_plane_2, extents[idx_2]]]


    def rotate_pcd_around_axis(self, pcd, centroid, angle, axis):
        T = get_transformation_matrix_about_arb_axis(centroid, angle, axis)
        pcd_copy = o3d.geometry.PointCloud(pcd)
        return pcd_copy.transform(T)
        
    def find_all_grasping_candidates(self):
        axis_and_extents = self.find_largest_axis()
        main = axis_and_extents[0]
        secondary = axis_and_extents[1]
        third = axis_and_extents[2]
        
        centroid = self.find_centroid()
        extents = np.asarray(self.bbox.extent).copy()
        max_extent_idx = np.argmax(extents)
        extents[max_extent_idx] = 0.025

        bbox = o3d.geometry.OrientedBoundingBox(centroid, self.bbox.R, extents)


        points = np.asarray(self.pointcloud.points)
        colors = np.asarray(self.pointcloud.colors)

        grasp_idxs = bbox.get_point_indices_within_bounding_box(self.pointcloud.points)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[grasp_idxs])
        pcd.colors = o3d.utility.Vector3dVector(colors[grasp_idxs])



        # rows = np.where(abs(points[:, max_idx] - centroid[max_idx]) < 0.008)
        colors[grasp_idxs] = (255,0,0)
        self.pointcloud.colors = o3d.utility.Vector3dVector(colors)
        
        # Save full PCD for illustration purposes
        self.save_pcd('pcd/graphics/grasp_candidates_highlighted_pcd.pcd')
        
        # self.visualize_geometries([self.pointcloud,])
        return grasp_idxs
    


    def find_grasping_points(self):
        rows = self.find_all_grasping_candidates()
        # Transform into camera frame
        # self.pointcloud.transform([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

        points = np.asarray(self.pointcloud.points)
        colors = np.asarray(self.pointcloud.colors)
        points = points[rows]
        colors = colors[rows]
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
        pcd.estimate_normals()
        pcd = pcd.uniform_down_sample(2)
        

        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)
        normals = np.asarray(pcd.normals)

        curr = 0
        last = len(points)
        print(last)
        # weights = np.asarray([2, -0.25, 2.5, -5, 5000, -100])
        weights = np.asarray([1, 1])

        best_partners = np.full((len(points), 3), -1.0)
        score = 0

        for i in range(0, last):
            point = points[i]
            normal = normals[i]
            max_score = -32000
            score = -32000

            for j in range(0, last):
                p = points[j]
                n = normals[j]

                distance = np.subtract(point, p)
                k0 = weights[0] * np.linalg.norm(distance)

                k1 = weights[1] * np.abs(point[2] - p[2])**2

                score = k0 + k1
                # print(f'{i} and {j}: {score}')
            
                if score > max_score:
                    # print(f'updated max score for {i} to {score} with {j}') 
                    max_score = score
                    max_idx = j
                    best_partners[i, 0] = max_idx
                    best_partners[i, 1] = score
            
            # Set indices to indicate the best pairing
            best_partners[i, 0] = max_idx
            best_partners[max_idx, 0] = i
            
            # Set score
            best_partners[i, 1] = score
            best_partners[max_idx, 1] = score
            
            # Set mark to done so that we can save some computation time 
            # If a point is done, we won't consider possible pairings again
            # We can do this because our evaluation function is symmetrical
            best_partners[i, 2] = 1
            best_partners[max_idx, 2] = 1     

                           
            
        # Find best scoring point and its partner
        if np.size(best_partners) != 0:
            max_idx = np.argmax(best_partners[:, 1])
            max_idx_partner = int(best_partners[max_idx, 0])
            print(f'{max_idx} and {max_idx_partner}')
            # Visualize with color
            colors[max_idx] = [0,255,0]
            colors[max_idx_partner] = [0,255,0]
            pcd.colors = o3d.utility.Vector3dVector(colors)
            pcd.points = o3d.utility.Vector3dVector(points)

            # New point cloud since visualization is strange otherwise
            grasp_points = [points[max_idx], points[max_idx_partner]]
            print(grasp_points)
            grasp_colors = [[0,255,0] for _ in range(len(grasp_points))]
            grasp_pcd = o3d.geometry.PointCloud()
            grasp_pcd.points = o3d.utility.Vector3dVector(grasp_points)
            grasp_pcd.colors = o3d.utility.Vector3dVector(grasp_colors)
            # self.grasp_pcd = grasp_pcd
            self.grasp_pcd = pcd
            return grasp_points
        else:
            return None



if __name__=='__main__':
    # grasp = GraspCandidate('pcd/pointcloud_bottle_91.pcd')
    grasp = GraspCandidate('pcd/pointcloud_teddy bear_315.pcd')
    axis_extent, _, _ = grasp.find_largest_axis()
    axis = axis_extent[0]
    cen = grasp.find_centroid()

    # pcd = grasp.rotate_pcd_around_axis(grasp.pointcloud, cen, math.pi, axis)
    # cen = grasp.find_centroid()
    # grasp.add_points_and_color_to_pcd([cen,], (255,0,0))
    # grasp.find_all_grasping_candidates()
    # grasp.find_grasping_points()
    # grasp.save_pcd('pcd/graphics/final_pcd_output.pcd')
    # grasp.visualize_geometries([grasp.pointcloud, grasp.grasp_pcd])
    # pcd.transform([[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    grasp.visualize_geometries([grasp.pointcloud])
    # grasp.visualise_pcd()

        
