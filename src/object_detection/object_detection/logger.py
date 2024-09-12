import numpy as np
import pandas as pd

class Logger:
    def __init__(self) -> None:
        # Store (global_x,global_y,global_z,t, confidence, class, drone_x, drone_y, drone_z, drone_roll, drone_pitch, drone_yaw) in here
        self.cols = 12
        self.records = np.empty((0,self.cols))


    def record_value(self, record):
        # The caller must make sure the dimensions match
        self.records = np.concatenate((self.records, record))
    
    def export_to_csv(self, output_location):
        print('exporting logs to csv')
        cols = ['global_x', 'global_y', 'global_z', 't', 'confidence', 'class', 'drone_x', 'drone_y', 'drone_z', 'drone_roll', 'drone_pitch', 'drone_yaw']
        df = pd.DataFrame(data=self.records, columns=cols)
        df.to_csv(output_location)


    def export_single_col_csv(self, output_location):
        print('exporting logs to csv')
        cols = ['delay']
        df = pd.DataFrame(data=self.records, columns=cols)
        df.to_csv(output_location)
    