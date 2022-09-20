import pandas as pd
import djitellopy
import time

class Logger:

    def __init__(self, filename: str):
        
        self.filename = filename
        self.df = pd.DataFrame(columns=['time', 'command', 'pitch', 'roll', 'Yaw', 'height', 'Vx', 'Vy', 'Vz', 'battery'])


    def add(self, data: dict, command: str):
        """
            Given a list of all parametrs, add them to the DF.
        """
        curr_time = time.time()
        roll = data['roll']
        pitch = data['pitch']
        yaw = data['yaw']
        height = data['h']
        vx = data['vgx']
        vy = data['vgy']
        vz = data['vgz']
        battery = data['bat']
        row = [curr_time, command, pitch, roll, yaw, height, vx, vy, vz, battery]
        print(row)
        self.df.loc[len(self.df)] = row
    
    def save_log(self):
        """
            This method saves the data frame to a csv file
        """
        self.df.to_csv(self.filename)


