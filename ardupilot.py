
from datetime import datetime
from pymavlink import mavutil


class ArduPilot:

    def __init__(self):
        self.data = {'ACC': {'timestamp': [], 'x': [], 'y': [], 'z': []},
                     'GYRO': {'timestamp': [], 'x': [], 'y': [], 'z': []},
                     'RAW_ACC': {'timestamp': [], 'x': [], 'y': [], 'z': []},
                     'RAW_GYRO': {'timestamp': [], 'x': [], 'y': [], 'z': []},
                     'VIBRATION': {'timestamp': [], 'x': [], 'y': [], 'z': []},
                     'SERVO_OUTPUT_RAW': {'timestamp': [], 'servo1': [], 'servo2': [], 'servo3': [], 'servo4': []},
                     'RC_CHANNELS': {'timestamp': [], 'chan1': [], 'chan2': [], 'chan3': [], 'chan4': []},
                     'ATTITUDE': {'timestamp': [], 'roll': [], 'pitch': [], 'yaw': []},
                     'BATTERY': {'timestamp': [], 'voltage': [], 'current': []},
                    }


    def set_field(self, attribute, field, value):
         self.data[attribute][field].append(value)

    def read_msg(self, msg):

        dateTimeNow = datetime.now()

        type = msg.get_type()
        if(type == 'SCALED_IMU'):
            self.set_field('ACC', 'x', msg.xacc if type == 'SCALED_IMU' else msg.AccX)
            self.set_field('ACC', 'y', msg.xacc if type == 'SCALED_IMU' else msg.AccY)
            self.set_field('ACC', 'z', msg.xacc if type == 'SCALED_IMU' else msg.AccZ)
            self.set_field('ACC', 'timestamp', msg.time_usec if type == 'SCALED_IMU' else msg.TimeUS)

            self.set_field('GYRO', 'x', msg.xgyro if type == 'SCALED_IMU' else msg.GyrX)
            self.set_field('GYRO', 'y', msg.ygyro if type == 'SCALED_IMU' else msg.GyrY)
            self.set_field('GYRO', 'z', msg.zgyro if type == 'SCALED_IMU' else msg.GyrZ)
            self.set_field('GYRO', 'timestamp', msg.time_usec if type == 'SCALED_IMU' else msg.TimeUS)
           
        elif(type=='RAW_IMU' or type== 'IMU'):
            self.set_field('RAW_ACC', 'x', msg.xacc if type == 'RAW_IMU' else msg.AccX)
            self.set_field('RAW_ACC', 'y', msg.xacc if type == 'RAW_IMU' else msg.AccY)
            self.set_field('RAW_ACC', 'z', msg.xacc if type == 'RAW_IMU' else msg.AccZ)
            self.set_field('RAW_ACC', 'timestamp', msg.time_usec if type == 'RAW_IMU' else msg.TimeUS)

            self.set_field('RAW_GYRO', 'x', msg.xgyro if type == 'RAW_IMU' else msg.GyrX)
            self.set_field('RAW_GYRO', 'y', msg.ygyro if type == 'RAW_IMU' else msg.GyrY)
            self.set_field('RAW_GYRO', 'z', msg.zgyro if type == 'RAW_IMU' else msg.GyrZ)
            self.set_field('RAW_GYRO', 'timestamp', msg.time_usec if type == 'RAW_IMU' else msg.TimeUS)
            
        elif(type in ['VIBRATION', 'VIBE']):
            self.set_field('VIBRATION', 'x', msg.vibration_x if type == 'VIBRATION' else msg.VibeX)
            self.set_field('VIBRATION', 'y', msg.vibration_y if type == 'VIBRATION' else msg.VibeY)
            self.set_field('VIBRATION', 'z', msg.vibration_z if type == 'VIBRATION' else msg.VibeZ)
            self.set_field('VIBRATION', 'timestamp', msg.time_usec if type == 'VIBRATION' else msg.TimeUS)
                        
        elif(type in ['SERVO_OUTPUT_RAW', 'RCOU']):
            self.set_field('SERVO_OUTPUT_RAW', 'servo1', msg.servo1_raw if type == 'SERVO_OUTPUT_RAW' else msg.C1)
            self.set_field('SERVO_OUTPUT_RAW', 'servo2', msg.servo1_raw if type == 'SERVO_OUTPUT_RAW' else msg.C2)
            self.set_field('SERVO_OUTPUT_RAW', 'servo3', msg.servo1_raw if type == 'SERVO_OUTPUT_RAW' else msg.C3)
            self.set_field('SERVO_OUTPUT_RAW', 'servo4', msg.servo1_raw if type == 'SERVO_OUTPUT_RAW' else msg.C4)
            self.set_field('SERVO_OUTPUT_RAW', 'timestamp', msg.time_usec if type == 'SERVO_OUTPUT_RAW' else msg.TimeUS)

        elif(type in ['RC_CHANNELS', 'RCIN']):
            self.set_field('RC_CHANNELS', 'chan1', msg.chan1_raw if type == 'RC_CHANNELS' else msg.C1)
            self.set_field('RC_CHANNELS', 'chan2', msg.chan2_raw if type == 'RC_CHANNELS' else msg.C2)
            self.set_field('RC_CHANNELS', 'chan3', msg.chan3_raw if type == 'RC_CHANNELS' else msg.C3)
            self.set_field('RC_CHANNELS', 'chan4', msg.chan4_raw if type == 'RC_CHANNELS' else msg.C4)
            self.set_field('RC_CHANNELS', 'timestamp', msg.time_boot_ms if type == 'RC_CHANNELS' else msg.TimeUS)
            
        elif(type in ['ATTITUDE', 'ATT']):
            self.set_field('ATTITUDE', 'roll', msg.roll if type == 'ATTITUDE' else msg.Roll)
            self.set_field('ATTITUDE', 'pitch', msg.pitch if type == 'ATTITUDE' else msg.Pitch)
            self.set_field('ATTITUDE', 'yaw', msg.yaw if type == 'ATTITUDE' else msg.Yaw)
            self.set_field('ATTITUDE', 'timestamp', msg.time_boot_ms if type == 'ATTITUDE' else msg.TimeUS)
            
        elif(type in ['SYS_STATUS', 'BAT']):
            self.set_field('BATTERY', 'voltage', msg.voltage_battery if type == 'SYS_STATUS' else msg.Volt)
            self.set_field('BATTERY', 'current', msg.current_battery if type == 'SYS_STATUS' else msg.Curr)
            if type == 'BAT':
                self.set_field('BATTERY', 'timestamp', msg.TimeUS)


    def read_tlog(self, file_name):
        name, ext = file_name.split('.')

        if 'bin' in ext:
            mavfile = mavutil.mavlink_connection(file_name, notimestamps=True, format='binary')
        else:
            mavfile = mavutil.mavlink_connection(file_name)

        while True:
            msg = mavfile.recv_match()
            if not msg:
                break

            self.read_msg(msg)
            


