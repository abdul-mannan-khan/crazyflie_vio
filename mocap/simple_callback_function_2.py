# -*- coding: utf-8 -*-
#
# ,---------,       ____  _ __
# |  ,-^-,  |      / __ )(_) /_______________ _____  ___
# | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
# | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
# Copyright (C) 2019 Bitcraze AB
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, in version 3.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
"""
Example of how to connect to a Qualisys QTM system and feed the position to a
Crazyflie. It uses the high level commander to upload a trajectory to fly a
figure 8.

Set the uri to the radio settings of the Crazyflie and modify the
rigid_body_name to match the name of the Crazyflie in QTM.
"""
import asyncio
import math
import time
import xml.etree.cElementTree as ET
from threading import Thread

import qtm
from scipy.spatial.transform import Rotation

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

#---------------- imports for Vicon ----------------------#
import sys
import os
#import time # already imported
import struct
import socket
import select
import numpy as np
#from threading import Thread # already imported
#---------------------------------------------------------#

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# The name of the rigid body in QTM that represents the Crazyflie
rigid_body_name = 'cf'

# True: send position and orientation; False: send position only
send_full_pose = True

# When using full pose, the estimator can be sensitive to noise in the orientation data when yaw is close to +/- 90
# degrees. If this is a problem, increase orientation_std_dev a bit. The default value in the firmware is 4.5e-3.
orientation_std_dev = 8.0e-3

# The trajectory to fly
# See https://github.com/whoenig/uav_trajectories for a tool to generate
# trajectories

# Duration,x^0,x^1,x^2,x^3,x^4,x^5,x^6,x^7,y^0,y^1,y^2,y^3,y^4,y^5,y^6,y^7,z^0,z^1,z^2,z^3,z^4,z^5,z^6,z^7,yaw^0,yaw^1,yaw^2,yaw^3,yaw^4,yaw^5,yaw^6,yaw^7
figure8 = [
    [1.050000, 0.000000, -0.000000, 0.000000, -0.000000, 0.830443, -0.276140, -0.384219, 0.180493, -0.000000, 0.000000, -0.000000, 0.000000, -1.356107, 0.688430, 0.587426, -0.329106, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.710000, 0.396058, 0.918033, 0.128965, -0.773546, 0.339704, 0.034310, -0.026417, -0.030049, -0.445604, -0.684403, 0.888433, 1.493630, -1.361618, -0.139316, 0.158875, 0.095799, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.620000, 0.922409, 0.405715, -0.582968, -0.092188, -0.114670, 0.101046, 0.075834, -0.037926, -0.291165, 0.967514, 0.421451, -1.086348, 0.545211, 0.030109, -0.050046, -0.068177, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.700000, 0.923174, -0.431533, -0.682975, 0.177173, 0.319468, -0.043852, -0.111269, 0.023166, 0.289869, 0.724722, -0.512011, -0.209623, -0.218710, 0.108797, 0.128756, -0.055461, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.560000, 0.405364, -0.834716, 0.158939, 0.288175, -0.373738, -0.054995, 0.036090, 0.078627, 0.450742, -0.385534, -0.954089, 0.128288, 0.442620, 0.055630, -0.060142, -0.076163, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.560000, 0.001062, -0.646270, -0.012560, -0.324065, 0.125327, 0.119738, 0.034567, -0.063130, 0.001593, -1.031457, 0.015159, 0.820816, -0.152665, -0.130729, -0.045679, 0.080444, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.700000, -0.402804, -0.820508, -0.132914, 0.236278, 0.235164, -0.053551, -0.088687, 0.031253, -0.449354, -0.411507, 0.902946, 0.185335, -0.239125, -0.041696, 0.016857, 0.016709, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.620000, -0.921641, -0.464596, 0.661875, 0.286582, -0.228921, -0.051987, 0.004669, 0.038463, -0.292459, 0.777682, 0.565788, -0.432472, -0.060568, -0.082048, -0.009439, 0.041158, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.710000, -0.923935, 0.447832, 0.627381, -0.259808, -0.042325, -0.032258, 0.001420, 0.005294, 0.288570, 0.873350, -0.515586, -0.730207, -0.026023, 0.288755, 0.215678, -0.148061, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [1.053185, -0.398611, 0.850510, -0.144007, -0.485368, -0.079781, 0.176330, 0.234482, -0.153567, 0.447039, -0.532729, -0.855023, 0.878509, 0.775168, -0.391051, -0.713519, 0.391628, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
]

# Define a class QtmWrapper that inherits from the Thread class. 
# This class is responsible for connecting to the Qualisys QTM system, 
# receiving pose data, and forwarding it to the Crazyflie.

# Inside the QtmWrapper class, there are methods for connecting, 
# discovering Qualisys QTM instances, handling received packets, 
# and closing the connection.
class ViconUDPDataRelay(Thread):
    def __init__(self, RX_sock):
        Thread.__init__(self)

        #self.body_name = body_name
        self.on_pose = None

        self.DEBUG = False
        # Define Message length
        self.MsgLen = 1024
        self.RX_sock = RX_sock
        self.MessageRX_flag = False
        self.SrvMsgReceived = True
        self.RxMessage = None
        
        # Entry dictionary
        self.object_dict = {}
        self.reset_object_dict()
        self.vicon_xyz_rpy = {'X':0,'Y':0,'Z':0,'Roll':0,'Pitch':0,'Yaw':0}
        
    def reset_object_dict(self):
        self.object_dict['number_objects'] = 0

    #async def receive_msg_over_udp(self):
    def receive_msg_over_udp(self):
        Header = 'Waiting!async def receive_msg_over_udp(self):'
        # Select RX_sock
        sock = self.RX_sock
        # Verify if data has been received from VICON
        ready = select.select([sock], [], [], 0.001)
        # If data has been received, process it
        while not ready[0]:
           ready = select.select([sock], [], [], 0.001)         
        if ready[0]:
            self.MessageRX_flag = True
            data, addr = sock.recvfrom(self.MsgLen)
            # Extract message header
            Sequence_B00 = data[0]
            Sequence_B01 = data[1]
            Sequence_B02 = data[2]
            Sequence_B03 = data[3]
            Sequence = (Sequence_B03<<32)+(Sequence_B02<<16)+(Sequence_B01<<8)+Sequence_B00
            return self.process_vicon_data(data[0:160])

    #async def process_vicon_data(self, data):
    def process_vicon_data(self, data):
        # Create struct with message format:
        # Data types according to https://docs.python.org/3/library/struct.html
        # FrameNumber                       -> B000 - B003 (uint32,  I)
        # ItemsInBlock                      -> B004        (uint8,   B)
        #              -----------------------------------------------
        #                      ItemID       -> B005        (uint8,   B)
        #              Header  ItemDataSize -> B006 - B007 (uint16,  H)
        #                      ItemName     -> B008 - B031 (uint8,   B)
        #              -----------------------------------------------
        # Item_raw_00          TransX       -> B032 - B039 (double,  d)
        #                      TransY       -> B040 - B047 (double,  d)
        #                      TransZ       -> B048 - B055 (double,  d)
        #              Data    RotX         -> B056 - B063 (double,  d)
        #                      RotY         -> B064 - B071 (double,  d)
        #                      RotZ         -> B072 - B079 (double,  d)    
        #              -----------------------------------------------
        #                      ItemID       -> B080        (uint8,   B)
        #              Header  ItemDataSize -> B081 - B082 (uint16,  H)
        #                      ItemName     -> B083 - B106 (uint8,   B)
        #              -----------------------------------------------
        # Item_raw_01          TransX       -> B107 - B114 (double,  d)
        #                      TransY       -> B115 - B122 (double,  d)
        #                      TransZ       -> B123 - B130 (double,  d)
        #              Data    RotX         -> B131 - B139 (double,  d)
        #                      RotY         -> B140 - B146 (double,  d)
        #                      RotZ         -> B147 - B154 (double,  d)    
        #              -----------------------------------------------
        s = struct.Struct('I2BH24c6dBH24c6d')
        UnpackedData = s.unpack(data)
        FrameNumber  = UnpackedData[0]
        ItemsInBlock = UnpackedData[1]
        Item_raw_00_ItemID       = UnpackedData[2]
        Item_raw_00_ItemDataSize = UnpackedData[3]
        Item_raw_00_ItemName     = UnpackedData[4:28]
        Item_raw_00_TransX       = UnpackedData[28]
        Item_raw_00_TransY       = UnpackedData[29]
        Item_raw_00_TransZ       = UnpackedData[30]
        Item_raw_00_RotX         = UnpackedData[31]
        Item_raw_00_RotY         = UnpackedData[32]
        Item_raw_00_RotZ         = UnpackedData[33]
        
        Item_raw_00_ItemDataSize_string = []
        
        for this_byte in range(0,len(Item_raw_00_ItemName)):
            if Item_raw_00_ItemName[this_byte]>= b'!' and Item_raw_00_ItemName[this_byte]<= b'~':
                Item_raw_00_ItemDataSize_string.append(Item_raw_00_ItemName[this_byte].decode('utf-8'))
        Item_raw_00_ItemDataSize_string = ''.join(Item_raw_00_ItemDataSize_string)
        self.object_dict[Item_raw_00_ItemDataSize_string] = {'PosX':Item_raw_00_TransX,'PosY':Item_raw_00_TransY,
                                                             'PosZ':Item_raw_00_TransZ,'RotX':Item_raw_00_RotX,
                                                             'RotY':Item_raw_00_RotY,'RotZ':Item_raw_00_RotZ}
        self.object_dict['number_objects'] += 1

        if self.DEBUG:
            print('\tPosition [cm]: {0:+3.4f}, {1:+3.4f}, {2:+3.4f}'.format(self.object_dict[Item_raw_00_ItemDataSize_string]['PosX']*1e-1,
                                                                               self.object_dict[Item_raw_00_ItemDataSize_string]['PosY']*1e-1,
                                                                               self.object_dict[Item_raw_00_ItemDataSize_string]['PosZ']*1e-1))
            print('\tAttitude [deg]: {0:+3.4f}, {1:+3.4f}, {2:+3.4f}'.format(np.rad2deg(self.object_dict[Item_raw_00_ItemDataSize_string]['RotX']),
                                                                           np.rad2deg(self.object_dict[Item_raw_00_ItemDataSize_string]['RotY']),
                                                                           np.rad2deg(self.object_dict[Item_raw_00_ItemDataSize_string]['RotZ'])))
            print('----------------------------------')
        
        self.vicon_xyz_rpy['X'] = self.object_dict[Item_raw_00_ItemDataSize_string]['PosX']*1e-1
        self.vicon_xyz_rpy['Y']  = self.object_dict[Item_raw_00_ItemDataSize_string]['PosY']*1e-1
        self.vicon_xyz_rpy['Z']  = self.object_dict[Item_raw_00_ItemDataSize_string]['PosZ']*1e-1
        self.vicon_xyz_rpy['Roll']  = np.rad2deg(self.object_dict[Item_raw_00_ItemDataSize_string]['RotX'])
        self.vicon_xyz_rpy['Pitch']  = np.rad2deg(self.object_dict[Item_raw_00_ItemDataSize_string]['RotY'])
        self.vicon_xyz_rpy['Yaw']  = np.rad2deg(self.object_dict[Item_raw_00_ItemDataSize_string]['RotZ'])
        #self.vicon_xyx_rpy = self.object_dict[Item_raw_00_ItemDataSize_string]
        #return self.vicon_xyx_rpy self.object_dict
        #return self.object_dict
        
        x = self.vicon_xyz_rpy['X']
        y = self.vicon_xyz_rpy['Y']
        z = self.vicon_xyz_rpy['Z']

        roll = self.vicon_xyz_rpy['Roll']
        pitch = self.vicon_xyz_rpy['Pitch']
        yaw = self.vicon_xyz_rpy['Yaw']
        
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]])
                
        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])
                
        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]])
                
        rot = np.dot(R_z, np.dot(R_y, R_x))
        
        print(self.on_pose)
        time.sleep(1)
        if self.on_pose:
            print("I am inside self on pose")
            print("---------------------------------------")
            # Make sure we got a position
            if math.isnan(x):
                return
            self.on_pose([x, y, z, rot])
                
        #return self.object_dict
        return self.vicon_xyz_rpy
    #async def close(self):
    def close(self):    
        pass

# The wait_for_position_estimator function waits for 
# the Crazyflie's position estimator to find the position based on
# the received data.
def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')
    print("-------------------------------------------")
    print("I am inside wait_for_position_estimator.")
    print("-------------------------------------------")
    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]
            #print("-----------------------------------------------------")
            #print("I am inside for loop of wait_for_position_estimator. ")
            #print("-----------------------------------------------------")
            # print(data)
            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            print("{} {} {}".
                   format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break

# The _sqrt function is a helper function to ensure that a 
# negative value is not passed to the square root function.
def _sqrt(a):
    """
    There might be rounding errors making 'a' slightly negative.
    Make sure we don't throw an exception.
    """
    if a < 0.0:
        return 0.0
    return math.sqrt(a)

# The send_extpose_rot_matrix function sends the current Crazyflie's 
# position and orientation to the Crazyflie's position estimator using the extpos module.
def send_extpose_rot_matrix(cf, x, y, z, rot):
    """
    Send the current Crazyflie X, Y, Z position and attitude as a (3x3)
    rotaton matrix. This is going to be forwarded to the Crazyflie's
    position estimator.
    """
    quat = Rotation.from_matrix(rot).as_quat()
    print("x is ", x)
    print("y is ", y)
    print("z is ", z)
    print("----------------------------------------------------")
    time.sleep(2)
    if send_full_pose:
        cf.extpos.send_extpose(x, y, z, quat[0], quat[1], quat[2], quat[3])
    else:
        cf.extpos.send_extpos(x, y, z)

# The reset_estimator function resets the Crazyflie's 
# position estimator and waits for it to find the position.
def reset_estimator(cf):
    print("------------------------------------")
    print("         Working on reset           ")
    print("------------------------------------")    
    cf.param.set_value('kalman.resetEstimation', '1')
    print("------------------------------------")
    print("         estimation completed       ")
    print("------------------------------------")
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    # time.sleep(1)
    wait_for_position_estimator(cf)
    print("------------------------------------")
    print("        wait completed  ")
    print("------------------------------------")

#---------------------------------------------------------------------------#
#                                 Send Data                                 #
#---------------------------------------------------------------------------#

# The adjust_orientation_sensitivity function adjusts the 
# sensitivity of the orientation data for the Crazyflie's position estimator.
def adjust_orientation_sensitivity(cf):
    cf.param.set_value('locSrv.extQuatStdDev', orientation_std_dev)

# The activate_kalman_estimator function activates 
# the Kalman estimator for the Crazyflie.
def activate_kalman_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '2')

    # Set the std deviation for the quaternion data pushed into the
    # kalman filter. The default value seems to be a bit too low.
    cf.param.set_value('locSrv.extQuatStdDev', 0.06)

# The activate_mellinger_controller function activates 
# the Mellinger controller for the Crazyflie.
def activate_mellinger_controller(cf):
    cf.param.set_value('stabilizer.controller', '2')

#---------------------------------------------------------------------------#
#                              Send Data Finish                             #
#---------------------------------------------------------------------------#

# The upload_trajectory function uploads a trajectory to the 
# Crazyflie, including its duration and coefficients.
def upload_trajectory(cf, trajectory_id, trajectory):
    trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
    trajectory_mem.trajectory = []

    total_duration = 0
    for row in trajectory:
        duration = row[0]
        x = Poly4D.Poly(row[1:9])
        y = Poly4D.Poly(row[9:17])
        z = Poly4D.Poly(row[17:25])
        yaw = Poly4D.Poly(row[25:33])
        trajectory_mem.trajectory.append(Poly4D(duration, x, y, z, yaw))
        total_duration += duration

    trajectory_mem.write_data_sync()
    cf.high_level_commander.define_trajectory(trajectory_id, 0, len(trajectory_mem.trajectory))
    return total_duration

# The run_sequence function executes the uploaded 
# trajectory using the high-level commander.
def run_sequence(cf, trajectory_id, duration):
    commander = cf.high_level_commander

    commander.takeoff(1.0, 2.0)
    time.sleep(1.0)
    relative = True
    commander.start_trajectory(trajectory_id, 1.0, relative)
    time.sleep(duration)
    #commander.go_to(-0.0, 0.0, 0.0, 0.0, 0.0)
    commander.go_to(-0.1, 0.1, 0.5, 0.0, 2.0)
    time.sleep(3.0)
    commander.land(0.0, 2.0)
    time.sleep(2)
    commander.stop()

# The main part of the code initiates Crazyflie drivers, 
# connects to the Qualisys QTM system, 
# sets up a callback to handle pose data, 
# adjusts the estimator's settings, 
# uploads the trajectory, resets the estimator, and 
# finally runs the trajectory sequence.

# Define a callback function

def main():

    cflib.crtp.init_drivers()

    #----------------- connecting to Vicont to read data ------------------#
    IP_Address01 = "0.0.0.0"
    Host01   = (IP_Address01, 51001)
    # Create RX socket
    RX_sock = socket.socket(socket.AF_INET,    # Internet
                                     socket.SOCK_DGRAM) # UDP
    RX_sock.bind(Host01)
    my_vicon_xyz_rpy = {'X':0,'Y':0,'Z':0,'Roll':0,'Pitch':0,'Yaw':0}
    #----------------------------------------------------------------------#
    my_vicon_data_relay = ViconUDPDataRelay(RX_sock)
    my_vicon_xyz_rpy = {'X':0,'Y':0,'Z':0,'Roll':0,'Pitch':0,'Yaw':0}
    my_vicon_data_relay.DEBUG = False

    pose = [0,0,0,np.zeros((3,3))]
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        trajectory_id = 1
        commander = cf.high_level_commander
        time.sleep(2)
        commander.stop()
        # reading Vicon Data
        while True:
            try:
                my_vicon_xyz_rpy = my_vicon_data_relay.receive_msg_over_udp()
                print(my_vicon_xyz_rpy)
                print("-------------------------------------------------------------")
                time.sleep(0.1)
                x = my_vicon_xyz_rpy['X']
                y = my_vicon_xyz_rpy['Y']
                z = my_vicon_xyz_rpy['Z']

                roll = my_vicon_xyz_rpy['Roll']
                pitch = my_vicon_xyz_rpy['Pitch']
                yaw = my_vicon_xyz_rpy['Yaw']
        
                R_x = np.array([[1, 0, 0],
                            [0, np.cos(roll), -np.sin(roll)],
                            [0, np.sin(roll), np.cos(roll)]])
                
                R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                            [0, 1, 0],
                            [-np.sin(pitch), 0, np.cos(pitch)]])
                
                R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                            [np.sin(yaw), np.cos(yaw), 0],
                            [0, 0, 1]])
                
                rot = np.dot(R_z, np.dot(R_y, R_x))
                print(rot)
                pose[0] = x
                pose[1] = y
                pose[2] = z
                pose[3] = rot
                print(pose)
                print("------------------------------------------------------")
                print(my_vicon_data_relay.on_pose)
                print("------------------------------------------------------")
                
                my_vicon_data_relay.on_pose = lambda pose: send_extpose_rot_matrix(
                cf, pose[0], pose[1], pose[2], pose[3])
                print("------------------------------------------------------")
                print(my_vicon_data_relay.on_pose)
                print("------------------------------------------------------")
                print("I am waiting for callback function to execute.")
                time.sleep(3)
                adjust_orientation_sensitivity(cf)
                print("------------------------------------")
                print("    Activating Kalman Estimator     ")
                print("------------------------------------")
                activate_kalman_estimator(cf)
                
                # duration = upload_trajectory(cf, trajectory_id, figure8)
                #print('The sequence is {:.1f} seconds long'.format(duration))
                reset_estimator(cf)
                time.sleep(5)
                print("------------------------------------")
                print("         Estimator reset            ")
                print("------------------------------------")
                #run_sequence(cf, trajectory_id, duration)
                
            except(KeyboardInterrupt,SystemExit):
                print("\nClosing program ...")
                commander.land(0.0, 2.0)
                time.sleep(0.1)
                commander.stop()
                time.sleep(0.1)
                # Close socket
                RX_sock.close()
                # Exit program1
                sys.exit()
            
            except socket.error as msg:
                print("Socket error!")
                # Close socket
                RX_sock.close()
                print(msg)
                sys.exit()

if __name__ == "__main__":
    # Execute the main function
    main()

