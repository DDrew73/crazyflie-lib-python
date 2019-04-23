# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2016 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Simple example that connects to the crazyflie at `URI` and runs a figure 8
sequence. This script requires some kind of location system, it has been
tested with (and designed for) the flow deck.

Change the URI variable to your Crazyflie configuration.
"""
import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

URI = 'radio://0/80/2M'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

m1stash = []
m2stash = []
m3stash = []
m4stash = []
zstash = []
stampstash = []

def start_attitude_printing(scf):
    log_conf = LogConfig(name='Stabilizer', period_in_ms=25)
    log_conf.add_variable('stabilizer.roll', 'float')
    log_conf.add_variable('stabilizer.pitch', 'float')
    log_conf.add_variable('stabilizer.yaw', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(attitude_callback)
    log_conf.start()

def attitude_callback(timestamp, data, logconf):
    r = data['stabilizer.roll']
    p = data['stabilizer.pitch']
    y = data['stabilizer.yaw']
    # print('pos: ({}, {}, {})'.format(r, p, y))

def start_ddmotor_printing(scf):
    log_conf = LogConfig(name='DDMotor', period_in_ms=25)
    log_conf.add_variable('motor.m1', 'float')
    log_conf.add_variable('motor.m2', 'float')
    log_conf.add_variable('motor.m3', 'float')
    log_conf.add_variable('motor.m4', 'float')
    log_conf.add_variable('stateEstimate.z', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(motor_callback)
    log_conf.start()

def motor_callback(timestamp, data, logconf):
    m1 = data['motor.m1']
    m2 = data['motor.m2']
    m3 = data['motor.m3']
    m4 = data['motor.m4']
    zest = data['stateEstimate.z']
    
    stampstash.append(time.time())
    m1stash.append(m1)
    m2stash.append(m2)
    m3stash.append(m3)
    m4stash.append(m4)
    zstash.append(zest)
    # print('ddmot: ({}, {}, {}, {}, {})'.format(m1, m2, m3, m4, zest))


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        cf = scf.cf

        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)

        start_attitude_printing(scf)
        start_ddmotor_printing(scf)
        
        
        for _ in range(10):
            cf.commander.send_bypass_setpoint(10000,10000,10000,10000)
            time.sleep(0.1)


        # for y in range(10):
        #     cf.commander.send_hover_setpoint(0,0,0,y/25)
        #     time.sleep(0.1)

        # # cf.commander.send_velocity_world_setpoint(0,0,0.0,0)
        # for _ in range(10):
        #     cf.commander.send_hover_setpoint(0,0,0,0.4)
        #     time.sleep(0.2)

        # for _ in range(4):
        #     cf.commander.send_velocity_world_setpoint(0, 0, 0.5, 0)
        #     time.sleep(0.40)
        #     cf.commander.send_velocity_world_setpoint(0,0,-1,0)
        #     time.sleep(0.40)

        # for _ in range(10):
        #     cf.commander.send_hover_setpoint(0,0,0,0.4)
        #     time.sleep(0.2)

        # for y in range(10):
        #     cf.commander.send_hover_setpoint(0, 0, 0, (10-y) / 25)
        #     time.sleep(0.1)

        cf.commander.send_stop_setpoint()
        time.sleep(1)

    m1stash = np.asarray(m1stash)
    m2stash = np.asarray(m2stash)
    m3stash = np.asarray(m3stash)
    m4stash = np.asarray(m4stash)
    zstash = np.asarray(zstash)
    stampstash = np.asarray(stampstash)
    stampstash = stampstash - stampstash[0]
    
    plt.subplot(2,1,1)
    plt.plot(stampstash,m1stash)
    plt.plot(stampstash,m2stash)
    plt.plot(stampstash,m3stash)
    plt.plot(stampstash,m4stash)
    
    plt.subplot(2,1,2)
    plt.plot(stampstash,zstash)
    plt.show()   

    s_ind = 80
    e_ind = 120

    m1stash = m1stash[s_ind:-e_ind]
    m2stash = m2stash[s_ind:-e_ind]
    m3stash = m3stash[s_ind:-e_ind]
    m4stash = m4stash[s_ind:-e_ind]
    zstash = zstash[s_ind:-e_ind]
    stampstash = stampstash[s_ind:-e_ind]

    plt.subplot(2,1,1)
    plt.plot(stampstash,m1stash)
    plt.plot(stampstash,m2stash)
    plt.plot(stampstash,m3stash)
    plt.plot(stampstash,m4stash)
    
    plt.subplot(2,1,2)
    plt.plot(stampstash,zstash)
    plt.show()   