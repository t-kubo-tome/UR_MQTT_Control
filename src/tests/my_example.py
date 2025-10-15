import psutil
import os
import sys
import time

import numpy as np

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive


def deg2rad(deg):
    return deg * np.pi / 180.0

def deg2rad_list(deg_list):
    return [deg2rad(deg) for deg in deg_list]

def rad2deg(rad):
    return rad * 180.0 / np.pi

def rad2deg_list(rad_list):
    return [rad2deg(rad) for rad in rad_list]

## Parameters

# Simulated robot
# robot_ip = "127.0.0.1"
robot_ip = "10.5.5.102"
# Real robot

velocity = 0.5
acceleration = 0.5
# -1.0 means use the robot’s default frequency, 500Hz for e-Series and UR-Series, while its 125Hz for the CB-series
rtde_frequency = 500.0
dt = 1.0 / rtde_frequency  # 2ms
# TODO: フラグの意味を調べる
flags = RTDEControl.FLAG_VERBOSE | RTDEControl.FLAG_UPLOAD_SCRIPT
# The port used for the External URCap interface (default: 50002) 
# RTDEは30004
ur_cap_port = 50002  # default
lookahead_time = 0.1
gain = 300
# ur_rtde realtime priorities
# プログラムのrt_app_priorityとないはず
# controlが相対的にreceiveより優位なのには意味がある
rt_receive_priority = 90
rt_control_priority = 85

# hostname, frequency, flags, ur_cap_port, rt_priority
rtde_c = RTDEControl(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority)
# hostname, frequency, variables, verbose, use_upper_range_registers, rt_priority
# variables: A vector of variable names to be monitored (empty vector means use all default variables)
# verbose: Enable verbose output for debugging purposes.
# use_upper_range_registers: ? realtime_control_example.pyではFalse
rtde_r = RTDEReceive(robot_ip, rtde_frequency, [], True, False, rt_receive_priority)

# この順番で指定する
# Set application real-time priority
os_used = sys.platform
process = psutil.Process(os.getpid())
if os_used == "win32":  # Windows (either 32-bit or 64-bit)
    process.nice(psutil.REALTIME_PRIORITY_CLASS)
elif os_used == "linux":  # linux
    rt_app_priority = 80
    param = os.sched_param(rt_app_priority)
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, param)
    except OSError:
        print("Failed to set real-time process scheduler to %u, priority %u" % (os.SCHED_FIFO, rt_app_priority))
    else:
        print("Process real-time priority set to: %u" % rt_app_priority)

# 現在の作業台でのデフォルト姿勢
default_joints = [0.0, -90.0, -90.0, -90.0, 90.0, 0.0]
# J6だけ回転
new_joints = [0.0, -90.0, -90.0, -90.0, 90.0, 90.0]

# 関節移動
# デフォルトではSync（Asyncではない）
# Move to joint position (linear in joint-space)
# Parameters:
#         q – joint positions
#         speed – joint speed of leading axis [rad/s]. defaults to 1.05
#         acceleration – joint acceleration of leading axis [rad/s^2]. defaults to 1.4
#         asynchronous – a bool specifying if the move command should be asynchronous. If asynchronous is true it is possible to stop a move command using either the stopJ or stopL function. Default is false, this means the function will block until the movement has completed.
rtde_c.moveJ(deg2rad_list(default_joints))
print(rad2deg_list(rtde_r.getActualQ()))
rtde_c.moveJ(deg2rad_list(new_joints))
print(rad2deg_list(rtde_r.getActualQ()))
rtde_c.moveJ(deg2rad_list(default_joints))
print(rad2deg_list(rtde_r.getActualQ()))

# TCP移動
pose = rtde_r.getActualTCPPose()
# X, Y, Z, Rx, Ry, Rz
# 単位はmとrad
# [0.492758731909551, -0.1328801058413286, 0.19206447671314636, -2.2212006216673545, 2.2207765897624494, -0.00123055083415886]
print(pose)
# Zを10cm上げる
pose[2] += 0.1
# Move to position (linear in tool-space)
# Parameters:
#         pose – target pose
#         speed – tool speed [m/s]. defaults to 0.25
#         acceleration – tool acceleration [m/s^2]. defaults to 1.2
#         asynchronous – a bool specifying if the move command should be asynchronous. If asynchronous is true it is possible to stop a move command using either the stopJ or stopL function. Default is false, this means the function will block until the movement has completed.
rtde_c.moveL(pose)
pose = rtde_r.getActualTCPPose()
print(pose)
rtde_c.moveJ(deg2rad_list(default_joints))

## サーボモード

# Move to initial joint position with a regular moveJ
joint_q = default_joints.copy()
rtde_c.moveJ(deg2rad_list(joint_q))

# Execute 500Hz control loop for 2 seconds, each cycle is 2ms
# 合計45度回転させる
for i in range(1000):
    # This function is used in combination with waitPeriod() and is used to get the start of a control period / cycle. 
    t_start = rtde_c.initPeriod()
    # Servo to position (linear in joint-space)
    # Parameters:
    #         q – joint positions [rad]
    #         speed – NOT used in current version
    #         acceleration – NOT used in current version
    #         time – time where the command is controlling the robot. The function is blocking for time t [S]
    #         lookahead_time – time [S], range [0.03,0.2] smoothens the trajectory with this lookahead time
    #         gain – proportional gain for following target position, range [100,2000]
    # あまりlookahead_time, gainを変えてもわからなそう
    rtde_c.servoJ(deg2rad_list(joint_q), velocity, acceleration, dt, lookahead_time, gain)
    joint_q[0] += 0.045
    joint_q[5] += 0.045
    # Used for waiting the rest of the control period, set implicitly as dt = 1 / frequency. 
    # NOTE: the function is to be used in combination with the initPeriod().
    # dtとrtde_frequencyは厳密には一緒じゃなくても可能
    rtde_c.waitPeriod(t_start)

# 無いと、
# TPで、Another thread is already controlling the robot
# 標準出力で、RTDEControlInterface: RTDE control script is not running!
# となる（プログラム側はしたがって例外は発生しない）
rtde_c.servoStop()

# Receive側のループ例
for i in range(5):
    t_start = rtde_r.initPeriod()
    print(t_start)
    rtde_r.waitPeriod(t_start)

# time.sleep(1)
rtde_c.moveJ(deg2rad_list(default_joints))
# This function will terminate the script on controller.
rtde_c.stopScript()
