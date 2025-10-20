import psutil
import os
import sys
import time

import numpy as np

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
# ur_rtde/src/rtde_python_bindings.cppから推定
from dashboard_client import DashboardClient

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

# TODO
# hostname, port, verbose
rtde_d = DashboardClient(robot_ip, 29999, False)
# timeout_ms
rtde_d.connect(2000)
print(rtde_d.isConnected())
rtde_d.disconnect()
print(rtde_d.isConnected())
rtde_d.connect(2000)
print(rtde_d.isConnected())
# Powers on the robot arm.
# rtde_d.powerOn()
# # Powers off the robot arm.
# rtde_d.powerOff()
# # Powers off the robot arm. ドキュメントにはこうあるが正しいか不明
# rtde_d.brakeRelease()
# TODO: この機能制限ならどういう用途で使う?勝手にProtectiveStopはUnlockされるということか
# Closes the current popup and unlocks protective stop.
# The unlock protective stop command fails with an exception if less than 5 seconds has passed since the protective stop occurred.
# rtde_d.unlockProtectiveStop()
# Use this when robot gets a safety fault or violation to restart the safety.
# After safety has been rebooted the robot will be in Power Off.
# You should always ensure it is okay to restart the system. 
# It is highly recommended to check the error log before using this command 
# (either via PolyScope or e.g. ssh connection).
# このコメントによればエラーログをRTDEで確認する方法がない?
# rtde_d.restartSafety()
print(rtde_d.isInRemoteControl())
# Returns the remote control status of the robot.
# If the robot is in remote control it returns false and if remote control is disabled or robot is in local control it returns false.
# Closes connection.
rtde_d.disconnect()
# rtde_d.quit()
# Shuts down and turns off robot and controller.
# rtde_d.shutdown()

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

# TODO: 要確認
# BEGIN

# Checks if the given joint position is reachable and within the current safety 
# limits of the robot.
# This check considers joint limits (if the target pose is specified as joint positions), 
# safety planes limits, TCP orientation deviation limits and range of the robot. 
# If a solution is found when applying the inverse kinematics to the given target 
# TCP pose, this pose is considered reachable
# 使えるが制御ループで毎回呼ぶのは重いかもしれない
print(rtde_c.isJointsWithinSafetyLimits(deg2rad_list(default_joints)))

# この有無でgetActualTCPForce()の値が変わるか確認する
# This function is used for enabling and disabling the use of external F/T measurements in the controller.
# NOTE: パラメータあり
enable = True
is_success = rtde_c.ftRtdeInputEnable(enable)

# Zeroes the TCP force/torque measurement from the builtin force/torque sensor by subtracting the current measurement from the subsequent.
is_success = rtde_c.zeroFtSensor()
print(is_success)

# Generalized forces in the TCP
print(rtde_r.getActualTCPForce())
# Get the raw force and torque measurement, not compensated for forces and torques caused by the payload.
print(rtde_r.getFtRawWrench())

# 排他的なモードになっていると思われる
robot_modes = {
    -1: "ROBOT_MODE_NO_CONTROLLER",
    0: "ROBOT_MODE_DISCONNECTED",
    1: "ROBOT_MODE_CONFIRM_SAFETY",
    2: "ROBOT_MODE_BOOTING",
    3: "ROBOT_MODE_POWER_OFF",
    4: "ROBOT_MODE_POWER_ON",
    5: "ROBOT_MODE_IDLE",
    6: "ROBOT_MODE_BACKDRIVE",
    7: "ROBOT_MODE_RUNNING",
    8: "ROBOT_MODE_UPDATING_FIRMWARE",
}
robot_mode = rtde_r.getRobotMode()
print(robot_mode, robot_modes.get(robot_mode, "Unknown"))

safety_status = rtde_r.getSafetyStatusBits()
IS_NORMAL_MODE = (safety_status & (1 << 0)) != 0
IS_REDUCED_MODE = (safety_status & (1 << 1)) != 0
IS_PROTECTIVE_STOPPED = (safety_status & (1 << 2)) != 0
IS_RECOVERY_MODE = (safety_status & (1 << 3)) != 0
IS_SAFEGUARD_STOPPED = (safety_status & (1 << 4)) != 0
IS_SYSTEM_EMERGENCY_STOPPED = (safety_status & (1 << 5)) != 0
IS_ROBOT_EMERGENCY_STOPPED = (safety_status & (1 << 6)) != 0
IS_EMERGENCY_STOPPED = (safety_status & (1 << 7)) != 0
IS_VIOLATION = (safety_status & (1 << 8)) != 0
IS_FAULT = (safety_status & (1 << 9)) != 0
IS_STOPPED_DUE_TO_SAFETY = (safety_status & (1 << 10)) != 0
print(f"{IS_NORMAL_MODE=}")
print(f"{IS_REDUCED_MODE=}")
print(f"{IS_PROTECTIVE_STOPPED=}")
print(f"{IS_RECOVERY_MODE=}")
print(f"{IS_SAFEGUARD_STOPPED=}")
print(f"{IS_SYSTEM_EMERGENCY_STOPPED=}")
print(f"{IS_ROBOT_EMERGENCY_STOPPED=}")
print(f"{IS_EMERGENCY_STOPPED=}")
print(f"{IS_VIOLATION=}")
print(f"{IS_FAULT=}")
print(f"{IS_STOPPED_DUE_TO_SAFETY=}")

print(rtde_r.isProtectiveStopped())
# どう復帰する?
# Triggers a protective stop on the robot. Can be used for testing and debugging.
# print(rtde_c.triggerProtectiveStop())
# Returns true if a program is running on the controller, 
# otherwise it returns false This is just a shortcut for getRobotStatus() & RobotStatus::ROBOT_STATUS_PROGRAM_RUNNING.
print(rtde_c.isProgramRunning())
# Robot status Bits 0-3:
# Is power on | Is program running | Is teach button pressed | Is power button pressed
# There is a synchronization gap between the three interfaces RTDE Control RTDE Receive and Dashboard Client.
# RTDE Control and RTDE Receive open its own RTDE connection and so the internal 
# state is not in sync. That means, if RTDE Control reports, that program is running, 
# RTDE Receive may still return that program is not running. 
# The update of the Dashboard Client even needs more time. 
# That means, the dashboard client still returns program not running after some 
# milliseconds have passed after RTDE Control already reports program running.
print(rtde_c.getRobotStatus())
print(rtde_r.getRobotStatus())
print(rtde_r.isEmergencyStopped())

rtde_r.disconnect()
# Noneなどが入る可能性がある?
print(rad2deg_list(rtde_r.getActualQ()))
print(rtde_r.isConnected())
rtde_r.reconnect()
print(rad2deg_list(rtde_r.getActualQ()))
print(rtde_r.isConnected())
# END

# time.sleep(1)
rtde_c.moveJ(deg2rad_list(default_joints))

# TODO: 要確認
# BEGIN
rtde_c.disconnect()
rtde_c.moveJ(deg2rad_list(default_joints))
print(rtde_c.isConnected())
rtde_c.reconnect()
rtde_c.moveJ(deg2rad_list(default_joints))
print(rtde_c.isConnected())
# END

# This function will terminate the script on controller.
rtde_c.stopScript()
