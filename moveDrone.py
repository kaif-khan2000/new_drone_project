from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
from pymavlink import mavutil
#######FUNCTIONS##########
def connectMyCopter():
    print "connecting to vehicle......"
    vehicle = connect('/dev/ttyAMA0',wait_ready=True,baud=57600)
    print "vehicle connected......."
    return vehicle

def arm_and_takeoff(height):
    vehicle.mode = 'GUIDED'
    while vehicle.mode != 'GUIDED':
        print "Arm and takeoff: changing mode to guided..."
        time.sleep(1)
    vehicle.armed =True
    while not vehicle.armed:
        print "Arm and takeoff: waiting to get armed..."
        time.sleep(1)
    print "ARMED!!!!!...."
    vehicle.simple_takeoff(height)
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=height*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)
def yaw(angle, direction):
    is_relative = 1
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        angle,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        direction,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    vehicle.send_mavlink(msg)

def velocity(velocity_x, velocity_y, velocity_z, distance):
    
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    duration = distance/velocity_x
    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
#######Main##############
vehicle = connectMyCopter()
arm_and_takeoff(3)
yaw(90,1)
yaw(90 ,-1)
