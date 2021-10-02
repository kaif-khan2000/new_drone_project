from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import exceptions
import math
import argparse
import threading
#######FUNCTIONS##########
def connectMyCopter():
    print "connecting to vehicle......"
    vehicle = connect('/dev/ttyAMA0',wait_ready=True,baud=57600)
    print "vehicle connected......."
    return vehicle
def distance(x1,y1,x2,y2):
    return geopy.distance.vincenty((x1,y1),(x2,y2)).km
def fetchNearestCoord():
    cord = [(),(),(),()]
    lat = vehicle.location.global_frame.lat
    lon = vehicle.location.global_frame.lon
    distances = []
    for x in cord:
        distances.append(distance(lat,lon,x[0],x[1]))
    min_dist = min(distances)
    min_ind = distances.index(min_dist)
    return cord[min_ind]

## Ignore below function its for debugging purpose 
def takeoff(x): 
    
    print "flying..."

    msg = vehicle.simple_takeoff(x)
    
    ##while True:
       ## print vehicle.location.global_relative_frame.alt
        ##if vehicle.location.global_relative_frame.alt >= .95*x:
          ##  break
    time.sleep(1)
    print msg
    
## the function which provides security.
## So basically, to control a vehicle by a Remote, The drone should be in stabilize mode.
## So this program runs infinitely and checks for drones "mode".
## If its not in guided mode then it will turn it back.
def secure():
    Land = True
    while True:
        print vehicle.mode
        if vehicle.mode == 'LAND' and Land:
            continue
        if vehicle.mode != 'GUIDED':
            #Land = False
            vehicle.armed = False
            print("Someone trying take control...")
            vehicle.mode = 'GUIDED'
            count = 0
            while vehicle.mode != 'GUIDED' and count < 6:
                print "Changing mode......"
                count+=1
                time.sleep(1)
            if count <= 6:
                print("Changed to guided.....")
<<<<<<< HEAD

                vehicle.armed = True
                while not vehicle.armed:
                    print "arming.. after check..."
                    time.sleep(1)
                ##lat,long = fetchNearestCoord()
                ##point1 = LocationGlobalRelative(lat,long,10)
                ##vehicle.simple_goto(point1)
=======
                ## here you can write the security measures code.
                ## like taking it to the safe place etc.
>>>>>>> 1c63468a522e1ec934396f92d8895510d120c509
                vehicle.simple_takeoff(2)
                time.sleep(2)

def arm_and_takeoff():
    vehicle.mode = 'GUIDED'
    while vehicle.mode != 'GUIDED':
        print "Arm and takeoff: changing mode to guided..."
        time.sleep(1)
    vehicle.armed =True
    while not vehicle.armed:
        print "Arm and takeoff: waiting to get armed..."
        time.sleep(1)
    print "ARMED!!!!!...."
    vehicle.simple_takeoff(2)
    time.sleep(5)
    secure()
    #confirm = raw_input("To fly pls enter y: ")
    #if True:
     #   t1 = threading.Thread(target=takeoff,args=(0.2,))
      #  t2 = threading.Thread(target=secure)
       # t1.start()
        #t2.start()
#######Main##############
vehicle = connectMyCopter()
arm_and_takeoff()

