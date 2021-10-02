from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import exceptions
import math
import argparse

#######FUNCTIONS##########
def connectMyCopter():
    print "connecting to vehicle......"
    vehicle = connect('/dev/ttyAMA0',wait_ready=True,baud=57600)
    print "vehicle connected......."
    return vehicle

def secure():
    while True:
        if vehicle.mode != 'GUIDED':
            vehicle.mode = 'GUIDED'

#######Main##############
vehicle = connectMyCopter()
secure()
