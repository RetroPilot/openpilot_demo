#!/usr/bin/env python3
from fileinput import close
import math
from selfdrive.swaglog import cloudlog


# Helper functions for GPS related activities



def isAppropriateStoppingDistance(DISTANCE_TO_STOP, SPEED, DECEL_RATE_TIME):
    if (DISTANCE_TO_STOP == None): return None
    if (SPEED > 0):
        if (DISTANCE_TO_STOP > 0 and SPEED > 0):
            TT3 = DISTANCE_TO_STOP / SPEED
            return TT3 < DECEL_RATE_TIME
    # Somethings zero, can stop.
    return True


def calculateDistance(startCords, destinationCords):
        # approximate radius of earth in km
    R = 6373.0

    lat1 = math.radians(float(startCords[0]))
    lon1 = math.radians(float(startCords[1]))
    lat2 = math.radians(float(destinationCords[0]))
    lon2 = math.radians(float(destinationCords[1]))

    dlon = lon2 - lon1
    dlat = lat2 - lat1

    a = math.sin(dlat / 2)**2 + math.cos(lat1) * \
            math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    distance = R * c * 1000  # convert km to m
    return distance



def getClosestPoint(WAYPOINTS, LAT, LON, BEARING): 
    closestPoint = None
    for point in WAYPOINTS.items():
        pointData = point[1]
        print(pointData)
        if BEARING in range(pointData['bearing'] - 50, pointData['bearing'] + 50):
            # Within tolerance
            print("target", pointData)
            
            distance = calculateDistance([LAT, LON], [pointData['lat'], pointData['lon']])
            print(distance)

            if (closestPoint != None):
                if (closestPoint[0] > distance):
                    closestPoint = [distance, pointData]
            else:
                closestPoint = [distance, pointData]
    return closestPoint


