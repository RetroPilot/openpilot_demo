#!/usr/bin/env python3
from fileinput import close
import math
import numpy
from selfdrive.swaglog import cloudlog
import time
import json
from selfdrive.controls.lib.helpers import getClosestPoint, isAppropriateStoppingDistance


# This basically just exists to be used as a radar point
# TODO: Provide more axises to allow the lead car marker to be correctly positioned on all axises
# TODO: Return confidence (derrived from GPS Accuracy and bearing accuracy)

class gpsPlanner():

    def __init__(self):
        # Navigation data
        self.LAT = None
        self.LON = None
        self.BEARING = 130 # Bearing from GPS (Reverse will invert) 
        self.SPEED = None # Vehicle speed m/s
        self.Enabled = False # Allows self disable for low GPS accuracy

        # Stop function data
        self.DISTANCE_TO_STOP = None # Meters to stop
        self.STOPPED_COUNTER = 0 # Counter to hold vehicle at stop for STOPPED_COUNTER_DURATION
        self.STOPPED = False # Vehicle speed 0 m/s
        self.CURRENT_STOP = None # Current point stopping at 
        self.LAST_STOP = None # Last point stopped at
        
        # Navigation points
        self.WAYPOINT_DATA_LAST_UPDATED = None # UNIX timestamp from last update
        self.WAYPOINTS = None # Waypoint data
        self.WAYPOINT_DATA_FS = open('/data/retropilot_waypoints.json') # FS location of json data

        # Configuration
        self.FULL_STOP_DISTANCE = 20 # Set by GPS Accuracy in runtime, default to 20
        self.STOPPED_COUNTER_DURATION = 60 # Could be set by each stop depending on conditions
        self.DECEL_RATE_TIME = 15


    def updateWaypoints(self):
        waypoint_string = self.WAYPOINT_DATA_FS.read()
        if waypoint_string is not None:
            self.WAYPOINTS = json.loads(waypoint_string)
            self.WAYPOINT_DATA_LAST_UPDATED = time.time()


    def distanceToNextStop(self): 
        closestPoint =  getClosestPoint(self.WAYPOINTS, self.LAT, self.LON, self.BEARING)
        print("returned point", closestPoint)
        if (closestPoint == None): return None
        if (closestPoint[0] and  isAppropriateStoppingDistance(closestPoint[0], self.SPEED, self.DECEL_RATE_TIME)):
            print("is good distance")
            self.CURRENT_STOP = closestPoint[1]
            self.DISTANCE_TO_STOP = closestPoint[0]
            return closestPoint[0]
        else:
            return None


    # Should be used as the entry point, will return None or a distance to stop
    def IsStopSignAhead(self):
        # is disabled so we won't continue
        if (self.Enabled == False): return None

        # Get distance to next stop
        self.distanceToNextStop()

        print(self.DISTANCE_TO_STOP)

        # Checking if a stop was identified and verifying counter is valid
        if (self.DISTANCE_TO_STOP and 
        self.CURRENT_STOP != None and 
        self.STOPPED_COUNTER <= self.STOPPED_COUNTER_DURATION):
        
            if ( isAppropriateStoppingDistance(self.DISTANCE_TO_STOP, self.SPEED, self.DECEL_RATE_TIME) == True):

                # Branch two ways:
                # 1: is within a reasonable stopping distance, start slowing
                # 2: vehicle needs to stop, is within FULL STOP DISTANCE
                if (self.DISTANCE_TO_STOP <= self.FULL_STOP_DISTANCE):
                    # We start our counter once the vehicle has stopped moving
                    if (self.SPEED <= 0):
                        self.STOPPED = True
                        self.STOPPED_COUNTER =+ 1
                    
                    return 0
                else:
                    # We don't really need to do any logic here, we'll just
                    # advise the distance to stop
                    return self.DISTANCE_TO_STOP
        else:

            if (self.STOPPED_COUNTER >= self.STOPPED_COUNTER_DURATION):
                self.stopSatisfied() # Resetting everything

            return None



    # Hard reset controller
    def reset(self):
        self.DISTANCE_TO_STOP = None
        self.STOPPED_COUNTER = 0
        self.STOPPED = False
        self.CURRENT_STOP = None
        self.LAST_STOP = None


    # Resets everyting and places current stop to last stop
    def stopSatisfied(self):

        # Set last stop to the current stop then clear it
        self.LAST_STOP = self.CURRENT_STOP
        self.CURRENT_STOP = None

        self.DISTANCE_TO_STOP = None
        self.STOPPED_COUNTER = 0
        self.STOPPED = False



    def update(self, sm):
        # Updating internal references
        gpsLocationExternal = sm["gpsLocationExternal"]
        carState = sm["carState"]


        # GPS
        self.LAT = gpsLocationExternal.latitude
        self.LON = gpsLocationExternal.longitude

        print(self.LAT, self.LON, self.BEARING)

        # Update speed
        self.SPEED = float(carState.wheelSpeeds.fr) 

        # Don't want to update bearing if we're going slow
        # as it will return inaccurate data
        if (self.SPEED  > 0.1):
            self.BEARING = gpsLocationExternal.bearingDeg
        
        if (self.WAYPOINTS == None or time.time() - self.WAYPOINT_DATA_LAST_UPDATED > 100): 
            print("updated waypoints")
            self.updateWaypoints()

        # Setting stopping distance to gps accuracy
        self.FULL_STOP_DISTANCE = gpsLocationExternal.accuracy / 2

        # Report low accuracy
        if (float(gpsLocationExternal.accuracy) > 1.5):
            cloudlog.warn("POOR GPS ACCURACY: "+ str(gpsLocationExternal.accuracy))
            self.Enabled = False
        else:
            self.Enabled = True


        return self.IsStopSignAhead()
