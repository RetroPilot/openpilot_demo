#!/usr/bin/env python3
import math
import numpy
from selfdrive.swaglog import cloudlog


"""

  30.573193, -96.394522
 name: lightsy lane exit
 bearing: 20.49


 30.568634, -96.389022
 name: stopEast
 bearing: 139.59

30.568672, -96.388921
 name: stopnorth
 bearing: 211.48


30.568577, -96.388854
 name stopwest
 bearing: 305


30.568541, -96.388992
 name stopsouth
 bearing: 8.3
"""

"""
 # Air bnb
            "30.573193, -96.394522": {
            "lat": "30.573193",
            "lon": "-96.394522",
            "name": "Lighsy Lane entry",
            "bearing": 20,
            "bearing_deviation": 5,
            "road": "NA"
        },

        # North south
       



"""


class gpsPlanner():

    def __init__(self):
        self.STOPPING_DISTANCE = 3
        self.STOPPED_COUNTER = 0
        self.SLOWDOWN_DISTANCE = 20
        self.BEARING_TOLERANCE = 5
        self.STOP_INITIATED = False
        self.MIN_STOP_DURATION = 60
        self.DECEL_RATE_TIME = 10




        self.distanceToStopTracker = 9999
        self.distanceToStopBreaches = 0


        self.bearing = 0
        self.speed = 0

        self.gps = {

        # Lane
        "30.568634, -96.389022": {
            "lat": 30.568634,
            "lon": -96.389022,
            "name": "Stop East",
            "bearing": 130,
            "bearing_deviation": 10,
            "street": "Lightsey Lane"
        },

        "30.568577, -96.388854": {
            "lat": 30.568577,
            "lon": -96.388854,
            "name": "Stop West",
            "bearing": 301,
            "bearing_deviation": 10,
            "street": "Lightsey Lane"
        },
         "30.568541, -96.388992": {
            "lat": 30.568541,
            "lon": -96.388992,
            "name": "Stop South",
            "bearing": 17,
            "bearing_deviation": 5,
            "road": "River Road"
        },
        "30.568672, -96.388921": {
            "lat": 30.568672,
            "lon": -96.388921,
            "name": "Stop North",
            "bearing": 211,
            "bearing_deviation": 5,
            "road": "River Road"
        },
    }

    #def make_radar(self, distance):
    #    return {'dRel': distance, 'yRel': -0.7599999904632568, 'vRel': 0.0, 'vLead': -1.064276618223392e-11, 'vLeadK': 1.3886331114062307e-07, 'aLeadK': 1.0457257474360904e-06, 'status': True, 'fcw': False, 'modelProb': 0.0, 'radar': True, 'aLeadTau': 1.5}

    def distance_gps(self, curLat, curLon, desLat, desLon):
        # approximate radius of earth in km
        R = 6373.0

        lat1 = math.radians(float(curLat))
        lon1 = math.radians(float(curLon))
        lat2 = math.radians(float(desLat))
        lon2 = math.radians(float(desLon))

        dlon = lon2 - lon1
        dlat = lat2 - lat1

        a = math.sin(dlat / 2)**2 + math.cos(lat1) * \
            math.cos(lat2) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        distance = R * c * 1000  # convert km to m
        return distance

    def is_gentle_stop(self, distance):
        if (self.speed > 0):
            speedMS = float(self.speed) / 2.237

            if (distance > 0 and speedMS > 0):
                TT3 = distance/speedMS

                return TT3 < self.DECEL_RATE_TIME
        # Somethings zero, can stop.
        return True


    def possibleStop(self, point, sm):
        if (True):
            self.speed = float(sm['carState'].wheelSpeeds.fr)

            if (self.STOPPED_COUNTER > 0):
                print("STOP COUNTER: ", str(self.STOPPED_COUNTER))

            # Calculating distance to the stop
            distanceToStop = self.distance_gps(
                self.lat, self.lon, point['lat'], point['lon']
            )

            # this will tell us if we should start feeding this data
            derrivedStopDistance = self.is_gentle_stop(distanceToStop)

            self.distanceToStopTracker = distanceToStop
            
            # We want to slow down prior to coming to a complete st
            if (derrivedStopDistance and
                #distanceToStop < self.SLOWDOWN_DISTANCE and
                self.STOPPED_COUNTER < self.MIN_STOP_DURATION and
                distanceToStop > self.STOPPING_DISTANCE and 
                self.speed > 0):
                cloudlog.debug("SLOW DOWN, STOP AHEAD")

                # Return how many meters to allow a gentle slowdown
                return distanceToStop

            if ((distanceToStop < self.STOPPING_DISTANCE and
            self.STOPPED_COUNTER < self.MIN_STOP_DURATION) or
            (self.STOP_INITIATED == True and
            self.STOPPED_COUNTER < self.MIN_STOP_DURATION)):

                cloudlog.debug(distanceToStop)
                if (self.speed <= 0):
                    cloudlog.debug("+1")
                    self.STOPPED_COUNTER += 1
                # Vehicle should stop
                cloudlog.debug("STOP")
                self.STOP_INITIATED = True
                # Return true since we want to stop ASAP
                return 0
            else:

                # We want to clear the stop counter once you're far enough away from the target
                if (distanceToStop > self.SLOWDOWN_DISTANCE and self.STOPPED_COUNTER > 0):
                    self.STOPPED_COUNTER = 0
                    cloudlog.debug("RESET COUNTER")

                # Stopped counter has finished, cleaning up state
                if (self.STOPPED_COUNTER >= self.MIN_STOP_DURATION):
                    self.STOP_INITIATED = False
                    self.distanceToStopTracker = 9999
                    cloudlog.debug("RESET STOP FORCE")

                return None

    def getClosestPoint(lat, lon):
        return [30.563902095024126, -96.25004729897357]

    def update(self, sm):
        return None
        # values to update:

        # longitudinalPlan.vCruise = float(0)
        # longitudinalPlan.aCruise = float(0)
        # longitudinalPlan.vStart = float(0)
        # longitudinalPlan.aStart = float(0)
        # longitudinalPlan.vTarget = float(0)
        # longitudinalPlan.aTarget = float(0)
        # longitudinalPlan.vTargetFuture = float(0)

        gpsLocationExternal = sm["gpsLocationExternal"]
        carState = sm["carState"]
        self.lat = gpsLocationExternal.latitude
        self.lon = gpsLocationExternal.longitude
        if (self.speed > 0.1):
            self.bearing = gpsLocationExternal.bearingDeg
        
        self.speed = float(sm['carState'].wheelSpeeds.fr) 
        if (float(gpsLocationExternal.accuracy) > 5.0):
            cloudlog.warn("POOR GPS ACCURACY: "+ str(gpsLocationExternal.accuracy))
        # Checking points to find ones on the correct bearing for us 
        #print("current bearing: " + str(self.bearing))
        # print("speed: " + str())


        if (carState.cruiseState.enabled != True and True == False):
            self.STOPPED_COUNTER = 0
            self.STOP_INITIATED = 0
            self.distanceToStopBreaches = 0
            self.distanceToStopTracker = 0
            return None


        for point in self.gps.items():
            pointData = point[1]
            if int(self.bearing) in range(pointData['bearing'] - pointData['bearing_deviation'], pointData['bearing'] + pointData['bearing_deviation']):
                # Within tolerance
                distanceToStop = self.possibleStop(pointData, sm)

                if (distanceToStop != None):
                    print("DIST FROM STOP: ", distanceToStop, pointData['name'])
                    return distanceToStop
                return None
        return None
