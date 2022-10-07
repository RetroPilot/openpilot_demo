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


class gpsPlanner():

    

    def __init__(self, CP):
        self.CP = CP

        self.STOPPING_DISTANCE = 7.5
        self.STOPPED_COUNTER = 0
        self.SLOWDOWN_DISTANCE = 20
        self.BEARING_TOLERANCE = 5


        self.lastCord = None

        self.bearing = None

        self.gps = {

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
        "30.568541, -96.388992": {
            "lat": 30.568541,
            "lon": -96.388992,
            "name": "Stop South",
            "bearing": 8,
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

        # Lane
        "30.568634, -96.389022": {
            "lat": 30.568634,
            "lon": -96.389022,
            "name": "Stop East",
            "bearing": 139,
            "bearing_deviation": 5,
            "name": "Lightsey Lane"
        },
        "30.568577, -96.388854": {
            "lat": 30.568577,
            "lon": -96.388854,
            "name": "Stop West",
            "bearing": 305,
            "bearing_deviation": 5,
            "name": "Lightsey Lane"
        },

        "30.564655, -": {
            "lat": 30.564655,
            "lon": -96.248350,
            "name": "Stop South yield TEST",
            "bearing": 168,
            "bearing_deviation": 5,
            "name": "Lapsis Court"
        }
    }


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

    def get_bearing(self, now, future):
        lat1 = now[0]
        long1 = now[1]
        lat2 = future[0]
        long2 = future[1]

        dLon = (long2 - long1)
        x = math.cos(math.radians(lat2)) * math.sin(math.radians(dLon))
        y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(
            math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(dLon))
        brng = numpy.arctan2(x, y)
        brng = numpy.degrees(brng)


        return abs(brng)

    def possibleStop(self, coords, bearing, point, sm):
        if (len(coords) > 0):
            actualSpeed = float(sm['carState'].wheelSpeeds.fr)

            # Calculating distance to the stop
            distanceToStop = self.distance_gps(
                coords[0], coords[1], point['lat'], point['lon']
            )
            
            # We want to slow down prior to coming to a complete st
            if (distanceToStop < self.SLOWDOWN_DISTANCE and
                self.STOPPED_COUNTER < 300 and
                distanceToStop > self.STOPPING_DISTANCE):

                # not sure how we're feeding this data back into long planner yet
                cloudlog.debug("SLOW DOWN, STOP AHEAD")

            if (distanceToStop < self.STOPPING_DISTANCE and self.STOPPED_COUNTER < 300):
                cloudlog.debug(distanceToStop)
                if (actualSpeed <= 0):
                    self.STOPPED_COUNTER += 1
                # Vehicle should stop
                cloudlog.debug("STOP")
            else:
                if (distanceToStop > self.STOPPING_DISTANCE):
                    self.STOPPED_COUNTER = 0

    def getClosestPoint(lat, lon):
        return [30.563902095024126, -96.25004729897357]

    def update(self, sm, CP):

        # values to update:

        # longitudinalPlan.vCruise = float(0)
        # longitudinalPlan.aCruise = float(0)
        # longitudinalPlan.vStart = float(0)
        # longitudinalPlan.aStart = float(0)
        # longitudinalPlan.vTarget = float(0)
        # longitudinalPlan.aTarget = float(0)
        # longitudinalPlan.vTargetFuture = float(0)

        gpsLocation = sm["liveLocationKalman"]
        #gpsLocationExternal = sm["gpsLocationExternal"]
        #print(gpsLocationExternal)
        print(gpsLocation.orientationNED)

        coords = gpsLocation.positionGeodetic.value

        bearing = None

        # Calculate bearing
        if (len(coords) > 0 and self.lastCord != None and len(self.lastCord) > 0):
            cloudlog.debug("last cord: " +str(self.lastCord) + " coords" + str(coords))
            debug = self.get_bearing(self.lastCord, coords)

            if (debug != 0.0):
                self.bearing = debug

            cloudlog.debug("BEARING: "+str(self.bearing))

        # Checking points to find ones on the correct bearing for us 
        for point in self.gps.items():
            pointData = point[1]
            if self.bearing in range(pointData['bearing'] - pointData['bearing_deviation'], pointData['bearing'] + pointData['bearing_deviation']):
                # Within tolerance
                if (self.bearing != None):
                    self.possibleStop(coords, self.bearing, pointData, sm)


        # Set lastCord to be used later to calculate bearing
        self.lastCord = coords

