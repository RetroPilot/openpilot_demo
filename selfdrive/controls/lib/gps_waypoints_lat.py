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


class gpsPlannerLat():

    def __init__(self):
        self.bearing = 0
        self.speed = 0

        self.track = [[0, 0], [0, 0], [0, 0]]

        self.tick = 0

        self.bend = [
            [30.5686366, -96.3890039],

            [30.5685806, -96.3889765],

            [30.568456, -96.3890158],
        ]

        self.running = False 

        self.gps = [

            [52.482408, 1.728524],
            [52.482499, 1.728568],
            [52.482530, 1.728732]
            # 0.0
            # 0.01123534650226293
            # 0.018558604978158973

            # [30.568658, -96.389044],
            # [30.568612, -96.388990],
            # [30.568561, -96.388976],
            # [30.568505, -96.388990],
            # [30.568475, -96.388999]
            #
            # 0.0
            # 0.00603144303160978
            # 0.0016797706265936508
            # 0.0017039777444447322
            # 0.0010673816567190872

            # [ 30.572882, -96.394014 ],
            # [ 30.572841, -96.393960 ],
            # [ 30.572812, -96.393920 ]

            # 0.0
            # 0.00602595244722111
            # 0.004462270554545567

        ]

        self.delay = 0

        self.curPoint = 0
        self.enabled = False

    def closest_node(self, node, nodes):
        nodes = numpy.asarray(nodes)
        deltas = nodes - node
        dist_2 = numpy.einsum('ij,ij->i', deltas, deltas)
        return numpy.argmin(dist_2)

    def get_bearing(self, lat1, long1, lat2, long2):
        dLon = (long2 - long1)
        x = math.cos(math.radians(lat2)) * math.sin(math.radians(dLon))
        y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(
            math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(dLon))
        brng = numpy.arctan2(x, y)
        brng = numpy.degrees(brng)

        return brng

    def test(self, gps0, gps1, gps2):
        x1 = float(gps0[0])
        y1 = float(gps0[1])

        x2 = float(gps1[0])
        y2 = float(gps1[1])

        x3 = float(gps2[0])
        y3 = float(gps2[1])

        xCoefficientArray = [x2 - x1, y2 - y1]
        yCoefficientArray = [x3 - x1, y3 - y1]

        coefficientArray = numpy.array(
            [xCoefficientArray, yCoefficientArray])
        constantArray = numpy.array([(pow(x2, 2) + pow(y2, 2) - pow(x1, 2) - pow(
            y1, 2))/2, (pow(x3, 2) + pow(y3, 2) - pow(x1, 2) - pow(y1, 2))/2])

        try:
            center = numpy.linalg.solve(
                coefficientArray, constantArray)

            return (math.sqrt(pow(x1-center[0], 2) + pow(y1-center[1], 2)) + math.sqrt(pow(x2-center[0], 2) + pow(y2-center[1], 2)) + math.sqrt(pow(x3-center[0], 2) + pow(y3-center[1], 2)) / 3)

            print("------------------------------------------------")
        except:
            print("straight!!!")

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

    def update(self, sm):
        return True;
        gpsLocationExternal = sm["gpsLocationExternal"]
        carState = sm["carState"]
        self.lat = gpsLocationExternal.latitude
        self.lon = gpsLocationExternal.longitude

        print(self.lat, self.lon)






        distanceToCurve = self.distance_gps(self.lat, self.lon, 31.0767027, -97.600979)
        distanceToCurveEnd = self.distance_gps(self.lat, self.lon, 31.076610799999997, -97.6011061)

        print("start dis", distanceToCurve)
        print("stop dis", distanceToCurveEnd)
        if (distanceToCurve < 0.25):
            print("RUNNING")
            self.running = True

        if (distanceToCurve > 10.):
            print("NOT RUNNING")
            self.running = False


        if (self.running == True):
            print("TURNING")
            return 0.06557377
        else:
            return None




        try:
            self.file = open('/data/angle.txt')
            tuneFile = self.file.read()
            if tuneFile is not None:
                return float(tuneFile)
        except:
            print("BAD FILE....")
            return None

        # values to update:

        # longitudinalPlan.vCruise = float(0)
        # longitudinalPlan.aCruise = float(0)
        # longitudinalPlan.vStart = float(0)
        # longitudinalPlan.aStart = float(0)
        # longitudinalPlan.vTarget = float(0)
        # longitudinalPlan.aTarget = float(0)
        # longitudinalPlan.vTargetFuture = float(0)

       

        return None
        if (self.speed > 0.1):
            self.bearing = gpsLocationExternal.bearingDeg

        self.speed = float(sm['carState'].wheelSpeeds.fr)
        if (float(gpsLocationExternal.accuracy) > 5.0):
            cloudlog.warn("POOR GPS ACCURACY: " +
                          str(gpsLocationExternal.accuracy))
        # Checking points to find ones on the correct bearing for us
        # print("current bearing: " + str(self.bearing))
        # print("speed: " + str())

        points = []

        # self.tick = self.tick + 1

        # if (self.tick > 20):
        #    self.tick = 0
        # else:
        #    return

        # self.track.append([self.lat, self.lon])

        # total = len(self.track) - 1

        closestNode = self.closest_node([self.lat, self.lon], self.bend)
        closestPoint = self.bend[closestNode]

        distanceToPoint = self.distance_gps(
            self.lat, self.lon, self.bend[0][0], self.bend[0][1])

       # print("distance to point: ", distanceToPoint, " closest point: ", closestPoint)

        # + 1 because its len so key 0 = 1
        # if (distanceToPoint < 1):
        #
        #

        #self.tick = self.tick + 1

        print("------------------")

        # for x in range(110):
        #print("{:.20f}".format(self.test(self.bend[0], self.bend[50], self.bend[100])) )

        # return self.test(self.bend[0], self.bend[1], self.bend[2])

        # return self.tick/1000

        #k = self.test(self.bend[0], self.bend[99], self.bend[100]) * 3
        # print(k)
        # return k

        # if (carState.cruiseState.enabled and self.curPoint + 1 < len(self.bend)):
        #    self.curPoint = self.curPoint + 1
        #    k = self.test(self.bend[self.curPoint], self.bend[len(self.bend)/2], self.bend[len(self.bend)])
        #    print(k, self.curPoint)
        #    return k
        # else:
        #    self.curPoint = 0

        # length = len(self.track) - 1
        # print("gpsd", self.haversine(
        #    self.track[length][0], self.track[length][1], self.lat, self.lon))
        # self.track.append([self.lat, self.lon])

        print("-----")
