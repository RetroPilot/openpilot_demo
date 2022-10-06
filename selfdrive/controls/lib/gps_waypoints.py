from math import sin, cos, sqrt, atan2, radians

class gpsPlanner():

    STOPPING_DISTANCE = 7.5
    stoppedCounter = 0

    def __init__(self, CP):
        self.CP = CP

    def distance_gps(curLat, curLon, desLat, desLon):

        # approximate radius of earth in km
        R = 6373.0

        lat1 = math.radians(curLat)
        lon1 = math.radians(curLon)
        lat2 = math.radians(desLat)
        lon2 = math.radians(desLon)

        dlon = lon2 - lon1
        dlat = lat2 - lat1

        a = math.sin(dlat / 2)**2 + math.cos(lat1) * \
            math.cos(lat2) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        distance = R * c * 1000  # convert km to m
        return distance

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
        coords = gpsLocation.positionGeodetic.value

        if len(coords) > 0:
            actualSpeed = float(sm['carState'].wheelSpeeds.fr)
            distanceToStop = distance_gps(
                             coords[0], coords[1], 30.563902095024126, -96.25004729897357) # these values should be pulled from a database
            if (distanceToStop < STOPPING_DISTANCE and stoppedCounter < 300):
                cloudlog.debug(distanceToStopOne)
                if (actualSpeed <= 0):
                    stoppedCounter += 1
                cloudlog.debug("STOP")
            else:
                if (distanceToStopOne > STOPPING_DISTANCE):
                    stoppedCounter = 0
