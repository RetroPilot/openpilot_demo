from cereal import car
from common.numpy_fast import mean
from opendbc.can.can_define import CANDefine
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from selfdrive.config import Conversions as CV
from selfdrive.car.ocelot.values import CAR, DBC, STEER_THRESHOLD


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    print(DBC[CP.carFingerprint]['chassis'])
    can_define = CANDefine(DBC[CP.carFingerprint]['chassis'])
    self.shifter_values = can_define.dv["GEAR_PACKET"]['GEAR']
    self.setSpeed = 0
    self.enabled = False
    self.oldEnabled = False

  def update(self, cp, cp_body):
    ret = car.CarState.new_message()

    #Car specific information
    if self.CP.carFingerprint == CAR.SMART_ROADSTER_COUPE:
        ret.doorOpen = False #any([cp_body.vl["BODYCONTROL"]['RIGHT_DOOR'], cp_body.vl["BODYCONTROL"]['LEFT_DOOR']]) != 0
        ret.seatbeltUnlatched = False
        ret.leftBlinker = False #cp_body.vl["BODYCONTROL"]['LEFT_SIGNAL']
        ret.rightBlinker = False #cp_body.vl["BODYCONTROL"]['RIGHT_SIGNAL']
        ret.espDisabled = False #cp_body.vl["ABS"]['ESP_STATUS']
        ret.wheelSpeeds.fl = 0 #cp_body.vl["SMARTROADSTERWHEELSPEEDS"]['WHEELSPEED_FL'] * CV.MPH_TO_MS
        ret.wheelSpeeds.fr = 0 #cp_body.vl["SMARTROADSTERWHEELSPEEDS"]['WHEELSPEED_FR'] * CV.MPH_TO_MS
        ret.wheelSpeeds.rl = 0 #cp_body.vl["SMARTROADSTERWHEELSPEEDS"]['WHEELSPEED_RL'] * CV.MPH_TO_MS
        ret.wheelSpeeds.rr = 0 #cp_body.vl["SMARTROADSTERWHEELSPEEDS"]['WHEELSPEED_RR'] * CV.MPH_TO_MS
        ret.brakeLights = False #cp_body.vl["ABS"]['BRAKEPEDAL']
        can_gear = 0 #int(cp_body.vl["GEARBOX"]['GEARPOSITION'])
        ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    #Ibooster data
    ret.brakePressed = False #cp.vl["BRAKE_STATUS"]['IBOOSTER_BRAKE_APPLIED']

    # if CP.enableGasInterceptor:
    #   ret.gas = (cp_body.vl["GAS_SENSOR"]['PED_GAS'] + cp_body.vl["GAS_SENSOR"]['PED_GAS2']) / 2.
    #   ret.gasPressed = ret.gas > 15

    ret.gas = 0
    ret.gasPressed = False

    #calculate speed from wheel speeds
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.001

    #Toyota SAS
    ret.steeringAngleDeg = 0 #cp.vl["TOYOTA_STEERING_ANGLE_SENSOR1"]['TOYOTA_STEER_ANGLE'] + cp.vl["TOYOTA_STEERING_ANGLE_SENSOR1"]['TOYOTA_STEER_FRACTION']
    ret.steeringRateDeg = 0 #cp.vl["TOYOTA_STEERING_ANGLE_SENSOR1"]['TOYOTA_STEER_RATE']


    #Steering information from smart standin ECU
    ret.steeringTorque = 0 #cp.vl["STEERING_STATUS"]['STEER_TORQUE_DRIVER']
    ret.steeringTorqueEps = 0 #cp.vl["STEERING_STATUS"]['STEER_TORQUE_EPS']
    ret.steeringPressed = False #abs(ret.steeringTorque) > STEER_THRESHOLD
    ret.steerWarning = False #cp.vl["STEERING_STATUS"]['STEERING_OK'] != 0

    ret.cruiseState.available = True
    ret.cruiseState.standstill = False
    ret.cruiseState.nonAdaptive = False

    #Logic for OP to manage whether it's enabled or not as controls board only sends button inputs
    self.oldEnabled = self.enabled

    self.setSpeed = ret.cruiseState.speed
    #if enabled from off (rising edge) set the speed to the current speed rounded to 5mph
    if self.enabled and not(self.oldEnabled):
        ret.cruiseState.speed = (self.myround((ret.vEgo * CV.MS_TO_MPH), 5)) * CV.MPH_TO_MS

    #increase or decrease speed in 5mph increments
    # if cp.vl["HIM_CTRLS"]['SPEEDUP_BTN']:
    #     ret.cruiseState.speed = self.setSpeed + 5*CV.MPH_TO_MS

    # if cp.vl["HIM_CTRLS"]['SPEEDDN_BTN']:
    #     ret.cruiseState.speed = self.setSpeed - 5*CV.MPH_TO_MS

    ret.cruiseState.enabled = self.enabled


    return ret



  @staticmethod
  def get_can_parser(CP):

    signals = [
      # sig_name, sig_address, default
      ("BRAKE_OK", "ACTUATOR_BRAKE_STATUS", 0),
      ("STEERING_OK", "ACTUATOR_STEERING_STATUS", 0),
    ]

    checks = [
      ("ACTUATOR_BRAKE_STATUS", 80),
    ]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

  @staticmethod
  def get_body_can_parser(CP):

    signals = [
    ]

    # use steering message to check if panda is connected to frc
    checks = [
    ]

    # if CP.carFingerprint == CAR.SMART_ROADSTER_COUPE:
    #     signals.append(("RIGHT_DOOR", "BODYCONTROL",0))
    #     signals.append(("LEFT_DOOR", "BODYCONTROL",0))
    #     signals.append(("LEFT_SIGNAL", "BODYCONTROL",0))
    #     signals.append(("RIGHT_SIGNAL", "BODYCONTROL",0))
    #     signals.append(("ESP_STATUS", "ABS",0))
    #     signals.append(("WHEELSPEED_FL", "SMARTROADSTERWHEELSPEEDS",0))
    #     signals.append(("WHEELSPEED_FR", "SMARTROADSTERWHEELSPEEDS",0))
    #     signals.append(("WHEELSPEED_RL", "SMARTROADSTERWHEELSPEEDS",0))
    #     signals.append(("WHEELSPEED_RR", "SMARTROADSTERWHEELSPEEDS",0))
    #     signals.append(("BRAKEPEDAL", "ABS",0))
    #     signals.append(("GEARPOSITION","GEARBOX", 0))

    return CANParser(DBC[CP.carFingerprint]['chassis'], signals, checks, 1)
