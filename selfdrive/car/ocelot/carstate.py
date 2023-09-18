from cereal import car
from common.numpy_fast import mean
from opendbc.can.can_define import CANDefine
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from selfdrive.config import Conversions as CV
from selfdrive.car.ocelot.values import DBC, BUTTON_STATES


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    print(DBC[CP.carFingerprint]['chassis'])
    can_define = CANDefine(DBC[CP.carFingerprint]['chassis'])
    self.shifter_values = can_define.dv["GEAR_PACKET"]['GEAR']
    self.setSpeed = 0
    self.enabled = False
    self.oldEnabled = False
    self.oldSpeedUp = False
    self.oldSpeedDn = False
    self.setSpeed = 10
    self.buttonStates = BUTTON_STATES.copy()

  def update(self, cp, cp_body):
    ret = car.CarState.new_message()

    # Car specific information
    ret.doorOpen = False
    ret.seatbeltUnlatched = False
    ret.leftBlinker = False
    ret.rightBlinker = False
    ret.espDisabled = False #cp_body.vl["ABS"]['ESP_STATUS']
    ret.wheelSpeeds.fl = 60 * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = 60 * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = 60 * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = 60 * CV.KPH_TO_MS
    ret.brakeLights = False
    ret.gearShifter = 'D'

    # iBooster data
    ret.brakePressed = False

    ret.gas = 0
    ret.gasPressed = False

    # calculate speed from wheel speeds
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.001

    # Toyota SAS
    # Do we need an angle sensor for demo?
    ret.steeringAngleDeg = 0 # cp_body.vl["STEER_ANGLE_SENSOR"]['STEER_ANGLE']
    ret.steeringRateDeg = 0 # cp_body.vl["VSC"]['STEER_RATE']


    # Steering information from smart standin ECU
    raw_torque = 0
    # if (cp_body.vl["STEER_TORQUE"]['DIRECTION'] == 1):
    #   raw_torque = cp_body.vl["STEER_TORQUE"]['DRIVER_TORQUE'] * -1
    # else:
    #   raw_torque = cp_body.vl["STEER_TORQUE"]['DRIVER_TORQUE']
    ret.steeringTorque = raw_torque

    # TODO: handle steering actuator
    ret.steeringTorqueEps = raw_torque * 10 #cp.vl["STEERING_STATUS"]['STEER_TORQUE_EPS']
    ret.steeringPressed = False #abs(ret.steeringTorque) > STEER_THRESHOLD
    ret.steerWarning = False #cp.vl["STEERING_STATUS"]['STEERING_OK'] != 0

    main_on = bool(cp.vl["CRUISE_BUTTONS"]['MAIN_ON'])

    ret.cruiseState.available = main_on
    ret.cruiseState.standstill = False
    ret.cruiseState.nonAdaptive = False

    # self.buttonStates["accelCruise"] = bool(cp_body.vl["CRUISE_BUTTONS"]['SET_PLUS'])
    # self.buttonStates["decelCruise"] = bool(cp_body.vl["CRUISE_BUTTONS"]['SET_MINUS'])
    # self.buttonStates["setCruise"] = bool(cp_body.vl["CRUISE_BUTTONS"]['CANCEL'])

    # if not main_on:
    #   self.enabled = False

    # if bool(cp_body.vl["CRUISE_BUTTONS"]['MAIN_ON']):
    #   if bool(self.buttonStates["setCruise"]) and not self.oldEnabled:
    #     self.enabled = self.enabled = not self.enabled
    #     if self.enabled:
    #         self.setSpeed = (int(round((ret.vEgo * CV.MS_TO_MPH)/5)) * 5)
    #         if ret.standstill:
    #           self.setSpeed = 10

    # #increase or decrease speed in 5mph increments
    # if cp_body.vl["CRUISE_BUTTONS"]['SET_PLUS']:
    #   self.setSpeed = self.setSpeed + 5

    # if cp_body.vl["CRUISE_BUTTONS"]['SET_MINUS']:
    #   self.setSpeed = self.setSpeed - 5

    ret.cruiseState.speed = cp.vl["CRUISE_STATE"]["SPEED"]
    ret.cruiseState.enabled = main_on

    ret.stockAeb = False
    ret.leftBlindspot = False
    ret.rightBlindspot = False

    # self.oldEnabled = bool(self.buttonStates["setCruise"])
    # self.oldSpeedDn = bool(self.buttonStates["decelCruise"])
    # self.oldSpeedUp = bool(self.buttonStates["accelCruise"])

    return ret



  @staticmethod
  def get_can_parser(CP):

    signals = [
      # sig_name, sig_address, default
      # ("BRAKE_OK", "OCELOT_BRAKE_STATUS", 0),
      # ("DRIVER_BRAKE_APPLIED", "OCELOT_BRAKE_STATUS", 0),
      # ("PED_GAS", "PEDAL_GAS_SENSOR", 0),
      # ("PED_GAS2", "PEDAL_GAS_SENSOR", 0),
      ("ENABLE", "CRUISE_STATE", 0),
      ("SPEED", "CRUISE_STATE", 0),
    ]

    checks = [
      #("OCELOT_BRAKE_STATUS", 80),
      #("PEDAL_GAS_SENSOR", 50),
      ("OCELOT_STEERING_STATUS", 80),
      ("CRUISE_STATE", 80)
    ]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 2)

  @staticmethod
  def get_body_can_parser(CP):

    signals = [
    ]

    # use steering message to check if panda is connected to frc
    checks = [
    ]

    return CANParser(DBC[CP.carFingerprint]['chassis'], signals, checks, 0)
