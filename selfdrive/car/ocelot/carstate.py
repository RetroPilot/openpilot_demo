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

    #Car specific information
    ret.doorOpen =any([cp_body.vl["DOORS"]['FL_DOOR'], cp_body.vl["DOORS"]['FR_DOOR'],
                        cp_body.vl["DOORS"]['RL_DOOR'], cp_body.vl["DOORS"]['RR_DOOR']]) != 0
    ret.seatbeltUnlatched = cp_body.vl["SEATBELTS"]["DRIVER_SEATBELT"]
    ret.leftBlinker = (cp_body.vl["SIGNAL_STALK"]['TURN_SIGNALS'] == 1)
    ret.rightBlinker = (cp_body.vl["SIGNAL_STALK"]['TURN_SIGNALS'] == 3)
    ret.espDisabled = False #cp_body.vl["ABS"]['ESP_STATUS']
    ret.wheelSpeeds.fl = cp_body.vl["WHEEL_SPEEDS"]['WHEEL_FL'] * CV.MPH_TO_MS
    ret.wheelSpeeds.fr = cp_body.vl["WHEEL_SPEEDS"]['WHEEL_FR'] * CV.MPH_TO_MS
    ret.wheelSpeeds.rl = cp_body.vl["WHEEL_SPEEDS"]['WHEEL_RL'] * CV.MPH_TO_MS
    ret.wheelSpeeds.rr = cp_body.vl["WHEEL_SPEEDS"]['WHEEL_RR'] * CV.MPH_TO_MS
    ret.brakeLights = cp_body.vl["GAS_BRAKE"]['BRAKE_PRESSED']
    can_gear = int(cp_body.vl["GEAR_PACKET"]['GEAR'])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    #Ibooster data
    ret.brakePressed = cp.vl["OCELOT_BRAKE_STATUS"]['DRIVER_BRAKE_APPLIED']

    ret.gas = ((cp_body.vl["PEDAL_GAS_SENSOR"]['PED_GAS'] / 2) + cp_body.vl["PEDAL_GAS_SENSOR"]['PED_GAS2']) / 2.
    ret.gasPressed = ret.gas > 260

    # ret.gas = 0
    # ret.gasPressed = False

    #calculate speed from wheel speeds
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.001

    #Toyota SAS
    ret.steeringAngleDeg = cp.vl["STEER_ANGLE_SENSOR"]['STEER_ANGLE']
    ret.steeringRateDeg = cp.vl["VSC"]['STEER_RATE']


    #Steering information from smart standin ECU
    raw_torque = 0
    if (cp.vl["STEER_TORQUE"]['DIRECTION'] == 1):
      raw_torque = cp.vl["STEER_TORQUE"]['DRIVER_TORQUE'] * -1
    else:
      raw_torque = cp.vl["STEER_TORQUE"]['DRIVER_TORQUE']
    ret.steeringTorque = raw_torque

    # TODO: handle steering actuator
    ret.steeringTorqueEps = raw_torque * 10 #cp.vl["STEERING_STATUS"]['STEER_TORQUE_EPS']
    ret.steeringPressed = False #abs(ret.steeringTorque) > STEER_THRESHOLD
    ret.steerWarning = False #cp.vl["STEERING_STATUS"]['STEERING_OK'] != 0

    main_on = bool(cp.vl["CRUISE_BUTTONS"]['MAIN_ON'])

    ret.cruiseState.available = main_on
    ret.cruiseState.standstill = False
    ret.cruiseState.nonAdaptive = False

    self.buttonStates["accelCruise"] = bool(cp.vl["CRUISE_BUTTONS"]['SET_PLUS'])
    self.buttonStates["decelCruise"] = bool(cp.vl["CRUISE_BUTTONS"]['SET_MINUS'])
    self.buttonStates["setCruise"] = bool(cp.vl["CRUISE_BUTTONS"]['CANCEL'])

    if not main_on:
      self.enabled = False

    if bool(cp.vl["CRUISE_BUTTONS"]['MAIN_ON']):
      if bool(self.buttonStates["setCruise"]) and not self.oldEnabled:
        self.enabled = not self.enabled
        if self.enabled:
            self.setSpeed = (int(round((ret.vEgo * CV.MS_TO_MPH)/5)) * 5)
            if ret.standstill:
              self.setSpeed = 10

    #increase or decrease speed in 5mph increments
    if cp.vl["CRUISE_BUTTONS"]['SET_PLUS']:
      self.setSpeed = self.setSpeed + 5

    if cp.vl["CRUISE_BUTTONS"]['SET_MINUS']:
      self.setSpeed = self.setSpeed - 5

    ret.cruiseState.speed = self.setSpeed * CV.MPH_TO_MS
    ret.cruiseState.enabled = self.enabled

    ret.stockAeb = False
    ret.leftBlindspot = False
    ret.rightBlindspot = False

    self.oldEnabled = bool(self.buttonStates["setCruise"])
    self.oldSpeedDn = bool(self.buttonStates["decelCruise"])
    self.oldSpeedUp = bool(self.buttonStates["accelCruise"])

    return ret



  @staticmethod
  def get_can_parser(CP):

    signals = [
      # sig_name, sig_address, default
      ("BRAKE_OK", "OCELOT_BRAKE_STATUS", 0),
      ("DRIVER_BRAKE_APPLIED", "OCELOT_BRAKE_STATUS", 0),
      ("PED_GAS", "PEDAL_GAS_SENSOR", 0),
      ("PED_GAS2", "PEDAL_GAS_SENSOR", 0),
    ]

    checks = [
      ("OCELOT_BRAKE_STATUS", 80),
      ("PEDAL_GAS_SENSOR", 50),
    ]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

  @staticmethod
  def get_body_can_parser(CP):

    signals = [
      ("MAIN_ON", "CRUISE_BUTTONS", 0),
      ("SET_MINUS", "CRUISE_BUTTONS", 0),
      ("SET_PLUS", "CRUISE_BUTTONS", 0),
      ("CANCEL", "CRUISE_BUTTONS", 0),
      ("STEER_ANGLE", "STEER_ANGLE_SENSOR", 0),  
      ("DIRECTION", "STEER_TORQUE", 0),
      ("DRIVER_TORQUE", "STEER_TORQUE", 0),
      ("BRAKE_PRESSED", "GAS_BRAKE", 0),
      ("FL_ROOR", "DOORS", 0),
      ("FR_ROOR", "DOORS", 0),
      ("RL_ROOR", "DOORS", 0),
      ("RR_ROOR", "DOORS", 0),
      ("DRIVER_SEATBELT", "SEATBELTS", 0),
      ("WHEEL_FL", "WHEEL_SPEEDS", 0),
      ("WHEEL_FR", "WHEEL_SPEEDS", 0),
      ("WHEEL_RL", "WHEEL_SPEEDS", 0),
      ("WHEEL_RR", "WHEEL_SPEEDS", 0),
      ("STEER_RATE", "VSC", 0),
      ("GEAR", "GEAR_PACKET", 0),
      ("TURN_SIGNALS", "SIGNAL_STALK", 0),
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
