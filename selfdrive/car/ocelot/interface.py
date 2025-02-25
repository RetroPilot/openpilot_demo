#!/usr/bin/env python3
from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.car.ocelot.values import CAR, FINGERPRINTS
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint
from selfdrive.swaglog import cloudlog
from selfdrive.car.interfaces import CarInterfaceBase

EventName = car.CarEvent.EventName

class CarInterface(CarInterfaceBase):
  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 3.0

  @staticmethod
  def myround(x, base=5):
    return base * round(x/base)

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=[]):  # pylint: disable=dangerous-default-value
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)

    ret.carName = "ocelot"
    #TODO: ocelot panda safety. allOutput is kinda cursed
    ret.safetyModel = car.CarParams.SafetyModel.allOutput

    ret.steerActuatorDelay = 0.12  # Default delay, Prius has larger delay
    ret.steerLimitTimer = 0.4

    ret.lateralTuning.init('pid')
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
    stop_and_go = True
    ret.safetyParam = 100
    ret.wheelbase = 2.36
    ret.steerRatio = 21
    tire_stiffness_factor = 0.444
    ret.mass = 810 + STD_CARGO_KG
    ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.3], [0.05]]
    ret.lateralTuning.pid.kf = 0.00007   # full torque for 20 deg at 80mph means 0.00007818594

    ret.steerRateCost = 1.
    ret.centerToFront = ret.wheelbase * 0.44

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)
    
    # detect hardware
    ret.enableGasInterceptor = 0x201 in fingerprint[0]
    ret.enableSteerInterceptor = 0x301 in fingerprint[0]
    ret.enableGasActuator = 0x401 in fingerprint[0]
    ret.enableiBooster = 0x20F in fingerprint[0]

    # only enable long if the hardware is present
    # TODO: do something with this information
    ret.openpilotLongitudinalControl = ret.enableiBooster and (ret.enableGasInterceptor or ret.enableGasActuator)

    if ret.enableGasInterceptor:
      cloudlog.warning("ECU Gas Interceptor: %r", ret.enableGasInterceptor)
    if ret.enableSteerInterceptor:
      cloudlog.warning("ECU Steer Interceptor: %r", ret.enableSteerInterceptor)
    if ret.enableGasActuator:
      cloudlog.warning("ECU Gas Actuator: %r", ret.enableGasActuator)
    if ret.enableiBooster:
      cloudlog.warning("ECU iBooster: %r", ret.enableiBooster)

    # min speed to enable ACC. if car can do stop and go, then set enabling speed
    # to a negative value, so it won't matter.
    ret.minEnableSpeed = -1.

    ret.longitudinalTuning.deadzoneBP = [0., 9.]
    ret.longitudinalTuning.deadzoneV = [0., .15]
    ret.longitudinalTuning.kpBP = [0., 5., 35.]
    ret.longitudinalTuning.kiBP = [0., 35.]

    if ret.enableGasInterceptor:
      ret.gasMaxBP = [0., 9., 35]
      ret.gasMaxV = [0.2, 0.5, 0.7]
      ret.longitudinalTuning.kpV = [1.2, 0.8, 0.5]
      ret.longitudinalTuning.kiV = [0.18, 0.12]
    if ret.enableGasActuator:
      ret.gasMaxBP = [0.]
      ret.gasMaxV = [0.5]
      ret.longitudinalTuning.kpV = [3.6, 2.4, 1.5]
      ret.longitudinalTuning.kiV = [0.54, 0.36]
    else:
      # generic values for now, but need to handle no gas control
      ret.gasMaxBP = [0.]
      ret.gasMaxV = [0.5]
      ret.longitudinalTuning.kpV = [3.6, 2.4, 1.5]
      ret.longitudinalTuning.kiV = [0.54, 0.36]

    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    # ******************* do can recv *******************
    self.cp.update_strings(can_strings)
    self.cp_body.update_strings(can_strings)

    ret = self.CS.update(self.cp)

    ret.canValid = True #self.cp.can_valid
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    # events
    events = self.create_common_events(ret)

    ret.events = events.to_msg()

    self.CS.out = ret.as_reader()
    return self.CS.out

  # pass in a car.CarControl
  # to be called @ 100hz
  def apply(self, c):

    # simple!
    can_sends = self.CC.update(c.enabled, self.CS, self.frame,
                               c.actuators)

    self.frame += 1
    return can_sends
