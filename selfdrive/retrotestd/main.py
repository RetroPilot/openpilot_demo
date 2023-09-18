#!/usr/bin/env python3
import os
import math
from cereal import car, log
from common.numpy_fast import clip
from common.realtime import sec_since_boot, config_realtime_process, Priority, Ratekeeper, DT_CTRL
from common.profiler import Profiler
from common.params import Params, put_nonblocking
import cereal.messaging as messaging
from selfdrive.config import Conversions as CV
from selfdrive.swaglog import cloudlog
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.car.car_helpers import get_car, get_startup_event, get_one_can
from selfdrive.controls.lib.lane_planner import CAMERA_OFFSET
from selfdrive.controls.lib.drive_helpers import update_v_cruise, initialize_v_cruise
from selfdrive.controls.lib.longcontrol import LongControl, STARTING_TARGET_SPEED
from selfdrive.controls.lib.latcontrol_pid import LatControlPID
from selfdrive.controls.lib.latcontrol_indi import LatControlINDI
from selfdrive.controls.lib.latcontrol_lqr import LatControlLQR
from selfdrive.controls.lib.latcontrol_angle import LatControlAngle
from selfdrive.controls.lib.events import Events, ET
from selfdrive.controls.lib.alertmanager import AlertManager
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.controls.lib.longitudinal_planner import LON_MPC_STEP
from selfdrive.locationd.calibrationd import Calibration
from selfdrive.hardware import HARDWARE, TICI

LDW_MIN_SPEED = 31 * CV.MPH_TO_MS
LANE_DEPARTURE_THRESHOLD = 0.1
STEER_ANGLE_SATURATION_TIMEOUT = 1.0 / DT_CTRL
STEER_ANGLE_SATURATION_THRESHOLD = 2.5  # Degrees

SIMULATION = "SIMULATION" in os.environ
NOSENSOR = "NOSENSOR" in os.environ
IGNORE_PROCESSES = set(["rtshield", "uploader", "deleter", "loggerd", "logmessaged", "tombstoned", "logcatd", "proclogd", "clocksd", "updated", "timezoned", "manage_athenad"])

ThermalStatus = log.DeviceState.ThermalStatus
State = log.ControlsState.OpenpilotState
PandaType = log.PandaState.PandaType
Desire = log.LateralPlan.Desire
LaneChangeState = log.LateralPlan.LaneChangeState
LaneChangeDirection = log.LateralPlan.LaneChangeDirection
EventName = car.CarEvent.EventName


class Retro:
  def __init__(self, sm=None, pm=None):

    # Setup sockets
    self.pm = pm
    if self.pm is None:
      self.pm = messaging.PubMaster(['retro'])

    self.sm = sm
    if self.sm is None:
      self.sm = messaging.SubMaster(['carState', 'modelV2',
                                    'longitudinalPlan', 'lateralPlan',
                                    'radarState'])
    self.SLOW_CAR = False

  def laneChange(self):
    radar = self.sm['radarState']
    model = self.sm['modelV2']
    lat = self.sm['lateralPlan']
    lines = model.laneLines

    if (len(lines) <= 0): return

    right_lane_visible = self.sm['lateralPlan'].rProb > 0.5
    left_lane_visible = self.sm['lateralPlan'].lProb > 0.5
    l_lane_close = left_lane_visible and (self.sm['modelV2'].laneLines[1].y[0] > -(1.08 + CAMERA_OFFSET))
    r_lane_close = right_lane_visible and (self.sm['modelV2'].laneLines[2].y[0] < (1.08 - CAMERA_OFFSET))
    dat = messaging.new_message('retro')
    dat.retro.cool = True


    self.pm.send('retro', dat)
      

    if (radar.leadOne.vRel < -2.7): #10km ish
      print("Slow Vehicle")







  def thread(self):
    while True:
      self.sm.update()
      self.laneChange()

def main(sm=None, pm=None):
  #retro = Retro(sm, pm)
  #retro.thread()
  print("ok")


if __name__ == "__main__":
  main()
