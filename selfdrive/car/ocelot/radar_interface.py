#!/usr/bin/env python3
import time
from cereal import car
from selfdrive.car.interfaces import RadarInterfaceBase

class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    self.pts = {}
    self.delay = 0
    self.no_radar_sleep = 0
    self.radar_ts = 0

  def update(self, can_strings):
    return car.RadarData.new_message()