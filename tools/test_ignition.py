#! /usr/bin/env python3

import cereal.messaging as messaging
import time

if __name__ == "__main__":
  health_sock = messaging.pub_sock('pandaState')

  try:
    while 1:

      msg = messaging.new_message('pandaState')
    
      msg.pandaState.pandaType = 3

      msg.pandaState.voltage = 12.0
      msg.pandaState.current = 1.0
      msg.pandaState.ignitionLine = True
      msg.pandaState.ignitionCan = True
      msg.pandaState.controlsAllowed = True

      health_sock.send(msg.to_bytes())

      time.sleep(0.1)
  except KeyboardInterrupt:
      msg = messaging.new_message('pandaState')

      msg.pandaState.pandaType = 3

      msg.pandaState.voltage = 12.0
      msg.pandaState.current = 1.0
      msg.pandaState.ignitionLine = False
      msg.pandaState.ignitionCan = False
      msg.pandaState.controlsAllowed = False

      health_sock.send(msg.to_bytes())
