#!/usr/bin/env python3
import os
import importlib
from multiprocessing import Process
from setproctitle import setproctitle

try:
  import usb1
  from usb1 import USBErrorIO, USBErrorOverflow  # pylint: disable=no-name-in-module
except Exception:
  pass

BASEDIR = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "../../"))

def f(proc):
    try:
        mod = importlib.import_module(proc)
        setproctitle(proc)
        mod.main()
    except KeyboardInterrupt:
        print("KEYBOARD INTERRUPT")
    except Exception:
        crash.capture_exception()
        raise

def g(pargs, cwd):
  os.chdir(cwd)
  os.execvp(pargs[0], pargs)

def can_init():
  # shamelessly stolen from boardd_old.py
  handle = None
  context = usb1.USBContext()
  #context.setDebug(9)

  for device in context.getDeviceList(skip_on_error=True):
    if device.getVendorID() == 0xbbaa and device.getProductID() == 0xddcc:
      handle = device.open()
      handle.claimInterface(0)
      handle.controlWrite(0x40, 0xdc, 17, 0, b'')
  if handle is None:
    exit(-1)

can_init()

joystick = Process(name = "joystickd", target=f, args=("joystickd",))
debug_controls = Process(name = "debug_controls", target=f, args=("debug_controls",))
boardd = Process(name = "boardd", target=g, args=(["./boardd"], os.path.join(BASEDIR, "selfdrive/boardd")))

boardd.start()
joystick.start()
debug_controls.start()

joystick.join()
debug_controls.join()
boardd.join()