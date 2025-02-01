#!/usr/bin/env python

# This process publishes joystick events. Such events can be suscribed by
# mocked car controller scripts.


### this process needs pygame and can't run on the EON ###

import pygame  # pylint: disable=import-error
import cereal.messaging as messaging

from tkinter import Tk, Canvas, Frame, BOTH
from PIL import Image, ImageTk

def main():

  # Tk stuff
  root = Tk()
  root.title("Joystick")
  root.geometry("1920x1024+300+300")
  canvas = Canvas()

  pil_img = Image.open("wheel.png").resize(size=(300, 300))

  canvas.create_line(15, 25, 200, 25)
  canvas.create_line(300, 35, 300, 200, dash=(4, 2))
  canvas.create_line(55, 85, 155, 85, 105, 180, 55, 85)

  # set up publishers and subscribers
  joystick_sock = messaging.pub_sock('testJoystick')
  sm = messaging.SubMaster(['carState'])

  pygame.init()

  # Used to manage how fast the screen updates
  clock = pygame.time.Clock()

  # Initialize the joysticks
  pygame.joystick.init()

  # Get count of joysticks
  joystick_count = pygame.joystick.get_count()
  if joystick_count > 1:
    raise ValueError("More than one joystick attached")
  elif joystick_count < 1:
    raise ValueError("No joystick found")

  # -------- Main Program Loop -----------
  inc = 0
  while True:
    # EVENT PROCESSING STEP
    for event in pygame.event.get():  # User did something
      if event.type == pygame.QUIT:  # If user clicked close
        pass
      # Available joystick events: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
      if event.type == pygame.JOYBUTTONDOWN:
        print("Joystick button pressed.")
      if event.type == pygame.JOYBUTTONUP:
        print("Joystick button released.")

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    # Usually axis run in pairs, up/down for one, and left/right for
    # the other.
    axes = []
    buttons = []

    for a in range(joystick.get_numaxes()):
      axes.append(joystick.get_axis(a))

    for b in range(joystick.get_numbuttons()):
      buttons.append(bool(joystick.get_button(b)))

    dat = messaging.new_message('testJoystick')
    dat.testJoystick.axes = axes
    dat.testJoystick.buttons = buttons
    joystick_sock.send(dat.to_bytes())

    # axis correction
    axes[2] = (axes[2]+1.0)/2
    axes[5] = (axes[5]+1.0)/2

    # Limit to 100 frames per second
    clock.tick(100)

    # update the tk window
    canvas.delete("all")
    # image
    pil_img_rot = pil_img.rotate(360*-axes[0])
    tk_img = ImageTk.PhotoImage(pil_img_rot)
    canvas.create_image(300, 300, image=tk_img)
    # bar graphs
    canvas.create_line(300, 600, 300+(axes[0]*100), 600, width=20) #steer
    canvas.create_line(200, 900, 200, 900 - (axes[2]*200), width=20) # brake
    canvas.create_line(400, 900, 400, 900 - (axes[5]*200), width=20) # gas
    # text
    canvas.create_text(300, 620, text=f"Steering: {axes[0]}")
    canvas.create_text(200, 920, text=f"Brake: {axes[2]}")
    canvas.create_text(400, 920, text=f"Gas: {axes[5]}")
    canvas.pack(fill=BOTH, expand=1)
    root.update()

    sm.update()
    for which, updated in sm.updated.items():
      if updated and (which == 'carState'):
        cs = sm[which]
        steer_angle = cs.steeringAngleDeg
        speed_mph = cs.vEgo * 3.6 * (1/1.609344)
        gear = cs.gearShifter


    #axes[0] = steer
    #axes[2]+1.0 = brake
    #axes[5]+1.0 = gas

if __name__ == "__main__":
  main()
