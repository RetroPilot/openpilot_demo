#!/usr/bin/env python

from tkinter import Tk, Canvas, Frame, BOTH
import cereal.messaging as messaging

class Example(Frame):

  def __init__(self):
    super().__init__()

    self.initUI()


  def initUI(self):

    self.master.title("Lines")
    self.pack(fill=BOTH, expand=1)

    canvas = Canvas(self)
    canvas.create_line(15, 25, 200, 25)
    canvas.create_line(300, 35, 300, 200, dash=(4, 2))
    canvas.create_line(55, 85, 155, 85, 105, 180, 55, 85)

    canvas.pack(fill=BOTH, expand=1)

def main()
  
  joystick_sock = messaging.sub_sock_sock('testJoystick')

  # Tkinter visuals
  root = Tk()
  ex = Example()
  root.geometry=("400x250+300+300")
  root.mainloop()

if __name__ == "__main__":
  main()