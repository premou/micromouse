#!/usr/bin/python2.7
#-*- coding: utf-8 -*-

##
# https://matplotlib.org/api/_as_gen/matplotlib.pyplot.show.html#matplotlib.pyplot.show
##

import sys
import datetime
import numpy as np
import matplotlib.pyplot as plt
import serial
from serial.tools import list_ports
from argparse import ArgumentParser
import io
import traceback
from tkinter import *
import time

# Debian installation command line
# apt install python-matplotlib

# Windows installation command line with pip
# pip install matplotlib
# pip install pyserial

# Global
global xKp
global xKi
global xKd
global wKp
global wKi
global wKd
global xSpeed
global wSpeed
global wt1
global wt2
global ut1
global ut2
global wheel_diameter
global sio

def GET():
  # Global
  global sio
  sio.write(str('get\n'))
  sio.flush()

def SET():
  # Global
  global xKp
  global xKi
  global xKd
  global wKp
  global wKi
  global wKd
  global xSpeed
  global wSpeed
  global wt1
  global wt2
  global ut1
  global ut2
  global wheel_diameter
  global sio
  
  cmd  = "set "
  cmd += xKp.get()    + " "    + xKi.get()    + " " + xKd.get() + " "
  cmd += wKp.get()    + " "    + wKi.get()    + " " + wKd.get() + " "
  cmd += xSpeed.get() + " " + wSpeed.get()                      + " "
  cmd += wt1.get()    + " "    + wt2.get()                      + " "       
  cmd += ut1.get()    + " "    + ut2.get()                      + " "
  cmd += wheel_diameter.get()                                   + '\n'
  sio.write(str(cmd))
  sio.flush()
	
def SAVE():
  global sio
  sio.write(str('save\n'))
  sio.flush()

# Main procedure
def main():
  # Global
  global xKp
  global xKi
  global xKd
  global wKp
  global wKi
  global wKd
  global xSpeed
  global wSpeed
  global wt1
  global wt2
  global ut1
  global ut2
  global wheel_diameter
  global sio

  # Global
  NB_SAMPLE_PER_LINE       = 13
  SPLIT_PATTERN            = ' '
  TELEMETRIE_START_PATTERN = 'LOG'
  TELEMETRIE_GET_PATTERN   = 'get'  
  TELEMETRIE_STOP_PATTERN  = 'EOL'

  # Available color
  # b : blue.
  # g : green.
  # r : red.
  # c : cyan.
  # m : magenta.
  # y : yellow.
  # k : black.
  # w : white.

  # Telemetrie context
  sampleCtx     = {}
  sampleCtx[0]  = { "title": "time",            "unit": "[ms]",    "color": "w",    "enable": 0, "factor": 1}  
  sampleCtx[1]  = { "title": "action number",   "unit": "",        "color": "--k",  "enable": 1, "factor": 10}
  sampleCtx[2]  = { "title": "phase",           "unit": "",        "color": "m",    "enable": 1, "factor": 100}
  sampleCtx[3]  = { "title": "target speed",    "unit": "[mm/ms]", "color": "-b",   "enable": 1, "factor": 1}
  sampleCtx[4]  = { "title": "set point speed", "unit": "[mm/ms]", "color": "--r",  "enable": 1, "factor": 1}
  sampleCtx[5]  = { "title": "real speed",      "unit": "[mm/ms]", "color": ":g",   "enable": 1, "factor": 1}
  sampleCtx[6]  = { "title": "pwm speed",       "unit": "",        "color": ":c",   "enable": 1, "factor": 1}
  sampleCtx[7]  = { "title": "speed error",     "unit": "[mm/ms]", "color": ":y",   "enable": 1, "factor": 1}
  sampleCtx[8]  = { "title": "target giro",     "unit": "[mm/ms]", "color": "-b",   "enable": 1, "factor": 1}
  sampleCtx[9]  = { "title": "set point giro",  "unit": "[mm/ms]", "color": "--r",  "enable": 1, "factor": 1}
  sampleCtx[10] = { "title": "real giro",       "unit": "[mm/ms]", "color": ":g",   "enable": 1, "factor": 1}
  sampleCtx[11] = { "title": "pwm giro",        "unit": "",        "color": ":c",   "enable": 1, "factor": 1}
  sampleCtx[12] = { "title": "giro error",      "unit": "[mm/ms]", "color": ":y",   "enable": 1, "factor": 1}

  # Specify plot range
  range_plot1 = range(1, 7)
  range_plot2 = range(8, 12)

  # Variable
  nbSample      = 1
  nbSampleError = 0
	
  # Check sample context consistency
  if len(sampleCtx) != NB_SAMPLE_PER_LINE:
    print ("#ERROR# sample context inconsistant")
    sys.exit(1)

  # Check parameters
  parser = ArgumentParser()
  parser.add_argument("-f", "--file", dest="filename",
                      help="file name for file mode", metavar="FILE", default=None)
  parser.add_argument("-p", "--port", dest="serialport",
                      help="port com for serial mode", type=str)
  parser.add_argument("-s", "--speed", dest="serialspeed",
                      help="serial speed for serial mode", type=int, default=115200)					  
  parser.add_argument("-w", "--window", dest="windowsize",
                      help="number of sample per plot", type=int, default=1500)					  
  args = parser.parse_args()

  # Populate variable
  WINDOW_SIZE = args.windowsize

  # Create the sample container
  sample_list = []
  for i in range(NB_SAMPLE_PER_LINE):
    sample_list.append([])
    sample_list[i] = [0] * WINDOW_SIZE
    
  # Fill the timestamp array
  for i in range(WINDOW_SIZE):
    sample_list[0][i] = i

  # Check the mode: file or serial
  log_file = None
  ser      = None
  sio      = None
  if  args.filename != None:
    print(f"# File mode with file {args.filename}")
    try:
      log_file = open(args.filename, "r")
    except IOError:
      print (f"#ERROR# unable to open file {args.filename}")
      sys.exit(1)
  elif args.serialport != None:
    print(f"# Serial mode port {args.serialport} with speed {args.serialspeed}")
    try:
      ser = serial.Serial(port=args.serialport, baudrate=args.serialspeed, timeout = 0)
      sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))
    except Exception as e:
      print (f"#ERROR unable to open serial port {args.serialport} with speed {args.serialspeed}")	
      print(e)
      sys.exit(0)	
  else:
    print("#ERROR# missing parameters")
    parser.print_help()
    sys.exit(0)
  
  # Print info
  print (f"# Number of expected sample: {NB_SAMPLE_PER_LINE}")
  print (f"# Split pattern: '{SPLIT_PATTERN}'")
  print (f"# Telemetrie START pattern: '{TELEMETRIE_START_PATTERN}'")
  print (f"# Telemetrie STOP pattern: '{TELEMETRIE_STOP_PATTERN}'")
  print (f"# Number max of sample per plot: {WINDOW_SIZE}")
  print ("# Sample context:")
  for i in range(len(sampleCtx)):
    print (f"#  - sample id {i}: enable: {sampleCtx[i]['enable']} factor: {sampleCtx[i]['factor']} => {sampleCtx[i]['title']} {sampleCtx[i]['unit']}")
  print ("")
  
  # Display plot
  f1, ax1 = plt.subplots()
  f2, ax2 = plt.subplots()
	
  # Main frame for get/set and save action
  frame = Tk()
  frame.title('GET/SET PID')

  Button(frame, text="Get", command=GET).grid(row=0, column=0)
  Label(frame, text="xKp").grid(row=1, column=1)
  xKp = Entry(frame)
  xKp.grid(row=1, column=2)

  Label(frame, text="xKi").grid(row=2, column=1)
  xKi = Entry(frame)
  xKi.grid(row=2, column=2)

  Label(frame, text="xKd").grid(row=3, column=1)
  xKd = Entry(frame)
  xKd.grid(row=3, column=2)

  Label(frame, text="wKp").grid(row=4, column=1)
  wKp = Entry(frame)
  wKp.grid(row=4, column=2)

  Label(frame, text="wKi").grid(row=5, column=1)
  wKi = Entry(frame)
  wKi.grid(row=5, column=2)

  Label(frame, text="wKd").grid(row=6, column=1)
  wKd = Entry(frame)
  wKd.grid(row=6, column=2)

  Label(frame, text="xSpeed").grid(row=7, column=1)
  xSpeed = Entry(frame)
  xSpeed.grid(row=7, column=2)

  Label(frame, text="wSpeed").grid(row=8, column=1)
  wSpeed = Entry(frame)
  wSpeed.grid(row=8, column=2)
  
  Label(frame, text="wt1").grid(row=9, column=1)
  wt1 = Entry(frame)
  wt1.grid(row=9, column=2)

  Label(frame, text="wt2").grid(row=10, column=1)
  wt2 = Entry(frame)
  wt2.grid(row=10, column=2)  

  Label(frame, text="ut1").grid(row=11, column=1)
  ut1 = Entry(frame)
  ut1.grid(row=11, column=2)

  Label(frame, text="ut2").grid(row=12, column=1)
  ut2 = Entry(frame)
  ut2.grid(row=12, column=2)  

  Label(frame, text="wheel_diameter").grid(row=13, column=1)
  wheel_diameter = Entry(frame)
  wheel_diameter.grid(row=13, column=2)  
  
  Button(frame, text="Set", command=SET).grid(row=14, column=0)
  Button(frame, text="Save", command=SAVE).grid(row=15, column=0)	
	
  # First update display
  plt.pause(0.05)
  frame.update_idletasks()
  frame.update()

  # Infinite loop
  while 1:
 
    # Init res to None
    res = None

    # Extract data from file
    if args.filename != None:
      line = log_file.readline()
    else:
      # Extract from serial link
      sio.flush() # it is buffering. required to get the data out *now*
      line = sio.readline()
	  
	# Check line value
    if len(line) == 0:
      try:	  
        ax1.grid(True)
        ax2.grid(True)		
        plt.pause(1)
      except:
        sys.exit(0)
        print("# Exit by user")
    else:
      res = line.rstrip('\n').rstrip(' ').split(SPLIT_PATTERN)	

    if res == None:
      continue	

    # Check the number of expected fields 
    if (len(res) >= 1) and (res[0] == TELEMETRIE_GET_PATTERN):
      print("Receive get response:")
      print(res)

      pattern_list = {'0' : xKp,    '1': xKi,  '2': xKd,\
                      '3' : wKp,    '4': wKi,  '5': wKd,\
    		          '6' : xSpeed, '7': wSpeed,\
                      '8' : wt1,    '9': wt2,\
                      '10': ut1,    '11': ut2, '12': wheel_diameter}
  
      # For test purpose, write the following line to the serial port
      # "get 0.0 9.4 7.3 1.0 2.1 100.0 1 9000 4 7 10 12 88"
      for g in range(1, len(res)):
        ctx = pattern_list[str(g-1)]
        ctx.delete(0,END)
        ctx.insert(0,res[g])
	
    elif (len(res) >= 1) and (res[0] == TELEMETRIE_STOP_PATTERN):
      print ("")
      print(f"Nb max sample received: {nbSample}")
      print(f"Nb sample error       : {nbSampleError}")
      print ("")	  
      # Display all the point 
      ax1.clear()
      ax2.clear()	  
      # Plot 1
      for i in range_plot1:
        if sampleCtx[i]["enable"] == 1:
          ax1.plot(np.array(sample_list[0]), np.array(sample_list[i]), sampleCtx[i]["color"], linewidth=1, label=sampleCtx[i]["title"])
        handles, labels = ax1.get_legend_handles_labels()
        ax1.legend(handles, labels)
        ax1.grid(True)
        txt_label = "Speed telemetry [%s]" % (datetime.datetime.now())
        ax1.set_title(txt_label)

      # Plot 2
      for i in range_plot2:
        if sampleCtx[i]["enable"] == 1:
          ax2.plot(np.array(sample_list[0]), np.array(sample_list[i]), sampleCtx[i]["color"], linewidth=1, label=sampleCtx[i]["title"])
        handles, labels = ax2.get_legend_handles_labels()
        ax2.legend(handles, labels)
        ax2.grid(True)
        txt_label = "Giro telemetry [%s]" % (datetime.datetime.now())
        ax2.set_title(txt_label)
		
      plt.pause(0.01)
      frame.update_idletasks()
      frame.update()

	  # restart the counter for the next run
      nbSample = 1
	  
    elif len(res) != (NB_SAMPLE_PER_LINE + 1):
      nbSampleError += 1
      print(res)
      continue
    else:
      # Check the fisrt pattern 
      if res[0] != TELEMETRIE_START_PATTERN:
        nbSampleError += 1
        print(res)
        continue
      else:        
        # -=-=-=-=-=-=-=-=-=-=-=-=-=-=
        # -=-=-=- Extract data -=-=-=-
        # -=-=-=-=-=-=-=-=-=-=-=-=-=-=
        for i in range(1, NB_SAMPLE_PER_LINE):
          if sampleCtx[i]["enable"] == 1:
            sample_list[i][nbSample] = int(res[1 + i]) * sampleCtx[i]["factor"]
         
        # Speed up the display 
        if (nbSample % 30) == 0:
		  # Plot 1 & 2
          try: 
            ax1.clear()
            for i in range_plot1:
              if sampleCtx[i]["enable"] == 1:
                txt_color = "%s" % (sampleCtx[i]["color"])
                ax1.plot(np.array(sample_list[0]), np.array(sample_list[i]), txt_color, linewidth=1)
            ax2.clear()
            for i in range_plot2:
              if sampleCtx[i]["enable"] == 1:
                txt_color = "%s" % (sampleCtx[i]["color"])
                ax2.plot(np.array(sample_list[0]), np.array(sample_list[i]), txt_color, linewidth=1)
            plt.pause(0.01)
          except Exception as e:
            print(f"# Error: {e}")
            traceback.print_exc(file=sys.stdout)
            sys.exit(0)
			
        # Sample rollover
        nbSample += 1
        if nbSample >= WINDOW_SIZE:
          nbSample = 0

# Main
if __name__ == '__main__':    
    main()

# End of file
