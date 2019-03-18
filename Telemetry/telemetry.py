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

# Debian installation command line
# apt install python-matplotlib

# Windows installation command line with pip
# pip install matplotlib
# pip install pyserial

# Main procedure
def main():

  # Global
  NB_SAMPLE_PER_LINE       = 13
  SPLIT_PATTERN            = ' '
  TELEMETRIE_START_PATTERN = 'LOG'
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
  sampleCtx[8]  = { "title": "target giro",    "unit": "[mm/ms]", "color": "-b",   "enable": 1, "factor": 1}
  sampleCtx[9]  = { "title": "set point giro", "unit": "[mm/ms]", "color": "--r",  "enable": 1, "factor": 1}
  sampleCtx[10] = { "title": "real giro",      "unit": "[mm/ms]", "color": ":g",   "enable": 1, "factor": 1}
  sampleCtx[11] = { "title": "pwm giro",       "unit": "",        "color": ":c",   "enable": 1, "factor": 1}
  sampleCtx[12] = { "title": "giro error",     "unit": "[mm/ms]", "color": ":y",   "enable": 1, "factor": 1}

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
	
  # First update display
  plt.pause(0.05)

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
    if (len(res) >= 1) and (res[0] == TELEMETRIE_STOP_PATTERN):
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
