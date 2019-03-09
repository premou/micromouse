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

# Debian installation command line
# apt install python-matplotlib

# Windows installation command line with pip
# pip install matplotlib
# pip install pyserial

# Main procedure
def main():

  # Global
  NB_SAMPLE_PER_LINE = 8
  SPLIT_PATTERN      = ' '
  TELEMETRIE_PATTERN = 'LOG'
  WINDOW_SIZE        = 1500

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
  sampleCtx    = {}
  sampleCtx[0] = { "title": "time",            "unit": "ms",    "color": "w",  "enable": 0, "factor": 1}  
  sampleCtx[1] = { "title": "action number",   "unit": "",      "color": "--k",  "enable": 1, "factor": 100}
  sampleCtx[2] = { "title": "phase",           "unit": "",      "color": "m",  "enable": 1, "factor": 100}
  sampleCtx[3] = { "title": "target speed",    "unit": "mm/ms", "color": "-b", "enable": 1, "factor": 1}
  sampleCtx[4] = { "title": "set point speed", "unit": "mm/ms", "color": "--r",  "enable": 1, "factor": 1}
  sampleCtx[5] = { "title": "real speed",      "unit": "mm/ms", "color": ":g",  "enable": 1, "factor": 1}
  sampleCtx[6] = { "title": "speed error",     "unit": "mm/ms", "color": ":y",  "enable": 1, "factor": 1}
  sampleCtx[7] = { "title": "pwm",             "unit": "",      "color": ":c",  "enable": 1, "factor": 1}

  # Variable
  nbSample      = 0
  nbSampleError = 0
  lastTime      = -1

  # Create the sample container
  sample_list = []
  for i in range(NB_SAMPLE_PER_LINE):
    sample_list.append([])
    sample_list[i] = [0] * WINDOW_SIZE
    
  # Fill the timestamp array
  for i in range(WINDOW_SIZE):
    sample_list[0][i] = i
	
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
  args = parser.parse_args()

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
      ser = serial.Serial(port=args.serialport, baudrate=args.serialspeed)
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
  print (f"# Telemetrie pattern: '{TELEMETRIE_PATTERN}'")
  print ("# Sample context:")
  for i in range(len(sampleCtx)):
    print (f"#  - sample id {i}: {sampleCtx[i]['title']} [{sampleCtx[i]['unit']}]")
  print ("")
  
  # Display plot
  fig, ax = plt.subplots()
  #ax.set_xlim(0, WINDOW_SIZE)
  #ax.grid(True)
  #ax.set_facecolor('xkcd:light khaki')
  #txt_label = "%s [%s] %d sample" % (sampleCtx[0]["title"], sampleCtx[0]["unit"], nbSample)
  #plt.xlabel(txt_label)
  #txt_label = "microMouse telemetry [%s]" % (datetime.datetime.now())
  #plt.title(txt_label)
  ax.legend(fancybox=True, framealpha=0.1)

  # First update display
  plt.pause(0.05)

  # Infinite loop
  while 1:
 
    # Extract data from file
    if args.filename != None:
      line = log_file.readline()
    else:
	  # Extract from serial link
      #line = ser.readline()
      sio.flush() # it is buffering. required to get the data out *now*
      line = sio.readline()
	  
	# Check line value
    if len(line) == 0:
      try:	  
        ax.grid(True)
        plt.pause(0.5)
      except:
        sys.exit(0)
        print("# Exit by user")
    else:
      res = line.rstrip('\n').rstrip(' ').split(SPLIT_PATTERN)	
	
    # Check the number of expected fields 
    if len(res) != (NB_SAMPLE_PER_LINE + 1):
      continue
    else:
      # Check the fisrt pattern 
      if res[0] != TELEMETRIE_PATTERN:
        nbSampleError += 1
        print ("#ERROR# missing first pattern in line:")
        print (res)
        continue
      else:        
        # Check time increment (rollover not supported)
        if lastTime != -1:
          if (lastTime + 1) != int(res[1]):
            print (f"#ERROR# timestamp sequence number error: previous={lastTime}, current={int(res[1])}")
            nbSampleError += 1
            # keep in mind last time
            lastTime = int(res[1])
            continue

        # Keep in mind last time
        lastTime = int(res[1])

        # -=-=-=-=-=-=-=-=-=-=-=-=-=-=
        # -=-=-=- Extract data -=-=-=-
        # -=-=-=-=-=-=-=-=-=-=-=-=-=-=
        for i in range(1, NB_SAMPLE_PER_LINE):
          if sampleCtx[i]["enable"] == 1:
            sample_list[i][nbSample] = int(res[1 + i]) * sampleCtx[i]["factor"]
         
		# Display every 100 points 
        if (nbSample % 25) == 0:
          try: 
            ax.clear()		
            for i in range(1, NB_SAMPLE_PER_LINE):
              if sampleCtx[i]["enable"] == 1:
                txt_label = "%s [%s]" % (sampleCtx[i]["title"], sampleCtx[i]["unit"])
                txt_color = "%s" % (sampleCtx[i]["color"])
                ax.plot(np.array(sample_list[0]), np.array(sample_list[i]), txt_color, linewidth=1, label=txt_label)
                plt.legend()
            ax.grid(True)
            plt.pause(0.01)
          except:
            print("# Exit by user...")
            sys.exit(0)

        # Sample rollover
        nbSample += 1
        if nbSample >= WINDOW_SIZE:
          nbSample = 0
		  # Clean the array 
          for i in range(1, NB_SAMPLE_PER_LINE):
            sample_list[i] = [0] * WINDOW_SIZE

# Main
if __name__ == '__main__':    
    main()

# End of file
