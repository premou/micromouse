#!/usr/bin/python2.7
#-*- coding: utf-8 -*-

##
# https://matplotlib.org/api/_as_gen/matplotlib.pyplot.show.html#matplotlib.pyplot.show
##

import sys
import datetime
import numpy as np
import matplotlib.pyplot as plt

# Debian installation command line
# apt install python-matplotlib

# Usage
def usage():
  print "Usage: %s file" % sys.argv[0]

# Main procedure
def main():

  # Global
  NB_SAMPLE_PER_LINE = 5
  SPLIT_PATTERN      = ' '
  TELEMETRIE_PATTERN = 'TEL'

  # Telemetrie context
  sampleCtx    = {}
  sampleCtx[0] = { "title": "time",         "unit": "ms",    "color": "b"}  
  sampleCtx[1] = { "title": "real speed",   "unit": "mm/ms", "color": "b"}
  sampleCtx[2] = { "title": "set up speed", "unit": "mm/ms", "color": "r"}
  sampleCtx[3] = { "title": "error",        "unit": "mm/ms", "color": "y"}
  sampleCtx[4] = { "title": "age du capitaine croquette",        "unit": "annee par ms", "color": "g"}

  # Variable
  nbSample      = 0
  nbSampleError = 0
  firstTime     = -1
  lastTime      = -1

  # Create the sample container
  sample_list = []
  for i in range(NB_SAMPLE_PER_LINE):
    sample_list.append([])

  # Check sample context consistency
  if len(sampleCtx) != NB_SAMPLE_PER_LINE:
    print "#ERROR# sample context inconsistant"
    sys.exit(1)

  # Check parameters
  if len(sys.argv) < 2:
    usage()
    sys.exit(1)

  # Open the log file
  try:
    logfile  = sys.argv[1]
    log_file = open(logfile, "r")
  except IOError:
    print "File %s does not exist !"
    sys.exit(1)
 
  # Print info
  print "# Mode: file (for the moment, serial mode in progress)"
  print "# Number of expected sample: %d" % (NB_SAMPLE_PER_LINE)
  print "# Split pattern: '%s'"           % (SPLIT_PATTERN)
  print "# Telemetrie pattern: '%s'"      % (TELEMETRIE_PATTERN)
  print "# Sample context:"
  for i in range(len(sampleCtx)):
    print "#  - sample id %d: %s [%s]" % (i, sampleCtx[i]["title"], sampleCtx[i]["unit"])

  # Parse all the line and build sample array
  for line in log_file.readlines():
    res = line.rstrip('\n').split(SPLIT_PATTERN)

    # Check the number of expected fields 
    if len(res) != (NB_SAMPLE_PER_LINE + 1):
      nbSampleError += 1
      print "#ERROR# missing filed in line:"
      print res
      continue
    else:
      # Check the fisrt pattern 
      if res[0] != TELEMETRIE_PATTERN:
        nbSampleError += 1
        print "#ERROR# missing first pattern in line:"
        print res
	continue
      else:
        # Check empty field
        for i in range(len(res)):
          if res[i] == '':
            nbSampleError += 1
            print "#ERROR# one filed is empty in line:"
            print res
            # keep in mind last time
            lastTime = int(res[1])
	    continue
        
        # Timebase
        if firstTime == -1:
          firstTime = int(res[1])

        # Check time increment (rollover not supported)
	if lastTime != -1:
          if (lastTime + 1) != int(res[1]):
            print "#ERROR# timestamp sequence number error: previous=%d, current=%d" % (lastTime, int(res[1]))
            nbSampleError += 1
            # keep in mind last time
    	    lastTime = int(res[1])
            continue

        # Keep in mind last time
    	lastTime = int(res[1])

        # -=-=-=-=-=-=-=-=-=-=-=-=-=-=
        # -=-=-=- Extract data -=-=-=-
        # -=-=-=-=-=-=-=-=-=-=-=-=-=-=
        for i in range(NB_SAMPLE_PER_LINE):
          sample_list[i].append(int(res[1 + i]))
        
        # One more set of valid sample
	nbSample += 1

  # Sum up
  print "# Nb sample: %d" % (nbSample)
  print "# Nb sample with error: %d" % (nbSampleError)
  print "# First timestamp: %d" % (firstTime)
  print "# Last timestamp: %d" % (lastTime)

  # Check nb sample
  for i in range(NB_SAMPLE_PER_LINE):
    if (len(sample_list[i]) != nbSample):
      print "#ERROR# nb sample inconsitant: expecting %d and found %d for sample id %d [%s]" % \
		(nbSample, len(sample_list[i]), i, sampleCtx[i]["title"])

  # Build plot
  fig, ax = plt.subplots()
  for i in range(NB_SAMPLE_PER_LINE - 1):
    txt_label = "%s [%s]" % (sampleCtx[i+1]["title"], sampleCtx[i+1]["unit"])
    txt_color = "%s" % (sampleCtx[i+1]["color"])
    ax.plot(np.array(sample_list[0]), np.array(sample_list[i+1]), txt_color, label=txt_label)

  # Display plot
  ax.set_xlim(firstTime, firstTime + nbSample)
  ax.grid(True)
  txt_label = "%s [%s] %d sample" % (sampleCtx[0]["title"], sampleCtx[0]["unit"], nbSample)
  plt.xlabel(txt_label)
  txt_label = "microMouse telemetry [%s]" % (datetime.datetime.now())
  plt.title(txt_label)
  plt.legend()
  plt.show()

# Main
if __name__ == '__main__':    
    main()

# End of file
