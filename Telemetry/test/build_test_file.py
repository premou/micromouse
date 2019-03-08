#!/usr/bin/python2.7
#-*- coding: utf-8 -*-

nbLine   = 800
timeOffset = 45698

sampleDico    = {}
sampleDico[0] = { "value":0,   "min":20,  "max":300, "step":10 }
sampleDico[1] = { "value":100, "min":10,  "max":500, "step":1  }
sampleDico[2] = { "value":22,  "min":0,   "max":100, "step":3  }
sampleDico[3] = { "value":1,   "min":0,   "max":421, "step":1 }

for i in range(nbLine):
  print "TEL %d %d %d %d %d" % 	((i+timeOffset),\
				sampleDico[0]["value"],\
				sampleDico[1]["value"],\
				sampleDico[2]["value"],\
				sampleDico[3]["value"])
  
  for i in range(len(sampleDico)):
    value = sampleDico[i]["value"] + sampleDico[i]["step"]
    if value > sampleDico[i]["max"]:
      value = sampleDico[i]["min"]
    sampleDico[i]["value"] = value

# End of file
