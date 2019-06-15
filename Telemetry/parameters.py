#!/usr/bin/python3
#-*- coding: utf-8 -*-

import sys
import base64
import binascii
import serial
import io
import traceback
import time
import struct
#
from struct import *
from serial.tools import list_ports
from argparse import ArgumentParser
from tkinter import *

# Define
SPLIT   = ' '
CONFIG  = 'configuration'
GET_API = 'get_api'
GET_ALL = 'get_all'
SET_ALL = 'set_all'
SAV_ALL = 'sav_all'

# Global
global sio
global theEnd
global nbParameters
global paramCtx

def GET():
  sio.write(str(GET_ALL+'\r\n'))
  sio.flush()
  time.sleep(0.5)
  line = sio.readline()
  if len(line) == 0:
    print('ERROR: no response to get_api command')
  res = line.rstrip('\n').rstrip(SPLIT).split(SPLIT)
  if (res[0]!=CONFIG) or (res[1]!=GET_ALL):
    print(f'ERROR: bad response to get_all command: {res}\n')
    return  
  base64_encoded = res[2]
  base64_decoded = base64.b64decode(base64_encoded)
  format = ''
  for x in range(nbParameters):
    format += 'I'
  var = unpack(format, base64_decoded)
  crc32_hex = hex(var[nbParameters-1]).split('x')[-1]
  crc32_computed     = binascii.crc32(base64_decoded[:((nbParameters-1) * 4)])
  crc32_computed_hex = hex(crc32_computed).split('x')[-1]
  if crc32_hex != crc32_computed_hex:
    print(f'ERROR: bad crc32 in get_all response: {crc32_hex} != {crc32_computed_hex}\n')
    return
  else:
    for p in range(nbParameters):
      ctx = paramCtx[p]['object']
      ctx.delete(0,END)
      ctx.insert(0,var[p])

def SET():
  paramValueList = []
  format = ''
  for x in range(nbParameters - 1):
    format += 'I'
    paramValueList.append(int(paramCtx[x]['object'].get()))
  var   = struct.pack(format, *paramValueList)
  crc32 = binascii.crc32(var)
  ctx = paramCtx[nbParameters - 1]['object']
  ctx.delete(0,END)
  ctx.insert(0,crc32)
  paramValueListWithCrc = []
  format = ''
  for x in range(nbParameters):
    format += 'I'
    paramValueListWithCrc.append(int(paramCtx[x]['object'].get()))
  var = struct.pack(format, *paramValueListWithCrc)
  base64_encoded = base64.b64encode(var)
  sio.write(str(SET_ALL + ' ') + base64_encoded.decode("utf-8") + str('\r\n'))
  sio.flush()
  time.sleep(0.5)
  line = sio.readline()
  if len(line) == 0:
    return
  else:
    print(f'{line}')

def SAVE():
  # Global
  global sio
  sio.write(str(SAV_ALL+'\r\n'))
  sio.flush()

def QUIT():
  # Global
  global theEnd
  theEnd = True

# Check parameters
parser = ArgumentParser()
parser.add_argument("-p", "--port", dest="serialport",
                    help="port com for serial mode", type=str)
parser.add_argument("-s", "--speed", dest="serialspeed",
                    help="serial speed for serial mode", type=int, default=115200)					  
args = parser.parse_args()

ser = None
sio = None
if args.serialport != None:
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

# First: send get_api command
cmd  = GET_API + "\r\n"
sio.write(str(cmd))
sio.flush()
# Read response
time.sleep(0.5)
line = sio.readline()
if len(line) == 0:
  print('ERROR: no response to get_api command')
  sys.exit(0)
res = line.rstrip('\n').rstrip(SPLIT).split(SPLIT)
if (res[0]!=CONFIG) or (res[1]!=GET_API):
  print('ERROR: bad response to get_api command: {res}')
  sys.exit(0)

nbParameters = int(res[2])
paramCtx = {}
for p in range(0, nbParameters):
  paramCtx[p] = {'label': res[3+p], 'value': 0, 'object': None}

# Main frame for get/set and save action
frame = Tk()
frame.title('GET/SET parameters')
for p in range(0, nbParameters):
  Label(frame, text=paramCtx[p]['label']).grid(row=p, column=0)
  paramCtx[p]['object'] = Entry(frame)
  paramCtx[p]['object'].grid(row=p, column=1)

Button(frame, text="Get ", command=GET ).grid(row=(nbParameters), column=2)
Button(frame, text="Set ", command=SET ).grid(row=(nbParameters), column=3)
Button(frame, text="Save", command=SAVE).grid(row=(nbParameters), column=4)	

# Request the values
frame.update_idletasks()
GET()

# Main loop entry
frame.mainloop()
  
# EOF