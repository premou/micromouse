#!/usr/bin/python2.7
#-*- coding: utf-8 -*-

import sys

try:
    log_file = open(sys.argv[1], "r")
except IOError:
        print (f"#ERROR# unable to open file {sys.argv[1]}")
        sys.exit(1)
for l in log_file.readlines():
    res = l.rstrip('\n').rstrip(' ').split(' ')
    if res[0] != 'LOG':
        txt_line = ''
        for k in range(len(res)):
            txt_line += res[k] + ' '
        print(f"{txt_line}")
    else:
        txt_line = ''
        for k in range(len(res)):
            txt_line += res[k] + ' '
        custom_range = range(2, 7)
        for k in custom_range:
            txt_line += res[k] + ' '
        print(f"{txt_line}")
        
# End of file
