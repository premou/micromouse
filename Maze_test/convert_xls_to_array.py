#!/usr/bin/python2.7
#-*- coding: utf-8 -*-

import sys
from xlrd import *

print("# Open file: ", sys.argv[1])

book = open_workbook(sys.argv[1], formatting_info=True)
sheet = book.sheet_by_index(0)
nrows = sheet.nrows
ncols = sheet.ncols
print("# Nb row:", nrows, " nb cols:",ncols)

array_txt = ["____",\
             "___N",\
             "__S_",\
             "__SN",\
             "_E__",\
             "_E_N",\
             "_ES_",\
             "_ESN",\
             "W___",\
             "W__N",\
             "W_S_",\
             "W_SN",\
             "WE__",\
             "WE_N",\
             "WES_",\
             "WESN"]

x_y = [ [0]*ncols for i in range(nrows)]

__hex = 0
N_hex = 1
S_hex = 2
E_hex = 4
W_hex = 8

for row in range(nrows):
    for col in range(ncols):
        state = __hex
        fmt = book.xf_list[sheet.cell(row, col).xf_index]
        if(fmt.border.bottom_line_style):
            state  += S_hex
        if(fmt.border.top_line_style):
            state  += N_hex
        if(fmt.border.left_line_style):
            state  += W_hex
        if(fmt.border.right_line_style):
            state  += E_hex
        x_y[row][col] = state
        print("(",row,",",col,"): ", state, " ", array_txt[x_y[row][col]])

# Display
for row in range(nrows):
    one_line = ''
    for col in range(ncols):
        one_line += array_txt[x_y[row][col]] + ' '
    print(one_line)
print("")

# Patch missing xls format
for row in range(nrows):
    one_line = ''
    for col in range(ncols):
        #    ?
        #   -x-
        #    -
        if (row - 1 > 0):
            if (N_hex & x_y[row][col]) == N_hex:
                x_y[row-1][col] |= S_hex
        #    -
        #   -x-
        #    ?
        if (row + 1 < nrows):
            if (S_hex & x_y[row][col]) == S_hex:
                x_y[row+1][col] |= N_hex
        #    -
        #   ?x-
        #    -
        if (col - 1 > 0):
            if (W_hex & x_y[row][col]) == W_hex:
                x_y[row][col-1] |= E_hex
        #    -
        #   -x?
        #    -
        if (col + 1 < ncols):
            if (E_hex & x_y[row][col]) == E_hex:
                x_y[row][col+1] |= W_hex

print("")

# Display
for row in range(nrows):
    one_line = ''
    for col in range(ncols):
        one_line += array_txt[x_y[row][col]] + ' '
    print(one_line)
print("")

# Format in C array
print("test_array_t test_array[MAX_MAZE_DEPTH][MAX_MAZE_DEPTH] =")
print("{")
for col in (range(ncols)):
    one_line = ''
    for row in reversed(range(nrows)):
        one_line += array_txt[x_y[row][col]] + ','
        if(row == 0):
            one_line = one_line[:-1]
    print("\t{", one_line, "},")
print("};")

# EOF
