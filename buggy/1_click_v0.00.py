#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import os
import PIL.Image
from argparse import ArgumentParser
from tkinter  import *

event2canvas = lambda e, c: (c.canvasx(e.x), c.canvasy(e.y))

# Only for my bloody Mac
########################
# PYTHONPATH='/Library/Frameworks/Python.framework/Versions/3.7/lib/python3.7/site-packages' python3 xxx

# Check parameters
parser = ArgumentParser()
parser.add_argument("-g", "--gif", dest="gif_directory",
                    help="file name for file mode", type=str, default=None)
args = parser.parse_args()

if args.gif_directory == None:
    print('#ERROR: missing gif directory')
    sys.exit(0)

# Convert jpeg to gif, if any
image_dir = args.gif_directory
included_extensions = ['jpg', 'jpeg']
file_names = [fn for fn in os.listdir(image_dir) if any(fn.endswith(ext) for ext in included_extensions)]
for j in file_names:
    print(f'# Converting {image_dir}/{j} to {image_dir}/{j}.gif')
    img     = PIL.Image.open(image_dir + '/' + j)
    img_rgb = img.convert('RGB')
    try:
        img_rgb.save(image_dir + '/' + j + '.gif', 'GIF')
    except:
        print('error')

# Find how many images should be displayed
image_dir = args.gif_directory
included_extensions = ['gif']
file_names = [fn for fn in os.listdir(image_dir) if any(fn.endswith(ext) for ext in included_extensions)]

if len(file_names)==0:
    print(f'#ERROR: no image file in directory')
    sys.exit(0)

exists = os.path.isfile('result.txt')
if exists:
    print(f'#ERROR: result file already exist. Rename or remove it.')
    sys.exit(0)

# Create file
global result_file
result_file = open("result.txt","w+")

# Scan all the files and display the image
for i in file_names:
    
    root = Tk()
    root.geometry("{0}x{1}+0+0".format(root.winfo_screenwidth(), root.winfo_screenheight()))
    
    # Setting up a tkinter canvas with scrollbars
    frame = Frame(root, bd=2, relief=SUNKEN)
    frame.grid_rowconfigure(0, weight=1)
    frame.grid_columnconfigure(0, weight=1)
    xscroll = Scrollbar(frame, orient=HORIZONTAL)
    xscroll.grid(row=1, column=0, sticky=E+W)
    yscroll = Scrollbar(frame)
    yscroll.grid(row=0, column=1, sticky=N+S)
    canvas = Canvas(frame, bd=0, xscrollcommand=xscroll.set, yscrollcommand=yscroll.set)
    canvas.grid(row=0, column=0, sticky=N+S+E+W)
    xscroll.config(command=canvas.xview)
    yscroll.config(command=canvas.yview)
    frame.pack(fill=BOTH,expand=1)
    
    f = image_dir + '/' + i
    root.title(f)
    print(f'# Processing file {f}')
    img = PhotoImage(file=f)
    canvas.create_image(0,0,image=img,anchor="nw")
    canvas.config(scrollregion=canvas.bbox(ALL))
    canvas.create_rectangle(0, 480, 1280, 600, outline="blue")

    # Function to be called when mouse 1 is clicked
    def printcoords(event):
        global result_file
        global f
        # Outputting x and y coords to console
        cx, cy = event2canvas(event, canvas)
        result_file.write("%s;%d;%d;" % (f, cx, cy))
        root.destroy()

    # Function to be called when mouse 2 is clicked
    def printemptycoords(event):
        global result_file
        global f
        # Outputting x and y coords to console
        cx, cy = event2canvas(event, canvas)
        result_file.write("%s;%d;%d;" % (f, -1, -1))
        root.destroy()

    # Mouseclick event
    canvas.bind("<ButtonPress-1>",printcoords)
    canvas.bind("<ButtonPress-2>",printemptycoords)
    root.mainloop()

print(f'# {len(file_names)} files in file result.txt')

result_file.close()

# End of file
