import numpy as np
import math
import cv2
from keras.models import load_model
import h5py
import time
import serial

# serial
port = serial.Serial("/dev/ttyS0",baudrate=115200,timeout=3.0)

# parameters
param_classes = 16 #16+1 (0..15 = lineposition, 16 = no line)
width = 320 # 1280 x 25% 
height_start_1 = 30 #30 720p x 25% x 30pix (lower)
height_end_1 = 60 #60 720p x 25% x 30pix (lower)
height_start_2 = 90
height_end_2 = 120
height = 30 #30 720p x 25% x 30pix (lower)

# load model
model = load_model('model.h5')
print("Loaded model from disk")
# summarize model.
model.summary()

# open video stream
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
last_time = time.time()
frame_counter = 0
while(True):
    return_value, image = camera.read()
    #cv2.imshow('frame',image)
    #assert(image.shape == (720,1280,3))
    assert(image.shape == (480,640,3))
    #print(str(image.shape))
    #resize (50%)
    dim = (320,240)
    im_resized = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
    assert(im_resized.shape == (240,320,3))
    
    # crop
    im_croped_1 = im_resized[240-height_end_1:240-height_start_1,:width]
    assert(im_croped_1.shape == (height,width,3))
    im_croped_2 = im_resized[240-height_end_2:240-height_start_2,:width]
    assert(im_croped_2.shape == (height,width,3))
    #make a prediction
    y_1 = model.predict(im_croped_1.reshape(1,30,320,3))
    y_2 = model.predict(im_croped_2.reshape(1,30,320,3))
    #print(str(y_1))
    #print(str(y_2))
    l_1 = 8
    l_2 = 8
    for l in range(0,param_classes):
        if y_1[0,l] >= 0.5:
            l_1 = l;
        if y_2[0,l] >= 0.5:
            l_2 = l
    cv2.line(image,(int((l_1+0.5)*640/param_classes),int((240-height_start_1-height/2)*2)),(int((l_2+0.5)*640/param_classes),int((240-height_end_2+height/2)*2)),(255,0,0),5)
    cv2.imshow('frame',image)
    #fps
    frame_counter += 1
    if time.time()>=last_time+1.0:
        last_time=time.time()
        print(str(frame_counter))
        frame_counter = 0
    # serial
    port.write("{:d}\r\n".format(l_1).encode('ascii'))    
    # stop condition
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
camera.release()
cv2.destroyAllWindows()
