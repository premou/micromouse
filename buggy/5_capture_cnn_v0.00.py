import numpy as np
import math
import cv2
import matplotlib.pyplot as plt
from keras.models import Sequential
from keras.layers import Dense
from keras.layers import Dropout
from keras.optimizers import Adam
from keras.models import load_model
import random
from datetime import datetime
import h5py
from sklearn.utils import shuffle
from keras.regularizers import l2
from keras.utils import to_categorical

# parameters
param_classes = 16 #16+1 (0..15 = lineposition, 16 = no line)
width = 320
height_start = 30
height_end = 60
height = 30

# load model
model = load_model('model.h5')
print("Loaded model from disk")
# summarize model.
model.summary()

# open video stream
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
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
    # crop to heigth start-end
    im_croped = im_resized[240-height_end:240-height_start,:width]
    cv2.imshow('frame',im_croped)
    assert(im_croped.shape == (height,width,3))
    #make a prediction
    y = model.predict(im_croped.reshape(1,30,320,3))
    #print(str(y))
    plt.cla()
    plt.imshow(im_croped)
    for l in range(0,param_classes):
        if y[0,l] >= 0.5:
            plt.axvline(x=(l+0.5)*(320/param_classes),linewidth=2)    
    #plt.show()
    plt.draw()
    plt.pause(0.001)
    
    # stop condition
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
camera.release()
cv2.destroyAllWindows()
