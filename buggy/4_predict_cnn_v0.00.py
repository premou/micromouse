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
import os

def image2vector(image):
    """
    Argument:
    image -- a numpy array of shape (length, height, depth)
    
    Returns:
    v -- a vector of shape (length*height*depth, 1)
    """
    size = image.shape[0] * image.shape[1] * image.shape[2]
    v = image.reshape(size, 1)
    return v

# parameters
param_classes = 16 #16+1 (0..15 = lineposition, 16 = no line)

# load model
model = load_model('model.h5')
print("Loaded model from disk")
# summarize model.
model.summary()

# input picture
width = 320 # 1280 x 25% 
height_start = 90 #30 720p x 25% x 30pix (lower)
height_end = 120 #60 720p x 25% x 30pix (lower)
height = 30 #30 720p x 25% x 30pix (lower)

# load images
list = os.listdir('images_test/')
##list =[ "images4/scene400169.jpg",
##        "images4/scene400089.jpg",
##        "images4/scene400193.jpg",
##        "images4/scene400273.jpg",
##        "images4/scene400561.jpg",
##        "images2/scene200945.jpg",
##        "images2/scene201473.jpg",
##        "images2/scene201377.jpg",
##        "images/scene00871.jpg",
##        "images/scene00731.jpg",
##        "images7/scene1500137.jpg",
##        "images7/scene700121.jpg",
##        "images7/scene1500177.jpg"
##        ]
for i in list:
    print("images_test/"+i)
    image = cv2.imread("images_test/"+i,cv2.IMREAD_COLOR)
    assert(image.shape == (720,1280,3))
    # resize (25%)
    dim = (width,180)
    im_resized = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
    assert(im_resized.shape == (180,width,3))
    # crop 50% bottom
    im_croped = im_resized[180-height_end:180-height_start,:width]
    assert(im_croped.shape == (height,width,3))
    #x = image2vector(im_croped)
    #assert(x.shape == (height*width*3,1))
    #make a prediction
    #y = model.predict(np.transpose(x))
    y = model.predict(im_croped.reshape(1,30,320,3))
    print(str(y))
    plt.imshow(im_croped)
    for l in range(0,param_classes):
        if y[0,l] >= 0.5:
            plt.axvline(x=(l+0.5)*(320/param_classes),linewidth=2)    
    plt.show()







