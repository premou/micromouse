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
height_start_1 = 30 #30 720p x 25% x 30pix (lower)
height_end_1 = 60 #60 720p x 25% x 30pix (lower)
height_start_2 = 90
height_end_2 = 120
height = 30 #30 720p x 25% x 30pix (lower)

# load images
list = os.listdir('images_test/')
for i in list:
    print("images_test/"+i)
    image = cv2.imread("images_test/"+i,cv2.IMREAD_COLOR)
    assert(image.shape == (720,1280,3))
    # resize (25%)
    dim = (width,180)
    im_resized = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
    assert(im_resized.shape == (180,width,3))
    # crop
    im_croped_1 = im_resized[180-height_end_1:180-height_start_1,:width]
    assert(im_croped_1.shape == (height,width,3))
    im_croped_2 = im_resized[180-height_end_2:180-height_start_2,:width]
    assert(im_croped_2.shape == (height,width,3))
    #make a prediction
    y_1 = model.predict(im_croped_1.reshape(1,30,320,3))
    y_2 = model.predict(im_croped_2.reshape(1,30,320,3))
    print(str(y_1))
    print(str(y_2))
    l_1 = 8
    l_2 = 8
    for l in range(0,param_classes):
        if y_1[0,l] >= 0.5:
            l_1 = l;
        if y_2[0,l] >= 0.5:
            l_2 = l
    plt.clf()
    plt.imshow(image)
    plt.plot([(l_1+0.5)*1280/param_classes, (l_2+0.5)*1280/param_classes], [(180-height_start_1-height/2)*4,(180-height_end_2+height/2)*4])
    #plt.show()
    plt.pause(1)







