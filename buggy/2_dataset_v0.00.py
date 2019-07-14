import numpy as np
import math
import cv2
import matplotlib.pyplot as plt
from keras.utils import to_categorical

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

# read input text file "picture";"x";"y";[...]
text_file = open("result.txt", "r")
content = text_file.read()
text_file.close()

# split fields into list
lines = content.split(';')
#print(lines)

# input picture
width = 320 # 1280 x 25% 
height_start = 30 # 720p x 25% x 30pix (lower)
height_end = 60 # 720p x 25% x 30pix (lower)
height = 30 # 720p x 25% x 30pix (lower)

# build data set
m = math.floor(len(lines)/3)
print(str(m))
X_train = np.zeros((width*height*3,m*3))
Y_train  = np.zeros((param_classes+1,m*3))
for i in range(0,m):
    fullpathname = lines[i*3].split('.')[0]+".jpg"
    print(fullpathname)
    print(lines[i*3+1])
    # read 1280x720x3 image jpeg
    image = cv2.imread(fullpathname,cv2.IMREAD_COLOR)
    assert(image.shape == (720,1280,3))
    # resize (25%)
    dim = (width,180)
    im_resized = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
    assert(im_resized.shape == (180,width,3))
    #cv2.imshow('Img', im_resized) 
    #cv2.waitKey(0)
    # crop to heigth start-end
    im_croped = im_resized[180-height_end:180-height_start,:width]
    assert(im_croped.shape == (height,width,3))
    #cv2.imshow('Img', im_croped) 
    #cv2.waitKey(0)
    #
    im_flipped_ver = cv2.flip(im_croped,0)
    im_flipped_hor = cv2.flip(im_croped,1)
    # list of images
    images = [im_croped, im_flipped_ver, im_flipped_hor]
    # read position
    position = int( lines[i*3+1] ) #0..1280
    if position == -1:
        position = 16
        # list of positions
        positions = [position, position, position]
    else:
        #resize Y too (25%)
        position = np.floor(position / 4) # 0..320
        #divide space into 16 segments
        position = np.floor(position / (320/param_classes)) # 0..15
        assert(position < param_classes)
        assert(position >= 0)
        # list of positions
        positions = [position, position, (param_classes-1)-position]
    # append into data set with data set augmentation (+3)
    for j in range(0,3):
        current_image = images[j]
        current_position = positions[j]
        # build x
        x = image2vector(current_image)
        assert(x.shape == (height*width*3,1))
        #plt.imshow(x.reshape((height, width, 3)))
        #plt.show()
        X_train [:,3*i+j]=np.transpose(x/255.0) # from integer to float RGB
        #plt.imshow(X[:,i+j].reshape((height, width, 3)))
        #plt.show()
        # build y
        Y_train [:,3*i+j] = to_categorical(current_position, num_classes=param_classes+1)
        #debug
        #plt.imshow(X_train[:,3*i+j].reshape((height, width, 3)))
        #for l in range(0,param_classes):
        #    if Y_train [l,3*i+j] == 1:
        #        plt.axvline(x=(l+0.5)*(320/param_classes),linewidth=3)
        #plt.show()

    
np.savetxt("Xtrain.txt",X_train,fmt="%f")
np.savetxt("Ytrain.txt",Y_train,fmt="%d")
