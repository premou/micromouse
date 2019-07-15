import numpy as np
import math
import cv2
import matplotlib.pyplot as plt
import random
from datetime import datetime
import h5py
from sklearn.utils import shuffle
import keras
from keras.datasets import mnist
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten, BatchNormalization, Activation
from keras.layers import Conv2D, MaxPooling2D
from keras.optimizers import Adam
from keras.regularizers import l2

# parameters
width = 320 # 1280 x 25%
height = 30 # 720p x 25% x 30pix (lower)
inmage_shape = (height, width, 3)
param_units_hidden_1 = 17 #17
param_units_hidden_2 = 17 #17
param_classes = 16 #16+1 (0..15 = lineposition, 16 = no line)

# hyperparameters
##Epoch 35/35
## - 9s - loss: 0.2584 - categorical_accuracy: 0.9426 - val_loss: 0.4981 - val_categorical_accuracy: 0.8435
##train 
##categorical_accuracy: 0.994
##test 
##categorical_accuracy: 0.844
hyp_epoch = 35 #35
hyp_bs = 128 #128
hyp_lr = 0.001 #0.001
hyp_lr_decay = 0.0 #0.0
hyp_l2_regularization = 0.001 #0.001
hyp_weight_dropout = 0.25 #0.25

random.seed(datetime.now())

# load dataset
X = np.transpose(np.loadtxt("Xtrain.txt", dtype=float))
Y = np.transpose(np.loadtxt('Ytrain.txt', dtype=int))
m = X.shape[0]
print(str(m))

# shuffle dataset (mini batch)
X_shuffled, Y_shuffled = shuffle(X, Y)

# split dataset
m_train = math.floor(m*0.85)

#X_train = X[:m_train,:]
#Y_train = Y[:m_train]
#X_test = X[m_train:,:]
#Y_test = Y[m_train:]
X_train = X_shuffled[:m_train,:].reshape(m_train,height,width,3)
Y_train = Y_shuffled[:m_train]
X_test = X_shuffled[m_train:,:].reshape(m-m_train,height,width,3)
Y_test = Y_shuffled[m_train:]

# input picture
width = 320 # 1280 x 25% 
height = 30 # 720p x 25% x 30pixels (lower)

# create model conv 32x5x5/2, conv 64x3x3/2, pol 2x2/1
model = Sequential()
model.add(Conv2D(32, kernel_size=(5, 5), strides=(2, 2), activation='relu', input_shape=inmage_shape))
model.add(Conv2D(64, kernel_size=(3, 3), strides=(2, 2), activation='relu'))
model.add(MaxPooling2D(pool_size=(2, 2), strides=(1, 1)))
model.add(Dropout(hyp_weight_dropout))
model.add(Flatten())
# FCN
#model.add(Dropout(hyp_weight_dropout, input_shape=(X_train.shape[1],)))
#model.add(Dense(param_units_hidden_1,use_bias=False,activity_regularizer=l2(hyp_l2_regularization))) #input_dim=X_train.shape[1],activity_regularizer=l2(hyp_l2_regularization)
#model.add(BatchNormalization())
#model.add(Activation("relu"))
#model.add(Dropout(hyp_weight_dropout))
model.add(Dense(param_units_hidden_2,use_bias=False,activity_regularizer=l2(hyp_l2_regularization)))
model.add(BatchNormalization())
model.add(Activation("relu"))
model.add(Dropout(hyp_weight_dropout))
model.add(Dense(param_classes+1)) # N-class output
model.add(Activation("softmax"))
# summarize model.
model.summary()

# Compile model
opt = Adam(lr=hyp_lr,decay=hyp_lr_decay)
model.compile(loss='categorical_crossentropy', optimizer=opt, metrics=['categorical_accuracy'])
# Fit the model
history = model.fit(X_train, Y_train, validation_data=(X_test, Y_test), epochs=hyp_epoch, batch_size=hyp_bs, verbose=2)
# evaluate the model
scores = model.evaluate(X_train, Y_train, verbose=0)
print("train \n%s: %.3f" % (model.metrics_names[1], scores[1]))
# evaluate the model
scores = model.evaluate(X_test, Y_test, verbose=0)
print("test \n%s: %.3f" % (model.metrics_names[1], scores[1]))

# list all data in history
print(history.history.keys())
# plot history
#plt.semilogy(history.history['acc'], label='train')
#plt.semilogy(history.history['val_acc'], label='test')
plt.plot(history.history['categorical_accuracy'], label='train')
plt.plot(history.history['val_categorical_accuracy'], label='test')
plt.ylabel('categorical_accuracy')
plt.xlabel('epoch')
plt.legend(['train', 'test'], loc='upper right')
plt.show()
plt.plot(history.history['loss'], label='train')
plt.plot(history.history['val_loss'], label='test')
plt.ylabel('loss')
plt.xlabel('epoch')
plt.legend(['train', 'test'], loc='upper right')
plt.show()

# save model and architecture to single file
model.save("model.h5")
print("Saved model to disk")





