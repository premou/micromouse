# Package imports
from keras.models import Sequential
from keras.layers import Dense
import h5py
import numpy as np
import random
from datetime import datetime
from matplotlib import pyplot

random.seed(datetime.now())
#np.random.seed(1) # set a seed so that the results are consistent

# Load data sets
dataset_train = np.loadtxt("data.txt")
print("Train data file shape : " + str(dataset_train.shape))
#dataset_crossvalidation = np.loadtxt("dataset_v.txt")
#print("Cross-validation data file shape : " + str(dataset_crossvalidation.shape))
#dataset_test = np.loadtxt("dataset_tst.txt")
#print("Test data file shape : " + str(dataset_test.shape))

# Extract data sets (X,Y)
X_train = dataset_train[1:-1,3:4] / 4096.0 # normalise 12-bit
Y_train = dataset_train[1:-1,0:1]
m = X_train.shape[0]
print("Train data set X shape : " + str(X_train.shape))
print("Train data set Y shape : " + str(Y_train.shape))
print("Train data set size : " + str(m))

# Add feature from x
X_train = np.append(X_train, 1.0/X_train, axis=1)
X_train = np.append(X_train, 1.0/pow(X_train[:,0:1],2.0), axis=1)
X_train = np.append(X_train, 1.0/np.sqrt(X_train[:,0:1]), axis=1)
X_train = np.append(X_train, 1.0/np.exp(X_train[:,0:1]), axis=1)
print("Train data set : " + str(X_train))


#X_crossvalidation = dataset_crossvalidation[:,0:4] / 4096.0 # normalise 12-bit
#Y_crossvalidation = dataset_crossvalidation[:,4:12]
#print("Cross-validation data set X shape : " + str(X_crossvalidation.shape))
#print("Cross-validation data set Y shape : " + str(Y_crossvalidation.shape))
#X_test = dataset_test[:,0:4] / 4096.0 # normalise 12-bit
#Y_test = dataset_test[:,4:12]
#print("Test data set X shape : " + str(X_test.shape))
#print("Test data set Y shape : " + str(Y_test.shape))
#assert X_train.shape[0] == Y_train.shape[0]
#assert X_crossvalidation.shape[0] == Y_crossvalidation.shape[0]
#assert X_test.shape[0] == Y_test.shape[0]
#assert X_train.shape[1] == X_crossvalidation.shape[1]
#assert X_train.shape[1] == X_test.shape[1]
#assert Y_train.shape[1] == Y_crossvalidation.shape[1]
#assert Y_train.shape[1] == Y_test.shape[1]

# create model
model = Sequential()
model.add(Dense(3, input_dim=5, activation='linear'))
#model.add(Dense(16, activation='relu')) ##deep learning
#model.add(Dense(16, activation='relu')) ##deep learning
#model.add(Dense(16, activation='relu')) ##deep learning
model.add(Dense(1, activation='linear'))
# Compile model
model.compile(loss='mean_squared_error', optimizer='adam', metrics=['mae'])
# Fit the model
model.fit(X_train, Y_train, epochs=1000, batch_size=None, shuffle=True, verbose=2)
# evaluate the model
#scores = model.evaluate(X_crossvalidation, Y_crossvalidation)
#print("\n%s: %.2f%%" % (model.metrics_names[1], scores[1]*100))
# evaluate the model
#scores = model.evaluate(X_test, Y_test)
#print("\n%s: %.2f%%" % (model.metrics_names[1], scores[1]*100))
prediction = model.predict(X_train,verbose=1)
print(str(prediction))
pyplot.plot(prediction, label='estimated')
pyplot.plot(Y_train, label='real')
pyplot.show()
# save model
model.save('my_model.h5')  # creates a HDF5 file 'my_model.h5'
