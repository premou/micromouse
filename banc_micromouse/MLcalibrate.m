% machine learning

%% initialization
clear ; close all; clc

%% load data set
load data.txt

%% size of data set (without first set)
m = size(data,1)-1;

%% build data set X (first 1 col) and y
y = data(2:m+1,1);
rawX = data(2:m+1,5);

% one feature with polynomial regression

% normalise feature
[X_norm, mu, sigma] = my_feature_normalize(1./(rawX));

% add x0
X = [ ones(m,1), X_norm ];

% initialize fitting parameters
theta = zeros(2, 1); 

% Some gradient descent settings
iterations = 600;
alpha = 0.01;

% run gradient descent
[theta, J_history] = my_gradient_descent(X, y, theta, alpha, iterations);

#error over distance
error = X*theta - y;

% plot x (ADC) => y (distance)
subplot(2,2,1)
plot(rawX,y);
hold on;
plot(rawX, X*theta, '-');
xlabel("ADC");
ylabel("Distance(mm)");
legend("Raw measures","H()")
title("Raw ADC value to Distance")

% plot polynomial regression x (ADC) => y (distance)
subplot(2,2,3)
plot(1./(rawX),y);
hold on;
plot(1./(rawX), X*theta, '-');
xlabel("Normalized(1/ADC)");
ylabel("Distance(mm)");
legend("Raw measures","H()");
title("Linear Regression")

subplot(2,2,2)
plot(y,error,"r");
hold on;
plot(y,zeros(m,1),'k-');
xlabel("Distance(mm)");
ylabel("Distnace Error(mm)");
legend("H()","reference")
title("Error")

subplot(2,2,4)
plot(J_history);
legend("Jtheta");
xlabel("Iterations");
ylabel("Cost");
title("Learning Rate")

%theta(2) = (theta(2)-mu)/sigma;
theta
mu
sigma

% test
real_distance = [y(50);y(100);y(150)]
adc = rawX(real_distance)
adc_norm = ((1./adc).-mu)./sigma;
calibrated_distance = [ones(3,1),adc_norm]*theta

% save results
save hypothesis.txt theta mu sigma
print -dpng ML_results
