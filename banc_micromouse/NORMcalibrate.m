% machine learning
%% data file order "DL"(2) ,"FL"(3) ,"FR"(4), "DR"(5)
name = [ "DL"; "FL"; "FR"; "DR" ];

%% initialization
clear ; close all; clc ;

%% load data set
load data.txt

%% size of data set (without first set)
m = size(data,1)-1;

%% build data set y
y = data(2:m+1,1);

for sensor = 1:4
  %% build data set X 
  rawX = data(2:m+1,sensor+1); %% SELECT DATA SET : "DL"(2) ,"FL"(3) ,"FR"(4), "DR"(5)

  %% one feature with polynomial regression x <=> 1/sqrt(ADC) 1/ADC 1/ADC²
  %% add x0 col
  %X = [ ones(m,1), 1./sqrt(rawX), 1./rawX, 1./(rawX.^2) ];
  X = [ ones(m,1), 1./sqrt(rawX), 1./(rawX) ];
 % X = [ ones(m,1), 1./sqrt(rawX), 1./(rawX.^2) ];
  %%X = [ ones(m,1), 1./sqrt(rawX), 1./(rawX) ];
  %X = [ ones(m,1), 1./log(rawX), 1./(rawX.^2) ];
  %X = [ ones(m,1), 1./sqrt(rawX), 1./(rawX) ];

  %% theta
  regul = eye(3,3);
  regul(1,1) = 0;
  regul(2,2) = 0.001;
  regul(3,3) = 0.001;
  theta = pinv(X'*X + regul)*X'*y
  %theta = pinv(X'*X)*X'*y

  #error over distance
  error = X*theta - y;

  % plot x (ADC) => y (distance)
  subplot(4,2,2*sensor-1)
  plot(rawX,y);
  hold on;
  plot(rawX, X*theta);
  xlabel("ADC");
  ylabel("Distance(mm)");
  %legend("Raw measures","H")
  title(sprintf("Raw ADC value to Distance %s", { "DL"; "FL"; "FR"; "DR" }{sensor}))
  subplot(4,2,2*sensor)
  plot(y(20:139),error(20:139),"r");
  hold on;
  plot(y,zeros(m,1),'k');
  xlabel("Distance(mm)");
  ylabel("Error(mm)");
  %legend("H","reference")
  title("Error")

  % save
  filename = sprintf("hypothesis_%s.txt", { "DL"; "FL"; "FR"; "DR" }{sensor})
  save(filename, "theta")
  print -dpng NORMresults
end