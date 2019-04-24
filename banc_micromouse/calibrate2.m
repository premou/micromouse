% y = e^(a/(x+b))
% x = a / ln(y) - b
clear
pkg load image
% cd ....
load data.txt
% extract raw data
distance = data([2:180],1);
raw = [ data([2:180],2), data([2:180],3), data([2:180],4), data([2:180],5) ];
% median filter on raw data
filt_raw = medfilt2(raw,[5 1]);
figure(1);
plot(filt_raw)
legend("DL","FL","FR","DR")
title("distance (mm) to filtered ADC (12bits)")
% we need Y = 1/ln(y)
Y = 1./log(filt_raw)
figure(2);
plot(Y)
legend("DL","FL","FR","DR")
title("distance (mm) to Y=1/ln(y) [LINEARISATION]")
% find a (ln or squareroot law)
A = zeros(40,4);
for index = [1:40]
  A(index,:) = (distance(index+30+20)-distance(index+30))./(Y(index+30+20,:) - Y(index+30,:));
end;
figure(3);
plot(A)
legend("DL","FL","FR","DR")
title("distance [30,70] (mm) to 'a' parameter ")
a = mean(A,1)
% find b (ln or squareroot law)
b = filt_raw(50,:)-X(50).*a
% final
synthetic_data = (a .* X ).+ b;
figure(4);
plot(synthetic_data([20:179],:))
axis([20,179,0,4000]);
legend("DL","FL","FR","DR")
title("distance (mm) to ADC recalculated using 'a' and 'b' parameters with formula y=1/(ax+b)^2")
% error
synthetic_distance = [ sqrt(a(1)./(filt_raw(:,1).-b(1))), sqrt(a(2)./(filt_raw(:,2).-b(2))), sqrt(a(3)./(filt_raw(:,3).-b(3))), sqrt(a(4)./(filt_raw(:,4).-b(4))) ];
error = synthetic_distance.-distance;
figure(5);
plot(synthetic_distance)
legend("DL","FL","FR","DR")
title("distance (mm) to recalculated distance from ADC using 'a' and 'b' parameters with x=(1/sqrt(y)-b)/a")
figure(6);
plot(error)
legend("DL","FL","FR","DR")
title("distance (mm) to distance error (mm) from ADC using 'a' and 'b' paramters")
% save results
save params.txt a b

