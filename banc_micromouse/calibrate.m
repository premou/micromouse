% http://www.micromouseonline.com/2010/07/07/calibrating-reflective-sensors/

% The sensor output follows very closely the photometry inverse square law [6]. 
% Thus, a simple equation can be used to model the sensor output s(x,? ) as a
% function of the distance x and the angle of incidence
% ? with the target surface :
% s(x, ? ) = (? / x^2) cos ? + ?

% with Bernard, 1/sqrt(ADC) is almost LINER
% y = 1/(ax+b)^2
% x = (1/sqrt(y)-b)/a

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
print -dpng raw
% find a (ln or squareroot law)
log_raw = 1./sqrt(filt_raw);
figure(2);
plot(log_raw)
legend("DL","FL","FR","DR")
title("distance (mm) to 1/sqrt(ADC) [linearisation]")
A = zeros(40,4);
for index = [1:40]
  A(index,:) = (log_raw(index+30+20,:) - log_raw(index+30,:))/20;
end;
figure(3);
plot(A)
legend("DL","FL","FR","DR")
title("distance [30,70] (mm) to 'a' parameter ")
a = mean(A,1)
% find b (ln or squareroot law)
b = log_raw(50,:)-50.*a
% final
synthetic_data = [ 1./((a(1)*distance+b(1)).^2), 1./((a(2)*distance+b(2)).^2),  1./((a(3)*distance+b(3)).^2),  1./((a(4)*distance+b(4)).^2) ];
figure(4);
plot(synthetic_data)
legend("DL","FL","FR","DR")
title("distance (mm) to ADC recalculated using 'a' and 'b' parameters with formula y=1/(ax+b)^2")
% error
synthetic_distance= [ (1./sqrt(filt_raw(:,1))-b(1))/a(1), (1./sqrt(filt_raw(:,2))-b(2))/a(2), (1./sqrt(filt_raw(:,3))-b(3))/a(3),  (1./sqrt(filt_raw(:,4))-b(4))/a(4)];
error = synthetic_distance.-distance;
figure(5);
plot(synthetic_distance)
legend("DL","FL","FR","DR")
title("distance (mm) to recalculated distance from ADC using 'a' and 'b' parameters with x=(1/sqrt(y)-b)/a")
print -dpng recalculated_distance 
figure(6);
plot(error)
legend("DL","FL","FR","DR")
title("distance (mm) to distance error (mm) from ADC using 'a' and 'b' paramters")
print -dpng error 
% save results
save params.txt a b

