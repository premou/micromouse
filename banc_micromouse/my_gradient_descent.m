function [theta, J_history] = my_gradient_descent(X, y, theta, alpha, num_iters)
% Performs gradient descent to learn theta
%   theta = my_gradient_descent(X, y, theta, alpha, num_iters) updates theta by 
%   taking num_iters gradient steps with learning rate alpha
  m = length(y);
  J_history = zeros(num_iters, 1);
  for iter = 1:num_iters
    theta = theta - alpha * ( X' * (X*theta-y) ) / m;
    J_history(iter) = my_cost(X, y, theta);
  end
end
