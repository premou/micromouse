function J = my_cost(X, y, theta)
  %Compute cost for linear regression
  %   J = my_cost(X, y, theta) computes the cost of using theta as the
  %   parameter for linear regression to fit the data points in X and y
  m = length(y);
  J=1/(2*m)*sum((X*theta-y).^2)
end
