function output = linearRegressionFilter(xInput, yInput, damping)
  output = zeros(length(xInput), 1);
  errors = zeros(length(xInput), 1);
  weight = 0;

  for n = 1:length(xInput)
    x = xInput(n);
    y = yInput(n);

    errors(n) = y - weight * x;
    totalSquaredError = sum(errors)^2 / length(xInput);
    weight = weight - totalSquaredError * damping;

    output(n) = weight * x;
  end
end
 