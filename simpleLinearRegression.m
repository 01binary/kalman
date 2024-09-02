function output = simpleLinearRegression(inputs, outputs, learningRate)
  output = zeros(length(inputs), 1);
  errors = zeros(length(inputs), 1);
  weight = 0;

  for n = 1:length(inputs)
    x = inputs(n);
    y = outputs(n);

    errors(n) = y - weight * x;
    totalSquaredError = sum(errors)^2 / length(inputs);
    weight = weight - totalSquaredError * learningRate;
    prediction = weight * x;

    output(n) = prediction;
  end
end
 