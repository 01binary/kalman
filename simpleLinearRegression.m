function predictions = linearRegression(inputs, outputs)
  xMean = mean(inputs);
  yMean = mean(outputs);

  xErrors = inputs - xMean;
  xErrorsSquared = xErrors .^2;
  yErrors = outputs - yMean;

  m = sum(xErrors .* yErrors) / sum(xErrorsSquared);
  b = yMean - m * xMean;

  predictions = inputs * m + b;
end