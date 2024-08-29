function outputs = linearRegressionFilter( ...
  inputs, ...
  bufferSize ...
)
  half = floor(bufferSize / 2);
  outputs = zeros(1, length(inputs));

  for n = 1:length(inputs)
    % Sample next and previous samples around this sample
    startIndex = max(1, n - half);
    endIndex = min(length(inputs), n + half);
    x = (startIndex:endIndex)';
    y = inputs(startIndex:endIndex);
    xMean = mean(x);
    yMean = mean(y);

    % Tune contribution of input to output
    weight = ...
      sum((x - xMean) .* (y - yMean)) / ...
      sum((x - xMean) .^ 2);

    % Tune initial state
    initialState = yMean - weight * xMean;

    % Predict
    outputs(n) = weight * n + initialState;
  end
end
