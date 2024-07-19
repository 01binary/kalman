function output = lowPassFilter(input, coefficient)
  estimate = input(1);
  output = zeros(length(input));

  for sampleIndex = 1:length(input)
    sample = input(sampleIndex);

    % Output is input blended with previous estimate
    estimate = ...
      (1.0 - coefficient) * estimate + ...
      coefficient * sample;

    output(sampleIndex, 1) = estimate;
  end
end