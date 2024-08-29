function output = movingAverageFilter(input, bufferSize)
  buffer = ones(bufferSize, 1) * input(1);
  output = zeros(length(input), 1);

  for sampleIndex = 1:length(input)
    bufferIndex = mod(sampleIndex - 1, bufferSize) + 1;
    buffer(bufferIndex) = input(sampleIndex);

    % Output is average of last bufferSize inputs
    output(sampleIndex) = ...
      sum(buffer) / bufferSize;
  end
end