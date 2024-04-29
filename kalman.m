global A;
global B;
global C;
global D;
global K;

% A weights (3x3 matrix)
A = [ ...
  0.9988,     0.05193, -0.02261;
  0.02222,   -0.01976,  0.7353;
  0.0009856, -0.2093,  -0.5957;
];

% B weights (3x1 vector)
B = [ ...
  -0.00000266;
  0.0000572747;
  -0.0001872152;
];

% C weights (1x3 vector)
C = [ ...
  -5316.903919, ...
  24.867656, ...
  105.92416 ...
];

% D weight (scalar)
D = 0;

% K weights (3x1 vector)
K = [ ...
  -0.0001655;
  -0.001508;
  6.209e-06;
];

% Initial state (3x1 vector) from model "x0"
initialState = [ ...
  -0.0458;
  0.0099;
  -0.0139;
];

% Initial state variance (3x1 vector) from model "dx0"
initialStateVariance = [ ...
  3571390965.7972;
  61462256677.8923;
  76677742001.5582;
];

% Input variance from PID algorithm (constant)
inputVariance = 1;

% Measurement variance from encoder (constant)
measurementVariance = 841.9616;

% Disturbance variance from model "NoiseVariance"
disturbanceVariance = 223.9915;

% Disturbance
disturbance = 0;

% Read input
csv = readmatrix('https://raw.githubusercontent.com/01binary/kalman/main/input.csv');
time = csv(:,1);
measurement = csv(:,2);
input = csv(:,3);
output = zeros(length(input), 1);

% Initial state
systemState = initialState;

% Initial state variance
stateVariance = initialStateVariance;

% Initial guess
[prediction] = systemModel(systemState, 0, 0);

% Initial guess variance
[estimateVariance] = systemVariance( ...
  initialStateVariance, inputVariance, disturbanceVariance);

% Filter
for i = 1:length(input)
  % Input
  measurement = input(i);

  % Blend estimate with prediction according to which has less variance
  % If prediction has less variance, we trust the model more
  % If estimate has less variance, we trust the measurement more
  gain = estimateVariance / ...
    (estimateVariance + measurementVariance);

  % Estimate
  estimate = prediction + gain * (measurement - prediction);
  estimateVariance = (1 - gain) * estimateVariance;

  % Predict and update state
  [y, x] = systemModel( ...
    systemState, ...
    measurement, ...
    disturbance ...
  );

  [ ...
    predictionVariance, ...
    stateVariance, ...
  ] = systemVariance( ...
    stateVariance, ...
    inputVariance, ...
    disturbanceVariance ...
  );

  % Output
  output(i) = y;
end

% Plot
plot(time, measurement, time, output);

function [y, x] = systemModel(x, u, e)
  global A;
  global B;
  global C;
  global D;
  global K;

  % Predict
  % y = Cx + Du + e
  y = ...
    C * x + ...  % Add contribution of state
    D * u + ...  % Add contribution of input
    e;           % Add disturbance

  % Update state
  % x = Ax + Bu + Ke
  x = ...
    A * x + ... % Add contribution of state
    B * u + ... % Add contribution of input
    e * K;      % Add contribution of disturbance
end

function [predictionVariance, stateVariance] = systemVariance( ...
  stateVariance, inputVariance, disturbanceVariance ...
)
  global A;
  global B;
  global C;
  global D;
  global K;

  % Update prediction variance
  predictionVariance = ...
    sum(C * stateVariance * C') + ...
    D^2 * inputVariance + ...
    disturbanceVariance;

  % Update state variance (vector)
  stateVariance = ...
    A * stateVariance + ...
    B * inputVariance + ...
    K * disturbanceVariance;
end
