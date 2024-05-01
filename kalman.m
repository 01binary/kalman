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

% Input variance
inputVariance = 1;

% Measurement variance
measurementVariance = 841.9616;

% Disturbance variance
disturbanceVariance = 223.9915;

% Disturbance
disturbance = 0;

% Read input
csv = readmatrix('https://raw.githubusercontent.com/01binary/kalman/main/input.csv');
time = csv(:,1);
measurements = csv(:,2);
inputs = csv(:,3);
outputs = zeros(length(inputs), 1);
simulation = zeros(length(inputs), 1);

% Initial state
systemState = initialState;

% Initial state variance
stateVariance = initialStateVariance;

% Initial guess
[prediction] = systemModel(systemState, 0, 0);

% Initial guess variance
[estimateVariance] = systemVariance( ...
  initialStateVariance, inputVariance, disturbanceVariance);

gain = 1;

% Filter
for i = 1:length(inputs)
  % Take measurement
  measurement = measurements(i)

  % Blend measurement with prediction
  estimate = prediction + gain * (measurement - prediction);
  estimateVariance = (1 - gain) * estimateVariance;

  % Weigh measurement against prediction depending on which has less variance
  gain = estimateVariance / (estimateVariance + measurementVariance);

  % Predict and update state
  input = inputs(i);
  [prediction, systemState] = systemModel(systemState, input, disturbance);
  simulation(i) = prediction;

  % Update variance
  [ ...
    predictionVariance, ...
    stateVariance, ...
  ] = systemVariance( ...
    stateVariance, ...
    inputVariance, ...
    disturbanceVariance ...
  );

  % Output
  outputs(i) = estimate;
end

% Plot
plot(time, measurements, time, outputs, time, simulation);

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
