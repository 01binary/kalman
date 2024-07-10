% A weights (3x3 matrix)
A = [ ...
  1.0005, -0.0050, 0.0001;
  0.0061, 0.9881, -0.0684;
  -0.0009, 0.0768, 0.9224;
];

% B weights (3x1 vector)
B = [ ...
  8.7913e-10;
  1.0489e-07;
  -2.4853e-05;
];

% C weights (1x3 vector)
C = [ ...
  -5.2908e+03, ...
  13.0803, ...
  -0.6389 ...
];

% D weight (scalar)
D = 0;

% K weights (3x1 vector)
K = [ ...
  -0.0003;
  0.0571;
  -0.3487;
];

% Initial state (3x1 vector)
x0 = [ ...
  -0.0461;
  -0.0198;
  0.0098;
];

% Initial state variance (3x1 vector)
dx0 = [ ...
  7.4356e+06;
  3.9306e+09;
  5.1495e+10;
];

% Input variance
inputVariance = 100;

% Measurement variance
measurementVariance = 344.5291;

% Disturbance variance
disturbanceVariance = 0.0016;

% Disturbance
disturbance = 0;

% Read input
csv = readmatrix('https://raw.githubusercontent.com/01binary/kalman/main/input.csv');
time = csv(:,1);
measurements = csv(:,2);
inputs = csv(:,3);
outputs = zeros(length(inputs), 1);
optimization = zeros(length(inputs), 1);

% Initial state
state = x0;

% Initial covariance
covariance = disturbanceVariance;

% Filter
for i = 1:length(inputs)
  % Input
  input = inputs(i);
  measurement = measurements(i);

  % Predict
  % y = Cx + Du + e
  prediction = ...
    C * state + ...  % Add contribution of state
    D * input + ...  % Add contribution of input
    disturbance;     % Add disturbance

  % Update state
  % x = Ax + Bu + Ke
  state = ...
    A * state + ...  % Add contribution of state
    B * input + ...  % Add contribution of input
    disturbance * K; % Add contribution of disturbance

  % Update covariance
  Aterm = sum(diag(A * covariance * 0.02 * A'));
  Kterm = sum(diag(K * disturbanceVariance * K'));
  %covariance = Aterm + Kterm;

  % Optimize covariance
  gain = covariance / (covariance + measurementVariance);

  % Correct prediction
  estimate = prediction + gain * (measurement - prediction);

  % Correct covariance
  covariance = (1 - gain) * covariance;

  % Output
  outputs(i) = prediction;
  optimization(i) = gain * 100;
end

% Plot measurements, outputs and Kalman gain
plot(time, measurements, time, outputs, time, optimization);
