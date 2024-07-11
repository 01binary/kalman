% Constants
global A;
global B;
global C;
global D;
global Q;
global R;
global I;

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

% Initial state (3x1 vector)
x0 = [ ...
  -0.0461;
  -0.0198;
  0.0098;
];

% Initial state standard deviation (3x1 vector)
dx0 = [ ...
  7.4356e+06;
  3.9306e+09;
  5.1495e+10;
];

% State identity matrix
I = eye(length(x0));

% Noise variance (scalar)
NoiseVariance = 1.539e-7;

% Noise covariance (3x3 matrix)
Q = I * NoiseVariance;

% Measurement variance
R = 3.4556e+03;

% Initial covariance
P0 = diag(dx0.^2);

% Read input
csv = readmatrix('https://raw.githubusercontent.com/01binary/kalman/main/input.csv');
time = csv(:,1);
measurements = csv(:,2);
inputs = csv(:,3);

% Initialize
state = x0;
covariance = P0;
gain = [0, 0, 0];
outputs = zeros(length(inputs), 1);
gains = zeros(length(inputs), 1);

% Filter
for i = 1:length(inputs)
  % Read input
  input = inputs(i);

  % Take measurement
  measurement = measurements(i);

  % Predict and update state
  [state, prediction] = systemModel( ...
    state, ...
    input ...
  );

  % Correct state
  [state, covariance, gain] = kalmanFilter( ...
    prediction, ...
    measurement, ...
    state, ...
    covariance, ...
    gain ...
  );

  % Output
  gains(i) = sum(gain);
  outputs(i) = prediction;
end

% Plot
plot( ...
  time, measurements, ...
  time, outputs, ...
  time, gains ...
);

function [x, y] = systemModel(x, u)
  global A;
  global B;
  global C;
  global D;
  global Q;

  % Predict
  % y = Cx + Du
  y = ...
    % Contribution of state
    C * x + ...
    % Contribution of input
    D * u;

  % Update state
  % x = Ax + Bu
  x = ...
    % Contribution of previous state
    A * x + ...
    % Contribution of input
    B * u;
end

function [x, P, K] = kalmanFilter(y, z, x, P, K)
  global A;
  global C;
  global Q;
  global R;
  global I;

  % Update covariance
  P = A * P * A' + Q;

  % Optimize gain
  K = (P * C') / (C * P * C' + R);

  % Correct state with measurement
  x = x + K * (z - y);

  % Correct covariance
  P = (I - K * C) * P * (I - K * C)' + K * R * K';
end
