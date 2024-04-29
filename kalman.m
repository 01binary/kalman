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

% Initial state (3x1 vector)
x0 = [ ...
  -0.0458;
  0.0099;
  -0.0139;
];

% Read input
csv = readmatrix('https://raw.githubusercontent.com/01binary/systemid/main/input.csv');
time = csv(:,1);
measurement = csv(:,2);
input = csv(:,3);

% Simulate
x = x0;
output = zeros(length(input), 1);

for i = 1:length(input)
  u = input(i);
  [y, x] = systemModel(A, B, C, D, K, x, u, 0);
  output(i) = y;
end

% Plot
plot(time, measurement, time, output);

function [y, x] = systemModel(A, B, C, D, K, x, u, e)
  % y = Cx + Du + e
  y = ...
    C * x + ...  % Add contribution of state
    D * u + ...  % Add contribution of input
    e;           % Add disturbance

  % x = Ax + Bu + Ke
  x = ...
    A * x + ... % Add contribution of state
    B * u + ... % Add contribution of input
    e * K;      % Add contribution of disturbance
end
