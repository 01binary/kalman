#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <random>
#include <limits>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

// A weights (3x3 matrix)
const MatrixXd A
{
  { 0.998800,   0.05193, -0.02261 },
  { 0.0222200, -0.01976,  0.7353  },
  { 0.0009856, -0.20930, -0.5957  }
};

// B weights (3x1 vector)
const VectorXd B {{
  -0.00000266,
  0.0000572747,
  -0.0001872152
}};

// C weights (1x3 vector)
const RowVectorXd C {{
  -5316.903919,
  24.867656,
  105.92416
}};

// D weight (scalar)
const double D = 0;

// Initial state (3x1 vector)
const VectorXd x0 {{
  -0.0458,
  0.0099,
  -0.0139
}};

// State standard deviation (3x1 vector)
const VectorXd dx0 {{
  7.4356e+06,
  3.9306e+09,
  5.1495e+10
}};

// State identity matrix
const Matrix<double, 3, 3> I = Matrix<double, 3, 3>::Identity();

// Noise variance (scalar)
const double NoiseVariance = 1.539e-7;

// Noise covariance
const MatrixXd Q = I * NoiseVariance;

// Measurement variance
const double R = 3.4556e+03;

// Initial covariance
const MatrixXd P0 {{
  { pow(dx0(0, 0), 2), 0.0, 0.0 },
  { 0.0, pow(dx0(1, 0), 2), 0.0 },
  { 0.0, 0.0, pow(dx0(2, 0), 2) }
}};

double systemModel(
  VectorXd& x, // state
  double u)       // input
{
  // Predict
  // y = Cx + Du
  double y =
    // Add contribution of state
    C.dot(x) +
    // Add contribution of input
    D * u;

  // Update state
  // x = Ax + Bu
  x =
    // Add contribution of state
    A * x +
    // Add contribution of input
    B * u;

  return y;
}

double kalmanFilter(
  double y,    // prediction
  double z,    // measurement
  VectorXd& x, // state
  MatrixXd& P, // covariance
  VectorXd& K) // gain
{
  // Update covariance
  P = A * P * A.transpose() + Q;

  // Optimize gain
  K = (
    (P * C.transpose()) /
    (C * P * C.transpose() + R)
  );

  // Correct state with measurement
  x = x + K * (z - y);

  // Correct variance
  P = (I - K * C) * P * (I - K * C).transpose() +
    K * R * K.transpose();

  return y;
}

bool openOutput(
  const string& path, ofstream& file)
{
  // Delete if exists
  remove(path.c_str());

  // Open file for writing
  file.open(path);
  if (!file.is_open()) return false;

  // Write headers
  file << "time,estimate,measurement" << endl;

  return true;
}

bool openInput(
  const string& path, ifstream& file)
{
  // Open file for reading
  file.open(path);
  if (!file.is_open()) return false;

  // Skip headers
  string headers;
  getline(file, headers);

  return true;
}

bool read(
  ifstream& file,
  double& time,
  double& measurement,
  double& input)
{
  static string line;
  getline(file, line);

  sscanf(
    line.c_str(),
    "%lf, %lf, %lf",
    &time,
    &measurement,
    &input);

  return !file.eof();
}

int main(int argc, char** argv)
{
  ifstream inputFile;

  if (!openInput("input.csv", inputFile))
  {
    cerr << "Failed to open input file" << endl;
    return 1;
  }

  ofstream outputFile;

  if (!openOutput("output.csv", outputFile))
  {
    cerr << "Failed to open output file" << endl;
    return 1;
  }

  VectorXd state = x0;
  VectorXd gain {{ 0, 0, 0 }};
  MatrixXd covariance = P0;
  double time, measurement, input;

  while(read(inputFile, time, measurement, input))
  {
    double estimate = kalmanFilter(
      systemModel(state, input),
      measurement,
      state,
      covariance,
      gain
    );

    outputFile
      << time << ","
      << estimate << ","
      << measurement
      << endl;
  }

  return 0;
}
