//
// Includes
//

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <random>
#include <limits>
#include <Eigen/Dense>

//
// Namespaces
//

using namespace std;
using namespace Eigen;

//
// Constants
//

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

// State identity (3x3 matrix)
const Matrix3d I = Matrix3d::Identity();

// Noise variance (scalar)
const double NoiseVariance = 1.539e-7;

// Noise covariance (3x3 matrix)
const MatrixXd Q = I * NoiseVariance;

// Measurement variance
const double R = 3.4556e+03;

// Initial covariance
const MatrixXd P0 {{
  { pow(dx0(0, 0), 2), 0.0, 0.0 },
  { 0.0, pow(dx0(1, 0), 2), 0.0 },
  { 0.0, 0.0, pow(dx0(2, 0), 2) }
}};

//
// Functions
//

double systemModel(
  VectorXd& x,  // state
  double u)     // input
{
  // Predict
  // y = Cx + Du
  double y =
    // Contribution of state
    C.dot(x) +
    // Contribution of input
    D * u;

  // Update state
  // x = Ax + Bu
  x =
    // Contribution of previous state
    A * x +
    // Contribution of input
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

  // Correct covariance
  P = (I - K * C) * P *
    (I - K * C).transpose() +
    K * R * K.transpose();

  return y;
}

//
// Forward Declarations
//

bool openInput(const string& path, ifstream& file);
bool openOutput(const string& path, ofstream& file);
bool read(ifstream& file, double& time, double& measurement, double& input);

//
// Entry point
//

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

  // Initialize
  VectorXd state = x0;
  VectorXd gain(3);
  MatrixXd covariance = P0;
  double time, measurement, input;
  double prediction = 0.0;

  // Filter
  while(read(inputFile, time, measurement, input))
  {
    // Correct state with measurement
    double estimate = kalmanFilter(
      prediction,
      measurement,
      state,
      covariance,
      gain
    );

    // Predict and update state
    prediction = systemModel(
      state,
      input
    );

    // Output
    outputFile
      << time << ","
      << estimate << ","
      << measurement
      << endl;
  }

  return 0;
}

//
// Utilities
//

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