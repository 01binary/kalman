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
const RowVectorXd B {{
  -0.00000266,
  0.0000572747,
  -0.0001872152
}};

// C weights (1x3 vector)
const VectorXd C {{
  -5316.903919,
  24.867656,
  105.92416
}};

// D weight (scalar)
const double D = 0;

// K weights (3x1 vector)
const RowVectorXd K {{
  -0.0001655,
  -0.001508,
  6.209e-06
}};

// Initial state (3x1 vector)
const RowVectorXd x0 {{
  -0.0458,
  0.0099,
  -0.0139
}};

/*
  * Discrete state-space system model
  * @param x: system state to update
  * @param u: system input
  * @param e: disturbance
  * @return: system output
*/
double systemModel(
  RowVectorXd& x, double u, double e)
{
  // Predict
  // y = Cx + Du + e
  double y =
    // Add contribution of state
    C.dot(x) +
    // Add contribution of input
    D * u +
    // Add disturbance
    e;

  // Update state
  // x = Ax + Bu + Ke
  x =
    // Add contribution of state
    x * A +
    // Add contribution of input
    B * u +
    // Add contribution of disturbance
    K * e;

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
  file << "time,prediction,measurement" << endl;

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

  RowVectorXd state = x0;
  double time, measurement, input;
  double disturbance = 0.0;

  while(read(inputFile, time, measurement, input))
  {
    double prediction = systemModel(
      state, input, disturbance);

    outputFile
      << time << ","
      << prediction << ","
      << measurement
      << endl;
  }

  return 0;
}
