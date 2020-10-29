#include "wma.hpp"

/**
 * This tutorial demonstrates weighted mean of anlgessimple sending of messages over the ROS system.
 */

using namespace std;

void print_output(float theta_1, float theta_2, float mean_out){
  cout<<"Mean of "<<theta_1 <<" and "<< theta_2 <<" degrees -> "<<mean_out<<endl;
}

int main(int argc, char **argv)
{

  float theta_1 = 0.;
  float theta_2 = 359.;
  print_output(theta_1, theta_2, (calc_mean(theta_1, theta_2, "deg")* RAD2DEG));

  theta_1 = 350.;
  theta_2 = 10.;
  print_output(theta_1, theta_2, (calc_mean(theta_1, theta_2, "deg")* RAD2DEG));

  theta_1 = 370.;
  theta_2 = 10.;
  print_output(theta_1, theta_2, (calc_mean(theta_1, theta_2, "deg")* RAD2DEG));

  theta_1 = -180.;
  theta_2 = 180.;
  print_output(theta_1, theta_2, (calc_mean(theta_1, theta_2, "deg")* RAD2DEG));

  theta_1 = 360.;
  theta_2 = 0.;
  print_output(theta_1, theta_2, (calc_mean(theta_1, theta_2, "deg")* RAD2DEG));

  theta_1 = 710.;
  theta_2 = 10.;
  print_output(theta_1, theta_2, (calc_mean(theta_1, theta_2, "deg")* RAD2DEG));

  theta_1 = -540.;
  theta_2 = 180.;
  print_output(theta_1, theta_2, (calc_mean(theta_1, theta_2, "deg")* RAD2DEG));

  std::vector<float> angles = {90., 180., 270., 360.};
  std::vector<float> weights = {1./4., 1./4., 1./4., 1./4.};
  cout<<(calc_weighted_mean(weights, angles, "deg") * RAD2DEG)<<endl;

  std::vector<float> angles2 = {10., 20., 30.};
  std::vector<float> weights2 = {1./3., 1./3., 1./3.};
  cout<<(calc_weighted_mean(weights2, angles2, "deg") * RAD2DEG)<<endl;
  std::vector<float> weights3 = {10., 1., 1.};
  cout<<(calc_weighted_mean(weights3, angles2, "deg") * RAD2DEG)<<endl;
  return 0;
}
