#include "math.h"
#include <sstream>
#include <vector>
#include <iostream>
#include <tf/tf.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#define DEG2RAD M_PI/180
#define RAD2DEG 180/M_PI

/**
 * Weighted Mean of Angles (WMA)
 */

using namespace std;



float calc_mean(float a1, float a2, std::string input_type="rad"){
  float a1_rad, a2_rad;
  if (input_type == "deg"){
    a1_rad = a1 * DEG2RAD;
    a2_rad = a2 * DEG2RAD;
  }else{
    a1_rad = a1;
    a2_rad = a2;
  }
  float x = 0; float y = 0;
  x = (cos(a1_rad) + cos(a2_rad)) / 2;
  y = (sin(a1_rad) + sin(a2_rad)) / 2;
  return atan2(y, x);
}

float calc_weighted_mean(std::vector<float>& weights, std::vector<float>& angles, std::string input_type="rad"){
  if (weights.size() != angles.size()){
    std::cout<<"\033[1;31m" <<"[Warning]: The sizes between weights and angles are not matched"<< "\033[0m"<<std::endl;
    return 0;
  }

  // Calc weights_sum in order to normalize weights.
  float weights_sum = 0;
  for(std::vector<float>::iterator it = weights.begin(); it != weights.end(); ++it){
      weights_sum += *it;
  }

  float x = 0.; float y = 0.;

  float w_norm, ang_rad;
  for (uint16_t i=0; i<weights.size(); ++i){
//    std::cout<<"\033[1;32m "<<weights[i]<<" , "<< weights_sum<<std::endl;
    w_norm = weights[i]/weights_sum;
    if (input_type == "deg"){
      ang_rad = angles[i] * DEG2RAD;
    }else{
      ang_rad = angles[i];
    }
//    std::cout<<"\033[1;32m "<<ang_rad<<" , "<< w_norm<<std::endl;
    x = x + cos(ang_rad) * w_norm;
    y = y + sin(ang_rad) * w_norm;
  }
  return atan2(y, x);
}

tf::Quaternion calc_weighted_mean(std::vector<tf::Quaternion>& qs, std::vector<float>& weights){

  if (weights.size() != qs.size()){
    std::cout<<"\033[1;31m" <<"[Warning]: The sizes between weights and angles are not matched"<< "\033[0m"<<std::endl;
    tf::Quaternion dummy(0, 0, 0, 1);
    return dummy;
  }

//  // Calc weights_sum in order to normalize weights.
  float weights_sum = 0;
  for(std::vector<float>::iterator it = weights.begin(); it != weights.end(); ++it){
      weights_sum += *it;
  }
  Eigen::MatrixXf Q_T(qs.size(), 4);

//  float w_norm, ang_rad;
  float w_norm;
  for (uint16_t i=0; i<weights.size(); ++i){
    w_norm = weights[i]/weights_sum;
    auto q = qs[i].normalize();
    Q_T(i, 0) = (float) (w_norm * q.x());
    Q_T(i, 1) = (float) (w_norm * q.y());
    Q_T(i, 2) = (float) (w_norm * q.z());
    Q_T(i, 3) = (float) (w_norm * q.w());
  }
  Eigen::MatrixXf Q_square = Q_T.transpose() * Q_T;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigensolver(Q_square);
  cout << "The eigenvalues of A are:\n" << eigensolver.eigenvalues() << endl;
  cout<< eigensolver.eigenvalues().size()<<std::endl;

  float max_eigen = 0;
  int max_idx = 0;
  auto evalues = eigensolver.eigenvalues();
  auto evectors = eigensolver.eigenvectors();
  for (uint16_t i=0; i<evalues.size(); ++i){
    if (max_eigen < evalues(i)){
      max_eigen = evalues(i);
      max_idx = i;
    }
  }
  cout<<max_eigen<<" , "<<max_idx<<endl;
  cout << "Here's a matrix whose columns are eigenvectors of A \n"
       << "corresponding to these eigenvalues:\n"
       << eigensolver.eigenvectors() << endl;

  tf::Quaternion q_out(evectors(0, max_idx), evectors(1, max_idx), evectors(2, max_idx), evectors(3, max_idx));

  return q_out.normalize();
}

tf::Quaternion slerp(tf::Quaternion& q0, tf::Quaternion& q1, double ratio) {
  /**< SLERP denotes Spherical Linear interpolation  */
    tf::Quaternion q_interpolated;

    q0.normalize();
    q1.normalize();

    double dot = q0.dot(q1);

    // If the dot product is negative, slerp won't take
    // the shorter path. Note that q1 and -q1 are equivalent when
    // the negation is applied to all four components. Fix by reversing one quaternion.
    if (dot < 0.0f) {
        q1 = q1 * (-1);
        dot = - dot;
    }
    const double DOT_THRESHOLD = 0.999;

    if (dot > DOT_THRESHOLD) {
        q_interpolated = q0 + (q1 - q0) * ratio;

    }else{
      // Since dot is in range [0, DOT_THRESHOLD], acos is safe
      q_interpolated = q0.slerp(q1, ratio);
    }

    return q_interpolated.normalize();

}

tf::Quaternion lerp(tf::Quaternion& q0, tf::Quaternion& q1, double ratio) {
  /**< LERP denotes Linear interpolation  */
    tf::Quaternion q_interpolated;
    q0.normalize();
    q1.normalize();
    q_interpolated = q0 + (q1 - q0) * ratio;
    return q_interpolated.normalize();

}

// https://github.com/BobMcFry/averaging_weighted_quaternions
tf::Quaternion getAverageQuaternion(
  const std::vector<tf::Quaternion>& quaternions,
  const std::vector<float>& weights)
{
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, quaternions.size());
  Eigen::Vector3d vec;
  for (size_t i = 0; i < quaternions.size(); ++i)
  {
    // Weigh the quaternions according to their associated weight
    tf::Quaternion quat = quaternions[i] * weights[i];
    // Append the weighted Quaternion to a matrix Q.
    Q(0,i) = quat.x();
    Q(1,i) = quat.y();
    Q(2,i) = quat.z();
    Q(3,i) = quat.w();
  }

  // Creat a solver for finding the eigenvectors and eigenvalues
  Eigen::EigenSolver<Eigen::MatrixXd> es(Q * Q.transpose());

  // Find index of maximum (real) Eigenvalue.
  auto eigenvalues = es.eigenvalues();
  size_t max_idx = 0;
  double max_value = eigenvalues[max_idx].real();
  for (size_t i = 1; i < 4; ++i)
  {
    double real = eigenvalues[i].real();
    if (real > max_value)
    {
      max_value = real;
      max_idx = i;
    }
  }

  // Get corresponding Eigenvector, normalize it and return it as the average quat
  auto eigenvector = es.eigenvectors().col(max_idx).normalized();

  tf::Quaternion mean_orientation(
    eigenvector[0].real(),
    eigenvector[1].real(),
    eigenvector[2].real(),
    eigenvector[3].real()
  );

  return mean_orientation;
}
