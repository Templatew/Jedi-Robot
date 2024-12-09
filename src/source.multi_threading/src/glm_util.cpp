// header-start
////////////////////////////////////////////////////////////////////////////////
//
// \file      glm_util.cpp
// \brief     Robot Inverse Kinematic with Cyclic Coordinate Descent
//
////////////////////////////////////////////////////////////////////////////////
// header-end
//

#include <algorithm>
#include <cmath>
#include <glm/gtc/matrix_access.hpp>  // need to be included to use these features.
#include <glm/mat4x4.hpp>             // mat4
#include <glm/vec3.hpp>               // vec3
#include <glm/vec4.hpp>               // vec3
#include <iomanip>
#include <iostream>
#include <numbers>
#include <vector>
//
#include "glm_util.h"

//
// how close is a number to 0 or 1
// if delta is < min_distance, we are cool!
//
static float min_distance = 5e-6f;

std::ostream &operator<<(std::ostream &os, const glm::vec3 &vs) {
  os << "(" << vs.x << ", " << vs.y << ", " << vs.z << ")";
  return os;
}

std::ostream &operator<<(std::ostream &os, const glm::vec4 &vs) {
  float x = vs.x;
  if (std::fabs(x) < min_distance) {
    x = 0.0f;
  }

  float y = vs.y;
  if (std::fabs(y) < min_distance) {
    y = 0.0f;
  }

  float z = vs.z;
  if (std::fabs(z) < min_distance) {
    z = 0.0f;
  }

  float w = vs.w;
  if (std::fabs(w) < min_distance) {
    w = 0.0f;
  }

  os << "  {" << std::setw(11) << std::setprecision(5) << std::right << x
     << ", " << std::setw(11) << std::setprecision(5) << std::right << y << ", "
     << std::setw(11) << std::setprecision(5) << std::right << z << ", "
     << std::setw(11) << std::setprecision(5) << std::right << w << "}";

  return os;
}

std::ostream &operator<<(std::ostream &os, const glm::mat4 &vs) {
  os << "{\n";
  for (size_t idx = 0; idx < 4; ++idx) {
    glm::vec4 arow = glm::row(vs, idx);
    os << arow << '\n';
  }
  os << "}";

  return os;
}

// gamma, X, roll
// beta, Y, pitch
// alpha, Z, yaw
//

namespace glmutil {

void
swap(glm::vec4 &lhs, glm::vec4 &rhs)
{
  std::swap(lhs[0], rhs[0]);
  std::swap(lhs[1], rhs[1]);
  std::swap(lhs[2], rhs[2]);
  std::swap(lhs[3], rhs[3]);
}

static bool
close_zero(float value)
{
  if (std::fabs(value) < min_distance) {
    return true;
  }
  return false;
}

float
normalize_angle(float angle)
{
  if (std::fabs(angle) <= std::numbers::pi_v<float>) {
    return angle;
  }
  if (angle > 0.0f) {
    // greater than pi
    // 2*pi = angle + res
    return angle - ( 2.0f * std::numbers::pi_v<float> ); 
  }
  return angle + ( 2.0f * std::numbers::pi_v<float> ); 
}

extern float
angle_cost(float gamma, float beta, float alpha)
{
  return std::fabs(gamma) + std::fabs(beta) + std::fabs(alpha);
}

bool
close_enough(float value, float target)
{
  if (target == 0.0f) {
    if (std::fabs(value) < min_distance) {
      return true;
    }
  } else if ((value > (target - min_distance)) and
             (value < (target + min_distance))) {
    return true;
  }
  return false;
}



//
// given a translation (x,y,z) and rotation (roll, pitch, yaw)
// return the matching 4x4 matrix
// res = translation * rotation(yaw) * rotation(pitch) * rotation(roll)
// rpy vector must be in radian
//
glm::mat4
to_matrix4x4(const std::vector<float> xyz, const std::vector<float> rpy)
{
  //
  // translation matrix
  glm::vec4 t4_xyz{xyz[0], xyz[1], xyz[2], 1.0f};
  glm::mat4 translation = glm::column(glm::mat4(1.0f), 3, t4_xyz);

  // roll(gamma)
  float cx = std::cos(rpy[0]);
  float sx = std::sin(rpy[0]);

  // layout appear to be correct for a rotation
  // but must be transposed as glm is in col major

  glm::mat4 rx{
    {   1,   0,  0,   0 },
    {   0,  cx, -sx,  0 },
    {   0,  sx,  cx,  0 },
    {   0,   0,   0,  1 }};

  rx = glm::transpose(rx);

  // pitch (beta)
  float cy = std::cos(rpy[1]);
  float sy = std::sin(rpy[1]);

  glm::mat4 ry{
    {  cy,   0,  sy,  0 },
    {   0,   1,   0,  0 },
    { -sy,   0,  cy,  0 },
    {   0,   0,   0,  1 }};

  ry = glm::transpose(ry);

  // yaw (alpha)
  float cz = std::cos(rpy[2]);
  float sz = std::sin(rpy[2]);

  glm::mat4 rz{
    {  cz, -sz,   0,  0 },
    {  sz,  cz,   0,  0 },
    {   0,   0,   1,  0 },
    {   0,   0,   0,  1 }};

  rz = glm::transpose(rz);

  glm::mat4 rotation = rz * ry * rx;
  glm::mat4 result = translation * rotation;

  return result;
}


//
// given a 4x4 transformation matrix
// returns the translation (x,y,z) and 
// rotation information (roll, pitch, yaw)
// ie. the Euler angles (gamma, beta, alpha)

// try to optimize a few common cases

std::vector<float> get_xyz_rpy(const glm::mat4 &m) {
  std::vector<float> results;
  results.reserve(6);

  glm::vec4 acol = glm::column(m, 3);

  results.push_back(acol.x);
  results.push_back(acol.y);
  results.push_back(acol.z);

  glm::vec4 acol0 = glm::column(m, 0);
  glm::vec4 acol1 = glm::column(m, 1);
  glm::vec4 acol2 = glm::column(m, 2);

  float gamma = 0.0f;
  float beta = 0.0f;
  float alpha = 0.0f;

  // special patterns
  //
  // Roll on the X axis   (Gamma)
  //  1   0   0
  //  0  cx -sx
  //  0  sx  cx
  if (close_enough(acol0.x, 1.0f) and close_enough(acol0.y, 0.0f) and
      close_enough(acol0.z, 0.0f) and close_enough(acol1.x, 0.0f) and
      close_enough(acol2.x, 0.0f)) {
    gamma = std::atan2(acol1[2], acol1[1]);

    //
    // Pitch on the Y axis (Beta)
    //  cy   0  sy
    //   0   1   0
    // -sy   0  cy
  } else if (close_enough(acol1.x, 0.0f) and close_enough(acol1.y, 1.0f) and
             close_enough(acol1.z, 0.0f) and close_enough(acol0.y, 0.0f) and
             close_enough(acol2.y, 0.0f)) {
    beta = std::atan2(acol2[0], acol2[2]);

    //
    // Yaw on the Z  (Alpha)
    //  cz  -sz  0
    //  sz   cz  0
    //   0   0   1
  } else if (close_enough(acol2.x, 0.0f) and close_enough(acol2.y, 0.0f) and
             close_enough(acol2.z, 1.0f) and close_enough(acol0.z, 0.0f) and
             close_enough(acol1.z, 0.0f)) {
    alpha = std::atan2(acol0[1], acol0[0]);

    // special case: -sin(beta) is -1 or 1
    // -1 =>  beta = pi/2,     sin(beta) = 1,  cos(beta) = 0
    //  1 =>  beta = -pi/2,    sin(beta) = -1, cos(beta) = 0
    //
    //  case -1)
    //  a = -sin(alpha)*cos(gamma) + cos(alpha)*sin(gamma) = -sin(alpha - gamma)
    //  b =  cos(alpha)*cos(gamma) + sin(alpha)*sin(gamma) =  cos(alpha - gamma)
    //
    //  case 1)
    //  a = -sin(alpha)*cos(gamma) - cos(alpha)*sin(gamma) = -sin(alpha + gamma)
    //  b =  cos(alpha)*cos(gamma) - sin(alpha)*sin(gamma) =  cos(alpha + gamma)

    // std::atan2(y,x) -> arctan(y/x)
    // std::atan2(sin_value, cos_value)
    // -pi < value < pi
    //alpha = std::atan2(-acol2[1], -acol2[0]);
    //
    // Pitch y = 90,  Pitch y = -90
    //  0  a  b       0  a -b
    //  0  b -a       0  b  a
    // -1  0  0       1  0  0
  } else if (close_zero(acol0.x) and close_zero(acol0.y) and
             close_zero(acol1.z) and close_zero(acol2.z) and
             close_enough(std::fabs(acol0.z), 1.0f)) {
    if (acol0.z > 0.0f) {
      //std::cout << "DEBUG: Case 1,  -sin(beta) = 1\n";
      beta = -std::numbers::pi_v<float> / 2.0f;
      float alpha_plus_gamma = std::atan2(-acol1[0], acol1[1]);
      alpha = alpha_plus_gamma / 2.0f;
      gamma = alpha_plus_gamma / 2.0f;
    } else {
      //std::cout << "DEBUG: Case -1, -sin(beta) = -1\n";
      beta = std::numbers::pi_v<float> / 2.0f;
      float alpha_minus_gamma = std::atan2(-acol1[0], acol1[1]);
      //std::cout << "DEBUG: alpha_minus_gamma = " << alpha_minus_gamma << "\n";
      gamma = 0.0f;
      alpha = alpha_minus_gamma;
    }
    //
    // Pitch -sin\beta = 0
    //  *  *  @
    //  *  *  *
    //  0  *  *
  } else if (close_enough(acol0.z, 0.0f)) {
    // it must be that @ is also zero
    // for sin\alpha * sin\gamma to be zero
    // \alpha is 0 or pi OR \gamma is 0 or pi
    float gamma0 = std::atan2(acol1[2], acol2[2]);
    float alpha0 = std::atan2(acol0[1], acol0[0]);

    float c_gamma0_c_alpha0 = std::cos(gamma0) * std::cos(alpha0);

    if (std::signbit(c_gamma0_c_alpha0) == std::signbit(acol1[1])) {
      beta = 0;
      gamma = gamma0;
      alpha = alpha0;
    } else {
      beta = std::numbers::pi_v<float>;
      gamma = std::atan2(-acol1[2], -acol2[2]);
      alpha = std::atan2(-acol0[1], -acol0[0]);
    }

  } else {
    // generic case sin(beta) is not 0 or 1 or -1
    // matrix is:
    // cos(alpha).cos(beta)      **complex**           **complex**
    // sin(alpha).cos(beta)      **complex**           **complex**
    // -sin(beta)                cos(beta).sin(gamma)   cos(beta).cos(gamma)
    //

    float sinbeta = acol0[2];
    float beta0 = -std::asin(sinbeta);
    float beta1 = (beta0 > 0.0f) ? std::numbers::pi_v<float> - beta0
                                 : -std::numbers::pi_v<float> - beta0;

    // depending on the sign
    // sinbeta > 0
    // beta could be either beta0  : -pi/2 < beta0 < pi/2
    // beta could be either beta1:   pi/2 < beta1 < pi        or -pi < beta1 < -pi/2


    alpha = std::atan2(acol0[1], acol0[0]);
    gamma = std::atan2(acol1[2], acol2[2]);

    // we need to confirm the sign of cos(beta)
    // using non null entries in the matrix

    if (std::fabs(acol0[0]) > std::fabs(acol0[1])) {
      // we use cos(alpha).cos(beta) for the sign determination
      float c_alpha_c_beta0 = std::cos(alpha) * std::cos(beta0);

      if (std::signbit(c_alpha_c_beta0) == std::signbit(acol0[0])) {
        beta = beta0;
      } else {
        beta = beta1;
      }
    } else {
      // we use sin(alpha).cos(beta) for the sign determination
      float s_alpha_c_beta0 = std::sin(alpha) * std::cos(beta0);
      if (std::signbit(s_alpha_c_beta0) == std::signbit(acol0[1])) {
        beta = beta0;
      } else {
        beta = beta1;
      }
    }
  }

  // check if we can minimize the cost by application of 
  // the rotation matrix theorem which say that
  // the rotation matrix is unchanged if
  // we replace (gamma, beta, alpha) by
  // (gamma + pi, pi - beta, alpha + pi)
  float gamma1 = normalize_angle(gamma + std::numbers::pi_v<float>); 
  float beta1 = normalize_angle(std::numbers::pi_v<float> - beta); 
  float alpha1 = normalize_angle(alpha + std::numbers::pi_v<float>); 

  float cost0 = angle_cost(gamma, beta, alpha);
  float cost1 = angle_cost(gamma1, beta1, alpha1);

  if (cost0 < cost1) {
    results.push_back(gamma);  // roll
    results.push_back(beta);   // pitch
    results.push_back(alpha);  // yaw
  } else {
    results.push_back(gamma1);  // roll
    results.push_back(beta1);   // pitch
    results.push_back(alpha1);  // yaw
  }

  return results;
}

glm::vec3 get_position(const glm::mat4 &m1) {
  glm::vec4 last_col = glm::column(m1, 3);
  glm::vec3 last_col_xyz{last_col.x, last_col.y, last_col.z};
  return last_col_xyz;
}

}  // namespace glmutil
   //
#if 0
    } else {

    }

    float c_beta0_s_gamma = std::cos(beta0) * std::sin(gamma);
    float c_beta0_c_gamma = std::cos(beta0) * std::cos(gamma);

    if ((std::signbit(c_beta0_s_gamma) == std::signbit(acol1[2])) and
        (std::signbit(c_beta0_c_gamma) == std::signbit(acol2[2]))) {
      beta = beta0;
    } else {
      beta = beta1;
    }
    std::cout << "DEBUG: Generic case results(rpy) = (" << gamma << "," << beta <<"," << alpha << ")\n";
  }
#endif
