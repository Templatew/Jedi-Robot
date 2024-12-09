#include <cmath>
#include <vector>
#include <algorithm>

#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/trigonometric.hpp>
#include <glm/ext/vector_bool4.hpp> // bvec4
#include <glm/ext/matrix_relational.hpp> // equal, all
#include <glm/ext/matrix_transform.hpp> // translate

#include "gtest/gtest.h"
#include "glm_util.h"


static float target_precision = 1.0e-6f;

//   1234567
// 1.9198621511459351,
// 1.9198622703552246, and

glm::mat4
static
compute_rotation_matrix(float roll_gamma, float pitch_beta, float yaw_alpha)
{
  // roll_gamma
  float cx = std::cos(roll_gamma);
  float sx = std::sin(roll_gamma);

  // layout appear to be correct for a rotation
  // but must be transposed as glm is in col major

  glm::mat4 rx{
    {   1,   0,  0,   0 },
    {   0,  cx, -sx,  0 },
    {   0,  sx,  cx,  0 },
    {   0,   0,   0,  1 }};

  rx = glm::transpose(rx);

  // pitch_beta
  float cy = std::cos(pitch_beta);
  float sy = std::sin(pitch_beta);

  glm::mat4 ry{
    {  cy,   0,  sy,  0 },
    {   0,   1,   0,  0 },
    { -sy,   0,  cy,  0 },
    {   0,   0,   0,  1 }};

  ry = glm::transpose(ry);

  // yaw_alpha
  float cz = std::cos(yaw_alpha);
  float sz = std::sin(yaw_alpha);

  glm::mat4 rz{
    {  cz, -sz,   0,  0 },
    {  sz,  cz,   0,  0 },
    {   0,   0,   1,  0 },
    {   0,   0,   0,  1 }};

  rz = glm::transpose(rz);

  glm::mat4 local_rotation = rz * ry * rx;
  return local_rotation;
}

TEST(GmlUtil, BasicMath)
{
  float angle0 = 0.0;
  float cos0 = std::cos(angle0);
  float sin0 = std::sin(angle0);
  EXPECT_NEAR(1.0f, cos0, target_precision);
  EXPECT_NEAR(0.0f, sin0, target_precision);

  float angle1 = std::numbers::pi_v<float> / 2.0;
  float cos1 = std::cos(angle1);
  float sin1 = std::sin(angle1);
  EXPECT_NEAR(0.0f, cos1, target_precision);
  EXPECT_NEAR(1.0f, sin1, target_precision);

  float angle2 = std::numbers::pi_v<float>;
  float cos2 = std::cos(angle2);
  float sin2 = std::sin(angle2);
  EXPECT_NEAR(-1.0f, cos2, target_precision);
  EXPECT_NEAR(0.0f, sin2, target_precision);

  float angle3 = -(std::numbers::pi_v<float> / 2.0);
  float cos3 = std::cos(angle3);
  float sin3 = std::sin(angle3);
  EXPECT_NEAR(0.0f, cos3, target_precision);
  EXPECT_NEAR(-1.0f, sin3, target_precision);
}

TEST(GmlUtil, BasicInverseMatrix1)
{
  float  roll_gamma = glm::radians(0.0f);
  float  pitch_beta = glm::radians(-110.0f);
  float  yaw_alpha = glm::radians(90.0f);

  glm::mat4 test_matrix = compute_rotation_matrix(roll_gamma, pitch_beta, yaw_alpha);
  std::vector<float> test_xyz_rpy = glmutil::get_xyz_rpy(test_matrix);

  //std::cout << "DEBUG: (Gamma) Roll  original=" << roll_gamma << ", extracted=" << test_xyz_rpy[3] << "\n";
  //std::cout << "DEBUG: (Beta)  Pitch original=" << pitch_beta << ", extracted=" << test_xyz_rpy[4] << "\n";
  //std::cout << "DEBUG: (Alpha) Yaw   original=" << yaw_alpha << ", extracted=" << test_xyz_rpy[5] << "\n";
  //std::cout << "DEBUG: Computed matrix = \n";
  //std::cout << test_matrix << "\n";

  EXPECT_EQ(6, test_xyz_rpy.size());
  EXPECT_NEAR(roll_gamma, test_xyz_rpy[3], target_precision);
  EXPECT_NEAR(pitch_beta,test_xyz_rpy[4], target_precision);
  EXPECT_NEAR(yaw_alpha,  test_xyz_rpy[5], target_precision);
}


TEST(GmlUtil, BasicInverseMatrix2)
{
  float  roll_gamma = glm::radians(0.0f);
  float  pitch_beta = glm::radians(90.0f);
  float  yaw_alpha = glm::radians(-110.0f);

  glm::mat4 test_matrix_computed = compute_rotation_matrix(roll_gamma, pitch_beta, yaw_alpha);

//  std::cout << "DEBUG: Given (gamma, beta, alpha) = (" 
//            << roll_gamma << ","
//            << pitch_beta << ","
//            << yaw_alpha << ")\n";


  // matrix in standard layout
  // but glm stores in col. major
  // so need to transpose to ensure internal representation
  // is aligned with what we see
  //
  glm::mat4 test_matrix_static = {
  {          0,   0.9396926,  -0.3420201,    0},
  {          0,  -0.3420201,  -0.9396926,    0},
  {         -1,           0,           0,    0},
  {          0,           0,           0,    1}};

  test_matrix_static = glm::transpose(test_matrix_static);

//std::cout << "DEBUG: Static matrix = \n";
//std::cout << test_matrix_static << "\n";
//std::cout << "DEBUG: Computed matrix = \n";
//std::cout << test_matrix_computed << "\n";

  glm::bvec4 const column_equal = glm::equal(test_matrix_static, test_matrix_computed, target_precision); // Evaluation per column
  bool result =  glm::all(column_equal);
  EXPECT_EQ(true, result);

  
  std::vector<float> test_xyz_rpy = glmutil::get_xyz_rpy(test_matrix_computed);


  EXPECT_EQ(6, test_xyz_rpy.size());
  EXPECT_NEAR(roll_gamma, test_xyz_rpy[3], target_precision);
  EXPECT_NEAR(pitch_beta,test_xyz_rpy[4], target_precision);
  EXPECT_NEAR(yaw_alpha,  test_xyz_rpy[5], target_precision);

  glm::mat4 test_matrix_second = compute_rotation_matrix(test_xyz_rpy[3], test_xyz_rpy[4], test_xyz_rpy[5]);


  glm::bvec4 const column_equal2 = glm::equal(test_matrix_static, test_matrix_second, target_precision); // Evaluation per column
  bool result2 =  glm::all(column_equal2);
  EXPECT_EQ(true, result2);



}

TEST(GmlUtil, BasicInverseMatrix3)
{
  float  roll_gamma = glm::radians(0.0f);
  float  pitch_beta = glm::radians(90.0f);
  float  yaw_alpha = glm::radians(110.0f);

  glm::mat4 test_matrix_computed = compute_rotation_matrix(roll_gamma, pitch_beta, yaw_alpha);

  glm::mat4 test_matrix_static = {
  {          0,  -0.9396926,  -0.3420201,    0},
  {          0,  -0.3420201,   0.9396926,    0},
  {         -1,           0,           0,    0},
  {          0,           0,           0,    1}};

  test_matrix_static = glm::transpose(test_matrix_static);

  glm::bvec4 const column_equal = glm::equal(test_matrix_static, test_matrix_computed, target_precision); // Evaluation per column
  bool result =  glm::all(column_equal);
  EXPECT_EQ(true, result);

  std::vector<float> test_xyz_rpy = glmutil::get_xyz_rpy(test_matrix_computed);

  EXPECT_NEAR(roll_gamma, test_xyz_rpy[3], target_precision);
  EXPECT_NEAR(pitch_beta,test_xyz_rpy[4], target_precision);
  EXPECT_NEAR(yaw_alpha,  test_xyz_rpy[5], target_precision);

  glm::mat4 test_matrix_second = compute_rotation_matrix(test_xyz_rpy[3], test_xyz_rpy[4], test_xyz_rpy[5]);

  glm::bvec4 const column_equal2 = glm::equal(test_matrix_static, test_matrix_second, target_precision); // Evaluation per column
  bool result2 =  glm::all(column_equal2);
  EXPECT_EQ(true, result2);
}

TEST(GmlUtil, BasicInverseMatrix4)
{
  float  roll_gamma = glm::radians(-55.0f);
  float  pitch_beta = glm::radians(-90.0f);
  float  yaw_alpha = glm::radians(-55.0f);

  glm::mat4 test_matrix_computed = compute_rotation_matrix(roll_gamma, pitch_beta, yaw_alpha);

  glm::mat4 test_matrix_static = {
  {          0,   0.9396926,   0.3420201,    0},
  {          0,  -0.3420201,   0.9396926,    0},
  {          1,           0,           0,    0},
  {          0,           0,           0,    1}};

  test_matrix_static = glm::transpose(test_matrix_static);

  glm::bvec4 const column_equal = glm::equal(test_matrix_static, test_matrix_computed, target_precision); // Evaluation per column
  bool result =  glm::all(column_equal);
  EXPECT_EQ(true, result);

  std::vector<float> test_xyz_rpy = glmutil::get_xyz_rpy(test_matrix_computed);

  EXPECT_NEAR(roll_gamma, test_xyz_rpy[3], target_precision);
  EXPECT_NEAR(pitch_beta,test_xyz_rpy[4], target_precision);
  EXPECT_NEAR(yaw_alpha,  test_xyz_rpy[5], target_precision);

  glm::mat4 test_matrix_second = compute_rotation_matrix(test_xyz_rpy[3], test_xyz_rpy[4], test_xyz_rpy[5]);
  glm::bvec4 const column_equal2 = glm::equal(test_matrix_static, test_matrix_second, target_precision); // Evaluation per column
  bool result2 =  glm::all(column_equal2);
  EXPECT_EQ(true, result2);
}


// FullInverseRPYMatrix
// computes the rotation matrix
// extract th Euler angles
// recompute a rotation matrix for these angles
// validate that these two matrix are equal
//
TEST(GmlUtil, CoarseInverseRPYMatrix)
{
  std::vector<float> angles;
  for(int a =-180; a <= 180; a += 45) {
    angles.push_back(static_cast<float>(a));
  }


  // triple loop on rpy
  for(const float roll_gamma_d: angles) {
    float  roll_gamma = glm::radians(roll_gamma_d);
    for(const float pitch_beta_d: angles) {
      float  pitch_beta = glm::radians(pitch_beta_d);
      for(const float yaw_alpha_d: angles) {
        float  yaw_alpha = glm::radians(yaw_alpha_d);
        glm::mat4 test_matrix = compute_rotation_matrix(roll_gamma, pitch_beta, yaw_alpha);

        std::vector<float> test_xyz_rpy = glmutil::get_xyz_rpy(test_matrix);
        glm::mat4 test_matrix1 = compute_rotation_matrix(test_xyz_rpy[3], test_xyz_rpy[4], test_xyz_rpy[5]);

        EXPECT_EQ(6, test_xyz_rpy.size());

        glm::bvec4 const column_equal = glm::equal(test_matrix, test_matrix1, 1e-4f); // Evaluation per column
        bool result =  glm::all(column_equal);

        EXPECT_EQ(true, result);
        if (not result) {
          std::cout << "DEBUG: First matrix = \n";
          std::cout << test_matrix << "\n";

          std::cout << "DEBUG: Re-Computed matrix = \n";
          std::cout << test_matrix1 << "\n";

          std::cout << "DEBUG: Given (gamma, beta, alpha) = (" 
                    << roll_gamma << ","
                    << pitch_beta << ","
                    << yaw_alpha << ")\n";

          std::cout << "DEBUG: Computed (gamma, beta, alpha) = (" 
                    << test_xyz_rpy[3] << ","
                    << test_xyz_rpy[4] << ","
                    << test_xyz_rpy[5] << ")\n";

        }
      }
    }
  }
}

TEST(GmlUtil, FineInverseRPYMatrix)
{
  std::vector<float> angles;
  for(int a =-180; a <= 180; a += 5) {
    angles.push_back(static_cast<float>(a));
  }

  // triple loop on rpy
  for(const float roll_gamma_d: angles) {
    float  roll_gamma = glm::radians(roll_gamma_d);
    for(const float pitch_beta_d: angles) {
      float  pitch_beta = glm::radians(pitch_beta_d);
      for(const float yaw_alpha_d: angles) {
        float  yaw_alpha = glm::radians(yaw_alpha_d);
        glm::mat4 test_matrix = compute_rotation_matrix(roll_gamma, pitch_beta, yaw_alpha);

        std::vector<float> test_xyz_rpy = glmutil::get_xyz_rpy(test_matrix);
        glm::mat4 test_matrix1 = compute_rotation_matrix(test_xyz_rpy[3], test_xyz_rpy[4], test_xyz_rpy[5]);

        EXPECT_EQ(6, test_xyz_rpy.size());

        glm::bvec4 const column_equal = glm::equal(test_matrix, test_matrix1, 1e-4f); // Evaluation per column
        bool result =  glm::all(column_equal);

        EXPECT_EQ(true, result);
      }
    }
  }
}

TEST(GmlUtil, RotationMatrixTheorem)
{
  std::vector<float> angles;
  for(int a =-180; a <= 180; a += 5) {
    angles.push_back(static_cast<float>(a));
  }

  // theorem say that given (gamma, beta, alpha)
  // the rotation matrix is unchanged if using
  // (gamma + pi, pi - beta, alpha + pi)
  //
  // cos(alpha).cos(beta)      **complex**           **complex**
  // sin(alpha).cos(beta)      **complex**           **complex**
  // -sin(beta)                cos(beta).sin(gamma)   cos(beta).cos(gamma)

  for(const float roll_gamma_d: angles) {
    float  roll_gamma = glm::radians(roll_gamma_d);
    for(const float pitch_beta_d: angles) {
      float  pitch_beta = glm::radians(pitch_beta_d);
      for(const float yaw_alpha_d: angles) {
        float  yaw_alpha = glm::radians(yaw_alpha_d);
        glm::mat4 test_matrix = compute_rotation_matrix(roll_gamma, pitch_beta, yaw_alpha);

        float pitch_beta1 = std::numbers::pi_v<float> -  pitch_beta;
        float roll_gamma1 = std::numbers::pi_v<float> + roll_gamma;
        float yaw_alpha1 = std::numbers::pi_v<float> + yaw_alpha;

        pitch_beta1 = glmutil::normalize_angle(pitch_beta1);
        roll_gamma1 = glmutil::normalize_angle(roll_gamma1);
        yaw_alpha1 = glmutil::normalize_angle(yaw_alpha1);

        glm::mat4 test_matrix1 = compute_rotation_matrix(roll_gamma1, pitch_beta1, yaw_alpha1);

        glm::bvec4 const column_equal = glm::equal(test_matrix, test_matrix1, target_precision); // Evaluation per column
        bool result =  glm::all(column_equal);
        EXPECT_EQ(true, result);
        if (not(result)) {
          std::cout << "DEBUG: (Gamma) Roll  original=" << roll_gamma << ", extracted=" << roll_gamma1 << "\n";
          std::cout << "DEBUG: (Beta)  Pitch original=" << pitch_beta << ", extracted=" << pitch_beta1 << "\n";
          std::cout << "DEBUG: (Alpha) Yaw   original=" << yaw_alpha << ", extracted=" << yaw_alpha1 << "\n";
          std::cout << "DEBUG: Computed matrix Original = \n";
          std::cout << test_matrix << "\n";
          std::cout << "DEBUG: Computed matrix Second = \n";
          std::cout << test_matrix1 << "\n";
        }

      }
    }
  }
}

TEST(GmlUtil, ForwardAndInverse)
{
  std::vector<float> angles;
  for(int a =-180; a <= 180; a += 5) {
    angles.push_back(static_cast<float>(a));
  }

  for(const float roll_gamma_d: angles) {
    float  roll_gamma = glm::radians(roll_gamma_d);
    for(const float pitch_beta_d: angles) {
      float  pitch_beta = glm::radians(pitch_beta_d);
      for(const float yaw_alpha_d: angles) {
        float  yaw_alpha = glm::radians(yaw_alpha_d);

        std::vector<float> rpy{roll_gamma, pitch_beta, yaw_alpha};
        std::vector<float> xyz{roll_gamma_d, pitch_beta_d, yaw_alpha_d};


        glm::mat4 test_matrix = glmutil::to_matrix4x4(xyz, rpy);

        glm::mat4 res_matrix1 = compute_rotation_matrix(roll_gamma, pitch_beta, yaw_alpha);
        glm::mat4 res_matrix2 = glm::translate(glm::mat4(1.0), glm::vec3(xyz[0], xyz[1], xyz[2])); 
        glm::mat4 res_matrix =  res_matrix2 * res_matrix1;

        glm::bvec4 column_equal = glm::equal(test_matrix, res_matrix, 1.0e-5f);
        bool result =  glm::all(column_equal);
        EXPECT_EQ(true, result);

        if (not result) {
          std::cout << "DEBUG: Computed matrix Original = \n";
          std::cout << test_matrix << "\n";
          std::cout << "DEBUG: Rotation matrix = \n";
          std::cout << res_matrix1 << "\n";
          std::cout << "DEBUG: Translation matrix = \n";
          std::cout << res_matrix2 << "\n";
          std::cout << "DEBUG: Computed matrix Second = \n";
          std::cout << res_matrix << "\n";
          std::cout << "(x,y,z) = (" << xyz[0] << "," << xyz[1] << "," << xyz[2] << ")\n";
        }

        auto float_compare = [](float a, float b) -> bool {
          float res = std::fabs(std::fabs(a) - std::fabs(b));
          return res < 1e-5f; 
        };

        std::vector<float> test_xyz_rpy = glmutil::get_xyz_rpy(test_matrix);
        std::vector<float> res_xyz(test_xyz_rpy.begin(),    test_xyz_rpy.begin() + 3);
        bool result_xyz =  std::ranges::equal(xyz, res_xyz, float_compare);
        EXPECT_EQ(true, result_xyz);

        if (not result_xyz) {
          std::cout << "DEBUG: First matrix = \n";
          std::cout << "(x,y,z) = (" << res_xyz[0] << "," << res_xyz[1] << "," << res_xyz[2] << ")\n";
        }
      }
    }
  }
}

/*

2024-04-25T16:10:22GMT [INFO]  Target rotation (r, p, y) = (0, 90, -109.999596)
Target Transform =
{
  {          0,      0.9397,    -0.34201,    -0.23507}
  {          0,    -0.34201,     -0.9397,    -0.64191}
  {         -1,           0,           0,     0.30754}
  {          0,           0,           0,           1}
}2024-04-25T16:10:22GMT [INFO]  ---------------------------------
2024-04-25T16:10:22GMT [INFO]  Compute forward kinematic with angles -80,-60,80,10,-40,-85
2024-04-25T16:10:22GMT [INFO]  Target position (x, y, z) = (-0.1080895, -0.71290296, 0.2189069)
2024-04-25T16:10:22GMT [INFO]  Target rotation (r, p, y) = (-132.4971, 74.553566, 135.0667)
Target Transform =
{
  {   -0.18855,     0.98024,   -0.059782,    -0.10809}
  {    0.18811,   -0.023698,    -0.98186,     -0.7129}
  {   -0.96388,    -0.19637,    -0.17992,     0.21891}
  {          0,           0,           0,           1}
*/
