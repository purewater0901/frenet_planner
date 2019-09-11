#ifndef STATELATTICEPLANNER_MATH_H
#define STATELATTICEPLANNER_MATH_H

#include <vector>
#include <Eigen/Eigen>
#include <cmath>
#include <cassert>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//スプラインの係数を計算する
static std::vector<double> calcSplineCoeffcient(const std::vector<double>& s, const std::vector<double>& y)
{
    assert(s.size()==4);
    assert(y.size()==4);

    std::vector<double> coefficients;
    coefficients.resize(4);

    double diff = s[3] - s[0];
    coefficients[0] = y[0];
    coefficients[1] = (-1.0/2.0) * (-2*y[3]+11*y[0]-18*y[1]+9*y[2]) / diff;
    coefficients[2] = (9.0/2.0) * (-y[3]+2*y[0]-5*y[1]+4*y[2]) / (diff*diff);
    coefficients[3] = (-9.0/2.0) * (-y[3]+y[0]-3*y[1]+3*y[2])/ (diff*diff*diff);

    return coefficients;
}

//calculate kurvature using cubic polynomials 
static double interpolation(const std::vector<double>& coefficients, const double& s)
{
    return coefficients[3]*s*s*s + coefficients[2]*s*s + coefficients[1]*s + coefficients[0];
}

//calculate yaw angle using cubic polynomials
static double integratedInterpolation(const std::vector<double>& coefficients, const double& s)
{
    return coefficients[3]*s*s*s*s/4 + coefficients[2]*s*s*s/3 + coefficients[1]*s*s/2 + coefficients[0]*s;
}

/*!
   * \brief normalize_angle_positive
   *
   *        Normalizes the angle to be 0 to 2*M_PI
   *        It takes and returns radians.
   */
  static inline double normalizeAnglePositive(double angle)
  {
    return fmod(fmod(angle, 2.0*M_PI) + 2.0*M_PI, 2.0*M_PI);
  }


  /*!
   * \brief normalize
   *
   * Normalizes the angle to be -M_PI circle to +M_PI circle
   * It takes and returns radians.
   *
   */
  static inline double normalizeAngle(double angle)
  {
    double a = normalizeAnglePositive(angle);
    if (a > M_PI)
      a -= 2.0 *M_PI;
    return a;
  }


static void cartesian_to_frenet(const double rs, const double rx, const double ry, const double rtheta,
    const double x, const double y, double* ptr_s, double* ptr_d) 
{
    const double dx = x - rx;
    const double dy = y - ry;

    const double cos_theta_r = std::cos(rtheta);
    const double sin_theta_r = std::sin(rtheta);

    const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
    *ptr_d = std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);
    *ptr_s = rs;
}

static cv::Point2i cv_offset(float x, float y,int image_width=1000, int image_height=1000)
{
    cv::Point2i output;
    output.x = int(x * 15) + 50;
    output.y = int(y * 30);
    return output;
};

#endif //STATELATTICEPLANNER_MATH_H