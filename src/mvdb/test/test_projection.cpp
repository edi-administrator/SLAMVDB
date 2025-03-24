#include <gtest/gtest.h>
#include "projection.hxx"

TEST(projection_test, cube)
{
  using namespace mvdb;

  Cube c { 1.0 };
  auto px = c.pixel_projection( vec_t { 0, 0, 10 }, intr_t::Identity(), pose_t::Identity() );
  for ( auto & _px : px )
  {
    std::cerr << "ij " << std::get<0>(_px) << " z " << std::get<1>(_px) << " ? " << std::get<2>(_px) << '\n';
  }
}

TEST(projection_test, pose_interp)
{
  using namespace mvdb;

  Eigen::Quaternion<double> q_start ( Eigen::AngleAxis<double>( 0, vec_t{ 0, 0, 1 } ) );
  Eigen::Quaternion<double> q_end ( Eigen::AngleAxis<double>( k_pi, vec_t{ 0, 0, 1 } ) );
  Eigen::Quaternion<double> q_middle ( Eigen::AngleAxis<double>( k_pi / 2, vec_t{ 0, 0, 1 } ) );

  vec_t t_start { 0, 0, 0 };
  vec_t t_end { 0, 0, 1 };
  vec_t t_middle { 0, 0, 0.5 };

  pose_t T_start = pose_t::Identity(), T_end = pose_t::Identity(), T_middle = pose_t::Identity();

  T_start.block<3,3>(0,0) = q_start.toRotationMatrix();
  T_end.block<3,3>(0,0) = q_end.toRotationMatrix();
  T_middle.block<3,3>(0,0) = q_middle.toRotationMatrix();

  T_start.block<3,1>(0,3) = t_start;
  T_end.block<3,1>(0,3) = t_end;
  T_middle.block<3,1>(0,3) = t_middle;

  pose_t T_interp = interpolate_pose( T_start, T_end, 0, 1, 0.5 );

  std::cerr << "T_start = \n" << T_start << "\n";
  std::cerr << "T_end = \n" << T_end << "\n";
  std::cerr << "T_middle = \n" << T_middle << "\n";
  std::cerr << "T_interp = \n" << T_interp << "\n";
  std::cerr << "T_delta = \n" << T_middle.inverse() * T_interp << "\n";
}