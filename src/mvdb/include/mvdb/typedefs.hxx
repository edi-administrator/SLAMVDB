#pragma once

#include <array>
#include <cstddef>
#include <Eigen/Dense>
#include <limits>

namespace mvdb
{

using coord_t = std::array<double, 3>;
using ockey_t = std::array<size_t,3>;
using vec_t = Eigen::Vector3d;
using pxf_t = Eigen::Vector2d;
using pxi_t = Eigen::Vector2i;
using pose_t = Eigen::Matrix4d;
using intr_t = Eigen::Matrix3d;

const size_t k_size_max = std::numeric_limits<size_t>::max();
constexpr double k_pi = 3.14159265358979323846;

struct _s_key_t
{
  static _s_key_t Zero() { return _s_key_t { 0 }; }
  uint64_t placeholder;
};

using sem_t = Eigen::Vector<float, 512>;
using dsc_t = Eigen::Vector<float, 10>;
using uid_t = std::string;
using skey_t = _s_key_t;
using xkey_t = size_t;
using nano_t = size_t;

const double k_epsilon = std::numeric_limits<sem_t::Scalar>::epsilon();

}