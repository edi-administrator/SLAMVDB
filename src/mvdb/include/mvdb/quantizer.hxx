#pragma once

#include "typedefs.hxx"
#include "math_utils.hxx"
#include "utils.hxx"

namespace mvdb
{

struct QuantizerParams
{
  std::string discrete_path = "discrete_vectors";
  std::string colors_path = "discrete_colors";
  std::string clusters_path = "";
};

class Quantizer
{
  public:

    Quantizer( const QuantizerParams& params = {} );

    vec_t color( const sem_t& v );
    vec_t color( const dsc_t& dsc );
    double color_mono( const sem_t& v );
    double color_mono( const dsc_t& dsc );
    dsc_t discrete_sim( const sem_t& v );
    skey_t semantic_key( const sem_t& v );
    std::string name( size_t i );

  protected:
    std::vector<std::string> m_class_names;
    std::vector<vec_t> m_colors;
    std::vector<double> m_colors_mono;
    Eigen::Matrix<sem_t::Scalar, sem_t::RowsAtCompileTime, dsc_t::RowsAtCompileTime> m_ref_discrete;
    Eigen::Matrix<sem_t::Scalar, sem_t::RowsAtCompileTime, -1> m_ref_clusters;
};

}
