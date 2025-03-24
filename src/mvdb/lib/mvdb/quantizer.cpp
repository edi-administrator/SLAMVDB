#include "quantizer.hxx"


namespace mvdb
{

Quantizer::Quantizer( const QuantizerParams& params )
{
  auto named_vec = named_mats_from_csvs<float, sem_t::RowsAtCompileTime, 1>( params.discrete_path );
  auto named_colors = named_mats_from_csvs<double, 3, 1>( params.colors_path );

  if ( named_vec.size() != dsc_t::RowsAtCompileTime )
  {
    throw std::out_of_range( "number of class vectors supplied is not correct! path = " + params.discrete_path );
  }

  if ( named_colors.size() != named_vec.size() )
  {
    throw std::out_of_range( "incorrect number of colors supplied! path = " +  params.colors_path );
  }

  m_colors_mono = decltype(m_colors_mono) {
    0.99, // asphalt
    0.1, // building
    0.3, // bush
    0.75, // dirt
    0.5, // grass
    0.01, // other
    0.85, // rock or rock-bed
    0.01, // sky
    0.25, // tree
    0.01 // water
  };

  size_t count = 0;
  auto color_it = named_colors.begin();
  for ( ; color_it != named_colors.end(); color_it++, count++ )
  {
    std::cerr << "Quantizer: " << color_it->first << " loaded\n";
    m_colors.push_back( color_it->second );
    // m_colors_mono.push_back( double(count) / double(named_colors.size()) ); 
    m_class_names.push_back( color_it->first );
    m_ref_discrete.col(count) = named_vec.at( color_it->first );
  }
}

vec_t Quantizer::color( const sem_t& v )
{
  return color( discrete_sim( v ) );
}

vec_t Quantizer::color( const dsc_t& dsc )
{
  int i;
  dsc.maxCoeff(&i);
  return m_colors[i];
}

double Quantizer::color_mono( const sem_t& v )
{
  return color_mono( discrete_sim( v ) );
}

double Quantizer::color_mono( const dsc_t& dsc )
{
  int i;
  dsc.maxCoeff(&i);
  return m_colors_mono[i];
}

dsc_t Quantizer::discrete_sim( const sem_t& v )
{
  return cosine_similarity( v, m_ref_discrete );
}

/*! \brief [TODO]: Currently unused! Outputs type zero */
skey_t Quantizer::semantic_key( const sem_t& v )
{
  return skey_t::Zero();
}

std::string Quantizer::name( size_t i )
{
  return m_class_names[i];
}

}