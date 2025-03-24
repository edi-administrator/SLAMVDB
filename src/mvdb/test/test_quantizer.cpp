#include <gtest/gtest.h>
#include <filesystem>
#include "quantizer.hxx"

using namespace mvdb;

TEST(test_quantizer, directory_load_test)
{
  auto current = std::filesystem::current_path();
  auto target = current / "../../src/mvdb/testdata/test_quant_load";
  QuantizerParams p { .discrete_path = target.string() };
  auto mats = named_mats_from_csvs<float,3,1>( p.discrete_path );

  for ( auto& [name,vector] : mats )
  {
    std::cout << "'"<< name << "' -> " << vector.transpose() << "\n";
  }
}

TEST(test_quantizer, cosine_dist)
{
  sem_t a = sem_t::Ones();
  sem_t b = -1 * sem_t::Ones();

  sem_t aa = 2 * a;
  sem_t bb = 2* b;

  Eigen::Matrix<sem_t::Scalar, sem_t::RowsAtCompileTime, 2> _bb;
  _bb = decltype(_bb)::Ones();
  _bb.col(0) *= -1;

  std::cerr << "cosine_dist(a,b)   = " << cosine_similarity(a, b) << "\n";
  std::cerr << "cosine_dist(aa,b)  = " << cosine_similarity(aa, b) << "\n";
  std::cerr << "cosine_dist(a,bb)  = " << cosine_similarity(a, bb) << "\n";
  std::cerr << "cosine_dist(a,_bb) = " << cosine_similarity(a, _bb).transpose() << "\n";
}

TEST(test_quantizer, test_load_discrete)
{
  auto current = std::filesystem::current_path();
  auto test_vectors_path = current / "../../src/mvdb/testdata/test_discrete_vectors";
  auto test_colors_path = current / "../../src/mvdb/testdata/test_discrete_colors";

  std::cerr << "vec path " << test_vectors_path << "\n";
  std::cerr << "col path " << test_colors_path << "\n";

  QuantizerParams p { .discrete_path = test_vectors_path.string(), .colors_path = test_colors_path.string() };

  Quantizer q { p };

  for ( auto i = 0; i < dsc_t::RowsAtCompileTime; i++ )
  {
    std::cerr << "class " << i <<" name '" <<  q.name( i ) << "'\n";
  }
}