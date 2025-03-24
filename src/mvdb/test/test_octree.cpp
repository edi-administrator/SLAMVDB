#include <gtest/gtest.h>
#include <iostream>
#include <bitset>
#include "octree.hxx"

using namespace mvdb;

TEST(octree_test, constructors)
{
  Octree tree;
  OctreeLeafOccNode nl (  0, 16, tree.params(), ockey_t {} );
  OctreeInternalNode ni ( 0, tree.params(), ockey_t {} );
  std::cerr << nl << std::endl;
  std::cerr << ni << std::endl;
}

TEST(octree_test, bbox)
{
  Octree tree { OctreeParams { .res=0.125 } };
  coord_t xyz { 1, 1, 1 };
  auto nh = tree.insert(xyz).value();
  std::cerr << "coord:     " << str(xyz) << "\n";
  std::cerr << "resolution:" << tree.resolution() << "\n";
  std::cerr << "inserted:  " << *nh << "\n";
  auto bbox = nh->bbox();
  std::cerr << "node bbox:\n";
  for ( auto& b : bbox )
  {
    std::cerr << b.transpose() << "\n";
  }
}

TEST(octree_test, coord_roundtrip)
{
  coord_t a { 0.1, 0.3, 0.4 };
  for ( auto i = 0; i <= 17; i++ )
  {
    auto b = coord_to_ockey( i, 0.25, a, 16 );
    auto c = ockey_to_coord( i, 0.25, b, 16 );
    std::cerr << str(a) << " " << str(b) << " " << str(c) << "\n";
  }
  coord_t aa { -0.51, 0.6, -0.8 };
  for ( auto i = 0; i <= 17; i++ )
  {
    auto bb = coord_to_ockey( i, 0.25, aa, 16 );
    auto cc = ockey_to_coord( i, 0.25, bb, 16 );
    std::cerr << "i=" << i << " "  << str(aa) << " " << str(bb) << " " << str(cc) << "\n";
  }
  coord_t aaa { 0, 0, 0 };
  for ( auto i = 0; i <= 17; i++ )
  {
    auto bbb = coord_to_ockey( i, 0.25, aaa, 16 );
    auto ccc = ockey_to_coord( i, 0.25, bbb, 16 );
    std::cerr << "i=" << i << " "  << str(aaa) << " " << str(bbb) << " " << str(ccc) << "\n";
  }

  std::cerr << "res =  " << 2.0 << "\n";
  coord_t aaaa { 10.625,  3.875, -0.875 };
  for ( auto i = 0; i <= 17; i++ )
  {
    auto bbbb = coord_to_ockey( i, 64.0, aaaa, 16 );
    auto cccc = ockey_to_coord( i, 64.0, bbbb, 16 );
    std::cerr << "i=" << i << " "  << str(aaaa) << " " << str(bbbb) << " " << str(cccc) << "\n";
  }

  std::cerr << "res =  " << 2.0 << "\n";
  OctreeParams p { .maxdepth = 4, .res=64.0  };
  for ( auto i = 0; i <= 17; i++ )
  {
    auto bbbb = p.coord_to_ockey( i, aaaa );
    auto cccc = p.ockey_to_coord( i, bbbb );
    std::cerr << "i=" << i << " "  << str(aaaa) << " " << str(bbbb) << " " << str(cccc) << "\n";
  }

  std::cerr << "res =  " << 2.0 << "\n";
  for ( auto i = 0; i <= 17; i++ )
  {
    OctreeParams pp { .maxdepth = size_t(i+1), .res=64.0  };
    auto bbbb = pp.coord_to_ockey_leaf( aaaa );
    auto cccc = pp.ockey_to_coord_leaf( bbbb );
    std::cerr << "maxd=" << i+1 << " "  << str(aaaa) << " " << str(bbbb) << " " << str(cccc) << "\n";
  }

  for ( double step = 0.0625; step < 1.0; step *= 2 )
  {
    for ( double x = 0.173; x < 10; x += step )
    {
        coord_t a { x, 0, 0 };
        coord_t b { 0, x, 0 };
        coord_t c { 0, 0, x };
        auto aa = coord_to_ockey( 15, 0.25, a, 16 );
        auto bb = coord_to_ockey( 15, 0.25, b, 16 );
        auto cc = coord_to_ockey( 15, 0.25, c, 16 );
        auto aaa = ockey_to_coord( 15, 0.25, aa, 16 );
        auto bbb = ockey_to_coord( 15, 0.25, bb, 16 );
        auto ccc = ockey_to_coord( 15, 0.25, cc, 16 );

        // std::cout << str(a) << " -> " << str(aaa) << "\n";
        // std::cout << str(b) << " -> " << str(bbb) << "\n";
        // std::cout << str(c) << " -> " << str(ccc) << "\n";

        EXPECT_NEAR(a[0], aaa[0], 0.125);
        EXPECT_NEAR(a[1], aaa[1], 0.125);
        EXPECT_NEAR(a[2], aaa[2], 0.125);
        EXPECT_NEAR(b[0], bbb[0], 0.125);
        EXPECT_NEAR(b[1], bbb[1], 0.125);
        EXPECT_NEAR(b[2], bbb[2], 0.125);
        EXPECT_NEAR(c[0], ccc[0], 0.125);
        EXPECT_NEAR(c[1], ccc[1], 0.125);
        EXPECT_NEAR(c[2], ccc[2], 0.125);
    }
  }
}

TEST(octree_test, key_to_int)
{
  ockey_t b { 
    0b111111111111, 
    0b000000000000, 
    0b000000111111 
    };
  std::cerr << str(b) << std::endl;
  auto c = ockey_to_xkey( b, 16 );
  std::cerr << std::setfill('0') << std::bitset<64>(c) << std::endl;
  auto d = xkey_to_ockey( c, 16 );
  std::cerr << str(d) << std::endl;
}

TEST(octree_test, insert_roundtrip)
{
  Octree tree { OctreeParams { .res=0.25 } };
  for ( double step = 0.0625; step < 1.0; step *= 2 )
  {
    for ( double x = step; x < 10; x += step )
    {
      auto nh = tree.insert( {x, 0, 0} );
      EXPECT_TRUE( nh.has_value() );
      if ( nh.has_value() )
      {
        EXPECT_NEAR( nh.value()->coord()[0], x, 0.125 );
      }
    }
  }
}

TEST(octree_test, t_init_ray)
{
  vec_t start { 0, 0, 0 };
  vec_t end { 10, -10, 0 };
  auto t_delta_init = t_initial( start, end, 0.125 );
  std::cerr << start.transpose() << "\n";
  std::cerr << end.transpose() << "\n";
  std::cerr << "sign function: " << vec_t { end.array().sign() }.transpose() << std::endl;
  std::cerr << t_delta_init.transpose() << "\n";
}

TEST(octree_test, ray_cast)
{
  coord_t start { 10, 10, 10 };
  coord_t end { 12, 10-2.05, 10 };
  std::cerr << "test start = " << str(start) << "\n";
  std::cerr << "test end   = " << str(end) << "\n";
  auto ray = key_rays( start, end, 0.125, 15, 16, 50 );
  for ( auto& key : ray )
  {
    std::cerr << str( ockey_to_coord(15, 0.125, key, 16) ) << "\n";
  }
}

TEST(octree_test, ray_cast_subsample)
{
  coord_t start { 10, 10, 10 };
  coord_t end { 20, -20, 10 };
  auto ray = key_rays( start, end, 0.125, 12, 16, 50 );
  for ( auto& key : ray )
  {
    std::cerr << str( ockey_to_coord(12, 0.125, key, 16) ) << "\n";
  }
}

TEST(octree_test, parent)
{
  auto a = xkey_to_ockey(0b100100100100100101, 16);
  auto b = xkey_to_ockey(0b100100100100100100, 16);
  EXPECT_EQ(last_same_parent_depth(a, b, 16), 14);
  auto c = xkey_to_ockey(0b100100100100110100, 16);
  auto d = xkey_to_ockey(0b100100100100100100, 16);
  EXPECT_EQ(last_same_parent_depth(c, d, 16), 13);
  auto e = xkey_to_ockey(0b100100100000100100, 16);
  auto f = xkey_to_ockey(0b100100100100100100, 16);
  EXPECT_EQ(last_same_parent_depth(e, f, 16), 12);
  auto g = xkey_to_ockey(0b100100100110100100, 16);
  auto h = xkey_to_ockey(0b100100100100100100, 16);
  EXPECT_EQ(last_same_parent_depth(g, h, 16), 12);
  auto i = xkey_to_ockey(0b101100100110100100, 16);
  auto j = xkey_to_ockey(0b100100100100100100, 16);
  EXPECT_EQ(last_same_parent_depth(i, j, 16), 9);
}

TEST(octree_test, logprob)
{
  auto logodds_h = log_from_prob(0.7);
  auto logodds_m = log_from_prob(0.4);
  auto ph = prob_from_log(logodds_h);
  auto pm = prob_from_log(logodds_m);
  std::cerr << "h: " << logodds_h << " " << ph << "\n";
  std::cerr << "n: " << logodds_m << " " << pm << "\n";
  std::cerr << "hhh: " << prob_from_log( logodds_h + logodds_h + logodds_h ) << "\n";
  std::cerr << "hhm: " << prob_from_log ( logodds_h + logodds_h + logodds_m )<< "\n";
  std::cerr << "hmm: " << prob_from_log ( logodds_h + logodds_m + logodds_m )<< "\n";
  std::cerr << "hhmm: " << prob_from_log ( logodds_h + logodds_h + logodds_m + logodds_m )<< "\n";
}


TEST(octree_test, param_methods)
{
  OctreeParams p { .maxdepth = 12, .res = 64 };

  vec_t coordinate { 10, 10, 10 };

  coord_t c_translation = coord_from_vec( coordinate );
  auto ockey = p.coord_to_ockey_leaf( c_translation );
  auto xkey_c = p.coord_to_xkey_leaf( c_translation );
  auto ockey_r = p.xkey_to_ockey( xkey_c );
  auto coord_r = p.xkey_to_coord_leaf( xkey_c );
  std::cerr << "c_translation     = " << str(c_translation) << "\n";
  std::cerr << "ockey_leaf(coord) = " << str( ockey ) << "\n";
  std::cerr << "xkey (coord)      = " << xkey_c << "\n";
  std::cerr << "ockey_leaf(round) = " << str(ockey_r) << "\n";
  std::cerr << "translation(round)= " << str(coord_r) << "\n";

  auto xk = p.vec_to_xkey_leaf( coordinate );
  auto rtrip = p.xkey_to_vec_leaf( xk );

  EXPECT_TRUE( (coordinate - rtrip).norm() < (64 / 2) * sqrt(2) );
}
