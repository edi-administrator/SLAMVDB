#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <optional>
#include <filesystem>
#include <fstream>
#include <string>
#include <sstream>
#include <Eigen/Dense>
#include <limits>
#include "math_utils.hxx"
#include "typedefs.hxx"

namespace mvdb
{

template<size_t rows, size_t cols>
inline
std::optional<std::vector<Eigen::Matrix<double,rows,cols>>> 
mat_from_csv( const std::string& path )
{
  using elem_t = Eigen::Matrix<double,rows,cols>;
  using ret_t = std::vector<elem_t>;

  ret_t rv;
  std::optional<ret_t> ret;

  std::string current_path = std::filesystem::current_path();
  std::ifstream file ( path, std::ifstream::in );
  std::string line;

  if ( !file.is_open() )
  {
    std::cerr << "file not found! path: \n" << path << "\nworking directory:\n" << current_path << "\n";
  }
  else
  {
    bool failure = false;
    while ( std::getline( file, line ) && !failure )
    {
      std::istringstream ifs_line (line);
      std::string token;
      elem_t value {};
      auto i = 0;
      while ( std::getline( ifs_line, token, ',' ) )
      {
        if ( i < rows * cols )
        {
          value( i / cols, i % cols ) = std::stod( token );
        }
        i++;
      }
      if ( i == rows * cols )
      {
        rv.push_back(value);
      }
      else 
      {
        std::cerr << "file : " << current_path << " wrong line length: " << i << "\n";
      }
    }
    if ( !failure )
    {
      ret = rv;
    }
  }

  return ret;
}

template<size_t rows, size_t cols>
inline
std::vector<std::pair<size_t, Eigen::Matrix<double,rows,cols>>> 
stamped_mats_from_csvs( const std::string& dir, const std::string& ext )
{
  using mat_t = Eigen::Matrix<double,rows,cols>;
  using elem_t = std::pair<size_t, mat_t>;
  using ret_t = std::vector<elem_t>;

  ret_t out;

  std::vector<std::filesystem::path> paths;

  for ( auto& dir_entry : std::filesystem::directory_iterator(dir) )
  {
    auto p = dir_entry.path().extension();
    std::cerr << dir_entry.path() << '\n';
    if ( p == ext )
    {
      paths.push_back( dir_entry.path() );
    }
  }

  auto comp = []( const std::filesystem::path& a, const std::filesystem::path& b ) -> bool
  {
    return std::stoul( a.filename().stem().string() ) <
     std::stoul( b.filename().stem().string() );
  };

  std::sort( paths.begin(), paths.end(), comp );

  for ( auto i = 0; i < paths.size(); i++ )
  {

    auto mat_opt = mat_from_csv<rows,cols>( paths[i] );

    if ( mat_opt.has_value() )
    {
      mat_t mat = mat_opt.value().front();
      size_t stamp = std::stoul( paths[i].filename().stem().string() );
      out.push_back( elem_t { stamp, mat } );
    }

  }

  return out;
}

template<typename scalar, size_t rows, size_t cols>
inline
std::map<std::string, Eigen::Matrix<scalar, rows, cols>>
named_mats_from_csvs( const std::string& dir, const std::string& ext = ".csv" )
{
  std::map<std::string, Eigen::Matrix<scalar, rows, cols>> out;

  for ( auto& dir_entry : std::filesystem::directory_iterator(dir) )
  {
    auto opt = mat_from_csv<rows,cols>( dir_entry.path() );
    if ( opt.has_value() )
    {
      Eigen::Matrix<double,rows,cols> m = opt.value().front();
      out.insert_or_assign( dir_entry.path().filename().stem(), m.template cast<scalar>() );
    }
  }

  return out;
}

std::vector<std::pair<size_t, pose_t>> stamped_poses_from_csvs( const std::string& dir, const std::string& ext );

std::optional<std::vector<coord_t>> coords_from_csv( const std::string& path );

std::optional<std::vector<pose_t>> poses_from_csv( const std::string& path );

std::optional<std::vector<vec_t>> vectors_from_csv( const std::string& path );

};