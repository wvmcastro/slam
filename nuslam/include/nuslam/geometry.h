#ifndef __NUSLAM_GEOMETRY_H
#define __NUSLAM_GEOMETRY_H

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Eigenvalues>
#include <iostream>

// Implementation of A. Al-Sharadqah and N. Chernov, 
// Error Analysis for Circle Fitting Algorithms 
// Based on the pratical summary in 
// https://nu-msr.github.io/navigation_site/lectures/circle_fit.html

double fit_error(Eigen::ArrayXf& x, Eigen::ArrayXf& y, double cx, 
  double cy, double r_squared)
{
  double s = 0;
  for(int i = 0; i < x.size(); i++)
  {
    s += std::pow((std::pow(x[i] - cx, 2) + std::pow(y[i] - cy, 2) - r_squared), 2);
  }

  return std::pow(s/x.size(), 0.5);
}

Eigen::Array4f circle_fit(Eigen::ArrayXf& x, Eigen::ArrayXf& y)
{
  float ux = x.mean();
  x -= ux;

  float uy = y.mean();
  y -= uy;

  auto z = x.square() + y.square();
  
  auto n = z.size();
  Eigen::MatrixXd Z(n, 4);
  for(int i = 0; i < n; i++) Z(i, 0) = z(i);
  for(int i = 0; i < n; i++) Z(i, 1) = x(i);
  for(int i = 0; i < n; i++) Z(i, 2) = y(i);
  for(int i = 0; i < n; i++) Z(i, 3) = 1.0;
  std::cout << Z << std::endl;
  
   //auto M = (1 / n) * Z.transpose()*Z;
  
  Eigen::MatrixXd Z_svd_matrixV;
  Eigen::VectorXd Z_singular_values;

  if(n < 16)
  {
    auto Z_svd = Z.jacobiSvd(Eigen::DecompositionOptions::ComputeFullV);
    Z_svd_matrixV = Z_svd.matrixV();
    Z_singular_values = Z_svd.singularValues();
  }
  else
  {
    auto Z_svd = Z.bdcSvd(Eigen::DecompositionOptions::ComputeFullV);
    Z_svd_matrixV = Z_svd.matrixV();
    Z_singular_values = Z_svd.singularValues();
  }
  
  Eigen::VectorXd a;
  if(Z_singular_values(3) < 1e-12)
    a = Z_svd_matrixV.col(3);
  else
  {
    auto S = Z_singular_values.asDiagonal();
    auto Y = Z_svd_matrixV * S * Z_svd_matrixV.transpose();
    
    auto uz = z.mean();
    Eigen::Matrix4d H_inv;
    H_inv << 0.0, 0.0, 0.0, 0.5,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0,
             0.5, 0.0, 0.0, -2*uz;
    
    Eigen::MatrixXd Q = Y * H_inv * Y;
    Eigen::EigenSolver<Eigen::MatrixXd> eigensolver{Q};
    
    int k = -1;
    double lambda = std::numeric_limits<double>::infinity();
    auto& lambdas = eigensolver.eigenvalues();
    for(int i = 0; i <  lambdas.size(); i++)
      if(lambdas[i].real() > 0 && lambdas[i].real() < lambda)
      {
        k = i;
        lambda = lambdas[i].real();
      }
    auto a_star = eigensolver.eigenvectors().col(k).real();
    a = Eigen::MatrixXd{Y.colPivHouseholderQr().solve(a_star).real()};
  }
  
  float c = -0.5/a(0);
  float cx = c * a(1);
  float cy = c * a(2);
  
  float r_squared = 0.25*(std::pow(a(1), 2) + std::pow(a(2), 2) 
                      - 4*a(0)*a(3)) / std::pow(a(0), 2);
  
  return Eigen::Array4f{cx + ux, 
                        cy + uy, 
                        std::sqrt(r_squared), 
                        (float) fit_error(x, y, cx, cy, r_squared)};
}

#endif
