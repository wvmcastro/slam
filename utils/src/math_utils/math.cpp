#include <math_utils/math.hpp>

namespace math
{

matd<2>
fromMsg(utils::Matrix2D const& msg)
{
  matd<2> m;
  m(0,0) = msg.data[0];
  
  if(msg.layout == Eigen::RowMajor)
  {
    m(0,1) = msg.data[1];
    m(1,0) = msg.data[2];
  }
  else
  {
    m(1,0) = msg.data[1];
    m(0,1) = msg.data[2];
  }
  
  m(1,1) = msg.data[3];

  return m;
}


spmatb removeMatrix(int dim, std::vector<uint16_t> const& indexes)
{
  spmatb M(dim, dim);
  std::vector<Eigen::Triplet<bool>> triplets;

  int j = 0;
  for(int i = 0; i < dim; i++)
  {
    if(j < indexes.size() && i == indexes[j])
      j++;
    else
      triplets.emplace_back(i, i-j, 1); 
  }

  M.setFromTriplets(triplets.begin(), triplets.end());

  return M;
}

}