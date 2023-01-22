#ifndef __BRESENHAM_HPP_
#define __BRESENHAM_HPP_

#include <vector>

#include <math_utils/math.hpp>
#include <misc_utils/misc.hpp>

using math::veci;

std::vector<veci<2>> bresenham_small_slope(veci<2> const& start, 
                                           veci<2> const& end)
{
  int dx = end[0] - start[0];
  int dy = end[1] - start[1];

  std::vector<veci<2>> cells;
  cells.push_back(start);

  int e = 0;
  int x = start[0];
  int y = start[1];
  int s = (dy < 0) ? -1 : 1;
  int c = s*dx - 2*dy;
  while(x < end[0])
  {
    if(s*2*e < s*c)
      e += dy;
    else
    {
      e += dy - s*dx;
      y += s;
    }
     x += 1;
     cells.emplace_back(x, y);
  }

  return cells;
}

std::vector<veci<2>> bresenham_big_slope(veci<2>& start, veci<2>& end)
{
  swap(start);
  swap(end);
  if(start[0] > end[0])
    swap(start, end);
  
  auto swaped = bresenham_small_slope(start, end);
  for(auto& cell : swaped)
    swap(cell);
  
  return swaped;
}

std::vector<veci<2>> bresenham(veci<2> start, veci<2> end)
{
  if(start[0] > end[0])
    swap(start, end);
  
  auto d = end - start;
  if(d[0] >= abs(d[1]))
    return bresenham_small_slope(start, end);
  else
    return bresenham_big_slope(start, end);
}

#endif 