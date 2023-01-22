#ifndef __SEIF_UTILS_HPP
#define __SEIF_UTILS_HPP

#include <math_utils/math.hpp>
#include <graph_utils/bfs.h>
#include <algorithm>
#include <sstream>

using math::vecd;
using math::matd;
using math::spmatd;
using math::spmatb;

namespace nuslam
{
class SEIFSlamDiffDrive;

std::vector<uint16_t>
getNeighbors(int landmark_index, double const* map, int map_size);

std::vector<uint16_t> 
getSetsUnion(std::vector<uint16_t> const& sorted_landmarks_set_a,
                  std::vector<uint16_t> const& sorted_landmarks_set_b,
                  bool& intersect);

namespace seif
{

struct MotionUpdateVariables
{
  spmatd Psi;
  spmatd Lambda;
  spmatd Phi; 
  spmatd Kappa; 
};

struct Data
{
  vecd<> state;
  vecd<> info_vector;
  spmatd info_matrix;
  int dim = -1;
};

Data
mapInfoMerge(SEIFSlamDiffDrive const& filter, Data const& other,
  std::vector<std::pair<uint16_t, uint16_t>> const& data_association);

class ActiveLandmarksBuffer
{
public:
  ActiveLandmarksBuffer(uint16_t size) : _size{size}, _count{0}
  {
    _timestamps = std::make_unique<double[]>(size);
    _ids = std::vector<uint16_t>();
    _ids.resize(size);
  }

  std::vector<uint16_t> insert(uint16_t const* new_ids, int n);
  bool remove(uint16_t landmark_id);
  bool update(uint16_t id);
  std::vector<uint16_t> landmarks(void) const;
  bool isActive(uint16_t landmark_id) const;
  bool changeLandmarkId(uint16_t old_id, uint16_t new_id);

private:
  uint16_t getIndexOfOldest() const;
  int getIndex(uint16_t landmark_id) const;
  void removeIndex(uint16_t index);

  uint16_t _size;
  uint16_t _count;
  std::unique_ptr<double[]> _timestamps;
  std::vector<uint16_t> _ids;
};

template <int N, int J>
class GraphView
{
public:
  GraphView(spmatd const& info_matrix_, uint32_t landmarks_count_):
    info_matrix{info_matrix_}, landmarks_count{landmarks_count_}
  {
    // do nothing
  }

  double operator()(uint32_t i, uint32_t j) const;
  std::vector<uint32_t> neighbors(uint32_t vertex_index) const;
  uint32_t size() const // number of vertexes
  {
    return landmarks_count+1;
  }

  spmatd const& info_matrix;
  uint32_t const landmarks_count;

private:
  uint32_t graph2MatIndex(uint32_t index) const
  {
    if(index > landmarks_count || index < 0)
    {
      std::stringstream ss;
      ss << "Index " << index << "out of range [0, " << landmarks_count << "]";
      throw std::out_of_range(ss.str());
    }

    if(index == landmarks_count)
      return 0;
    
    return N + J*index;
  }
};

template <int N, int J>
double 
GraphView<N, J>::operator()(uint32_t i, uint32_t j) const
{
  i = graph2MatIndex(i);
  j = graph2MatIndex(j);

  int block_rows_size = (i == 0) ? N : J;
  int block_cols_size = (j == 0) ? N : J;
  auto block = info_matrix.block(i, j, block_rows_size, block_cols_size);

  return block.norm() > 0 ? 1 : 0;
}

template <int N, int J>
std::vector<uint32_t>
GraphView<N, J>::neighbors(uint32_t vertex_index) const
{
  std::vector<uint32_t> neighbors;

  uint32_t col = graph2MatIndex(vertex_index);
  int index = vertex_index;
  if(col == 0) // looking for robot neighbors, which must have the active set within it
    index = -1;

  for(int i = 0; i < index; i++)
  {
    if(info_matrix.coeff(N+J*i, col) != 0)
      neighbors.push_back(i);
  }

  for(int i = index+1; i < landmarks_count; i++)
  {
    if(info_matrix.coeff(N+J*i, col) != 0)
      neighbors.push_back(i);
  }

  if(col != 0) 
    if(info_matrix.coeff(0, col) != 0) // robot
      neighbors.push_back(landmarks_count);

  return neighbors;
}

template <int N, int J>
std::vector<uint16_t>
shortestPathToRobot(GraphView<N, J> const& graph, uint16_t landmark_index)
{
  uint32_t parent[graph.size()];
  uint32_t distance = bfs(graph, graph.size()-1, landmark_index, parent);
  if(distance == std::numeric_limits<uint32_t>::infinity())
    return std::vector<uint16_t>();
  
  std::vector<uint16_t> path;
  path.reserve(distance);
  path.push_back(landmark_index);
  uint32_t next = parent[landmark_index];
  while(next != graph.size()-1) 
  {
    path.push_back(next);
    next = parent[next];
  }

  std::reverse(path.begin(), path.end());
  return path;
}

template <int N, int J>
spmatb 
makeProjectionMatrix(
  uint16_t dim,
  std::vector<uint16_t> const& landmarks_indexes,
  bool robot)
{
  int triplet_list_size = landmarks_indexes.size()*J;
  triplet_list_size += ((int)robot) * N;
  std::vector<Eigen::Triplet<bool>> triplet_list;
  triplet_list.reserve(triplet_list_size);

  int j = 0;
  if(robot)
    for(int i = 0; i < N; i++)
      triplet_list.emplace_back(j++, i, 1);

  for(auto idx: landmarks_indexes)
  {
    triplet_list.emplace_back(j++, N + J*idx, 1);
    triplet_list.emplace_back(j++, N + J*idx+1, 1);
  }

  spmatb proj{triplet_list_size, dim};
  proj.setFromTriplets(triplet_list.begin(), triplet_list.end());
  return proj;
}

template <int J>
spmatb 
makeProjectionMatrix(
  uint16_t dim,
  std::vector<uint16_t> const& block_indexes)
{
  int triplet_list_size = block_indexes.size()*J;
  
  std::vector<Eigen::Triplet<bool>> triplet_list;
  triplet_list.reserve(triplet_list_size);

  int j = 0;
  for(auto idx: block_indexes)
  {
    triplet_list.emplace_back(j++, J*idx, 1);
    triplet_list.emplace_back(j++, J*idx+1, 1);
  }

  spmatb proj{triplet_list_size, dim};
  proj.setFromTriplets(triplet_list.begin(), triplet_list.end());
  return proj;
}

}
}

#endif