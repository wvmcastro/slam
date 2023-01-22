#include <nuslam/seif_utils.hpp>
#include <nuslam/seif.hpp>
#include <ros/ros.h>

namespace nuslam
{
std::vector<uint16_t>
getNeighbors(int landmark_index, double const* adjacency, int size)
{
  std::vector<uint16_t> neighbors;
  for(int i = 0; i < landmark_index; i++)
    if(adjacency[2*i] != 0)
      neighbors.push_back(i);

  for(int i = landmark_index+1; i < size; i++)
    if(adjacency[2*i] != 0)
      neighbors.push_back(i);

  return neighbors;
}

std::vector<uint16_t> 
getSetsUnion(std::vector<uint16_t> const& sorted_landmarks_set_a,
                  std::vector<uint16_t> const& sorted_landmarks_set_b,
                  bool& intersect)
{
  intersect = math::intersect(
    sorted_landmarks_set_a.begin(), sorted_landmarks_set_a.end(),
    sorted_landmarks_set_b.begin(), sorted_landmarks_set_b.end()
  );

  std::vector<uint16_t> blanket;
  if(intersect == false)
    return blanket;

  blanket.resize(sorted_landmarks_set_a.size() + sorted_landmarks_set_b.size());
  auto end = std::set_union(
                sorted_landmarks_set_a.begin(), sorted_landmarks_set_a.end(),
                sorted_landmarks_set_b.begin(), sorted_landmarks_set_b.end(),
                blanket.begin()
             );
  blanket.resize(end - blanket.begin());
  return blanket;
}

namespace seif
{

Data
mapInfoMerge(SEIFSlamDiffDrive const& filter, Data const& other,
  std::vector<std::pair<uint16_t, uint16_t>> const& data_association)
{
  // create augmented info vector and matrix
  uint32_t n = filter.internalStateSize();
  auto augmented_info_mat_triplets = math::sparseMatrixToTriplets(
    filter._info_matrix);
  math::sparseMatrixToTriplets(other.info_matrix,
    augmented_info_mat_triplets, n);

  Data augmented;

  uint32_t augmented_size = n + other.info_vector.size();

  ROS_INFO_STREAM("Other state size: " << other.info_vector.size());
  ROS_INFO_STREAM("Augmented size before any merge: " << augmented_size);

  augmented.info_matrix = spmatd(augmented_size, augmented_size);
  augmented.info_matrix.setFromTriplets(augmented_info_mat_triplets.begin(),
    augmented_info_mat_triplets.end());

  augmented.info_vector = vecd<>(augmented_size);
  augmented.info_vector.head(n) = filter._info_vector.head(n);
  augmented.info_vector.tail(other.info_vector.size()) = other.info_vector;

  int merge_count = 0;
  for(auto& association : data_association)
  {
    uint16_t landmark_id = association.second;
    uint16_t other_landmark_id = association.first - merge_count;

    ROS_INFO_STREAM("Merging landmark filter " << landmark_id
      << " with landmark " << other_landmark_id << " from other"
      << " (previously " << association.first << ")" );
    
    // the merge count can be subtracted like that because we start from
    // the minor indexes to the big one
    auto M = math::mergeMatrix<filter.J>(augmented.info_vector.size(),
      landmark_id, filter._landmarks_count+other_landmark_id,
      filter.N);

    augmented.info_matrix = M.cast<double>().transpose() * augmented.info_matrix
      * M.cast<double>();
    
    augmented.info_vector = M.cast<double>().transpose() * augmented.info_vector;

    merge_count++;
  }

  int filter_max_size = filter.N + filter.J * filter.map_size;
  if(augmented.info_vector.size() > filter_max_size)
  {
    ROS_ERROR_STREAM("Resize augmented filter info, because it exceeds"
      " maximum filter size");
    augmented.info_matrix.resize(filter_max_size, filter_max_size);
    augmented.info_vector.resize(filter_max_size);
  }

  augmented.dim = -1;
  return augmented;
}

std::vector<uint16_t>
ActiveLandmarksBuffer::landmarks() const
{
  return {_ids.begin(), _ids.begin()+_count};
}

bool
ActiveLandmarksBuffer::update(uint16_t id)
{
  for(int j = 0; j < _count; j++)
  {
    if(_ids[j] == id)
    {
      _timestamps[j] = ros::Time::now().toSec();
      return true;
    }
  }
  return false;
}

int 
ActiveLandmarksBuffer::getIndex(uint16_t landmark_id) const
{
  for(int i = 0; i < _count; i++)
    if(_ids[i] == landmark_id)
      return i;
  
  return -1;
}

void
ActiveLandmarksBuffer::removeIndex(uint16_t index)
{
  for(int i = index; i < _count-1; i++)
    {
      _ids[i] = _ids[i+1];
      _timestamps[i] = _timestamps[i+1];
    }
    _count--;
}

bool 
ActiveLandmarksBuffer::remove(uint16_t landmark_id)
{
  int index = getIndex(landmark_id);
  if(index == -1)
    return false;

  removeIndex(index);

  return true;
}

bool
ActiveLandmarksBuffer::isActive(uint16_t landmark_id) const
{
  if(getIndex(landmark_id) != -1)
    return true;
  
  return false;
}

std::vector<uint16_t> 
ActiveLandmarksBuffer::insert(uint16_t const* new_ids, int n)
{
  std::vector<uint16_t> dropped; // ie landmarks that will become passive
  double now = ros::Time::now().toSec();
  int i = 0;
  while(i < n && _count < _size)
  {
    _ids[_count] = new_ids[i];
    _timestamps[_count] = now;
    _count++;
    i++;
  }
  while(i < n)
  {
    int removed = getIndexOfOldest();
    dropped.push_back(_ids[removed]);
    _ids[removed] = new_ids[i++];
    _timestamps[removed] = now;
  }

  ROS_INFO_STREAM("Active landmarks: " << landmarks());
  ROS_INFO_STREAM("Dropped landmarks: " << dropped);

  return dropped;
}

uint16_t
ActiveLandmarksBuffer::getIndexOfOldest() const
{
  double time = std::numeric_limits<double>::max();
  uint16_t index = -1;
  for(uint16_t i = 0; i < _count; i++)
  {
    if(_timestamps[i] < time)
    {
      time = _timestamps[i];
      index = i;
    }
  }
  return index;
}

bool
ActiveLandmarksBuffer::changeLandmarkId(uint16_t old_id, uint16_t new_id)
{
  int index = getIndex(old_id);
  if(index == -1)
    return false;
  
  _ids[index] = new_id;

  return true;
}

}
}