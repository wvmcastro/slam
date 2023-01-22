#include "nuslam/landmarks_manager.hpp"
#include <algorithm>
#include <misc_utils/misc.hpp>
#include <math_utils/math.hpp>
#include <ros/ros.h>

namespace nuslam
{

bool
LandmarksManager::isPermanent(uint16_t landmark) const
{
  return find(landmark, _permanent) != _permanent.end();
}

std::vector<uint16_t>::const_iterator 
LandmarksManager::find(uint16_t landmark, 
  std::vector<uint16_t> const& vector) const
{
  return std::find(vector.begin(), vector.end(), landmark);
}

std::vector<LandmarksManager::Candidate>::const_iterator
LandmarksManager::findCandidate(uint16_t landmark_id) const
{
  auto it = _candidates.begin();
  while(it != _candidates.end())
  {
    if(it->id == landmark_id)
      return it;
  }

  return it;
}

std::vector<uint16_t> 
LandmarksManager::update(std::vector<uint16_t> const& revisited_landmarks)
{
  std::vector<uint16_t> revisited = revisited_landmarks;
  std::sort(revisited.begin(), revisited.end());
  updateCandidatesScores(revisited);

  {
    for(auto& candidate: _candidates)
      if(shouldUpgradeToPermanent(candidate) == true)
        insertToPermanent(candidate.id);

    _candidates.erase(std::remove_if(_candidates.begin(), _candidates.end(), 
      LandmarksManager::shouldUpgradeToPermanent), _candidates.end());
  }

  std::vector<uint16_t> remove_ids;
  for(auto& candidate : _candidates)
    if(shouldRemove(candidate) == true)
      remove_ids.push_back(candidate.id);


  ROS_INFO_STREAM("revisited_landmarks: " << revisited_landmarks);
  ROS_INFO_STREAM("candidates: " << _candidates);
  ROS_INFO_STREAM("permanent" << _permanent);
  ROS_INFO_STREAM("landmarks to remove: " << remove_ids);
  

  return remove_ids;
}

void
LandmarksManager::removeCandidates(std::vector<uint16_t> const& remove_list)
{
  _candidates.erase(std::remove_if(_candidates.begin(), _candidates.end(), 
    [remove_list](Candidate const& candidate)
    {
      auto it =std::find(remove_list.begin(), remove_list.end(), candidate.id);
      return it != remove_list.end();
    }
    ), _candidates.end());

  correctIds(remove_list);
}

void
LandmarksManager::insertCandidate(uint16_t landmark_id)
{
  _candidates.emplace_back(landmark_id, 1);
}

void
LandmarksManager::insertPermanent(uint16_t landmark_id)
{
  if(_permanent.size() > 0 && landmark_id <= _permanent.at(_permanent.size() - 1))
    throw std::invalid_argument("Landmark id cannot be greater than the "
                                "last landmark in buffer");

  _permanent.emplace_back(landmark_id);
}
  
void
LandmarksManager::upgradeCandidateToPermanent(uint16_t landmark_id)
{
  auto it = findCandidate(landmark_id);
  if(it == _candidates.end())
  {
    ROS_ERROR_STREAM("landmark " << landmark_id << " is not in candidates");
    throw std::invalid_argument("Landmark id not found in candidates");
  }

  _candidates.erase(it);
  insertToPermanent(landmark_id);
}

void
LandmarksManager::updateCandidatesScores(
  std::vector<uint16_t> const& revisited_landmarks)
{
  int r = 0;
  int c = 0;
  int revisited_size = revisited_landmarks.size();
  int candidates_size = _candidates.size();
  while(r < revisited_size && c < candidates_size)
  {
    if(revisited_landmarks[r] > _candidates[c].id)
    {
      _candidates[c].score -= 2;
      c++;
    }
    else if(revisited_landmarks[r] < _candidates[c].id) 
    {
      r++;
    }
    else
    {
      _candidates[c].score += 1;
      r++;
      c++;
    }
  }

  while(c < candidates_size)
  {
    _candidates[c].score -= 2;
    c++;
  }
}

bool
LandmarksManager::shouldRemove(Candidate const& candidate)
{
  return candidate.score <= -5;
}

bool
LandmarksManager::shouldUpgradeToPermanent(Candidate const& candidate)
{
  return candidate.score >= 10;
}

void
LandmarksManager::insertToPermanent(uint16_t landmark_id)
{
  // insert sorted
  _permanent.insert(
    std::upper_bound(_permanent.begin(), _permanent.end(), landmark_id),
      landmark_id
  );
}

void 
LandmarksManager::correctIds(std::vector<uint16_t> const& removed_landmarks)
{
  correctIdsAfterRemoval<uint16_t>(_permanent, removed_landmarks);


  std::vector<uint16_t> local_candidates(_candidates.size());
  for(int i = 0; i < _candidates.size(); i++)
    local_candidates[i] = _candidates[i].id;
  
  correctIdsAfterRemoval(local_candidates, removed_landmarks);
  for(int i = 0; i < _candidates.size(); i++)
    _candidates[i].id = local_candidates[i];
}

}

std::basic_ostream<char>& operator<<(std::basic_ostream<char> &ss, 
  nuslam::LandmarksManager::Candidate const &candidate) 
{
  ss << "{ id: " << candidate.id << ", score: " << (int) candidate.score << " }";
  return ss;
}