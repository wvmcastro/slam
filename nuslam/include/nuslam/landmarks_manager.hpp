#ifndef __LANDMARKS_MANAGER_HPP
#define __LANDMARKS_MANAGER_HPP

#include <vector>
#include <cstdint>
#include <sstream>

namespace nuslam
{

class LandmarksManager
{
public:
struct Candidate {
  Candidate(uint16_t landmark_id, int8_t score_):
    id{landmark_id}, score{score_}
  {
    // do nothing
  }
  uint16_t id;
  int8_t score;
};
  LandmarksManager()
  {
    // do nothing
  }

  std::vector<uint16_t> update(std::vector<uint16_t> const&);
  void insertCandidate(uint16_t);
  void insertPermanent(uint16_t);
  void upgradeCandidateToPermanent(uint16_t);
  bool isPermanent(uint16_t) const;
  void removeCandidates(std::vector<uint16_t> const&);

  std::vector<uint16_t> const& permanents() const
  {
    return _permanent;
  }

private:
  std::vector<uint16_t> _permanent;
  std::vector<Candidate> _candidates;
  
  std::vector<uint16_t>::const_iterator find(uint16_t,
    std::vector<uint16_t> const&) const;
  std::vector<Candidate>::const_iterator findCandidate(uint16_t) const;
  
  static bool shouldRemove(Candidate const&);
  static bool shouldUpgradeToPermanent(Candidate const&);
  void insertToPermanent(uint16_t);
  void updateCandidatesScores(std::vector<uint16_t> const&);
  void correctIds(std::vector<uint16_t> const&);
};

}

std::basic_ostream<char>& operator<<(std::basic_ostream<char> &ss, 
  nuslam::LandmarksManager::Candidate const &candidate); 

#endif