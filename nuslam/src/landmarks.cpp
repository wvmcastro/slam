#include <string>
#include <cmath>
#include <limits>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <nuslam/geometry.h>

#include <nuslam/TurtleMap.h>
#include <nuslam/LandmarkDebug.h>
#include <math_utils/math.hpp>

class LandMarkDetector
{
public:
  float derivative_thld; 
  LandMarkDetector(std::string const& laser_topic, 
                   std::string const& landmarks_topic,
                   float derivative_threshold=0)
  {
    _laser_data = nullptr;
    
    _laser_data_sub = _nh.subscribe(laser_topic,
        1, &LandMarkDetector::detect_and_publish_landmarks, this);
   
    _landmarks_pub = _nh.advertise<nuslam::TurtleMap>(landmarks_topic, 1);
    
    ros::NodeHandle p_nh("~");
    
    derivative_thld = derivative_threshold;
    if(derivative_thld == 0)
      p_nh.getParam("derivative_threshold", derivative_thld);

    p_nh.getParam("debug", _debug);
    if(_debug == true)
      _landmarks_debug_pub = _nh.advertise<nuslam::LandmarkDebug>(
          "nuslam/debug", 1);
  }

private:
  bool _debug{false};
  sensor_msgs::LaserScan::ConstPtr const* _laser_data; 
  std::vector<float> _laser_signal_derivative;
  std::vector<float> _laser_signal;
  ros::NodeHandle _nh;
  ros::Publisher _landmarks_pub;
  ros::Publisher _landmarks_debug_pub;
  ros::Subscriber _laser_data_sub;

  void detect_and_publish_landmarks(
    sensor_msgs::LaserScan::ConstPtr const& data)
  {
    _laser_data = &data;
    std::vector<int> peaks_indexes = process_laser_signal();
    auto cylinders_boundaries_indexes = extract_cylinder_boundaries(
        peaks_indexes);
    
    std::vector<Eigen::Array4f> landmarks;
    landmarks.reserve(cylinders_boundaries_indexes.size() / 2);
    
    auto& c = cylinders_boundaries_indexes;
    for(int i = 0; i < c.size(); i += 2)
    {
      auto x = get_xy_points_from_range_and_theta(c[i], c[i+1], 'x');
      if(x.size() > 3)
      {
        auto y = get_xy_points_from_range_and_theta(c[i], c[i+1], 'y');
        auto landmark = circle_fit(x, y);

        float fitness_error = landmark(3);
        float radius = landmark(2);
        if(fitness_error < 1e-3 && radius > 0.065)
        {
          math::cartesianToPolar(landmark.data());
          landmarks.push_back(landmark);
        }
      }
    }
    nuslam::TurtleMap landmarks_message;
    fillLandmarksMessage(landmarks, landmarks_message);
    _landmarks_pub.publish(landmarks_message);

    if(_debug == true)
      pubDebugMessage(cylinders_boundaries_indexes);
  }
  
  std::vector<int>
  process_laser_signal()
  {
    clamp_laser_data();
    laser_data_centered_derivative();
    
    std::vector<int> peaks_indexes;
    for(int i = 0; i < _laser_signal.size(); i++)
    {
      if(std::abs(_laser_signal_derivative[i]) > derivative_thld)
      {
        peaks_indexes.push_back(i);
      }
    }

    return peaks_indexes;
  }
  
  std::vector<int> extract_cylinder_boundaries(std::vector<int> peaks_indexes)
  {
    bool on_cylinder = false;
    int start, end;
    std::vector<int> cylinders_boundaries_indexes;
    for(auto i : peaks_indexes)
    {
      if(_laser_signal_derivative[i] < 0)
      {
        on_cylinder = true;
        start = i;
      }
      else if(_laser_signal_derivative[i] > 0 && on_cylinder == true)
      {
        end = i;
        cylinders_boundaries_indexes.push_back(start);
        cylinders_boundaries_indexes.push_back(end);
        on_cylinder = false;
      }
    }

    // Threat the special case when the scan starts in the middle of a 
    // cylinder. When this happens the first peak is positive, instead of
    // negative, and the flag on_cylinder finishes the above loop with
    // true state.
    if(on_cylinder == true)
      if(_laser_signal_derivative[peaks_indexes[0]] > 0)
      {
        end = peaks_indexes[0];
        cylinders_boundaries_indexes.push_back(start);
        cylinders_boundaries_indexes.push_back(end);
      }
    return cylinders_boundaries_indexes;
  }

  Eigen::ArrayXf get_xy_points_from_range_and_theta(int left_index,
                                                    int right_index,
                                                    char coord)
  {
    auto f = (coord == 'x') ? std::cos<float> : std::sin<float>;
    float theta0 = (*_laser_data)->angle_min; 
    auto theta_increment = (*_laser_data)->angle_increment;
    Eigen::ArrayXf v;
    if(right_index > left_index)
    {
      v = Eigen::ArrayXf(right_index - left_index + 1);
      int k = 0;
      for(int i = left_index; i <= right_index; i++)
        v(k++) = ((*_laser_data)->ranges[i] * 
                  f(theta0 + i*theta_increment)).real();
    }
    else
    {
      auto n = (*_laser_data)->ranges.size() + right_index - left_index + 1;
      v = Eigen::ArrayXf(n);
      int k = 0;
      for(int i = left_index; i < (*_laser_data)->ranges.size(); i++)
        v(k++) = ((*_laser_data)->ranges[i] *
                  f(theta0 + i*theta_increment)).real();

      for(int i = 0; i <= right_index; i++)
        v(k++) = ((*_laser_data)->ranges[i] * f(i*theta_increment)).real();
    }

    return v;
  }

  void fillLandmarksMessage(std::vector<Eigen::Array4f>& landmarks,
                            nuslam::TurtleMap& message)
  {
    int n = landmarks.size();
    message.range.resize(n);
    message.bearing.resize(n);
    message.radius.resize(n);
    message.id.resize(n);
    for(int i = 0 ; i < n; i++)
    {
      message.id[i] = i;
      message.range[i] = landmarks[i](0);
      message.bearing[i] = landmarks[i](1);
      message.radius[i] = landmarks[i](2);
    }
    message.header.frame_id = (*_laser_data)->header.frame_id;
    message.header.stamp = ros::Time::now();  
  }

  void pubDebugMessage(std::vector<int> const& peaks)
  {
    static nuslam::LandmarkDebug debug_msg;
    
    debug_msg.header.frame_id = (*_laser_data)->header.frame_id;
    debug_msg.header.stamp = ros::Time::now();
    debug_msg.threshold = derivative_thld;

    debug_msg.signal_derivative = _laser_signal_derivative;
    debug_msg.peaks_index = peaks;
    _landmarks_debug_pub.publish(debug_msg);
  }

  void clamp_laser_data()
  {
    _laser_signal = std::vector<float>((*_laser_data)->ranges);
    
    auto min = (*_laser_data)->range_min;
    auto max = (*_laser_data)->range_max;

    auto& v = _laser_signal;
    for(int i = 0; i < v.size(); i++)
    {
      if(v[i] < min) v[i] = min;
      else if(v[i] > max) v[i] = max;
    }
  }

  void laser_data_centered_derivative()
  {
    auto& signal = _laser_signal;
    
    if(_laser_signal_derivative.size() != signal.size())
      _laser_signal_derivative = std::vector<float>(signal.size());

    double delta_inv = 0.5 / (*_laser_data)->angle_increment; 
    auto n = signal.size() - 1;
    for(int i = 1; i < n; i++)
    {
      _laser_signal_derivative[i] = delta_inv * (signal[i+1] - signal[i-1]);
    }

    // calculate derivative in both extremes
    _laser_signal_derivative[0] = delta_inv * (signal[1] - signal[n]);
    _laser_signal_derivative[n] = delta_inv * (signal[0] - signal[n-1]);
  }
  
  float index2angle(int i)
  {
    return (*_laser_data)->angle_min + i * (*_laser_data)->angle_increment;
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "landmarks_detector");
  ros::NodeHandle nh;
  
  std::string laser_topic, landmarks_topic;
  nh.getParam("laser_topic", laser_topic);
  nh.getParam("landmarks_topic", landmarks_topic);
  LandMarkDetector detector{laser_topic, landmarks_topic};

  ros::spin();
}


