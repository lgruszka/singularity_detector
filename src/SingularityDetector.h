 /******************************************************************************
 * Detect and classify the position of a robot in the proximity of a singular position
 * EasyRobots @2022
 *      Author: Lukasz Gruszka
 *****************************************************************************/

#ifndef SINGULARITY_DETECTOR_H_
#define SINGULARITY_DETECTOR_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include <rtt/extras/SlaveActivity.hpp>

#include <string>

#include <eigen3/Eigen/Dense>
#include <std_msgs/UInt8.h>
#include <vector>


/**
 * @brief Class to detect and classify the position of a robot in the proximity of a singular position
 * 
 */
class SingularityDetector : public RTT::TaskContext {
 public:
  /**
   * @brief Construct a new Singularity Detector:: Singularity Detector object
   * 
   * @param name 
   */
  explicit SingularityDetector(const std::string& name);

  /**
   * @brief Destroy the Singularity Detector:: Singularity Detector object
   * 
   */
  virtual ~SingularityDetector();

  bool configureHook();
  void updateHook();
  bool checkAllLimitsSize(int number_of_joints_, std::vector<double> l1_lower_, std::vector<double> l1_upper_, std::vector<double> l2_lower_, std::vector<double> l2_upper_, std::vector<double> l3_lower_, std::vector<double> l3_upper_);
  int checkSingularityLevel(int number_of_joints_, Eigen::VectorXd joint_position, std::vector<double> l1_lower_, std::vector<double> l1_upper_, std::vector<double> l2_lower_, std::vector<double> l2_upper_, std::vector<double> l3_lower_, std::vector<double> l3_upper_);

 protected:
  /// Input port to read actual position
  RTT::InputPort<Eigen::VectorXd> port_joint_position; 
  /// Output port to send singularity scaling coefficient
  RTT::OutputPort<std_msgs::UInt8> port_singularity_scaling;  
  
 private:
  std::vector<double> l1_lower;
  std::vector<double> l1_upper;
  std::vector<double> l2_lower;
  std::vector<double> l2_upper;
  std::vector<double> l3_lower;
  std::vector<double> l3_upper;
  int number_of_joints;
  std_msgs::UInt8 singularity_scaling;
  Eigen::VectorXd joint_position;
};

#endif  // SINGULARITY_DETECTOR_H_

