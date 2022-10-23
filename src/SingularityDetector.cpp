 /******************************************************************************
 * Detect and classify the position of a robot in the proximity of a singular position
 * EasyRobots @2022
 *      Author: Lukasz Gruszka
 *****************************************************************************/

#include "SingularityDetector.h"

SingularityDetector::SingularityDetector(const std::string& name)
    : RTT::TaskContext(name, PreOperational) {

  this->addProperty("number_of_joints", number_of_joints);
  this->addProperty("singularity_level1_lower", l1_lower);
  this->addProperty("singularity_level1_upper", l1_upper);
  this->addProperty("singularity_level2_lower", l2_lower);
  this->addProperty("singularity_level2_upper", l2_upper);
  this->addProperty("singularity_level3_lower", l3_lower);
  this->addProperty("singularity_level3_upper", l3_upper);
  this->addPort("JointPosition", port_joint_position);
  this->addPort("SingularityScaler", port_singularity_scaling);
}

SingularityDetector::~SingularityDetector() {
}


/**
 * @brief Implement this method such that it contains the code which will be executed when configure() is called. 
 * 
 * @return true   to indicate that configuration succeeded and the Stopped state may be entered. 
 * @return false  to indicate that configuration failed and the Preoperational state is entered. 
 */
bool SingularityDetector::configureHook() {  
  try {
    if (number_of_joints <= 0)
      return false;
    if (!checkAllLimitsSize(number_of_joints, l1_lower, l1_upper, l2_lower, l2_upper, l3_lower, l3_upper))
      return false;
    joint_position.resize(number_of_joints);
    singularity_scaling.data = 1.0;   // init scaling parameter value

    return true;
  } catch (std::exception &e) {
    RTT::Logger::log(RTT::Logger::Error) << e.what() << RTT::endlog();
    return false;
  } catch (...) {
    RTT::Logger::log(RTT::Logger::Error) << "unknown exception !!!"
                                         << RTT::endlog();
    return false;
  }
  return true;
}


/**
 * @brief Check singularity level in each periodic step 
 * 
 */
void SingularityDetector::updateHook() {
  if (port_joint_position.read(joint_position) == RTT::NewData) {
    int singularity_level = checkSingularityLevel(number_of_joints, joint_position, l1_lower, l1_upper, l2_lower, l2_upper, l3_lower, l3_upper);
    singularity_scaling.data = singularity_level+1;
  }
  port_singularity_scaling.write(singularity_scaling);
}


/**
 * @brief Check if all of singularity level limits have proper size
 * 
 * @param number_of_joints_   number of robot joints
 * @param l1_lower_           vector of lower limits for 1st singularity stage for all joints
 * @param l1_upper_           vector of upper limits for 1st singularity stage for all joints
 * @param l2_lower_           vector of lower limits for 2st singularity stage for all joints
 * @param l2_upper_           vector of upper limits for 2st singularity stage for all joints
 * @param l3_lower_           vector of lower limits for 3st singularity stage for all joints
 * @param l3_upper_           vector of upper limits for 3st singularity stage for all joints
 * @return true   if all of singularity level limits have proper size 
 * @return false  if any of singularity level limits has wrong size
 */
bool SingularityDetector::checkAllLimitsSize(int number_of_joints_,
                                              std::vector<double> l1_lower_, std::vector<double> l1_upper_, 
                                              std::vector<double> l2_lower_, std::vector<double> l2_upper_, 
                                              std::vector<double> l3_lower_, std::vector<double> l3_upper_) {
  std::vector<std::vector<double> > lower_limits {l1_lower_,l2_lower_,l3_lower_};
  std::vector<std::vector<double> > upper_limits {l1_upper_,l2_upper_,l3_upper_};
  for (int i=0; i<lower_limits.size(); i++){
    if (lower_limits[i].size() != number_of_joints_){
      RTT::Logger::log(RTT::Logger::Error)  << i << " lower limit wrong size: " << lower_limits[i].size() 
                                            << ", should be: " << number_of_joints_ << RTT::endlog();
      return false;
    }
  }
  for (int i=0; i<upper_limits.size(); i++){
    if (upper_limits[i].size() != number_of_joints_){
      RTT::Logger::log(RTT::Logger::Error)  << i << " upper limit wrong size: " << upper_limits[i].size() 
                                            << ", should be: " << number_of_joints_ << RTT::endlog();
      return false;
    }
  }
  return true;
}

/**
 * @brief Compare every joint position to upper and lower limits of different singularity level
 * 
 * @param number_of_joints_   number of robot joints
 * @param joint_position      joint position being checked
 * @param l1_lower_           vector of lower limits for 1st singularity stage for all joints
 * @param l1_upper_           vector of upper limits for 1st singularity stage for all joints
 * @param l2_lower_           vector of lower limits for 2st singularity stage for all joints
 * @param l2_upper_           vector of upper limits for 2st singularity stage for all joints
 * @param l3_lower_           vector of lower limits for 3st singularity stage for all joints
 * @param l3_upper_           vector of upper limits for 3st singularity stage for all joints
 * @return int                index of the singularity level of actual position 
 */
int SingularityDetector::checkSingularityLevel(int number_of_joints_, Eigen::VectorXd joint_position, 
                                                  std::vector<double> l1_lower_, std::vector<double> l1_upper_, 
                                                  std::vector<double> l2_lower_, std::vector<double> l2_upper_, 
                                                  std::vector<double> l3_lower_, std::vector<double> l3_upper_) {
  int max = 0;
  int temp = 0;
  //find the highest level of proximity to the singularity achieved by any axis
  for (int i=0; i< number_of_joints; i++) {
    if (joint_position[i]<l3_upper_[i] && joint_position[i]>l3_lower_[i]) {
      temp = 3;
      max = 3;
      break;
    }
    else if (joint_position[i]<l2_upper_[i] && joint_position[i]>l2_lower_[i]) {
      temp = 2;
    }
    else if (joint_position[i]<l1_upper_[i] && joint_position[i]>l1_lower_[i]) {
      temp = 1;
    }
    else {
      temp = 0;
    }
    if (temp >= max) {
      max = temp;
    }
  }
  return max;
}

ORO_CREATE_COMPONENT(SingularityDetector)
