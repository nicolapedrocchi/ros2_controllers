// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef JOINT_TRAJECTORY_CONTROLLER__TRAJECTORY_UTILS_HPP_
#define JOINT_TRAJECTORY_CONTROLLER__TRAJECTORY_UTILS_HPP_

#include <cmath>
#include <cstddef>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <vector>
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "joint_trajectory_controller/trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace joint_trajectory_controller
{
namespace trajectory_utils
{

inline
std::tuple<TrajectoryPointConstIter, TrajectoryPointConstIter> find_segment(
  const std::shared_ptr<trajectory_msgs::msg::JointTrajectory>& trajectory_msg, 
  const rclcpp::Duration& time_from_start)
{
  TrajectoryPointConstIter start_segment_itr = trajectory_msg->points.begin();
  TrajectoryPointConstIter end_segment_itr = trajectory_msg->points.begin();
  if (trajectory_msg->points.empty() || time_from_start < trajectory_msg->points.front().time_from_start)
  {
    if (trajectory_msg->points.empty())
    {
      RCLCPP_WARN( rclcpp::get_logger("joint_trajectory_controller"),"Received empty trajectory message");
    }
  }
  else if (time_from_start >= trajectory_msg->points.back().time_from_start)
  {
    start_segment_itr = trajectory_msg->points.end() - 1;
    end_segment_itr = trajectory_msg->points.end();
  }
  else
  {
    // time_from_start + trajectory time is the expected arrival time of trajectory
    const auto last_idx = trajectory_msg->points.size() - 1;
    for (size_t i = 0; i < last_idx; ++i)
    {
      auto & point = trajectory_msg->points[i];
      auto & next_point = trajectory_msg->points[i + 1];

      const rclcpp::Duration t0 = point.time_from_start;
      const rclcpp::Duration t1 = next_point.time_from_start;
      if (time_from_start >= t0 && time_from_start < t1)
      {
        start_segment_itr = trajectory_msg->points.begin() + static_cast<TrajectoryPointConstIter::difference_type>(i);
        end_segment_itr = trajectory_msg->points.begin() + static_cast<TrajectoryPointConstIter::difference_type>(i + 1);
      }
    }
  }
  return {start_segment_itr, end_segment_itr};
}

/// 1: OK
/// 0: out of limits
/// -1: joint name not found
inline
int leqt(const std::map<std::string, double>& limits, const std::string& joint_name, double value, bool abs = true)
{
  auto it = limits.find(joint_name);
  if(it != limits.end())
  {
    return abs ? std::fabs(value) <= it->second ? 1 : 0 : value <= it->second ? 1 : 0;
  }
  return -1;
}

inline
int leqt(const std::map<std::string, double>& limits, const std::vector<std::string> joint_names, const std::vector<double>& v, bool abs = true)
{
  for(std::size_t j=0; j<joint_names.size();j++)
  {
    int res = leqt(limits, joint_names.at(j), v[j], abs);
    if(res==-1 || res==0)
      return res;
  }
  return 1;
}

inline
std::vector<double> multiply(const double factor, const std::vector<double>& v)
{
  std::vector<double> ret = v;
  std::transform(ret.begin(), ret.end(), ret.begin(),
                   [factor](double val) { return val * factor; });;
  return ret;
}

inline
std::vector<double> add(const std::vector<double>& a, const std::vector<double>& b)
{
  std::vector<double> ret=a;
  std::transform(ret.begin(), ret.end(), b.begin(), ret.begin(), std::plus<double>());
  return ret;
}

inline 
void apply_scaling_factor(
  const double scaling_factor,
  const double scaling_factor_derivative,
  trajectory_msgs::msg::JointTrajectoryPoint & p)
{
  
  if (!p.velocities.empty())
  {
    p.velocities = multiply(scaling_factor, p.velocities);
  }
  if (!p.accelerations.empty())
  {
    auto a0 = trajectory_utils::multiply(scaling_factor*scaling_factor, p.accelerations);
    auto a1 = trajectory_utils::multiply(scaling_factor_derivative, p.velocities);
    p.accelerations = add(a0, a1);
  }
}

inline
std::tuple<rclcpp::Duration,double, double, TrajectoryPointConstIter, TrajectoryPointConstIter> compute_interval_and_scaling(
  const std::shared_ptr<trajectory_msgs::msg::JointTrajectory>& trajectory_msg,
  const rclcpp::Duration & sample_time_from_start, 
  const rclcpp::Duration & period, 
  const double& scaling_factor, 
  const double& prev_scaling_factor,
  const std::map<std::string, double>& max_velocities,
  const std::map<std::string, double>& max_accelerations, 
  const double min_allowed_scaling_factor = 1e-4)
{
  std::vector<double> zeros(trajectory_msg->joint_names.size(), 0.0);
  double dtau_i = scaling_factor;
  double dtau_i_1 = prev_scaling_factor;
  double ddtau_i = (dtau_i-dtau_i_1)/period.seconds();
  rclcpp::Duration tau_i_1 = sample_time_from_start;
  rclcpp::Duration tau_i = sample_time_from_start + period * dtau_i;
  TrajectoryPointConstIter k_1_itr, k_itr;
  // ========
  do
  {
    std::cout << "--------- sample time: " << tau_i.seconds() << std::endl;
    std::tie(k_1_itr, k_itr) = trajectory_utils::find_segment(trajectory_msg, tau_i);
    if(k_itr == trajectory_msg->points.end())
    {
      break;
    }
    const std::vector<double> & v_k = k_itr->velocities.empty() ? zeros : k_itr->velocities;
    const std::vector<double> & a_k = k_itr->accelerations.empty() ? zeros : k_itr->accelerations;
    if(1 == trajectory_utils::leqt(
      max_velocities, 
      trajectory_msg->joint_names, 
      trajectory_utils::multiply(dtau_i, v_k),
      true))
    {
      ddtau_i = (dtau_i-dtau_i_1)/period.seconds();
      auto a_k_first_term = trajectory_utils::multiply(dtau_i*dtau_i, a_k);
      auto a_k_second_term = trajectory_utils::multiply(ddtau_i, v_k);
      if(1 == trajectory_utils::leqt(max_accelerations, trajectory_msg->joint_names, trajectory_utils::add(a_k_first_term, a_k_second_term),true))
      {
        break;
      }
    }
    // This algorithm is brutal, TODO: implement a more graceful and fast degradation
    dtau_i = 0.94*dtau_i; // from 1 to 1e-4 in about 150 iterations
    if(dtau_i<min_allowed_scaling_factor)
    {
      RCLCPP_WARN(rclcpp::get_logger("joint_trajectory_controller"),"Severe scaling factor reduction, trajectory is probably not feasible");
      tau_i  = tau_i_1;
      dtau_i = 0.0;
      dtau_i_1 = 0.0;
      break;
    }
    else
    {
      tau_i  = tau_i_1 + period * dtau_i;
    }
  } while (true);

  
  return {tau_i, dtau_i, ddtau_i, k_1_itr, k_itr};
}



}  // namespace trajectory_utils
}  // namespace joint_trajectory_controller

#endif  // JOINT_TRAJECTORY_CONTROLLER__TRAJECTORY_UTILS_HPP_