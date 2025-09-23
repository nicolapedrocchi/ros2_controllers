// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef JOINT_TRAJECTORY_CONTROLLER__TRAJECTORY_UTILS_HPP_
#define JOINT_TRAJECTORY_CONTROLLER__TRAJECTORY_UTILS_HPP_

#include <cmath>
#include <cstddef>
#include <rclcpp/duration.hpp>
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
  TrajectoryPointConstIter start_segment_itr = trajectory_msg->points.end();
  TrajectoryPointConstIter end_segment_itr = trajectory_msg->points.end();
  // time_from_start + trajectory time is the expected arrival time of trajectory
  const auto last_idx = trajectory_msg->points.size() - 1;
  for (size_t i = 0; i < last_idx; ++i)
  {
    auto & point = trajectory_msg->points[i];
    auto & next_point = trajectory_msg->points[i + 1];

    const rclcpp::Duration t0 =point.time_from_start;
    const rclcpp::Duration t1 = next_point.time_from_start;

    if (time_from_start >= t0 && time_from_start < t1)
    {
      // If int
      start_segment_itr = trajectory_msg->points.begin() + static_cast<TrajectoryPointConstIter::difference_type>(i);
      end_segment_itr = trajectory_msg->points.begin() + static_cast<TrajectoryPointConstIter::difference_type>(i + 1);
    }
  }
  return {start_segment_itr, end_segment_itr};
}

inline
std::tuple<TrajectoryPointConstIter, TrajectoryPointConstIter> find_segment(
  const std::shared_ptr<trajectory_msgs::msg::JointTrajectory>& trajectory_msg, 
  const rclcpp::Time& sample_time, 
  const rclcpp::Time& trajectory_start_time)
{
  TrajectoryPointConstIter start_segment_itr = trajectory_msg->points.end();
  TrajectoryPointConstIter end_segment_itr = trajectory_msg->points.end();
  // time_from_start + trajectory time is the expected arrival time of trajectory
  const auto last_idx = trajectory_msg->points.size() - 1;
  for (size_t i = 0; i < last_idx; ++i)
  {
    auto & point = trajectory_msg->points[i];
    auto & next_point = trajectory_msg->points[i + 1];

    const rclcpp::Time t0 = trajectory_start_time + point.time_from_start;
    const rclcpp::Time t1 = trajectory_start_time + next_point.time_from_start;

    if (sample_time >= t0 && sample_time < t1)
    {
      // If int
      start_segment_itr = trajectory_msg->points.begin() + static_cast<TrajectoryPointConstIter::difference_type>(i);
      end_segment_itr = trajectory_msg->points.begin() + static_cast<TrajectoryPointConstIter::difference_type>(i + 1);
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
}  // namespace trajectory_utils
}  // namespace joint_trajectory_controller

#endif  // JOINT_TRAJECTORY_CONTROLLER__TRAJECTORY_UTILS_HPP_