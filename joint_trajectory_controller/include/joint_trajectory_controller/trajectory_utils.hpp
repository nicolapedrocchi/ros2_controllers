// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef JOINT_TRAJECTORY_CONTROLLER__TRAJECTORY_UTILS_HPP_
#define JOINT_TRAJECTORY_CONTROLLER__TRAJECTORY_UTILS_HPP_


#include <iterator>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <sstream>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include "joint_trajectory_controller/trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace joint_trajectory_controller
{
namespace trajectory_utils
{

/**
 * \param trajectory_msg The trajectory message to search in.
 * \param time_from_start The time from the start of the trajectory to sample at.
 * \return A tuple of the start and end segment iterators. 
 *   The start segment iterator points to the
 *   trajectory point with the largest time_from_start that is smaller than or equal to the given
 *   time_from_start. The end segment iterator points to the trajectory point with the smallest
 *   time_from_start that is larger than the given time_from_start. 
 *   If the given time_from_start is before the first trajectory point, the start segment iterator 
 *   will be equal to the end segment iterator, which will point to the first trajectory point. 
 *   If the given time_from_start is after the last trajectory point, the start segment iterator 
 *   will point to the last trajectory point and the end segment iterator will be equal to 
 *   the trajectory_msg->points.end().
 */
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
        break;
      }
    }
  }
  return {start_segment_itr, end_segment_itr};
}

/**
 * \param limits A map of joint names to their limits.
 * \param joint_name The name of the joint to check.
 * \param value The value to check against the limits.
 * \param abs Whether to check the absolute value.
 * \return 1 if the value is within limits, 0 if out of limits, -1 if joint name not found.
 */
inline
int leqt(const std::map<std::string, double>& limits, const std::string& joint_name, const double& value, bool abs = true)
{
  auto it = limits.find(joint_name);
  if(it != limits.end())
  {
    return (abs ? std::fabs(value) <= it->second : value <= it->second) ? 1 : 0;
  }
  return -1;
}

/**
 * \param limits A map of joint names to their limits.
 * \param joint_names The names of the joints to check.
 * \param v The values to check against the limits.
 * \param abs Whether to check the absolute value.
 * \return 1 if the value is within limits, 0 if out of limits, -1 if joint name not found.
 * Note: this function assumes that the size of joint_names and v are the same and that the order 
 * of joint_names corresponds to the order of values in v. In the case this is not true, the 
*  function raises a std::runtime_error
 * The limits, instead, may be store more information than limits of the joint_names.
 */
inline
int leqt(const std::map<std::string, double>& limits, const std::vector<std::string>&joint_names, const std::vector<double>& v, bool abs = true)
{
  if(joint_names.size()!=v.size())
  {
    throw std::runtime_error("Caught an exception in performing 'leqt' operation in the"
    " joint_trajectory_controller math. The input vectors 'joint_names' and 'v'"
    " have different sizes");
  }
  for(std::size_t j=0; j<joint_names.size();j++)
  {
    int res = leqt(limits, joint_names.at(j), v[j], abs);
    if(res==-1 || res==0)
      return res;
  }
  return 1;
}

/**
 * \param factor The factor to multiply the vector by.
 * \param v The vector to multiply.
 * \return The result of multiplying the vector by the factor.
 * Note: this function does not modify the input vector, it returns a new vector.
 */
inline
std::vector<double> multiply(const double& factor, const std::vector<double>& v)
{
  std::vector<double> ret = v;
  std::transform(ret.begin(), ret.end(), ret.begin(),
                   [factor](double val) { return val * factor; });
  return ret;
}

/**
 * \param a A vector of double.
 * \param b A vector of double.
 * \return The sum of a and b
 * Note: this function assumes that the size of a and b are the same. In the case this is not true,
 * the function raises a std::runtime_error
 */
inline
std::vector<double> add(const std::vector<double>& a, const std::vector<double>& b) 
{
  if(a.size()!=b.size())
  {
    throw std::runtime_error("Caught an exception in performing 'add' operation in the"
    " joint_trajectory_controller math. The input vectors 'a' and 'b'"
    " have different sizes");
  }
  std::vector<double> ret(a.size(), 0.0);
  std::transform(a.begin(), a.end(), b.begin(), ret.begin(), std::plus<double>());
  return ret;
}

/**
 * @brief Applies the scaling factor to the given trajectory point.
 * @param scaling_factor The scaling factor to apply.
 * @param scaling_factor_derivative The derivative of the scaling factor to apply.
 * @param p The trajectory point to apply the scaling factor to.
 * Note: this function modifies the input trajectory point.
 */
inline 
void apply_scaling_factor(
  const double& scaling_factor,
  const double& scaling_factor_derivative,
  trajectory_msgs::msg::JointTrajectoryPoint & p)
{
  if (!p.accelerations.empty())
  {
    auto a0 = trajectory_utils::multiply(scaling_factor*scaling_factor, p.accelerations);
    auto a1 = !p.velocities.empty() ? trajectory_utils::multiply(scaling_factor_derivative, p.velocities) : std::vector<double>(a0.size(), 0.0);
    p.accelerations = add(a0, a1);
  }
  if (!p.velocities.empty())
  {
    p.velocities = multiply(scaling_factor, p.velocities);
  }
}

/**  @brief Solve per-joint quadratic constraints on  x == ddtau_i.
 * 
 * The acceleration limits are defined on the second derivative of the trajectory with respect to time 'i', i.e. ddtau_i.
 * The accleraion can be expressed as: 
 *  (dtau_i_1 + ddtau_i)^2 * a_k[i]  +  ddtau_i * v_k[i]  <=  a_max[i]
 * where:
 *  dtau_i_1 is the scaling factor at the step i-1
 *  ddtau_i is the scaling factor modification at the step i that I have to add to grant the system dynamics
 *
 * Solve per-joint quadratic constraints on  x == ddtau_i.
 * For each joint j: a_k[i]*x^2 + (2*a_k[i]*dtau_i_1 + v_k[i])*x + (a_k[i]*dtau_i_1^2 - a_max[i]) <= 0
 * Returns the largest feasible ddtau_i, or NaN if infeasible.
 *
 * @param v_k velocity at the k-th knot of the trajectory (given sampling the trajectory at time_from_start + dtau_i)
 * @param a_k velocity at the k-th knot of the trajectory (given sampling the trajectory at time_from_start + dtau_i)
 * @param joint_names vector of the joint names
 * @param max_accelerations vector of the acceleration boundaries
 * NOTE: This algorithm is not exact, since it assumes that v_k and a_k do not change if we change dtau_i, that is not true!!!
 */
inline
double solve_max_ddtau(
    const std::vector<double>& v_k,
    const std::vector<double>& a_k,
    const std::vector<std::string>& joint_names,
    const std::map<std::string, double>& max_accelerations,
    const double dtau_i_1)
{
  /// numerical epsilon for convergence
  constexpr double kEps = 1e-8;

  /// This structure is then needed to tackle the multiple solutions of the equation
  struct Interval
  {
    double lo;
    double hi;
  };
  const double INF = std::numeric_limits<double>::infinity();
  std::vector<Interval> feasible = {{-INF, INF}};

  /// This function is designed to find the intersection between sets of intervals
  auto intersect = [](const std::vector<Interval>& a,
                      const std::vector<Interval>& b)
  {
    std::vector<Interval> result;

    for(const auto& ia : a)
    {
      for(const auto& ib : b)
      {
        double lo = std::max(ia.lo, ib.lo);
        double hi = std::min(ia.hi, ib.hi);

        if(lo <= hi)
          result.push_back({lo,hi});
      }
    }

    return result;
  };

  for (std::size_t j = 0; j < joint_names.size(); j++)
  {
    // Ease the notation:
    // FROM:
    //    a_k[i]*x^2 + (2*a_k[i]*dtau_i_1 + v_k[i])*x + (a_k[i]*dtau_i_1^2 - a_max[i]) <= 0
    // TO:
    //    A*x^2 + (2*A*d + V)*x + (A*d^2 - L) <= 0
    const double A = a_k[j];
    const double V = v_k[j];
    const double L = std::fabs(max_accelerations.at(joint_names[j]));
    const double d = dtau_i_1;

    // Ease the notation:
    // FROM: 
    //    A*x^2 + (2*A*d + V)*x + (A*d^2 - L) <= 0
    // TO: 
    //    A*x^2 + B*x + C <= 0
    const double B = 2.0 * A * d + V;
    const double C = A * d * d - L;

    // The vector to store the solution's validity intervals
    std::vector<Interval> constraint;

    // Solve the quadratic equation
    if (std::fabs(A) < kEps)
    {
      // Linear case: B*x + C <= 0
      // we have only 1 solution, so we should assign or x_min or x_max for the j-th joint.
      if (std::fabs(B) < kEps)
      {
        if (C > 0.0) 
        {
          // The sclaing is infeasible for the j-th joint 
          // therefore we must consider the scaling infeasible for all the joints!
          return std::numeric_limits<double>::quiet_NaN();
        }
        else
        {
          // The sclaing is always satisfied for the j-th joint. 
          // Therefore, we impose a vector with the validity range from -inf to +inf
          constraint = {{-INF, INF}};
        }
      }
      else if (B > 0.0)
      {
        constraint = {{-INF, -C/B}}; // x <= -C/B
      }
      else
      {
        constraint = {{-C/B, INF}};  // x >= -C/B
      }
    }
    else //if (std::fabs(A) > kEps)
    {
      // Full Quadratic Case
      // We have eventually one or two range of solutions 
      
      // computation of the equation discriminant
      const double discriminant = B * B - 4.0 * A * C;

      if (discriminant < 0.0)
      {  
        // No real roots.
        if (A > 0.0)
        {
          // A > 0: parabola always > 0, no constraint from the j-th joint
          // The sclaing is infeasible for the j-th joint 
          // therefore we must consider the scaling infeasible for all the joints!
          return std::numeric_limits<double>::quiet_NaN(); // parabola always > 0, infeasible
        }
        else
        {
          // A < 0: parabola always <= 0, no constraint from the j-th joint
         constraint = {{-INF, INF}};
        }
      }
      else
      {
        // Two real roots!
        const double sqrt_disc = std::sqrt(discriminant);
        const double x1 = (-B - sqrt_disc) / (2.0 * A);
        const double x2 = (-B + sqrt_disc) / (2.0 * A);
        const double r_lo = std::min(x1, x2);
        const double r_hi = std::max(x1, x2);

        if (A > 0.0)
        {
          // Feasible region: [r_lo, r_hi] — intersect with running bounds
          constraint = {{r_lo, r_hi}};
        }
        else
        {
          // A < 0: feasible region is (-inf, r_lo] ∪ [r_hi, +inf)
          constraint = {{-INF, r_lo}, {r_hi, INF}};
        }
      }
    }

    // Now we can intersect the range we found for the solution of the j-th joint
    // with the interval that sotr the information for all the joints
    feasible = intersect(feasible,constraint);

    // In the case the feasible interval is empty, it does mean that there is not
    // any ddtau_i that allows the scaling for all the joints. It does mean that
    // one joint should "accelerate", while another should "decelerate" 
    if(feasible.empty())
    {
      return std::numeric_limits<double>::quiet_NaN();
    }
  }

  // Final selection. We choice the maximum value among the ones in the feasible intervals
  double best = -INF;


  // The case hi is +inf and lo is -inf can happen only when a_k, v_k are both zero.
  // This is, however, a meaningless case! To check it, we will introduce a further
  // check after thi for loop
  for(const auto& interval : feasible)
  {
    double candidate = std::isfinite(interval.hi) ? interval.hi : interval.lo;
    best = std::max(best, candidate);
  }

  if(!std::isfinite(best))
  {
    // No meaningful bound!
    best = std::numeric_limits<double>::quiet_NaN();
  }

  return best;
  
}

/**
 * @brief Computes the next trajectory interval and an adjusted scaling factor to respect
 *        joint velocity and acceleration limits.
 *
 * This function implements an online time-scaling algorithm for trajectory execution.
 * Given a desired scaling factor (dtau_i), it iteratively reduces it until the scaled
 * velocities and accelerations at the next sample point fall within the provided limits.
 *
 * The "scaling factor" (dtau_i) represents how fast to traverse the trajectory in
 * path-parameter space relative to wall-clock time. A value of 1.0 means nominal speed;
 * values < 1.0 slow down execution.
 *
 * @param trajectory_msg       Shared pointer to the JointTrajectory message to be executed.
 * @param sample_time_from_start  Current time offset into the trajectory (tau_{i-1}).
 * @param period               Controller update period (wall-clock delta-t).
 * @param scaling_factor       Desired scaling factor for this step (dtau_i candidate).
 * @param prev_scaling_factor  Scaling factor applied in the previous step (dtau_{i-1}).
 * @param max_velocities       Map of joint name vs maximum allowed velocity magnitude.
 * @param max_accelerations    Map of joint name vs maximum allowed acceleration magnitude.
 * @param min_allowed_scaling_factor  Lower bound on dtau_i before aborting reduction
 *                             and clamping to the previous step's value (default: 1e-4).
 *
 * @return A tuple containing:
 *   - tau_i        (rclcpp::Duration)  : Next sample time from trajectory start.
 *   - dtau_i       (double)            : Accepted scaling factor for this step.
 *   - ddtau_i      (double)            : Rate of change of scaling factor (acceleration in path-parameter).
 *   - k_1_itr      (TrajectoryPointConstIter) : Iterator to the segment start point.
 *   - k_itr        (TrajectoryPointConstIter) : Iterator to the segment end point.
 *
 * @throws std::runtime_error  If a trajectory point has a velocity or acceleration vector
 *                             whose size does not match the number of joints.
 */
inline
std::tuple<rclcpp::Duration,double, double, TrajectoryPointConstIter, TrajectoryPointConstIter> compute_interval_and_scaling(
  const std::shared_ptr<trajectory_msgs::msg::JointTrajectory>& trajectory_msg,
  const rclcpp::Duration & sample_time_from_start, 
  const rclcpp::Duration & period, 
  const double& scaling_factor, 
  const double& prev_scaling_factor,
  const std::map<std::string, double>& max_velocities,
  const std::map<std::string, double>& max_accelerations)
{
  // Zero vector used as a fallback when a trajectory point omits velocities or accelerations.
  std::vector<double> zeros(trajectory_msg->joint_names.size(), 0.0);

  // dtau_i  : current (candidate) scaling factor — may be reduced by the loop below.
  // dtau_i_1: previous step's accepted scaling factor (read-only reference).
  double dtau_i = scaling_factor;
  const double& dtau_i_1 = prev_scaling_factor;

  // Rate of change of the scaling factor (path-parameter "jerk" analog).
  double ddtau_i = (dtau_i-dtau_i_1)/period.seconds();

  // tau_i_1: current time in trajectory space (start of this control step).
  // tau_i  : projected next time in trajectory space after applying dtau_i.
  const rclcpp::Duration& tau_i_1 = sample_time_from_start;
  rclcpp::Duration tau_i = sample_time_from_start + period * dtau_i;
  
  // Iterators that will point to the bounding trajectory segment for tau_i.
  TrajectoryPointConstIter k_1_itr = trajectory_msg->points.end();
  TrajectoryPointConstIter k_itr = trajectory_msg->points.end();

  // -----------------------------------------------------------------
  // Build local copies of limit maps, inserting "no limit" sentinels
  // for any joint that is missing from the caller-supplied maps.
  // -----------------------------------------------------------------
  std::map<std::string, double> _max_velocities;
  std::map<std::string, double> _max_accelerations;
  for(const auto& jname : trajectory_msg->joint_names)
  {
    if(max_velocities.find(jname) == max_velocities.end())
    {
      RCLCPP_WARN(rclcpp::get_logger("joint_trajectory_controller"), "Joint '%s' not found in max velocity limits, assuming no limit.", jname.c_str());
      _max_velocities[jname] = std::numeric_limits<double>::max();
    }
    else
    {
      _max_velocities[jname] = std::fabs(max_velocities.at(jname)) > 0.0 ? std::fabs(max_velocities.at(jname)) : std::numeric_limits<double>::max();
    }

    if(max_accelerations.find(jname) == max_accelerations.end())
    {
      RCLCPP_WARN(rclcpp::get_logger("joint_trajectory_controller"), "Joint '%s' not found in max acceleration limits, assuming no limit.", jname.c_str());
      _max_accelerations[jname] = std::numeric_limits<double>::max();
    }
    else
    {
      _max_accelerations[jname] = std::fabs(max_accelerations.at(jname)) > 0.0 ? std::fabs(max_accelerations.at(jname)) : std::numeric_limits<double>::max();
    }
  }

  // -----------------------------------------------------------------
  // Iterative scaling-factor reduction loop.
  //
  // Each iteration:
  //   1. Locate the trajectory segment that contains the projected tau_i.
  //   2. Compute scaled velocity  : v_scaled = dtau_i * v_k
  //   3. Compute scaled accel     : a_scaled  = dtau_i^2 * a_k + ddtau_i * v_k
  //   4. If both are within limits then accept dtau_i and break.
  //   5. If velocity is OK but acceleration is violated we try to recompute dtau_i.
  //      If dtau_i falls below the minimum threshold, clamp to the previous value and
  //      break (trajectory likely infeasible).
  //   6. If velocity itself is violated then override dtau_i with the tightest
  //      per-joint velocity-ratio bound and retry.
  // -----------------------------------------------------------------
  constexpr int kMaxIter = 10;
  int iter = 0;
  do
  {
    if(++iter > kMaxIter)
    {
      RCLCPP_ERROR(rclcpp::get_logger("joint_trajectory_controller"),  "compute_interval_and_scaling did not converge in %d iterations.", kMaxIter);
      dtau_i = dtau_i_1;
      ddtau_i = 0.0;
      break;
    }
    // update the tau_i corresponding to the current candidate scaling factor
    tau_i = sample_time_from_start + period * dtau_i;

    // Find the two bounding waypoints (k-1, k) that bracket tau_i.
    std::tie(k_1_itr, k_itr) = trajectory_utils::find_segment(trajectory_msg, tau_i);
    
    // If tau_i is beyond the last trajectory point, nothing to scale — exit.
    if(k_itr == trajectory_msg->points.end())
    {
      break;
    }
    
    // Use zero vectors if the trajectory point omits velocity / acceleration data.
    const std::vector<double> & v_k = k_itr->velocities.empty() ? zeros : k_itr->velocities;
    const std::vector<double> & a_k = k_itr->accelerations.empty() ? zeros : k_itr->accelerations;
    
    // Validate sizes to avoid silent undefined behaviour from mismatched vectors.
    if(v_k.size()!=zeros.size() || a_k.size()!=zeros.size())
    {
      std::string msg = "A Trajectory Point has invalid velocity or acceleration size. "
       "Expected "+std::to_string(zeros.size())+", got "+std::to_string(v_k.size()) +" and " +std::to_string(a_k.size()) + "respectively. "
       "Aborting scaling factor computation to prevent undefined behavior. Please fix the trajectory message.";
       throw std::runtime_error(msg);
    }

    // Scaled velocity: v_k_scaled = dtau_i * v_k
    std::vector<double> _v_k = trajectory_utils::multiply(dtau_i, v_k);

    // Recompute ddtau_i in case dtau_i was updated in a previous iteration.
    ddtau_i = (dtau_i-dtau_i_1)/period.seconds();
    
    // --- Check velocity constraint ---
    // leqt returns 1 when all |v_k_scaled[j]| <= v_max[j].
    if(1 == trajectory_utils::leqt(
      _max_velocities, 
      trajectory_msg->joint_names, 
      _v_k,
      true))
    {
      // Velocity is within limits; now check acceleration.
      // Scaled accel: a_k_scaled = dtau_i^2 * a_k  +  ddtau_i * v_k
      auto a_k_first_term = trajectory_utils::multiply(dtau_i*dtau_i, a_k);
      auto a_k_second_term = trajectory_utils::multiply(ddtau_i, v_k);
      auto _a_k = add(a_k_first_term, a_k_second_term);
      
      // --- Check acceleration constraint ---
      if(1 == trajectory_utils::leqt(_max_accelerations, trajectory_msg->joint_names, _a_k,true))
      {
        // Both velocity and acceleration are within limits → accept and exit.
        break;
      }
      else
      {
        // Acceleration constraint violated.
        // NOTE: This algorithm is not exact, since it assumes that v_k and a_k do not change if we change dtau_i, that is not true!!!
        const double ddtau_opt = solve_max_ddtau(v_k, a_k, trajectory_msg->joint_names,
                                                  _max_accelerations, dtau_i_1);

        if (std::isnan(ddtau_opt) || dtau_i_1 + ddtau_opt < 0.0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("joint_trajectory_controller"), "Trajectory segment is dynamically infeasible.");
            ddtau_i = 0.0;
            dtau_i  = dtau_i_1;
            break;
        }

        ddtau_i = ddtau_opt;
        dtau_i = std::clamp(dtau_i_1 + ddtau_i, 0.0, scaling_factor);
        // NOTE: 
        // If dtau_i_1 + ddtau_opt is legitimately negative (the robot must decelerate to zero), 
        // clamping to 1e-4 overrides the analytic solution and re-enters the loop with a value 
        // that will fail the acceleration check again — potentially burning all kMaxIter 
        // iterations. 
        // The floor should be 0.0, with a separate check for near-zero
        if (dtau_i < 1e-4) 
        {
          dtau_i = 0.0;  // commit to full stop rather than oscillating near zero
          ddtau_i = -dtau_i_1 / period.seconds();
          break;
        }
      }
    }
    else
    {
      // Velocity constraint violated even before checking acceleration.
      // Override dtau_i with the tightest per-joint velocity-ratio:
      //   dtau_i = min_j( v_max[j] / |v_k[j]| )  * 0.99  (safety margin)
      std::stringstream ss; 
      ss << "dtau_i" << dtau_i << " ";
      ss << "v_k=["; for(const auto & v : v_k) { ss<<v<<","; } ss << "] ";
      ss << "_v_k=["; for(const auto & v : _v_k) { ss<<v<<","; } ss << "] ";
      ss << "v_max=["; for(const auto & v : max_velocities) { ss<<v.second<<","; } ss << "] ";
      RCLCPP_WARN(rclcpp::get_logger("joint_trajectory_controller"), 
          "\u001B[32m"
          "Override Scaling factor to preserve max velocity! %s ", ss.str().c_str());

      // Since the leqt check above already verified that at least one joint violates the velocity constraint, 
      // we are guaranteed to reduce dtau_i by at least some positive amount in this step, 
      // so we do not need a safety floor check here (unlike in the acceleration reduction branch).
      for(std::size_t i = 0; i < trajectory_msg->joint_names.size(); i++)
      {
        const auto& jname = trajectory_msg->joint_names[i];
        if(std::fabs(v_k[i]) > 0)
        {
          dtau_i = std::min(dtau_i, _max_velocities.at(jname) / std::fabs(v_k[i]));
        }
      }

      // Apply a 1 % safety margin to stay strictly inside the velocity limit.
      dtau_i = 0.99 * dtau_i;
      ddtau_i = (dtau_i - dtau_i_1) / period.seconds();
    }

  } while (true);
  
  // Despite tau_i was compute at the begin of the loop
  // I must recompute tau_i accordingly to the new value of dtau_i
  tau_i  = tau_i_1 + period * dtau_i;
  return {tau_i, dtau_i, ddtau_i, k_1_itr, k_itr};
}

}  // namespace trajectory_utils
}  // namespace joint_trajectory_controller

#endif  // JOINT_TRAJECTORY_CONTROLLER__TRAJECTORY_UTILS_HPP_