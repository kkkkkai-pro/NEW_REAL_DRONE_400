#include <plan_manage/competition_mission_common.h>

#include <algorithm>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>

namespace ego_planner
{
namespace
{

enum class Block4State
{
  IDLE = 0,
  APPROACH_PRE_GATE = 1,
  ALIGN_TO_GATE = 2,
  COMMIT_THROUGH_GATE = 3,
  POST_GATE_RECOVER = 4,
  EMERGENCY_HOLD = 5
};

} // namespace

class Block4GateManager
{
public:
  explicit Block4GateManager(ros::NodeHandle &nh) : nh_(nh) {}

  bool init()
  {
    std::string error;
    if (!loadBlock4Config(nh_, config_, error))
    {
      ROS_ERROR_STREAM("[block4_gate_manager] " << error);
      return false;
    }

    nh_.param("competition/frame_id", frame_id_, std::string("world"));
    nh_.param("competition/goal_topic", goal_topic_, std::string("/competition/mission_goal"));
    nh_.param("competition/odom_topic", odom_topic_, std::string("/Odom_high_freq"));
    nh_.param("competition/block4_start_topic", start_topic_, std::string("/competition/block4/start"));
    nh_.param("competition/block4_status_topic", status_topic_, std::string("/competition/block4/status"));

    if (config_.gates.empty())
    {
      ROS_ERROR("[block4_gate_manager] no gates configured");
      return false;
    }

    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic_, 1, true);
    status_pub_ = nh_.advertise<std_msgs::Int32>(status_topic_, 1, true);

    odom_sub_ = nh_.subscribe(odom_topic_, 1, &Block4GateManager::odomCallback, this, ros::TransportHints().tcpNoDelay());
    start_sub_ = nh_.subscribe(start_topic_, 1, &Block4GateManager::startCallback, this, ros::TransportHints().tcpNoDelay());

    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, config_.publish_rate)), &Block4GateManager::timerCallback, this);
    publishStatus(STATUS_IDLE);
    return true;
  }

private:
  void publishStatus(int code)
  {
    std_msgs::Int32 msg;
    msg.data = code;
    status_pub_.publish(msg);
  }

  void publishGoal(const Eigen::Vector3d &point)
  {
    current_goal_ = point;
    goal_pub_.publish(makePoseStamped(point, frame_id_));
  }

  void reset()
  {
    active_ = false;
    state_ = Block4State::IDLE;
    selected_gate_ = -1;
    publishStatus(STATUS_IDLE);
  }

  bool isNear(const Eigen::Vector3d &target, double tol) const
  {
    return have_odom_ && (odom_pos_ - target).norm() <= tol;
  }

  double headingErrorDeg(const Eigen::Vector3d &normal) const
  {
    const Eigen::Matrix3d rot = odom_orient_.normalized().toRotationMatrix();
    const Eigen::Vector3d forward = normalizedOr(rot.col(0), Eigen::Vector3d::UnitX());
    const Eigen::Vector3d gate_normal = normalizedOr(normal, Eigen::Vector3d::UnitX());
    const double dot = std::max(-1.0, std::min(1.0, forward.dot(gate_normal)));
    return std::acos(dot) * 180.0 / 3.14159265358979323846;
  }

  void odomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    have_odom_ = true;
    odom_pos_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    odom_vel_ = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    odom_orient_ = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  }

  void startCallback(const std_msgs::Int32ConstPtr &msg)
  {
    if (msg->data < 0)
    {
      reset();
      return;
    }
    if (msg->data >= static_cast<int>(config_.gates.size()))
    {
      ROS_ERROR_STREAM("[block4_gate_manager] invalid gate index " << msg->data);
      return;
    }

    selected_gate_ = msg->data;
    state_ = Block4State::APPROACH_PRE_GATE;
    active_ = true;
    publishStatus(STATUS_RUNNING);

    gate_ = config_.gates[selected_gate_];
    pre_gate_ = gate_.center - normalizedOr(gate_.normal, Eigen::Vector3d::UnitX()) * gate_.pre_distance;
    post_gate_ = gate_.center + normalizedOr(gate_.normal, Eigen::Vector3d::UnitX()) * gate_.post_distance;
    publishGoal(pre_gate_);
    ROS_INFO_STREAM("[block4_gate_manager] gate " << selected_gate_ << " activated");
  }

  void timerCallback(const ros::TimerEvent &)
  {
    if (!active_ || !have_odom_ || selected_gate_ < 0)
    {
      return;
    }

    const double yaw_tol = config_.align_yaw_tolerance_deg;

    switch (state_)
    {
    case Block4State::APPROACH_PRE_GATE:
      if (!isNear(pre_gate_, config_.goal_tolerance))
      {
        publishGoal(pre_gate_);
      }
      else
      {
        state_ = Block4State::ALIGN_TO_GATE;
        ROS_INFO("[block4_gate_manager] pre-gate reached, aligning");
      }
      break;

    case Block4State::ALIGN_TO_GATE:
    {
      const double lateral = distancePointToLine(odom_pos_, gate_.center, gate_.normal);
      const double speed = odom_vel_.norm();
      const double heading_err = headingErrorDeg(gate_.normal);
      if (lateral <= config_.align_position_tolerance && speed <= config_.align_speed_tolerance && heading_err <= yaw_tol)
      {
        state_ = Block4State::COMMIT_THROUGH_GATE;
        publishGoal(gate_.center);
        ROS_INFO("[block4_gate_manager] aligned, committing through gate");
      }
      else if (!isNear(pre_gate_, config_.goal_tolerance * 1.5))
      {
        publishGoal(pre_gate_);
      }
      break;
    }

    case Block4State::COMMIT_THROUGH_GATE:
      if (!isNear(gate_.center, config_.goal_tolerance))
      {
        publishGoal(gate_.center);
      }
      else
      {
        state_ = Block4State::POST_GATE_RECOVER;
        publishGoal(post_gate_);
      }
      break;

    case Block4State::POST_GATE_RECOVER:
      if (!isNear(post_gate_, config_.goal_tolerance))
      {
        publishGoal(post_gate_);
      }
      else
      {
        active_ = false;
        state_ = Block4State::IDLE;
        publishStatus(STATUS_DONE);
        ROS_INFO("[block4_gate_manager] gate completed");
      }
      break;

    case Block4State::EMERGENCY_HOLD:
    case Block4State::IDLE:
    default:
      break;
    }
  }

  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber start_sub_;
  ros::Publisher goal_pub_;
  ros::Publisher status_pub_;
  ros::Timer timer_;

  Block4Config config_;
  std::string frame_id_{"world"};
  std::string odom_topic_{"/Odom_high_freq"};
  std::string goal_topic_{"/competition/mission_goal"};
  std::string start_topic_{"/competition/block4/start"};
  std::string status_topic_{"/competition/block4/status"};

  bool have_odom_{false};
  bool active_{false};
  int selected_gate_{-1};
  Block4State state_{Block4State::IDLE};
  GateConfig gate_;
  Eigen::Vector3d pre_gate_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d post_gate_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d current_goal_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d odom_vel_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond odom_orient_{Eigen::Quaterniond::Identity()};
};

} // namespace ego_planner

int main(int argc, char **argv)
{
  ros::init(argc, argv, "block4_gate_manager");
  ros::NodeHandle nh("~");

  ego_planner::Block4GateManager node(nh);
  if (!node.init())
  {
    return 1;
  }

  ros::spin();
  return 0;
}
