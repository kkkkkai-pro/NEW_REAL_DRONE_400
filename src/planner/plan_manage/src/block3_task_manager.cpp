#include <plan_manage/competition_mission_common.h>

#include <algorithm>
#include <boost/bind.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <limits>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

namespace ego_planner
{
namespace
{

struct DynamicObstacle
{
  bool have_pose{false};
  geometry_msgs::PoseStamped pose;
  ros::Time stamp;
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_position{Eigen::Vector3d::Zero()};
  ros::Time last_update;
};

enum class Block3State
{
  IDLE = 0,
  APPROACH_WAIT_POINT = 1,
  OBSERVE_DYNAMIC_OBS = 2,
  EVAL_CROSSING_GAP = 3,
  COMMIT_CROSS = 4,
  EXIT_RECOVER = 5,
  EMERGENCY_HOLD = 6
};

} // namespace

class Block3TaskManager
{
public:
  explicit Block3TaskManager(ros::NodeHandle &nh) : nh_(nh) {}

  bool init()
  {
    std::string error;
    if (!loadBlock3Config(nh_, config_, error))
    {
      ROS_ERROR_STREAM("[block3_task_manager] " << error);
      return false;
    }

    nh_.param("competition/frame_id", frame_id_, std::string("world"));
    nh_.param("competition/goal_topic", goal_topic_, std::string("/competition/mission_goal"));
    nh_.param("competition/odom_topic", odom_topic_, std::string("/Odom_high_freq"));
    nh_.param("competition/block3_activate_topic", activate_topic_, std::string("/competition/block3/activate"));
    nh_.param("competition/block3_status_topic", status_topic_, std::string("/competition/block3/status"));

    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic_, 1, true);
    status_pub_ = nh_.advertise<std_msgs::Int32>(status_topic_, 1, true);

    odom_sub_ = nh_.subscribe(odom_topic_, 1, &Block3TaskManager::odomCallback, this, ros::TransportHints().tcpNoDelay());
    activate_sub_ = nh_.subscribe(activate_topic_, 1, &Block3TaskManager::activateCallback, this, ros::TransportHints().tcpNoDelay());

    for (size_t i = 0; i < config_.dynamic_topics.size(); ++i)
    {
      dynamic_subs_.push_back(nh_.subscribe<geometry_msgs::PoseStamped>(
          config_.dynamic_topics[i], 10, boost::bind(&Block3TaskManager::dynamicPoseCallback, this, _1, i), ros::TransportHints().tcpNoDelay()));
      obstacles_.push_back(DynamicObstacle{});
    }

    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, config_.publish_rate)), &Block3TaskManager::timerCallback, this);
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
    state_ = Block3State::IDLE;
    clear_count_ = 0;
    active_ = false;
    publishStatus(STATUS_IDLE);
  }

  bool isNear(const Eigen::Vector3d &target, double tol) const
  {
    return have_odom_ && (odom_pos_ - target).norm() <= tol;
  }

  Eigen::Vector3d obstaclePositionAt(const DynamicObstacle &obs, double dt) const
  {
    if (!obs.have_pose)
    {
      return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    }
    return obs.last_position + obs.velocity * dt;
  }

  bool corridorClear(const Eigen::Vector3d &segment_start, const Eigen::Vector3d &segment_end, double horizon) const
  {
    const double sample_dt = 0.2;
    const double radius = 0.5 * config_.corridor_width + config_.dynamic_object_radius;
    for (const auto &obs : obstacles_)
    {
      if (!obs.have_pose)
      {
        continue;
      }
      for (double t = 0.0; t <= horizon; t += sample_dt)
      {
        const Eigen::Vector3d predicted = obstaclePositionAt(obs, t);
        if (!predicted.allFinite())
        {
          continue;
        }
        if (distancePointToSegment(predicted, segment_start, segment_end) <= radius)
        {
          return false;
        }
      }
    }
    return true;
  }

  void dynamicPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg, size_t index)
  {
    if (index >= obstacles_.size())
    {
      return;
    }

    DynamicObstacle &obs = obstacles_[index];
    const ros::Time now = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    const Eigen::Vector3d position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    if (obs.have_pose)
    {
      const double dt = (now - obs.last_update).toSec();
      if (dt > 1e-3)
      {
        obs.velocity = (position - obs.last_position) / dt;
      }
    }
    obs.have_pose = true;
    obs.pose = *msg;
    obs.last_position = position;
    obs.last_update = now;
  }

  void odomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    have_odom_ = true;
    odom_pos_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    odom_vel_ = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
  }

  void activateCallback(const std_msgs::BoolConstPtr &msg)
  {
    if (msg->data)
    {
      active_ = true;
      state_ = Block3State::APPROACH_WAIT_POINT;
      clear_count_ = 0;
      publishStatus(STATUS_RUNNING);
      publishGoal(config_.wait_point);
      ROS_INFO("[block3_task_manager] activated");
    }
    else
    {
      reset();
    }
  }

  void timerCallback(const ros::TimerEvent &)
  {
    if (!active_ || !have_odom_)
    {
      return;
    }

    const double eval_horizon = std::max(config_.prediction_horizon, config_.min_gap_time) + config_.safety_margin;

    switch (state_)
    {
    case Block3State::APPROACH_WAIT_POINT:
      if (!isNear(config_.wait_point, config_.goal_tolerance))
      {
        publishGoal(config_.wait_point);
      }
      else
      {
        state_ = Block3State::OBSERVE_DYNAMIC_OBS;
        clear_count_ = 0;
        ROS_INFO("[block3_task_manager] wait point reached, observing gap");
      }
      break;

    case Block3State::OBSERVE_DYNAMIC_OBS:
      if (corridorClear(config_.cross_entry, config_.cross_exit, eval_horizon))
      {
        ++clear_count_;
      }
      else
      {
        clear_count_ = 0;
      }

      if (clear_count_ >= config_.observe_hold_count)
      {
        state_ = Block3State::EVAL_CROSSING_GAP;
        publishGoal(config_.cross_entry);
        ROS_INFO("[block3_task_manager] gap opened, moving to entry");
      }
      break;

    case Block3State::EVAL_CROSSING_GAP:
      if (!corridorClear(config_.cross_entry, config_.cross_exit, eval_horizon))
      {
        state_ = Block3State::OBSERVE_DYNAMIC_OBS;
        clear_count_ = 0;
        publishGoal(config_.wait_point);
        ROS_WARN("[block3_task_manager] gap closed, back to observe");
        break;
      }

      if (isNear(config_.cross_entry, config_.commit_tolerance))
      {
        state_ = Block3State::COMMIT_CROSS;
        publishGoal(config_.cross_exit);
        ROS_INFO("[block3_task_manager] committed to cross");
      }
      break;

    case Block3State::COMMIT_CROSS:
      if (!corridorClear(odom_pos_, config_.cross_exit, std::max(1.0, config_.min_gap_time)))
      {
        state_ = Block3State::EMERGENCY_HOLD;
        active_ = false;
        publishGoal(odom_pos_);
        publishStatus(STATUS_EMERGENCY);
        ROS_ERROR("[block3_task_manager] emergency hold");
        break;
      }

      if (isNear(config_.cross_exit, config_.goal_tolerance))
      {
        state_ = Block3State::EXIT_RECOVER;
        active_ = false;
        publishStatus(STATUS_DONE);
        ROS_INFO("[block3_task_manager] block3 done");
      }
      break;

    case Block3State::EXIT_RECOVER:
    case Block3State::EMERGENCY_HOLD:
    case Block3State::IDLE:
    default:
      break;
    }
  }

  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber activate_sub_;
  std::vector<ros::Subscriber> dynamic_subs_;
  ros::Publisher goal_pub_;
  ros::Publisher status_pub_;
  ros::Timer timer_;

  Block3Config config_;
  std::vector<DynamicObstacle> obstacles_;

  std::string frame_id_{"world"};
  std::string odom_topic_{"/Odom_high_freq"};
  std::string goal_topic_{"/competition/mission_goal"};
  std::string activate_topic_{"/competition/block3/activate"};
  std::string status_topic_{"/competition/block3/status"};

  bool have_odom_{false};
  bool active_{false};
  Block3State state_{Block3State::IDLE};
  int clear_count_{0};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d odom_vel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d current_goal_{Eigen::Vector3d::Zero()};
};

} // namespace ego_planner

int main(int argc, char **argv)
{
  ros::init(argc, argv, "block3_task_manager");
  ros::NodeHandle nh("~");

  ego_planner::Block3TaskManager node(nh);
  if (!node.init())
  {
    return 1;
  }

  ros::spin();
  return 0;
}
