#include <plan_manage/competition_mission_common.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <quadrotor_msgs/TakeoffLand.h>

namespace ego_planner
{
namespace
{

enum class MissionState
{
  WAIT_FOR_ODOM = 0,
  WAIT_FOR_TAKEOFF = 1,
  RUNNING = 2,
  WAIT_FOR_LAND_HANDOFF = 3,
  LANDING_REQUESTED = 4,
  EMERGENCY_HOLD = 5,
  COMPLETED = 6
};

} // namespace

class CompetitionMissionManager
{
public:
  explicit CompetitionMissionManager(ros::NodeHandle &nh) : nh_(nh) {}

  bool init()
  {
    std::string error;
    if (!loadCompetitionMissionConfig(nh_, config_, error))
    {
      ROS_ERROR_STREAM("[competition_mission_manager] " << error);
      return false;
    }

    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(config_.goal_topic, 1, true);
    takeoff_land_pub_ = nh_.advertise<quadrotor_msgs::TakeoffLand>(config_.takeoff_land_topic, 1);
    block3_activate_pub_ = nh_.advertise<std_msgs::Bool>(config_.block3_activate_topic, 1, true);
    block4_start_pub_ = nh_.advertise<std_msgs::Int32>(config_.block4_start_topic, 1, true);
    mission_status_pub_ = nh_.advertise<std_msgs::Int32>(config_.mission_status_topic, 1, true);

    odom_sub_ = nh_.subscribe(config_.odom_topic, 1, &CompetitionMissionManager::odomCallback, this, ros::TransportHints().tcpNoDelay());
    block3_status_sub_ = nh_.subscribe(config_.block3_status_topic, 1, &CompetitionMissionManager::block3StatusCallback, this, ros::TransportHints().tcpNoDelay());
    block4_status_sub_ = nh_.subscribe(config_.block4_status_topic, 1, &CompetitionMissionManager::block4StatusCallback, this, ros::TransportHints().tcpNoDelay());

    timer_ = nh_.createTimer(ros::Duration(0.1), &CompetitionMissionManager::timerCallback, this);
    publishMissionStatus(STATUS_IDLE);
    return true;
  }

private:
  void publishMissionStatus(int code)
  {
    std_msgs::Int32 msg;
    msg.data = code;
    mission_status_pub_.publish(msg);
  }

  void publishGoal(const Eigen::Vector3d &point)
  {
    last_goal_ = point;
    goal_pub_.publish(makePoseStamped(point, config_.frame_id));
  }

  void publishTakeoff()
  {
    quadrotor_msgs::TakeoffLand msg;
    msg.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::TAKEOFF;
    takeoff_land_pub_.publish(msg);
  }

  void publishLand()
  {
    quadrotor_msgs::TakeoffLand msg;
    msg.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::LAND;
    takeoff_land_pub_.publish(msg);
  }

  void publishBlock3Active(bool active)
  {
    std_msgs::Bool msg;
    msg.data = active;
    block3_activate_pub_.publish(msg);
  }

  void publishBlock4Start(int gate_index)
  {
    std_msgs::Int32 msg;
    msg.data = gate_index;
    block4_start_pub_.publish(msg);
  }

  bool nearPoint(const Eigen::Vector3d &point, double tolerance) const
  {
    return have_odom_ && (odom_pos_ - point).norm() <= tolerance;
  }

  void enterStage(size_t index)
  {
    current_stage_ = index;
    stage_entered_ = false;
    stage_ready_time_ = ros::Time(0);
    goal_reached_time_ = ros::Time(0);
  }

  void enterEmergencyHold()
  {
    state_ = MissionState::EMERGENCY_HOLD;
    publishGoal(odom_pos_);
    publishBlock3Active(false);
    publishBlock4Start(-1);
    publishMissionStatus(STATUS_EMERGENCY);
    ROS_ERROR("[competition_mission_manager] emergency hold");
  }

  void odomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    have_odom_ = true;
    odom_pos_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    odom_vel_ = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
  }

  void block3StatusCallback(const std_msgs::Int32ConstPtr &msg)
  {
    block3_status_ = msg->data;
  }

  void block4StatusCallback(const std_msgs::Int32ConstPtr &msg)
  {
    block4_status_ = msg->data;
  }

  void timerCallback(const ros::TimerEvent &)
  {
    if (!have_odom_)
    {
      return;
    }

    const ros::Time now = ros::Time::now();
    switch (state_)
    {
    case MissionState::WAIT_FOR_ODOM:
      publishMissionStatus(STATUS_IDLE);
      if (config_.auto_takeoff)
      {
        publishTakeoff();
        takeoff_start_time_ = now;
        state_ = MissionState::WAIT_FOR_TAKEOFF;
        publishMissionStatus(STATUS_RUNNING);
        ROS_INFO("[competition_mission_manager] takeoff requested");
      }
      else
      {
        state_ = MissionState::RUNNING;
        enterStage(0);
      }
      break;

    case MissionState::WAIT_FOR_TAKEOFF:
      if (odom_pos_.z() >= config_.takeoff_ready_height || (now - takeoff_start_time_).toSec() > config_.takeoff_wait_timeout)
      {
        state_ = MissionState::RUNNING;
        enterStage(0);
        ROS_INFO("[competition_mission_manager] takeoff complete, starting mission");
      }
      else
      {
        publishTakeoff();
      }
      break;

    case MissionState::RUNNING:
      if (current_stage_ >= config_.mission_sequence.size())
      {
        if (config_.auto_land)
        {
          state_ = MissionState::WAIT_FOR_LAND_HANDOFF;
          stage_ready_time_ = now;
          publishBlock3Active(false);
          publishBlock4Start(-1);
          ROS_INFO("[competition_mission_manager] mission finished, waiting to land");
        }
        else
        {
          state_ = MissionState::COMPLETED;
          publishMissionStatus(STATUS_DONE);
        }
        break;
      }

      handleCurrentStage(now);
      break;

    case MissionState::WAIT_FOR_LAND_HANDOFF:
      if ((now - stage_ready_time_).toSec() >= config_.land_handoff_delay)
      {
        publishLand();
        state_ = MissionState::LANDING_REQUESTED;
        ROS_INFO("[competition_mission_manager] landing requested");
      }
      break;

    case MissionState::LANDING_REQUESTED:
      if (odom_pos_.z() <= 0.20)
      {
        state_ = MissionState::COMPLETED;
        publishMissionStatus(STATUS_DONE);
        ROS_INFO("[competition_mission_manager] mission completed");
      }
      break;

    case MissionState::EMERGENCY_HOLD:
    case MissionState::COMPLETED:
      break;
    }
  }

  void handleCurrentStage(const ros::Time &now)
  {
    MissionStage &stage = config_.mission_sequence[current_stage_];
    if (!stage_entered_)
    {
      stage_entered_ = true;
      stage_ready_time_ = now;
      publishMissionStatus(STATUS_RUNNING);
      ROS_INFO_STREAM("[competition_mission_manager] stage " << current_stage_ << ": " << missionStageTypeToString(stage.type));

      switch (stage.type)
      {
      case MissionStageType::WAYPOINT:
      case MissionStageType::HOVER_LAND:
        publishGoal(stage.point);
        break;
      case MissionStageType::BLOCK3:
        block3_status_ = STATUS_RUNNING;
        publishBlock3Active(true);
        break;
      case MissionStageType::BLOCK4:
        block4_status_ = STATUS_RUNNING;
        publishBlock4Start(stage.gate_index);
        break;
      }
    }

    switch (stage.type)
    {
    case MissionStageType::WAYPOINT:
      if (!nearPoint(stage.point, config_.waypoint_goal_tolerance))
      {
        goal_reached_time_ = ros::Time(0);
        publishGoal(stage.point);
      }
      else
      {
        if (goal_reached_time_.isZero())
        {
          goal_reached_time_ = now;
        }
        else if ((now - goal_reached_time_).toSec() >= config_.waypoint_hold_time)
        {
          ++current_stage_;
          stage_entered_ = false;
        }
      }
      break;

    case MissionStageType::BLOCK3:
      if (block3_status_ == STATUS_DONE)
      {
        publishBlock3Active(false);
        ++current_stage_;
        stage_entered_ = false;
      }
      else if (block3_status_ == STATUS_EMERGENCY)
      {
        enterEmergencyHold();
      }
      break;

    case MissionStageType::BLOCK4:
      if (block4_status_ == STATUS_DONE)
      {
        publishBlock4Start(-1);
        ++current_stage_;
        stage_entered_ = false;
      }
      else if (block4_status_ == STATUS_EMERGENCY)
      {
        enterEmergencyHold();
      }
      break;

    case MissionStageType::HOVER_LAND:
      if (!nearPoint(stage.point, config_.waypoint_goal_tolerance))
      {
        goal_reached_time_ = ros::Time(0);
        publishGoal(stage.point);
      }
      else
      {
        if (goal_reached_time_.isZero())
        {
          goal_reached_time_ = now;
        }
        else if ((now - goal_reached_time_).toSec() >= config_.hover_land_hold_time)
        {
          ++current_stage_;
          stage_entered_ = false;
        }
      }
      break;
    }
  }

  ros::NodeHandle nh_;
  ros::Publisher goal_pub_;
  ros::Publisher takeoff_land_pub_;
  ros::Publisher block3_activate_pub_;
  ros::Publisher block4_start_pub_;
  ros::Publisher mission_status_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber block3_status_sub_;
  ros::Subscriber block4_status_sub_;
  ros::Timer timer_;

  CompetitionMissionConfig config_;
  MissionState state_{MissionState::WAIT_FOR_ODOM};
  bool have_odom_{false};
  bool stage_entered_{false};
  size_t current_stage_{0};
  int block3_status_{STATUS_IDLE};
  int block4_status_{STATUS_IDLE};
  ros::Time takeoff_start_time_;
  ros::Time stage_ready_time_;
  ros::Time goal_reached_time_;
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d odom_vel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_goal_{Eigen::Vector3d::Zero()};
};

} // namespace ego_planner

int main(int argc, char **argv)
{
  ros::init(argc, argv, "competition_mission_manager");
  ros::NodeHandle nh("~");

  ego_planner::CompetitionMissionManager node(nh);
  if (!node.init())
  {
    return 1;
  }

  ros::spin();
  return 0;
}
