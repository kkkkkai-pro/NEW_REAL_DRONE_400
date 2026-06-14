#ifndef EGO_PLANNER_COMPETITION_MISSION_COMMON_H
#define EGO_PLANNER_COMPETITION_MISSION_COMMON_H

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <xmlrpcpp/XmlRpcValue.h>

namespace ego_planner
{

enum MissionStatusCode
{
  STATUS_IDLE = 0,
  STATUS_RUNNING = 1,
  STATUS_DONE = 2,
  STATUS_EMERGENCY = 3
};

enum class MissionStageType
{
  WAYPOINT,
  BLOCK3,
  BLOCK4,
  HOVER_LAND
};

struct MissionStage
{
  MissionStageType type{MissionStageType::WAYPOINT};
  Eigen::Vector3d point{Eigen::Vector3d::Zero()};
  int gate_index{0};
};

struct GateConfig
{
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  Eigen::Vector3d normal{Eigen::Vector3d::UnitX()};
  double width{0.5};
  double height{0.5};
  double pre_distance{1.0};
  double post_distance{1.0};
};

struct Block3Config
{
  std::vector<std::string> dynamic_topics;
  Eigen::Vector3d wait_point{Eigen::Vector3d::Zero()};
  Eigen::Vector3d cross_entry{Eigen::Vector3d::Zero()};
  Eigen::Vector3d cross_exit{Eigen::Vector3d::Zero()};
  double corridor_width{1.0};
  double dynamic_object_radius{0.35};
  double prediction_horizon{3.0};
  double safety_margin{0.5};
  double min_gap_time{2.0};
  double goal_tolerance{0.35};
  double commit_tolerance{0.45};
  int observe_hold_count{5};
  double publish_rate{10.0};
};

struct Block4Config
{
  std::vector<GateConfig> gates;
  double goal_tolerance{0.35};
  double align_position_tolerance{0.20};
  double align_speed_tolerance{0.20};
  double align_yaw_tolerance_deg{30.0};
  double publish_rate{10.0};
};

struct CompetitionMissionConfig
{
  std::string frame_id{"world"};
  std::string odom_topic{"/Odom_high_freq"};
  std::string goal_topic{"/competition/mission_goal"};
  std::string takeoff_land_topic{"/px4ctrl/takeoff_land"};
  std::string block3_activate_topic{"/competition/block3/activate"};
  std::string block3_status_topic{"/competition/block3/status"};
  std::string block4_start_topic{"/competition/block4/start"};
  std::string block4_status_topic{"/competition/block4/status"};
  std::string mission_status_topic{"/competition/mission/status"};

  bool auto_takeoff{true};
  bool auto_land{true};
  double takeoff_wait_timeout{8.0};
  double takeoff_ready_height{0.35};
  double land_handoff_delay{1.0};
  double waypoint_goal_tolerance{0.35};
  double waypoint_hold_time{0.5};
  double hover_land_hold_time{0.8};

  std::vector<MissionStage> mission_sequence;
  Block3Config block3;
  Block4Config block4;
};

bool loadCompetitionMissionConfig(ros::NodeHandle &nh, CompetitionMissionConfig &config, std::string &error);
bool loadBlock3Config(ros::NodeHandle &nh, Block3Config &config, std::string &error);
bool loadBlock4Config(ros::NodeHandle &nh, Block4Config &config, std::string &error);
bool loadGateConfigArray(ros::NodeHandle &nh, const std::string &key, std::vector<GateConfig> &gates, std::string &error);
bool loadMissionSequence(ros::NodeHandle &nh, std::vector<MissionStage> &sequence, std::string &error);

bool readVector3Param(ros::NodeHandle &nh, const std::string &key, Eigen::Vector3d &value, std::string &error);
bool readStringListParam(ros::NodeHandle &nh, const std::string &key, std::vector<std::string> &values, std::string &error);
bool readIntParam(ros::NodeHandle &nh, const std::string &key, int &value, const int default_value);
bool readDoubleParam(ros::NodeHandle &nh, const std::string &key, double &value, const double default_value);
bool readBoolParam(ros::NodeHandle &nh, const std::string &key, bool &value, const bool default_value);

geometry_msgs::PoseStamped makePoseStamped(const Eigen::Vector3d &point, const std::string &frame_id);
double distancePointToSegment(const Eigen::Vector3d &point, const Eigen::Vector3d &segment_start, const Eigen::Vector3d &segment_end);
double distancePointToLine(const Eigen::Vector3d &point, const Eigen::Vector3d &line_point, const Eigen::Vector3d &line_dir);
Eigen::Vector3d normalizedOr(const Eigen::Vector3d &value, const Eigen::Vector3d &fallback);

std::string missionStageTypeToString(MissionStageType type);

} // namespace ego_planner

#endif
