#include <plan_manage/competition_mission_common.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>

namespace ego_planner
{
namespace
{

bool getXmlValue(ros::NodeHandle &nh, const std::string &key, XmlRpc::XmlRpcValue &value, std::string &error)
{
  if (!nh.getParam(key, value))
  {
    error = "missing param: " + key;
    return false;
  }
  return true;
}

bool xmlValueToDouble(const XmlRpc::XmlRpcValue &value, double &out)
{
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    out = static_cast<int>(value);
    return true;
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
  {
    out = static_cast<double>(value);
    return true;
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
  {
    out = static_cast<bool>(value) ? 1.0 : 0.0;
    return true;
  }
  return false;
}

bool xmlValueToInt(const XmlRpc::XmlRpcValue &value, int &out)
{
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    out = static_cast<int>(value);
    return true;
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
  {
    out = static_cast<int>(static_cast<double>(value));
    return true;
  }
  return false;
}

std::string toLowerCopy(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

bool parseVector3(const XmlRpc::XmlRpcValue &value, Eigen::Vector3d &vec, std::string &error)
{
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() != 3)
  {
    error = "expected 3-element array";
    return false;
  }

  for (int i = 0; i < 3; ++i)
  {
    double scalar = 0.0;
    if (!xmlValueToDouble(value[i], scalar))
    {
      error = "vector element is not numeric";
      return false;
    }
    vec(i) = scalar;
  }
  return true;
}

bool parseStringList(const XmlRpc::XmlRpcValue &value, std::vector<std::string> &values, std::string &error)
{
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    error = "expected array";
    return false;
  }

  values.clear();
  values.reserve(value.size());
  for (int i = 0; i < value.size(); ++i)
  {
    if (value[i].getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      error = "array element is not string";
      return false;
    }
    values.push_back(static_cast<std::string>(value[i]));
  }
  return true;
}

MissionStageType parseStageType(const XmlRpc::XmlRpcValue &value, std::string &error)
{
  if (value.getType() != XmlRpc::XmlRpcValue::TypeString)
  {
    error = "stage type must be a string";
    return MissionStageType::WAYPOINT;
  }

  const std::string type = toLowerCopy(static_cast<std::string>(value));
  if (type == "waypoint")
  {
    return MissionStageType::WAYPOINT;
  }
  if (type == "block3")
  {
    return MissionStageType::BLOCK3;
  }
  if (type == "block4")
  {
    return MissionStageType::BLOCK4;
  }
  if (type == "hover_land" || type == "hoverland" || type == "land")
  {
    return MissionStageType::HOVER_LAND;
  }

  error = "unknown stage type: " + type;
  return MissionStageType::WAYPOINT;
}

bool parseMissionStage(const XmlRpc::XmlRpcValue &value, MissionStage &stage, std::string &error)
{
  if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    error = "mission stage must be a map";
    return false;
  }

  if (!value.hasMember("type"))
  {
    error = "mission stage missing type";
    return false;
  }

  stage.type = parseStageType(value["type"], error);
  if (!error.empty())
  {
    return false;
  }

  stage.gate_index = 0;
  if (value.hasMember("gate_index"))
  {
    if (!xmlValueToInt(value["gate_index"], stage.gate_index))
    {
      error = "gate_index must be numeric";
      return false;
    }
  }

  if (stage.type == MissionStageType::WAYPOINT || stage.type == MissionStageType::HOVER_LAND)
  {
    if (!value.hasMember("point"))
    {
      error = "mission stage missing point";
      return false;
    }
    if (!parseVector3(value["point"], stage.point, error))
    {
      return false;
    }
  }

  return true;
}

bool parseGate(const XmlRpc::XmlRpcValue &value, GateConfig &gate, std::string &error)
{
  if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    error = "gate entry must be a map";
    return false;
  }

  if (!value.hasMember("center") || !value.hasMember("normal"))
  {
    error = "gate entry missing center or normal";
    return false;
  }

  if (!parseVector3(value["center"], gate.center, error) || !parseVector3(value["normal"], gate.normal, error))
  {
    return false;
  }

  gate.normal = normalizedOr(gate.normal, Eigen::Vector3d::UnitX());

  if (value.hasMember("width") && !xmlValueToDouble(value["width"], gate.width))
  {
    error = "gate width must be numeric";
    return false;
  }
  if (value.hasMember("height") && !xmlValueToDouble(value["height"], gate.height))
  {
    error = "gate height must be numeric";
    return false;
  }
  if (value.hasMember("pre_distance") && !xmlValueToDouble(value["pre_distance"], gate.pre_distance))
  {
    error = "gate pre_distance must be numeric";
    return false;
  }
  if (value.hasMember("post_distance") && !xmlValueToDouble(value["post_distance"], gate.post_distance))
  {
    error = "gate post_distance must be numeric";
    return false;
  }
  return true;
}

bool loadVector3WithDefault(ros::NodeHandle &nh, const std::string &key, Eigen::Vector3d &value, std::string &error, const Eigen::Vector3d &default_value)
{
  XmlRpc::XmlRpcValue xml;
  if (!nh.getParam(key, xml))
  {
    value = default_value;
    return true;
  }
  return parseVector3(xml, value, error);
}

} // namespace

bool readVector3Param(ros::NodeHandle &nh, const std::string &key, Eigen::Vector3d &value, std::string &error)
{
  return loadVector3WithDefault(nh, key, value, error, value);
}

bool readStringListParam(ros::NodeHandle &nh, const std::string &key, std::vector<std::string> &values, std::string &error)
{
  XmlRpc::XmlRpcValue xml;
  if (!getXmlValue(nh, key, xml, error))
  {
    return false;
  }
  return parseStringList(xml, values, error);
}

bool readIntParam(ros::NodeHandle &nh, const std::string &key, int &value, const int default_value)
{
  nh.param(key, value, default_value);
  return true;
}

bool readDoubleParam(ros::NodeHandle &nh, const std::string &key, double &value, const double default_value)
{
  nh.param(key, value, default_value);
  return true;
}

bool readBoolParam(ros::NodeHandle &nh, const std::string &key, bool &value, const bool default_value)
{
  nh.param(key, value, default_value);
  return true;
}

bool loadGateConfigArray(ros::NodeHandle &nh, const std::string &key, std::vector<GateConfig> &gates, std::string &error)
{
  XmlRpc::XmlRpcValue xml;
  if (!nh.getParam(key, xml))
  {
    error = "missing param: " + key;
    return false;
  }
  if (xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    error = key + " must be an array";
    return false;
  }

  gates.clear();
  gates.reserve(xml.size());
  for (int i = 0; i < xml.size(); ++i)
  {
    GateConfig gate;
    if (!parseGate(xml[i], gate, error))
    {
      return false;
    }
    gates.push_back(gate);
  }
  return true;
}

bool loadMissionSequence(ros::NodeHandle &nh, std::vector<MissionStage> &sequence, std::string &error)
{
  sequence.clear();
  XmlRpc::XmlRpcValue xml;
  if (nh.getParam("competition/mission_sequence", xml))
  {
    if (xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      error = "competition/mission_sequence must be an array";
      return false;
    }

    for (int i = 0; i < xml.size(); ++i)
    {
      MissionStage stage;
      if (!parseMissionStage(xml[i], stage, error))
      {
        return false;
      }
      sequence.push_back(stage);
    }
  }

  if (sequence.empty())
  {
    XmlRpc::XmlRpcValue waypoints_xml;
    if (!nh.getParam("competition/global_waypoints", waypoints_xml))
    {
      error = "competition/mission_sequence or competition/global_waypoints must be provided";
      return false;
    }
    if (waypoints_xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      error = "competition/global_waypoints must be an array";
      return false;
    }

    for (int i = 0; i < waypoints_xml.size(); ++i)
    {
      MissionStage stage;
      stage.type = MissionStageType::WAYPOINT;
      if (!parseVector3(waypoints_xml[i], stage.point, error))
      {
        return false;
      }
      sequence.push_back(stage);
    }
  }

  return !sequence.empty();
}

bool loadBlock3Config(ros::NodeHandle &nh, Block3Config &config, std::string &error)
{
  readDoubleParam(nh, "competition/block3/corridor_width", config.corridor_width, config.corridor_width);
  readDoubleParam(nh, "competition/block3/dynamic_object_radius", config.dynamic_object_radius, config.dynamic_object_radius);
  readDoubleParam(nh, "competition/block3/prediction_horizon", config.prediction_horizon, config.prediction_horizon);
  readDoubleParam(nh, "competition/block3/safety_margin", config.safety_margin, config.safety_margin);
  readDoubleParam(nh, "competition/block3/min_gap_time", config.min_gap_time, config.min_gap_time);
  readDoubleParam(nh, "competition/block3/goal_tolerance", config.goal_tolerance, config.goal_tolerance);
  readDoubleParam(nh, "competition/block3/commit_tolerance", config.commit_tolerance, config.commit_tolerance);
  readDoubleParam(nh, "competition/block3/publish_rate", config.publish_rate, config.publish_rate);
  readIntParam(nh, "competition/block3/observe_hold_count", config.observe_hold_count, config.observe_hold_count);

  if (!loadVector3WithDefault(nh, "competition/block3/wait_point", config.wait_point, error, config.wait_point))
  {
    return false;
  }
  if (!loadVector3WithDefault(nh, "competition/block3/cross_entry", config.cross_entry, error, config.cross_entry))
  {
    return false;
  }
  if (!loadVector3WithDefault(nh, "competition/block3/cross_exit", config.cross_exit, error, config.cross_exit))
  {
    return false;
  }

  XmlRpc::XmlRpcValue xml;
  if (nh.getParam("competition/block3/dynamic_topics", xml))
  {
    if (!parseStringList(xml, config.dynamic_topics, error))
    {
      return false;
    }
  }
  if (config.dynamic_topics.empty())
  {
    config.dynamic_topics.push_back("/dynamic/pose_0");
  }

  return true;
}

bool loadBlock4Config(ros::NodeHandle &nh, Block4Config &config, std::string &error)
{
  readDoubleParam(nh, "competition/block4/goal_tolerance", config.goal_tolerance, config.goal_tolerance);
  readDoubleParam(nh, "competition/block4/align_position_tolerance", config.align_position_tolerance, config.align_position_tolerance);
  readDoubleParam(nh, "competition/block4/align_speed_tolerance", config.align_speed_tolerance, config.align_speed_tolerance);
  readDoubleParam(nh, "competition/block4/align_yaw_tolerance_deg", config.align_yaw_tolerance_deg, config.align_yaw_tolerance_deg);
  readDoubleParam(nh, "competition/block4/publish_rate", config.publish_rate, config.publish_rate);
  return loadGateConfigArray(nh, "competition/block4/gates", config.gates, error);
}

bool loadCompetitionMissionConfig(ros::NodeHandle &nh, CompetitionMissionConfig &config, std::string &error)
{
  readBoolParam(nh, "competition/auto_takeoff", config.auto_takeoff, config.auto_takeoff);
  readBoolParam(nh, "competition/auto_land", config.auto_land, config.auto_land);
  readDoubleParam(nh, "competition/takeoff_wait_timeout", config.takeoff_wait_timeout, config.takeoff_wait_timeout);
  readDoubleParam(nh, "competition/takeoff_ready_height", config.takeoff_ready_height, config.takeoff_ready_height);
  readDoubleParam(nh, "competition/land_handoff_delay", config.land_handoff_delay, config.land_handoff_delay);
  readDoubleParam(nh, "competition/waypoint_goal_tolerance", config.waypoint_goal_tolerance, config.waypoint_goal_tolerance);
  readDoubleParam(nh, "competition/waypoint_hold_time", config.waypoint_hold_time, config.waypoint_hold_time);
  readDoubleParam(nh, "competition/hover_land_hold_time", config.hover_land_hold_time, config.hover_land_hold_time);

  nh.param("competition/frame_id", config.frame_id, config.frame_id);
  nh.param("competition/odom_topic", config.odom_topic, config.odom_topic);
  nh.param("competition/goal_topic", config.goal_topic, config.goal_topic);
  nh.param("competition/takeoff_land_topic", config.takeoff_land_topic, config.takeoff_land_topic);
  nh.param("competition/block3_activate_topic", config.block3_activate_topic, config.block3_activate_topic);
  nh.param("competition/block3_status_topic", config.block3_status_topic, config.block3_status_topic);
  nh.param("competition/block4_start_topic", config.block4_start_topic, config.block4_start_topic);
  nh.param("competition/block4_status_topic", config.block4_status_topic, config.block4_status_topic);
  nh.param("competition/mission_status_topic", config.mission_status_topic, config.mission_status_topic);

  if (!loadMissionSequence(nh, config.mission_sequence, error))
  {
    return false;
  }
  if (!loadBlock3Config(nh, config.block3, error))
  {
    return false;
  }
  if (!loadBlock4Config(nh, config.block4, error))
  {
    return false;
  }

  return true;
}

geometry_msgs::PoseStamped makePoseStamped(const Eigen::Vector3d &point, const std::string &frame_id)
{
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = frame_id;
  pose.pose.position.x = point.x();
  pose.pose.position.y = point.y();
  pose.pose.position.z = point.z();
  pose.pose.orientation.w = 1.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  return pose;
}

double distancePointToSegment(const Eigen::Vector3d &point, const Eigen::Vector3d &segment_start, const Eigen::Vector3d &segment_end)
{
  const Eigen::Vector3d segment = segment_end - segment_start;
  const double length_sq = segment.squaredNorm();
  if (length_sq < 1e-9)
  {
    return (point - segment_start).norm();
  }

  const double t = std::max(0.0, std::min(1.0, (point - segment_start).dot(segment) / length_sq));
  const Eigen::Vector3d projection = segment_start + t * segment;
  return (point - projection).norm();
}

double distancePointToLine(const Eigen::Vector3d &point, const Eigen::Vector3d &line_point, const Eigen::Vector3d &line_dir)
{
  const Eigen::Vector3d dir = normalizedOr(line_dir, Eigen::Vector3d::UnitX());
  return ((point - line_point) - ((point - line_point).dot(dir)) * dir).norm();
}

Eigen::Vector3d normalizedOr(const Eigen::Vector3d &value, const Eigen::Vector3d &fallback)
{
  const double norm = value.norm();
  if (norm < 1e-9)
  {
    return fallback;
  }
  return value / norm;
}

std::string missionStageTypeToString(MissionStageType type)
{
  switch (type)
  {
  case MissionStageType::WAYPOINT:
    return "waypoint";
  case MissionStageType::BLOCK3:
    return "block3";
  case MissionStageType::BLOCK4:
    return "block4";
  case MissionStageType::HOVER_LAND:
    return "hover_land";
  }
  return "unknown";
}

} // namespace ego_planner
