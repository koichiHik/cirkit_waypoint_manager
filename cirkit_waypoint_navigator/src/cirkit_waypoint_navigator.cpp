/*-------------------------------------------------
参考プログラム
read_csv.cpp : https://gist.github.com/yoneken/5765597#file-read_csv-cpp

-------------------------------------------------- */

#include <actionlib/client/simple_action_client.h>
#include <cirkit_waypoint_navigator/TeleportAbsolute.h>
#include <geometry_msgs/Pose.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <laser_geometry/laser_geometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <boost/shared_array.hpp>
#include <boost/tokenizer.hpp>

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "ros_colored_msg.h"  // FIXME: this header depend ROS, but exclude ros header. Now must be readed after #include"ros/ros.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

typedef boost::tokenizer<boost::char_separator<char>> tokenizer;

namespace RobotBehaviors {
enum State {
  INIT_NAV,
  WAYPOINT_NAV,
  WAYPOINT_REACHED_GOAL,
  WAYPOINT_NAV_PLANNING_ABORTED,
};
}

enum WayPointType {
  NORMAL,
  WAIT_USER_INPUT,
};

class WayPoint {
 public:
  WayPoint();
  WayPoint(move_base_msgs::MoveBaseGoal goal, WayPointType waypoint_type, double reach_threshold)
      : goal_(goal),
        waypoint_type_(waypoint_type),
        reach_threshold_(reach_threshold),
        user_input_received_(false) {}
  ~WayPoint() {}  // FIXME: Don't declare destructor!!

  WayPointType GetWayPointType() { return waypoint_type_; }
  bool WaitUserInput() {
    bool tmp_user_input_received_ = user_input_received_;
    user_input_received_ = true;
    return waypoint_type_ == WayPointType::WAIT_USER_INPUT && !tmp_user_input_received_;
  }
  move_base_msgs::MoveBaseGoal goal_;
  WayPointType waypoint_type_;
  double reach_threshold_;
  bool user_input_received_;
};

class CirkitWaypointNavigator {
 public:
  CirkitWaypointNavigator();

  ~CirkitWaypointNavigator();

  void SendNewGoal(geometry_msgs::Pose pose);

  void SendNextWaypointMarker(const geometry_msgs::Pose waypoint, int target_object_mode);

  void CancelGoal();

  int ReadWaypointFile(std::string filename);

  std::shared_ptr<WayPoint> RetrieveNextWaypoint();

  bool IsFinalGoal();

  double ComputeDistanceToWaypoint(geometry_msgs::Pose a, geometry_msgs::Pose b);

  void SendNextGoal(const WayPoint& waypoint);

  double GetWaypointReachThreshold();

  geometry_msgs::Pose GetRobotCurrentPosition();

  geometry_msgs::Pose GetCurrentGoalPosition();

  void Run();

 private:
  MoveBaseClient ac_;
  RobotBehaviors::State robot_behavior_state_;
  ros::Rate rate_;
  std::vector<std::shared_ptr<WayPoint>> waypoints_buff_;
  ros::NodeHandle nh_;
  tf::TransformListener listener_;
  int target_waypoint_index_;           // 次に目指すウェイポイントのインデックス
  double dist_thres_to_target_object_;  // 探索対象にどれだけ近づいたらゴールとするか
  double reach_threshold_;  // 今セットされてるゴール（waypointもしくは探索対象）へのしきい値
  geometry_msgs::Pose now_goal_;  // 現在目指しているゴールの座標
  ros::Publisher cmd_vel_pub_;
  ros::Publisher next_waypoint_marker_pub_;
};

CirkitWaypointNavigator::CirkitWaypointNavigator() : ac_("move_base", true), rate_(10) {
  robot_behavior_state_ = RobotBehaviors::INIT_NAV;
  std::string filename;

  ros::NodeHandle n("~");

  // Read Parameter.
  n.param<std::string>("waypointsfile", filename,
                       ros::package::getPath("cirkit_waypoint_navigator") +
                           "/waypoints/garden_waypoints.csv");  // FIXME: Don't find!
  n.param("dist_thres_to_target_object", dist_thres_to_target_object_, 1.8);
  n.param("start_waypoint", target_waypoint_index_, 0);

  next_waypoint_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/next_waypoint", 1);

  // X. Reading Way Points.
  ROS_INFO("[Waypoints file name] : %s", filename.c_str());
  ROS_INFO("Reading Waypoints.");
  ReadWaypointFile(filename.c_str());
  ROS_INFO("Waiting for action server to start.");

  // X. Wait for Server.
  ac_.waitForServer();
}

CirkitWaypointNavigator::~CirkitWaypointNavigator() { this->CancelGoal(); }

void CirkitWaypointNavigator::SendNewGoal(const geometry_msgs::Pose pose) {
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.pose = pose;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  ac_.sendGoal(goal);
  now_goal_ = goal.target_pose.pose;
}

void CirkitWaypointNavigator::SendNextWaypointMarker(const geometry_msgs::Pose waypoint,
                                                     int target_object_mode) {
  visualization_msgs::Marker waypoint_marker;
  waypoint_marker.header.frame_id = "map";
  waypoint_marker.header.stamp = ros::Time();
  waypoint_marker.id = 0;
  waypoint_marker.type = visualization_msgs::Marker::ARROW;
  waypoint_marker.action = visualization_msgs::Marker::ADD;
  waypoint_marker.pose = waypoint;
  waypoint_marker.pose.position.z = 0.3;
  waypoint_marker.scale.x = 0.8;
  waypoint_marker.scale.y = 0.5;
  waypoint_marker.scale.z = 0.1;
  waypoint_marker.color.a = 0.7;
  waypoint_marker.color.r = 0.05 + 1.0 * (float)target_object_mode;
  waypoint_marker.color.g = 0.80;
  waypoint_marker.color.b = 0.05 + 1.0 * (float)target_object_mode;
  next_waypoint_marker_pub_.publish(waypoint_marker);
}

void CirkitWaypointNavigator::CancelGoal() {
  ROS_INFO("CancelGoal() is called !!");
  ac_.cancelGoal();
}

int CirkitWaypointNavigator::ReadWaypointFile(std::string filename) {
  const int rows_num = 9;  // x, y, z, Qx,Qy,Qz,Qw, waypoint_type, reach_threshold
  boost::char_separator<char> sep(",", "", boost::keep_empty_tokens);
  std::ifstream ifs(filename.c_str());
  std::string line;
  while (ifs.good()) {
    getline(ifs, line);
    if (line.empty()) break;
    tokenizer tokens(line, sep);
    std::vector<double> data;
    tokenizer::iterator it = tokens.begin();
    for (; it != tokens.end(); ++it) {
      std::stringstream ss;
      double d;
      ss << *it;
      ss >> d;
      data.push_back(d);
    }
    if (data.size() != rows_num) {
      ROS_ERROR("Row size is mismatch!!");
      return -1;
    } else {
      move_base_msgs::MoveBaseGoal waypoint;
      waypoint.target_pose.pose.position.x = data[0];
      waypoint.target_pose.pose.position.y = data[1];
      waypoint.target_pose.pose.position.z = data[2];
      waypoint.target_pose.pose.orientation.x = data[3];
      waypoint.target_pose.pose.orientation.y = data[4];
      waypoint.target_pose.pose.orientation.z = data[5];
      waypoint.target_pose.pose.orientation.w = data[6];
      waypoints_buff_.push_back(std::shared_ptr<WayPoint>(
          new WayPoint(waypoint, static_cast<WayPointType>((int)data[7]), data[8] / 2.0)));
    }
  }
  return 0;
}

std::shared_ptr<WayPoint> CirkitWaypointNavigator::RetrieveNextWaypoint() {
  ROS_INFO_STREAM("Next Waypoint : " << target_waypoint_index_);
  return waypoints_buff_[target_waypoint_index_++];
}

bool CirkitWaypointNavigator::IsFinalGoal() {
  if ((target_waypoint_index_) == ((int)waypoints_buff_.size())) {
    return true;
  } else {
    return false;
  }
}

double CirkitWaypointNavigator::ComputeDistanceToWaypoint(geometry_msgs::Pose a,
                                                          geometry_msgs::Pose b) {
  return sqrt(pow((a.position.x - b.position.x), 2.0) + pow((a.position.y - b.position.y), 2.0));
}

// 通常のwaypointの場合
void CirkitWaypointNavigator::SendNextGoal(const WayPoint& waypoint) {
  reach_threshold_ = waypoint.reach_threshold_;
  this->SendNextWaypointMarker(waypoint.goal_.target_pose.pose,
                               0);  // 現在目指しているwaypointを表示する
  this->SendNewGoal(waypoint.goal_.target_pose.pose);
}

double CirkitWaypointNavigator::GetWaypointReachThreshold() { return reach_threshold_; }

geometry_msgs::Pose CirkitWaypointNavigator::GetRobotCurrentPosition() {
  // tfを使ってロボットの現在位置を取得する
  tf::StampedTransform transform;
  geometry_msgs::Pose pose;
  try {
    listener_.lookupTransform("/map", "/base_link", ros::Time(0), transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
  }
  pose.position.x = transform.getOrigin().x();
  pose.position.y = transform.getOrigin().y();
  return pose;
}

geometry_msgs::Pose CirkitWaypointNavigator::GetCurrentGoalPosition() { return now_goal_; }

void CirkitWaypointNavigator::Run() {
  robot_behavior_state_ = RobotBehaviors::INIT_NAV;

  // X. Process running loop.
  while (ros::ok()) {
    bool is_set_next_as_target = false;
    std::shared_ptr<WayPoint> next_waypoint = this->RetrieveNextWaypoint();

    // X. Wait user input if necessary.
    if (next_waypoint->WaitUserInput()) {
      ROS_INFO("Wait for key input from user.....");
      getchar();
    }

    // X. Next Waypoint.
    {
      ROS_GREEN_STREAM("Next WayPoint is got");
      ROS_INFO("Go next_waypoint.");
      this->SendNextGoal(*next_waypoint);
      robot_behavior_state_ = RobotBehaviors::WAYPOINT_NAV;
    }

    // X. Time that this waypoint starts.
    ros::Time cur_waypnt_start_time = ros::Time::now();
    ros::Time verbose_start = ros::Time::now();
    double last_distance_to_goal = 0;
    double delta_distance_to_goal = 1.0;  // 0.1[m]より大きければよい

    // X. Loop till robot state changed.
    while (ros::ok()) {
      geometry_msgs::Pose robot_current_position = this->GetRobotCurrentPosition();
      geometry_msgs::Pose current_goal_position = this->GetCurrentGoalPosition();

      // X. Compute distance to reach targeted waypoint.
      double distance_to_goal =
          this->ComputeDistanceToWaypoint(robot_current_position, current_goal_position);

      // X. Judge if robot is stuck.
      delta_distance_to_goal = last_distance_to_goal - distance_to_goal;
      if (delta_distance_to_goal < 0.1) {
        // X. Judge based on spent time in current waypoint.
        ros::Duration spent_time_in_cur_waypnt = ros::Time::now() - cur_waypnt_start_time;
        if (spent_time_in_cur_waypnt.toSec() > 10.0) {
          if (robot_behavior_state_ == RobotBehaviors::WAYPOINT_NAV) {
            robot_behavior_state_ =
                RobotBehaviors::WAYPOINT_NAV_PLANNING_ABORTED;  // プランニング失敗とする
            break;
          } else {
            break;
          }
        } else {  // 30秒おきに進捗を報告する
          ros::Duration verbose_time = ros::Time::now() - verbose_start;
          if (verbose_time.toSec() > 30.0) {
            ROS_INFO_STREAM("Waiting Abort: passed 30s, Distance to goal: " << distance_to_goal);
            verbose_start = ros::Time::now();
          }
        }
      } else {
        // X. Robot is not stuck.
        last_distance_to_goal = distance_to_goal;
        cur_waypnt_start_time = ros::Time::now();
      }

      // X. Check if waypoint has to be updated.
      if (distance_to_goal < this->GetWaypointReachThreshold()) {
        ROS_INFO_STREAM("Distance: " << distance_to_goal);
        if (robot_behavior_state_ == RobotBehaviors::WAYPOINT_NAV) {
          robot_behavior_state_ = RobotBehaviors::WAYPOINT_REACHED_GOAL;
          break;
        } else {
          break;
        }
      }
      rate_.sleep();
      ros::spinOnce();
    }

    // X. Decide next action.
    switch (robot_behavior_state_) {
      case RobotBehaviors::WAYPOINT_REACHED_GOAL: {
        ROS_INFO("WAYPOINT_REACHED_GOAL");
        // If this is final waypoint, cancel goal and terminate.
        if (this->IsFinalGoal()) {
          this->CancelGoal();
          return;
        }
        break;
      }
      case RobotBehaviors::WAYPOINT_NAV_PLANNING_ABORTED: {
        ROS_INFO("!! WAYPOINT_NAV_PLANNING_ABORTED !!");
        // Cancel current goal.
        this->CancelGoal();
        target_waypoint_index_ -= 1;
        break;
      }
      default: {
        ROS_WARN_STREAM("!! UNKNOWN STATE !!");
        break;
      }
    }
    rate_.sleep();
    ros::spinOnce();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "cirkit_waypoint_navigator");
  CirkitWaypointNavigator cirkit_waypoint_navigator;
  cirkit_waypoint_navigator.Run();

  return 0;
}
