/*******************************************/
/*  Bas Janssen                            */
/*  Fontys Hogeschool Engineering          */
/*  2017                                   */
/*******************************************/

#include <ros/ros.h>
// Includes the descartes robot model we will be using
#include <descartes_moveit/moveit_state_adapter.h>
// Includes the descartes trajectory type we will be using
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
#include <descartes_trajectory/joint_trajectory_pt.h>
// Includes the planner we will be using
#include <descartes_planner/dense_planner.h>

#include <visualization_msgs/Marker.h>

//The path is build using points in the eigen convention
#include <eigen_conversions/eigen_msg.h>

#include <time.h>

#include "descartes_planning_service/generate_motion_plan.h"

typedef std::vector<descartes_core::TrajectoryPtPtr> TrajectoryVec;
typedef TrajectoryVec::const_iterator TrajectoryIter;

//There is a cleaner way, but this works...
std::vector<std::string> names;
ros::Publisher marker_publisher_;

void publishPosesMarkers(const EigenSTL::vector_Affine3d& poses, std::string WORLD_FRAME, float AXIS_LINE_WIDTH, float AXIS_LINE_LENGHT);

//Convert the poseArray to Eigen
EigenSTL::vector_Affine3d toEigenArray(const geometry_msgs::PoseArray &poses)
{
  EigenSTL::vector_Affine3d result (poses.poses.size());
  std::transform(poses.poses.begin(), poses.poses.end(), result.begin(), [](const geometry_msgs::Pose& pose) {
    Eigen::Affine3d e;
    tf::poseMsgToEigen(pose, e);
    return e;
  });
  return result;
}

//Should make axis and interpolation parameters.
descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d& pose, std::string underdefinedAxis, float interpolation)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;

  if(underdefinedAxis.compare("X_AXIS") == 0)
  {
    return TrajectoryPtPtr( new AxialSymmetricPt(pose, interpolation, AxialSymmetricPt::X_AXIS) );
  }
  else if(underdefinedAxis.compare("Y_AXIS") == 0)
  {
    return TrajectoryPtPtr( new AxialSymmetricPt(pose, interpolation, AxialSymmetricPt::Y_AXIS) );
  }
  else if(underdefinedAxis.compare("Z_AXIS") == 0)
  {
    return TrajectoryPtPtr( new AxialSymmetricPt(pose, interpolation, AxialSymmetricPt::Z_AXIS) );
  }
}

//Turn the Descartes trajectory into something ROS understands
trajectory_msgs::JointTrajectory
toROSJointTrajectory(const TrajectoryVec& trajectory,
                     const descartes_core::RobotModel& model,
                     const std::vector<std::string>& joint_names,
                     double time_delay)
{
  // Fill out information about our trajectory
  trajectory_msgs::JointTrajectory result;
  ros::Time stamp(0.0);
  result.header.stamp = stamp;//ros::Time::now();
  result.header.frame_id = "base_link";
  result.joint_names = joint_names;

  // For keeping track of time-so-far in the trajectory
  double time_offset = 0.0;
  // Loop through the trajectory
  for (TrajectoryIter it = trajectory.begin(); it != trajectory.end(); ++it)
  {
    // Find nominal joint solution at this point
    std::vector<double> joints;
    it->get()->getNominalJointPose(std::vector<double>(), model, joints);

    // Fill out a ROS trajectory point
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions = joints;
    // velocity, acceleration, and effort are given dummy values
    // we'll let the controller figure them out
    pt.velocities.resize(joints.size(), 0.0);
    pt.accelerations.resize(joints.size(), 0.0);
    pt.effort.resize(joints.size(), 0.0);
    // set the time into the trajectory
    pt.time_from_start = ros::Duration(time_offset);
    // increment time
    time_offset += time_delay;

    result.points.push_back(pt);
  }

  return result;
}

//Need to add options for world_frame, tcp_frame, execution time, return computation time and path lenght.
bool generate_motion_plan(descartes_planning_service::generate_motion_plan::Request &req, descartes_planning_service::generate_motion_plan::Response &res)
{
  EigenSTL::vector_Affine3d points = toEigenArray(req.path);
  publishPosesMarkers(points, req.world_frame, 0.01, 0.01);
  
  TrajectoryVec path;
  
  path.push_back(descartes_core::TrajectoryPtPtr( new descartes_trajectory::JointTrajectoryPt(req.joint_states.position) ));
  
  for(unsigned int i = 0; i < points.size(); i++)
  {
    const Eigen::Affine3d& pose = points[i];
    descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pose, req.underdefinedAxis, req.interpolation);
    path.push_back(pt);
  }
    
  //Create a robot model and initialize it
  descartes_core::RobotModelPtr model (new descartes_moveit::MoveitStateAdapter);
  
  model->setCheckCollisions(true);

  // Name of description on parameter server. Typically just "robot_description".
  const std::string robot_description = "robot_description";

  // name of the kinematic group you defined when running MoveitSetupAssistant
  const std::string group_name = "manipulator";

  // Name of frame in which you are expressing poses. Typically "world_frame" or "base_link".
  //const std::string world_frame = "/base_link";
  const std::string world_frame = req.world_frame;

  // tool center point frame (name of link associated with tool)
  //const std::string tcp_frame = "pencil_tip";
  //const std::string tcp_frame = "tool0";
  const std::string tcp_frame = req.tcp_frame;

  if (!model->initialize(robot_description, group_name, world_frame, tcp_frame))
  {
    ROS_INFO("Could not initialize robot model");
    return -1;
  }

  // 3. Create a planner and initialize it with our robot model
  descartes_planner::DensePlanner plannerDense;
  plannerDense.initialize(model);

  TrajectoryVec result;

  ROS_INFO("Starting path planning");

  clock_t t;
  t = clock();

  // Feed the trajectory to the planner
  if (!plannerDense.planPath(path))
  {
    ROS_ERROR("Could not solve for a valid path");
    return -2;
  }
  ROS_INFO("Finished path planning, searching for shortest path");
  if (!plannerDense.getPath(result))
  {
    ROS_ERROR("Could not retrieve path");
    return -3;
  }

  t = clock() -t;

  ROS_INFO("Path planning took %f seconds.", ((float) t) / CLOCKS_PER_SEC);

  // 5. Translate the result into a type that ROS understands
  // Generate a ROS joint trajectory with the result path, robot model, given joint names,
  // a certain time delta between each trajectory point
  trajectory_msgs::JointTrajectory joint_solution = toROSJointTrajectory(result, *model, req.names, req.dt);
  
  if(joint_solution.points.size() > 0)
  {
    res.plan = joint_solution;
    res.computeTime = ((float) t) / CLOCKS_PER_SEC;
    res.pointCount = joint_solution.points.size();
    return true;
  }
  else
  {
    return false;
  }
}

//Publish the path visualization to rviz
void publishPosesMarkers(const EigenSTL::vector_Affine3d& poses, std::string WORLD_FRAME, float AXIS_LINE_WIDTH, float AXIS_LINE_LENGHT)
{
  // creating rviz markers
  visualization_msgs::Marker z_axes, y_axes, x_axes, line;
  visualization_msgs::MarkerArray markers_msg;

  z_axes.type = y_axes.type = x_axes.type = visualization_msgs::Marker::LINE_LIST;
  z_axes.ns = y_axes.ns = x_axes.ns = "axes";
  z_axes.action = y_axes.action = x_axes.action = visualization_msgs::Marker::ADD;
  z_axes.lifetime = y_axes.lifetime = x_axes.lifetime = ros::Duration(0);
  z_axes.header.frame_id = y_axes.header.frame_id = x_axes.header.frame_id = WORLD_FRAME;
  z_axes.scale.x = y_axes.scale.x = x_axes.scale.x = AXIS_LINE_WIDTH;

  // z properties
  z_axes.id = 0;
  z_axes.color.r = 0;
  z_axes.color.g = 0;
  z_axes.color.b = 1;
  z_axes.color.a = 1;

  // y properties
  y_axes.id = 1;
  y_axes.color.r = 0;
  y_axes.color.g = 1;
  y_axes.color.b = 0;
  y_axes.color.a = 1;

  // x properties
  x_axes.id = 2;
  x_axes.color.r = 1;
  x_axes.color.g = 0;
  x_axes.color.b = 0;
  x_axes.color.a = 1;

  // line properties
  line.type = visualization_msgs::Marker::LINE_STRIP;
  line.ns = "line";
  line.action = visualization_msgs::Marker::ADD;
  line.lifetime = ros::Duration(0);
  line.header.frame_id = WORLD_FRAME;
  line.scale.x = AXIS_LINE_WIDTH;
  line.id = 0;
  line.color.r = 1;
  line.color.g = 1;
  line.color.b = 0;
  line.color.a = 1;

  // creating axes markers
  z_axes.points.reserve(2*poses.size());
  y_axes.points.reserve(2*poses.size());
  x_axes.points.reserve(2*poses.size());
  line.points.reserve(poses.size());
  geometry_msgs::Point p_start,p_end;
  double distance = 0;
  Eigen::Affine3d prev = poses[0];
  for(unsigned int i = 0; i < poses.size(); i++)
  {
    const Eigen::Affine3d& pose = poses[i];
    distance = (pose.translation() - prev.translation()).norm();

    tf::pointEigenToMsg(pose.translation(),p_start);

    if(distance > 0.01)
    {
      Eigen::Affine3d moved_along_x = pose * Eigen::Translation3d(AXIS_LINE_LENGHT,0,0);
      tf::pointEigenToMsg(moved_along_x.translation(),p_end);
      x_axes.points.push_back(p_start);
      x_axes.points.push_back(p_end);

      Eigen::Affine3d moved_along_y = pose * Eigen::Translation3d(0,AXIS_LINE_LENGHT,0);
      tf::pointEigenToMsg(moved_along_y.translation(),p_end);
      y_axes.points.push_back(p_start);
      y_axes.points.push_back(p_end);

      Eigen::Affine3d moved_along_z = pose * Eigen::Translation3d(0,0,AXIS_LINE_LENGHT);
      tf::pointEigenToMsg(moved_along_z.translation(),p_end);
      z_axes.points.push_back(p_start);
      z_axes.points.push_back(p_end);

      // saving previous
      prev = pose;
    }

    line.points.push_back(p_start);
  }

  markers_msg.markers.push_back(x_axes);
  markers_msg.markers.push_back(y_axes);
  markers_msg.markers.push_back(z_axes);
  markers_msg.markers.push_back(line);

  marker_publisher_.publish(markers_msg);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "descartes_generate_motion_plan");
  
  ros::NodeHandle nh;
  
  // Get Joint Names
  nh.getParam("controller_joint_names", names);
  marker_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("visualize_trajectory_curve",1,true);
  
  ros::ServiceServer ur_generate_motion_plan_server = nh.advertiseService("descartes_generate_motion_plan", generate_motion_plan);
  //ros::ServiceServer visualize_trajectory_server = nh.advertiseService("visualize_trajectory", visualize_trajectory);
  ROS_INFO("Motion planner service online");
  ros::spin();
  
  return 0;
}
