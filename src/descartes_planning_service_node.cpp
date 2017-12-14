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
// Includes the planner we will be using
#include <descartes_planner/dense_planner.h>

//The path is build using points in the eigen convention
#include <eigen_conversions/eigen_msg.h>

#include <time.h>

#include "descartes_planning_service/generate_motion_plan.h"

typedef std::vector<descartes_core::TrajectoryPtPtr> TrajectoryVec;
typedef TrajectoryVec::const_iterator TrajectoryIter;

//There is a cleaner way, but this works...
std::vector<std::string> names;

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
descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d& pose, std::string underdefinedAxis)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;

  if(underdefinedAxis.compare("X_AXIS") == 0)
  {
    return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI/30, AxialSymmetricPt::X_AXIS) );
  }
  else if(underdefinedAxis.compare("Y_AXIS") == 0)
  {
    return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI/30, AxialSymmetricPt::Y_AXIS) );
  }
  else if(underdefinedAxis.compare("Z_AXIS") == 0)
  {
    return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI/30, AxialSymmetricPt::X_AXIS) );
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
  
  TrajectoryVec path;
  for(unsigned int i = 0; i < points.size(); i++)
  {
    const Eigen::Affine3d& pose = points[i];
    descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pose, req.underdefinedAxis);
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

  ROS_INFO("Startong path planning");

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


int main(int argc, char** argv)
{
  ros::init(argc, argv, "descartes_generate_motion_plan");
  
  ros::NodeHandle nh;
  
  // Get Joint Names
  nh.getParam("controller_joint_names", names);
  
  ros::ServiceServer ur_generate_motion_plan_server = nh.advertiseService("descartes_generate_motion_plan", generate_motion_plan);
  ROS_INFO("Motion planner service online");
  ros::spin();
  
  return 0;
}
