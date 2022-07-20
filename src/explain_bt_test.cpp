#include <iostream>
#include <fstream>
#include <string>
#include <boost/filesystem.hpp>
#include "ros/ros.h"
#include <behaviortree_cpp_v3/bt_factory.h>
#include "explain_bt/Explain.h"
#include "ExplainableBT.h"
#include "ur5e_nodes.h"

#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace BT;
using boost::filesystem::current_path;

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(4);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "base_link";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.4;
  collision_objects[0].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = -0.5;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.2;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;

  // BEGIN_SUB_TUTORIAL table2
  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "base_link";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.4;
  collision_objects[1].primitives[0].dimensions[1] = 0.2;
  collision_objects[1].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0;
  collision_objects[1].primitive_poses[0].position.y = -0.5;
  collision_objects[1].primitive_poses[0].position.z = 0.2;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;
  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[1].ADD;

  // BEGIN_SUB_TUTORIAL object
  // Define the object that we will be manipulating
  collision_objects[2].header.frame_id = "base_link";
  collision_objects[2].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.02;
  collision_objects[2].primitives[0].dimensions[1] = 0.02;
  collision_objects[2].primitives[0].dimensions[2] = 0.2;

  /* Define the pose of the object. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = -0.5;
  collision_objects[2].primitive_poses[0].position.y = 0;
  collision_objects[2].primitive_poses[0].position.z = 0.5;
  collision_objects[2].primitive_poses[0].orientation.w = 1.0;
  // END_SUB_TUTORIAL

  collision_objects[2].operation = collision_objects[2].ADD;

   // Add a big table for everything
  collision_objects[3].id = "table3";
  collision_objects[3].header.frame_id = "base_link";

  /* Define the primitive and its dimensions. */
  collision_objects[3].primitives.resize(1);
  collision_objects[3].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[3].primitives[0].dimensions.resize(3);
  collision_objects[3].primitives[0].dimensions[0] = 2.0;
  collision_objects[3].primitives[0].dimensions[1] = 2.0;
  collision_objects[3].primitives[0].dimensions[2] = 5.0;

  /* Define the pose of the table. */
  collision_objects[3].primitive_poses.resize(1);
  collision_objects[3].primitive_poses[0].position.x = 0;
  collision_objects[3].primitive_poses[0].position.y = 0;
  collision_objects[3].primitive_poses[0].position.z = -2.5;
  collision_objects[3].primitive_poses[0].orientation.w = 1.0;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main (int argc, char **argv)
{
   BehaviorTreeFactory factory;

   using namespace UR5eNodes;/*
   factory.registerNodeType<GoHome>("GoHome");
   factory.registerNodeType<GoVertical>("GoVertical");
   factory.registerNodeType<MoveToPose>("MoveToPose");
   factory.registerNodeType<TranslateToPose>("TranslateToPose");
   factory.registerNodeType<BadMove>("BadMove");
   // factory.registerNodeType<Pick>("Pick");*/

   GripperInterface gripper;
   factory.registerSimpleAction("OpenGripper", 
                                 std::bind(&GripperInterface::open, &gripper));
   factory.registerSimpleAction("CloseGripper", 
                                 std::bind(&GripperInterface::close, &gripper));

   std::string cwd = get_current_dir_name();
   auto tree = factory.createTreeFromFile(cwd + "/src/explain_bt/src/test_tree.xml");
   ROS_INFO("BT created from file.");

   ExplainableBT explainable_tree(tree);

   ros::init(argc, argv, "explainable_bt_server");
   ros::NodeHandle n;
   ros::ServiceServer service = n.advertiseService("explainable_bt", &ExplainableBT::explain_callback, &explainable_tree);
   ros::AsyncSpinner spinner(1); // Use 1 thread
   spinner.start();
   
   ROS_INFO("Explainable BT node started.");

   //// Adding Objects ////
   moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
   addCollisionObjects(planning_scene_interface);
   ////////////////////////

   // This executeTick method runs the BT without explainability
   // tree.root_node->executeTick();
   
   // This option runs the BT with explainability
   explainable_tree.execute();

   ros::waitForShutdown();

   return 0;
}