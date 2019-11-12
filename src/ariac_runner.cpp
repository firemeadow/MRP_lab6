#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/Kit.h"
#include "osrf_gear/KitObject.h"
#include "osrf_gear/StorageUnit.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "ur_kinematics/ur_kin.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "std_msgs/String.h"
#include <cstdlib>
#include <string>
#include <vector>

using namespace std;

bool has_started_competition = false;
vector<osrf_gear::Order> current_orders;
osrf_gear::LogicalCameraImage camera_data;
sensor_msgs::JointState joint_states;

int current_kit_index = 0;
int current_kit_object_index = 0;
string current_model_type = "";
int camera_model_index = -1;

bool is_current_object_known()
{
    return !current_model_type.empty();
}

void try_start_competition(ros::ServiceClient& begin_client, ros::Rate* loop_rate)
{
    if(!has_started_competition)
    {
		ROS_INFO("Calling Service...");
        std_srvs::Trigger begin_comp;
        if(begin_client.call(begin_comp))
        {
            ROS_INFO("Service was called");
            if(begin_comp.response.success)
            {
                ROS_INFO("Successful service call");
                has_started_competition = true;
                *loop_rate = ros::Rate(10);
            }
            else
            {
                ROS_WARN("Start competition unsuccessful. Message %s", begin_comp.response.message.c_str());
            }
        }
        else
        {
            ROS_ERROR("Error contacting competition service.");
            exit(0);
        }
    }
}

void current_kit_location(ros::ServiceClient& kit_lookup_client)
{
    if(is_current_object_known() || current_orders.size() == 0 || current_orders.front().kits.size() == 0)
    {
        ROS_INFO("There is no need to lookup the current order locaiton again.");
        return;
    }
    
    ROS_INFO("Order has %d kits", current_orders.front().kits.size());
    
    for(vector<osrf_gear::KitObject>::iterator current = current_orders.front().kits.front().objects.begin(); current != current_orders.front().kits.front().objects.end(); ++current)
    {
        ROS_INFO("Kit types: %s", current->type.c_str());
    }
    
    osrf_gear::Kit& current_kit = current_orders.front().kits.at(current_kit_index);
    current_model_type = current_kit.objects.at(current_kit_object_index).type;
    osrf_gear::GetMaterialLocations kit_lookup;
    kit_lookup.request.material_type = current_model_type;
    
    
    ROS_INFO("Looking up location for kit type %s", current_kit.kit_type.c_str());
    if(kit_lookup_client.call(kit_lookup))
    {
        //Iterate t
        ROS_INFO("Called kit type lookup service found %d", kit_lookup.response.storage_units.size());
        
        
        for(vector<osrf_gear::StorageUnit>::iterator current = kit_lookup.response.storage_units.begin(); current != kit_lookup.response.storage_units.end(); ++current)
        {
            ROS_INFO("Found kit type %s in storage unit %s", current_kit.kit_type.c_str(), current->unit_id.c_str());
        }
    }
    else
    {
        ROS_ERROR("Looking up kit location falied.");
    }
}

void print_pose(string message, geometry_msgs::Pose& pose)
{
    geometry_msgs::Point position = pose.position;
    geometry_msgs::Quaternion orientation = pose.orientation;
    ROS_INFO("%s \r\n at position %f %f %f \r\n and orientation %f %f %f %f", 
                message.c_str(),
                position.x,
                position.y,
                position.z,
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w);
}

geometry_msgs::Pose lookup_object_location(string type)
{
    for(int i = 0; camera_data.models.size(); i++)
    {
        osrf_gear::Model current = camera_data.models.at(i);
        if(current.type.compare(type) == 0)
        {
            camera_model_index = i;
            print_pose("Object of type" + type, current.pose); 
            return current.pose;
        }
    }
}


void new_order_callback(const osrf_gear::Order::ConstPtr& new_order)
{
    ROS_INFO("Received order");
    current_orders.push_back(*new_order);
}

void camera_callback(const osrf_gear::LogicalCameraImage& camera_info)
{
    camera_data = camera_info;
}

void joint_callback(const sensor_msgs::JointState& joint_info)
{
	joint_states = joint_info;
}

bool have_valid_orders(ros::ServiceClient& begin_client, ros::Rate* loop_rate, ros::ServiceClient& kit_lookup_client)
{
    try_start_competition(begin_client, loop_rate);
    if(has_started_competition)
    {
        ROS_INFO("Competition is started, looking up orders.");
        current_kit_location(kit_lookup_client);
        return is_current_object_known();
    }

    return false;
}

string remove_first_char(string string_val)
{
    if(string_val.size() > 1 && string_val.substr(0, 1).compare("/") == 0)
    {
        return string_val.substr(1, string_val.size() - 1);
    }
    else
    {
        return string_val;
    }
}

void offset_target_position(geometry_msgs::TransformStamped* goal_pose)
{
    goal_pose->transform.translation.z += 0.10;
    goal_pose->transform.rotation.w = 0.707;
    goal_pose->transform.rotation.x = 0.0;
    goal_pose->transform.rotation.y = 0.707;
    goal_pose->transform.rotation.z = 0.0;
}

trajectory_msgs::JointTrajectory plan_movement(geometry_msgs::TransformStamped desired)
{
	double T_pose[4][4], T_des[4][4];
	double q_pose[6], q_des[8][6];
	trajectory_msgs::JointTrajectory joint_trajectory;

	q_pose[0] = joint_states.position[1];
	q_pose[1] = joint_states.position[2];
	q_pose[2] = joint_states.position[3];
	q_pose[3] = joint_states.position[4];
	q_pose[4] = joint_states.position[5];
	q_pose[5] = joint_states.position[6];
	ur_kinematics::forward((double *)&q_pose, (double *)&T_pose);

	T_des[0][3] = desired.transform.translation.x;
	T_des[1][3] = desired.transform.translation.y;
	T_des[2][3] = desired.transform.translation.z + 0.3; // above part
	T_des[3][3] = 1.0;
	// The orientation of the end effector so that the end effector is down.
	T_des[0][0] = 0.0; T_des[0][1] = -1.0; T_des[0][2] = 0.0;
	T_des[1][0] = 0.0; T_des[1][1] = 0.0; T_des[1][2] = 1.0;
	T_des[2][0] = -1.0; T_des[2][1] = 0.0; T_des[2][2] = 0.0;
	T_des[3][0] = 0.0; T_des[3][1] = 0.0; T_des[3][2] = 0.0;

	joint_trajectory.header.seq++; // Each joint trajectory should have an incremented sequence number
	joint_trajectory.header.stamp = ros::Time::now(); // When was this message created.
	joint_trajectory.header.frame_id = "/world"; // Frame in which this is specified.
	// Set the names of the joints being used. All must be present.
	joint_trajectory.joint_names.clear();
	joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
	joint_trajectory.joint_names.push_back("shoulder_pan_joint");
	joint_trajectory.joint_names.push_back("shoulder_lift_joint");
	joint_trajectory.joint_names.push_back("elbow_joint");
	joint_trajectory.joint_names.push_back("wrist_1_joint");
	joint_trajectory.joint_names.push_back("wrist_2_joint");
	joint_trajectory.joint_names.push_back("wrist_3_joint");
	// Set a start and end point.
	joint_trajectory.points.resize(2);
	// Set the start point to the current position of the joints from joint_states.
	joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
	for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++) {
		for (int indz = 0; indz < joint_states.name.size(); indz++) {
			if (joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
				joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
				break;
			}
		}
	}
	// When to start (immediately upon receipt).
	joint_trajectory.points[0].time_from_start = ros::Duration(0.0);

	// Must select which of the num_sols solution to use.
	int q_des_indx = 0;

	int num_sols = ur_kinematics::inverse((double *)&T_des, (double *)&q_des);

	// Set the end point for the movement
	joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());

	// Set the linear_arm_actuator_joint from joint_states as it is not part of the inverse kinematics solution.
	joint_trajectory.points[1].positions[0] = joint_states.position[1];

	double q_sols[8][6];

	// The actuators are commanded in an odd order, enter the joint positions in the correct positions
	for (int indy = 0; indy < 6; indy++) {
		joint_trajectory.points[1].positions[indy + 1] = q_sols[q_des_indx][indy];
	}
	// How long to take for the movement.
	joint_trajectory.points[1].time_from_start = ros::Duration(1.0);

	return joint_trajectory;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lab_3_ariac");
    ros::NodeHandle node_handle;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::ServiceClient begin_client = node_handle.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    ros::ServiceClient kit_lookup_client = node_handle.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");

    ros::Subscriber order_subscriber = node_handle.subscribe("/ariac/orders", 200, new_order_callback);
    ros::Subscriber logical_camera_subscriber = node_handle.subscribe("/ariac/logical_camera", 200, camera_callback);
    ros::Subscriber joint_states_h = node_handle.subscribe("ariac/joint_states", 10, joint_callback);
	ros::Publisher joint_trajectories = node_handle.advertise<std_msgs::String>("ariac/joint_states", 1000);

    
    //Spin slow until competition starts
    ros::Rate loop_rate(0.2);

    //Main loop
    while(ros::ok())
    {
        if(have_valid_orders(begin_client, &loop_rate, kit_lookup_client))
        {			
            geometry_msgs::TransformStamped object_pose_world = tfBuffer.lookupTransform(
            	"base_link",
           		"logical_camera_frame", ros::Time(0.0), ros::Duration(1.0)
       		);
			
            offset_target_position(&object_pose_world);

            ROS_INFO("Starting to plan...");

			trajectory_msgs::JointTrajectory joint_trajectory = plan_movement(object_pose_world);

			// Publish the specified trajectory.
			joint_trajectories.publish(joint_trajectory);

			ros::AsyncSpinner spinner(1);
            spinner.start();
            
			// Instantiate the Action Server client
			actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>trajectory_as("ariac/arm/follow_joint_trajectory", true);

			// The “true” at the end of the instantiation causes a separate thread to be spun for the client.
			// Create the structure to populate for running the Action Server.
			control_msgs::FollowJointTrajectoryAction joint_trajectory_as;

			// It is possible to reuse the JointTrajectory from above
			joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;

			// The header and goal (not the tolerances) of the action must be filled out as well.
			// (rosmsg show control_msgs/FollowJointTrajectoryAction)
			actionlib::SimpleClientGoalState state = \
			trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, \
			ros::Duration(30.0), ros::Duration(30.0));

			ROS_INFO("Action Server returned with status: [%i] %s", state.state_, \
			state.toString().c_str());

            spinner.stop();
        }

        //process all callbacks
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;   
}
