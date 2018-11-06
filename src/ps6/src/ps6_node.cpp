#include <ros/ros.h>
#include <iostream>
#include <string>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/ConveyorBeltControl.h>
#include <osrf_gear/DroneControl.h>
#include <std_srvs/Trigger.h>

using namespace std;

bool g_take_new_snapshot = false;
osrf_gear::LogicalCameraImage g_cam2_data;

void cam2CB(const osrf_gear::LogicalCameraImage& message_holder)
{
	if(g_take_new_snapshot)
	{
		g_cam2_data = message_holder;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ps6_node");
	ros::NodeHandle n;

	ros::ServiceClient startup_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
	std_srvs::Trigger startup_srv;
	ros::ServiceClient conveyor_client = n.serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control");
	osrf_gear::ConveyorBeltControl conveyor_srv;
	ros::ServiceClient drone_client = n.serviceClient<osrf_gear::DroneControl>("/ariac/drone");
	osrf_gear::DroneControl drone_srv;

	ros::Subscriber cam2_subscriber = n.subscribe("/ariac/logical_camera_2", 1, cam2CB);

	//start up the system
	startup_srv.response.success = false;
	while(!startup_srv.response.success)
	{
		ROS_WARN("Waiting to start up...");
		startup_client.call(startup_srv);
		ros::Duration(0.5).sleep();
	}
	ROS_INFO("Received successful response from startup service");

	//start up conveyor movement to 100% power
	conveyor_srv.request.power = 100.0;
	conveyor_srv.response.success = false;
	while(!conveyor_srv.response.success)
	{
		ROS_WARN("Waiting to start conveyor...");
		conveyor_client.call(conveyor_srv);
		ros::Duration(0.5).sleep();
	}
	ROS_INFO("Received successful response from conveyor service");

	g_take_new_snapshot = true;
	while(g_cam2_data.models.size()<1)
	{
		ros::spinOnce();
		ros::Duration(0.5).sleep();
	}
	ROS_INFO("I see a box");

	//pause conveyor when box is beneath logical camera 2
	bool marker = false;
	while(!marker)
	{
		if(g_cam2_data.models[0].pose.position.z < -0.3)
		{
			ROS_INFO("Waiting for box...");
			ros::spinOnce();
			ros::Duration(0.5).sleep();
		}
		else
		{
			ROS_INFO("Halting conveyor for 5 seconds...");
			conveyor_srv.request.power = 0.0;
			conveyor_srv.response.success = false;
			marker = true;
			conveyor_client.call(conveyor_srv);
			ros::Duration(5.0).sleep();
		}
	}
	
	//start up conveyor movement again	
	ROS_INFO("Resuming conveyor movement...");
	conveyor_srv.request.power = 100.0;
	conveyor_srv.response.success = false;
	conveyor_client.call(conveyor_srv);
	
	ros::Duration(15.0).sleep();
	
	//start up the drone
	drone_srv.request.shipment_type = "shipping_box_0";
	drone_srv.response.success = false;
	while(!drone_srv.response.success)
	{
		ROS_WARN("Waiting to call drone...");
		drone_client.call(drone_srv);
		ros::Duration(0.5).sleep();
	}
	ROS_INFO("Received successful response from drone service");
	
	//end the simulation
	conveyor_srv.request.power = 0.0;
	conveyor_srv.response.success = false;
	conveyor_client.call(conveyor_srv);
	
	ROS_INFO("Simulation completed");
	return 0;
}
