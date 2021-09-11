#include <iostream>
#include <ros/ros.h>
#include<ros/time.h>
#include <ros/duration.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <perception_msgs/ObstacleArray.h>
#include <perception_msgs/Obstacle.h>

#include <nav_msgs/Odometry.h>

#include <cmath>

using namespace std;

class object_I2V
{
private:
	std::string sub_object_str;
	std::string sub_obstacle_str;
	std::string pub_object_str;
	std::string pub_obstacle_str;
	std::string sub_odom_str;
	std::string frame_id;
	
	double vehicle_x = 0;
	double vehicle_y = 0;
	double vehicle_yaw = 0;
	bool show_myself = true;
private:
	//创建节点句柄
	ros::NodeHandle nh;
	//声明订阅器
	ros::Subscriber sub_object;
	ros::Subscriber sub_obstacle;
	ros::Subscriber sub_odom;
	//声明发布器
	ros::Publisher pub_object;
	ros::Publisher pub_obstacle;
	//定时器
public:
	object_I2V();

	void init_para();
	//回调函数
	void callback_object(visualization_msgs::MarkerArray t);
	void callback_obstacle(perception_msgs::ObstacleArray t);
	void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
};

object_I2V::object_I2V() {
	//launch文件获得参数
	init_para();
	//订阅
	sub_odom = nh.subscribe(sub_odom_str, 1, &object_I2V::odom_callback, this);
	sub_object = nh.subscribe(sub_object_str, 1, &object_I2V::callback_object, this);
	sub_obstacle = nh.subscribe(sub_obstacle_str, 1, &object_I2V::callback_obstacle, this);

	//发布
	pub_object = nh.advertise<visualization_msgs::MarkerArray>(pub_object_str, 1);
	pub_obstacle = nh.advertise<perception_msgs::ObstacleArray>(pub_obstacle_str, 1);
}
void object_I2V::init_para() {
	ros::param::get("~sub_object_str", sub_object_str);
	ros::param::get("~sub_obstacle_str", sub_obstacle_str);
	ros::param::get("~pub_object_str", pub_object_str);
	ros::param::get("~pub_obstacle_str", pub_obstacle_str);
	ros::param::get("~sub_odom_str", sub_odom_str);
	ros::param::get("~frame_id", frame_id);
	ros::param::get("~show_myself", show_myself);
}

void object_I2V::odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
	vehicle_x = msg->pose.pose.position.x;
	vehicle_y = msg->pose.pose.position.y;
	vehicle_yaw = msg->pose.covariance[0];
	std::cout << "Get vehcile pose:\n" << "x = " << vehicle_x << "\ty = " << vehicle_y << "\tyaw = " << vehicle_yaw << std::endl;
}

void object_I2V::callback_object(visualization_msgs::MarkerArray t) {
	std::cout << "callback_object" << std::endl;
	visualization_msgs::MarkerArray tmp_t, tmp_pub;
	tmp_t = t;
	double tmp_x, tmp_y, tmp_theta;
	//将tmp_t中的大地坐标转换为车辆坐标系坐标
	for (int i = 0; i < tmp_t.markers.size(); i++) {
		tmp_t.markers[i].header.frame_id = frame_id;
		tmp_x = tmp_t.markers[i].pose.position.x - vehicle_x;
		tmp_y = tmp_t.markers[i].pose.position.y - vehicle_y;
		tmp_theta = 2 * asin(tmp_t.markers[i].pose.orientation.z);
		tmp_t.markers[i].pose.position.x = tmp_x * cos(vehicle_yaw) + tmp_y * sin(vehicle_yaw);
		tmp_t.markers[i].pose.position.y = -tmp_x * sin(vehicle_yaw) + tmp_y * cos(vehicle_yaw);
		tmp_t.markers[i].pose.orientation.z = sin(0.5*(tmp_theta - vehicle_yaw));
		tmp_t.markers[i].pose.orientation.w = cos(0.5*(tmp_theta - vehicle_yaw));
	}
	//将自己从障碍物中删去
	if (show_myself == false) {
		for (int i = 0; i < tmp_t.markers.size(); i++) {
			tmp_x = tmp_t.markers[i].pose.position.x;
			tmp_y = tmp_t.markers[i].pose.position.y;
			if (tmp_x*tmp_x + tmp_y * tmp_y < 4 * 4) {
				continue;
			}
			tmp_pub.markers.push_back(tmp_t.markers[i]);
		}
	}
	pub_object.publish(tmp_pub);
}

void object_I2V::callback_obstacle(perception_msgs::ObstacleArray t) {
	std::cout << "callback_obstacle" << std::endl;
	perception_msgs::ObstacleArray tmp_t, tmp_pub;
	tmp_t = t;
	double tmp_x, tmp_y, tmp_theta;
	//将tmp_t中的大地坐标转换为车辆坐标系坐标
	for (int i = 0; i < tmp_t.obstacles.size(); i++) {
		tmp_x = tmp_t.obstacles[i].pose.position.x - vehicle_x;
		tmp_y = tmp_t.obstacles[i].pose.position.y - vehicle_y;
		tmp_theta = 2 * asin(tmp_t.obstacles[i].pose.orientation.z);
		tmp_t.obstacles[i].pose.position.x = tmp_x * cos(vehicle_yaw) + tmp_y * sin(vehicle_yaw);
		tmp_t.obstacles[i].pose.position.y = -tmp_x * sin(vehicle_yaw) + tmp_y * cos(vehicle_yaw);
		tmp_t.obstacles[i].pose.orientation.z = sin(0.5*(tmp_theta - vehicle_yaw));
		tmp_t.obstacles[i].pose.orientation.w = cos(0.5*(tmp_theta - vehicle_yaw));
	}
	//将自己从障碍物中删去
	if (show_myself == false) {
		for (int i = 0; i < tmp_t.obstacles.size(); i++) {
			tmp_x = tmp_t.obstacles[i].pose.position.x;
			tmp_y = tmp_t.obstacles[i].pose.position.y;
			if (tmp_x*tmp_x + tmp_y * tmp_y < 4 * 4) {
				continue;
			}
			tmp_pub.obstacles.push_back(tmp_t.obstacles[i]);
		}
	}
	pub_obstacle.publish(tmp_pub);
}

//回调函数
int main(int argc, char **argv)
{
	ros::init(argc, argv, "obj_markers");
	ros::NodeHandle nh("~");
	// omp_set_num_threads(4);
	object_I2V xy_convert;

	ros::spin();
	return 0;
}
