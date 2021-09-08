#include <iostream>
#include <ros/ros.h>
#include<ros/time.h>
#include <ros/duration.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <perception_msgs/ObstacleArray.h>
#include <perception_msgs/Obstacle.h>

#include <cmath>

using namespace std;

class object_I2V
{
private:
	std::string sub_object_str;
	std::string sub_obstacle_str;
	std::string pub_object_str;
	std::string pub_obstacle_str;
	
	double position_x = 0;
	double position_y = 0;
	double theta = 0;

private:
	//创建节点句柄
	ros::NodeHandle nh;
	//声明订阅器
	ros::Subscriber sub_object;
	ros::Subscriber sub_obstacle;
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
};

object_I2V::object_I2V() {
	//launch文件获得参数
	init_para();
	//订阅
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

	ros::param::get("~position_x", position_x);
	ros::param::get("~position_y", position_y);
	ros::param::get("~theta", theta);
	theta+=1.57;
	std::cout << "Get road pose:\n" << "x = " << position_x << "\ty = " << position_y << "\ttheta = " << theta << std::endl;
}

void object_I2V::callback_object(visualization_msgs::MarkerArray t) {
    std::cout<<"callback_object"<<std::endl;
	visualization_msgs::MarkerArray tmp_t,tmp_pub;
	tmp_t = t;
	double tmp_x, tmp_y, tmp_theta;
	//将tmp_t中的局部坐标转换为大地坐标系坐标
	for (int i = 0; i < tmp_t.markers.size(); i++) {
		tmp_x = tmp_t.markers[i].pose.position.x;
		tmp_y = tmp_t.markers[i].pose.position.y;
		tmp_theta = 2 * asin(tmp_t.markers[i].pose.orientation.z);
		tmp_t.markers[i].pose.position.x = position_x + tmp_x * cos(theta) - tmp_y * sin(theta);
		tmp_t.markers[i].pose.position.y = position_y + tmp_x * sin(theta) + tmp_y * cos(theta);
		tmp_t.markers[i].pose.orientation.z = sin(0.5*(theta - tmp_theta));
		tmp_t.markers[i].pose.orientation.w = cos(0.5*(theta - tmp_theta));
	}
	std::cout << "road2global over!" << std::endl;
	pub_object.publish(tmp_pub);
}

void object_I2V::callback_obstacle(perception_msgs::ObstacleArray t) {
    std::cout<<"callback_obstacle"<<std::endl;
	perception_msgs::ObstacleArray tmp_t,tmp_pub;
	tmp_t = t;
	double tmp_x, tmp_y, tmp_theta;
	//将tmp_t中的局部坐标转换为大地坐标系坐标
	for (int i = 0; i < tmp_t.obstacles.size(); i++) {
		tmp_x = tmp_t.obstacles[i].pose.position.x;
		tmp_y = tmp_t.obstacles[i].pose.position.y;
		tmp_theta = 2 * asin(tmp_t.obstacles[i].pose.orientation.z);
		tmp_t.obstacles[i].pose.position.x = position_x + tmp_x * cos(theta) - tmp_y * sin(theta);
		tmp_t.obstacles[i].pose.position.y = position_y + tmp_x * sin(theta) + tmp_y * cos(theta);
		tmp_t.obstacles[i].pose.orientation.z = sin(0.5*(theta - tmp_theta));
		tmp_t.obstacles[i].pose.orientation.w = cos(0.5*(theta - tmp_theta));
	}
	std::cout << "road2global over!" << std::endl;
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