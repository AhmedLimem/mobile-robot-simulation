#include <vector>
#include <string>
#include <math.h>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "location_monitor/LandmarkDistance.h"

using std::vector;
using std::string;
using location_monitor::LandmarkDistance;

class Landmark {
	public:
		string name;
		double x;
		double y;
		Landmark(string name, double x, double y)
			: name(name), x(x), y(y) {}
};

class LandmarkMonitor{
	private:
		vector<Landmark> landmarks_;
		ros::Publisher landmark_pub_;

		void InitLandmarks(){
			landmarks_.push_back(Landmark("Cube", 0.31, 0.49));
			landmarks_.push_back(Landmark("Dumpster", 0.11, -1.42));
			landmarks_.push_back(Landmark("Barrier", -1.59, -0.83));
		}

		LandmarkDistance FindClosest(double x, double y){
			LandmarkDistance result;
			result.distance = -1;

			for (size_t i = 0; i < landmarks_.size(); ++i){
				const Landmark& landmark = landmarks_[i];
				double dx = landmark.x - x;
				double dy = landmark.y - y;
				double distance = sqrt(dx*dx + dy*dy);

				if (result.distance < 0 || distance < result.distance){
					result.name = landmark.name;
					result.distance = distance;

				}
			}

			return result;
		}
	public:
		LandmarkMonitor(const ros::Publisher& landmark_pub)
			: landmarks_(), landmark_pub_(landmark_pub) {
			InitLandmarks();
		}

		void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
			 double x = msg->pose.pose.position.x;
       			 double y = msg->pose.pose.position.y;
			 LandmarkDistance ld = FindClosest(x, y);
       			 //ROS_INFO("name: %s, d: %f", ld.name.c_str(), ld.distance);
			 landmark_pub_.publish(ld);
			 if (ld.distance <= 0.5){
				 ROS_INFO("I'm near the %s", ld.name.c_str());
			 }
		}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "location_monitor");
	ros::NodeHandle nh;
	ros::Publisher landmark_pub = nh.advertise<LandmarkDistance>("closest_landmark", 10);
	LandmarkMonitor monitor(landmark_pub);
	ros::Subscriber sub = nh.subscribe("odom", 10, &LandmarkMonitor::odomCallback, &monitor);
	ros::spin();
	return 0;
}
