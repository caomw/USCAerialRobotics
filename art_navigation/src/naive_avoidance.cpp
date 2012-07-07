#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Byte.h>
#include <std_msgs/ByteMultiArray.h>

using namespace std;


ros::Subscriber sub_scan;
ros::Publisher pub_attitude;


std::vector<float> linspace(double min_range, double max_range, int total_no_points) {
    std::vector<float> phi = std::vector<float>(total_no_points);
    phi[0] = min_range;
    double delta = (max_range-min_range)/(total_no_points - 1);
    for(int i = 1 ; i < phi.size() ; i++)
        phi[i]=phi[i-1]+(float)delta;

    return phi;
}


void lrf_callback(const sensor_msgs::LaserScan::ConstPtr& msg_lrf) {

    int tot_scans = 681;
    double min_angle = -2.08621;
    double max_angle = 2.08621;

    float critical_avoid_threshold = 0.25;
    float critical_avoid_pitch = 3;
    float critical_avoid_roll = critical_avoid_pitch;
    float min_range_threshold = 0.05;

    float forward_pitch = 2;
    float forward_roll = 0;
    float forward_yaw = 0;
    float move_forward_threshold = 1;

    float turn_pitch = 0;
    float turn_roll = 0;
    float turn_yaw = 5;

    float desired_pitch = 0;
    float desired_roll = 0;
    float desired_yaw = 0;

    vector<float> current_scan;

    for (uint32_t i = 0; i < msg_lrf->ranges.size(); i++) {
        current_scan.push_back(msg_lrf->ranges[i]);
        }

    vector<float> phi = linspace(min_angle, max_angle, tot_scans);

    bool critical_avoid_behavior = false;
    vector<int> forward_cone;
    for (int i = 0; i<tot_scans; i++){
        if (abs(phi[i]) * 180 / 3.1415 < 45){
            forward_cone.push_back(1);
        }
        else{
            forward_cone.push_back(0);
        }
    }

    int closest_point = 0;
    int closest_distance = 0;

    for (int i = 0; i<tot_scans; i++){
        if (current_scan[i] > min_range_threshold && current_scan[i] < critical_avoid_threshold){
            critical_avoid_behavior = true;
            closest_point = i;
            closest_distance = current_scan[i];
        }
    }

    bool move_forward = true;

    int closest_point_deg, opposite_point_deg;
    if (critical_avoid_behavior == true){
        closest_point_deg = phi[closest_point]*180/3.1415;
        opposite_point_deg = closest_point_deg + 180;
        if (opposite_point_deg > 180){
            opposite_point_deg = opposite_point_deg - 360;
        }
        desired_pitch = critical_avoid_pitch*cos(opposite_point_deg*3.1415/180);
        desired_roll = critical_avoid_roll*sin(opposite_point_deg*3.1415/180);
        desired_yaw = 0;
    }
    else{
        for (int i=0; i<tot_scans; i++){
            if (forward_cone[i] && (current_scan[i] < move_forward_threshold)){
                move_forward = false;
                break;
            }
        }
        if (move_forward){
            desired_pitch = forward_pitch;
            desired_roll = forward_roll;
            desired_yaw = forward_yaw;
        }
        else{
            desired_pitch = turn_pitch;
            desired_roll = turn_roll;
            desired_yaw = turn_yaw;
	}
    }

    union AttitudePacket {
	struct {
	    float pitch; 
	    float roll;
	    float yaw;
	};
	int8_t data[12];
    } pack;

    pack.pitch = desired_pitch;
    pack.roll = desired_roll;
    pack.yaw = desired_yaw;

    std_msgs::ByteMultiArray attitude_data;

    for (int i=0; i<12; i++) {
	attitude_data.data[i] = pack.data[i];
    }

    pub_attitude.publish(attitude_data);
}


int main (int argc, char** argv) {
	ros::init(argc, argv, "naive_avoidance");
	ros::NodeHandle nh;

	pub_attitude = nh.advertise<std_msgs::ByteMultiArray>("/arduino/attitude", 10);
	sub_scan = nh.subscribe("/scan", 10, lrf_callback);

	ros::Rate loop_rate(10);
	while(ros::ok()) {
  		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}










