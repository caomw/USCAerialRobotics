#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <boost/shared_ptr.hpp>
#include <art_lrf/Lines.h>
#include <Eigen/Eigen>
#include <cmath>
#include <vector>
#include <iostream>

#define MIN_VAL(X,Y) ((X < Y) ? (X) : (Y))
#define MAX_VAL(X,Y) ((X > Y) ? (X) : (Y))

using namespace std;
using namespace Eigen;

class line {

public:
    int theta_index;
    int rho_index;
    float est_rho;
    int votes;
    vector<int> pixels;
    vector<int> line_pixels;
    vector<float> lengths;
    vector<bool> partial_observe;
    vector<vector<int> > support_points;
    vector<vector<int> > endpoints;
    vector<vector<float> > endpoint_ranges;
    
    line()
    {
        pixels = vector<int>();
        endpoints = vector<vector<int> >();
        line_pixels = vector<int>();
	lengths = vector<float>();
	endpoint_ranges = vector<vector<float> >();
	partial_observe = vector<bool>();
	support_points = vector<vector<int> >();
    }
};

bool doNextRun = false;
bool firstRun = true;
int est_rot = 0;
int previous_est_rot = 0;
Vector2f est_translation;
Vector2f previous_translation;

MatrixXd observed_projection(2,2);


vector<float> linspace(double min_range, double max_range, int total_no_points)  {
    vector<float> phi = std::vector<float>(total_no_points);
    phi[0] = min_range;
    double delta = (max_range-min_range)/(total_no_points - 1);
    for(unsigned int i = 1 ; i < phi.size() ; i++)
        phi[i]=phi[i-1]+delta;

    return phi;
}




class Compare {
	public:
		ros::NodeHandle nh;
		ros::Subscriber sub_lines;
		ros::Publisher pub_pos;
		std_msgs::String pos_msg_old;

		double counter;
		boost::shared_ptr<vector<line> > scan1, scan2, original_scan;

		Compare(ros::NodeHandle& _nh): nh(_nh) {
			pub_pos = nh.advertise<std_msgs::String>("/arduino/pos", 1);
			sub_lines = nh.subscribe("/lrfLines", 1, &Compare::lines_callback, this);	
			counter = 0;
		}

		~Compare() { 
			scan1.reset();
			scan2.reset();
		}


		int find_pos_index(float pos) {
			vector<float> temp;
	
			for (int i=0; i < 2001; i++) {
				temp.push_back(-10.0+(0.01*i));
			}
	
			int current_index = 0;
	
			for (unsigned int i=0; i<temp.size(); i++) {
				if (pos > temp[i]) {
					current_index = i;
				} else {
					break;
				}
			}
	
			return current_index;
		}


		int find_heading_index(int angle) {
			vector<int> temp;
	
			for (int i=0; i < 361; i++) {
				temp.push_back(-180+i);
			}
	
			int current_index = 0;
	
			for (unsigned int i=0; i<temp.size(); i++) {
				if (angle > temp[i]) {
					current_index = i;
				} else {
					break;
				}
			}
	
			return current_index;
		}


	    void find_window_path(boost::shared_ptr<vector<line> > scan, vector<float> angle, float desired_x_translation, float desired_y_translation, int desired_rotation) {

			float window_size = 1.0;
			float window_tolerance = 0.2;
			int window_line = -1;
			int window_gap = 0;
	
			MatrixXd p1(2,1);                       //position of first window edge
			MatrixXd p2(2,1);                       //position of second window edge
			MatrixXd p(2,2);                        //Matrix of window edges
			MatrixXd current_position(2,1);	
			MatrixXd window_center(2,1);
			MatrixXd slope(2,1);
			MatrixXd perpindicular(2,1);
			MatrixXd norm_current_position(2,1);
			MatrixXd projection(2,2);
			MatrixXd middle_objective(2,1);
			MatrixXd objective_error(2,1);
			MatrixXd target(2,1);
			MatrixXd motion(2,1);
			MatrixXd new_position(2,1);
			float middle_error;


			for (unsigned int i=0; i<scan->size(); i++)
			{
				for (unsigned int j=1; j<scan->at(i).lengths.size(); j=j+2)
				{
					if (abs(scan->at(i).lengths[j]-window_size) < window_tolerance){
						window_line = i;
						window_gap = (j-1)/2;
					}
				}	
			}

			vector<int> gap_points(2,0);

			if (window_line >= 0){
				gap_points[0] = scan->at(window_line).endpoints[window_gap][1];
				gap_points[1] = scan->at(window_line).endpoints[window_gap+1][0];
				p1(0) = scan->at(window_line).endpoint_ranges[window_gap][1]*cos(angle[scan->at(window_line).theta_index]);
				p1(1) = scan->at(window_line).endpoint_ranges[window_gap][1]*sin(angle[scan->at(window_line).theta_index]);
				p2(0) = scan->at(window_line).endpoint_ranges[window_gap+1][0]*cos(angle[scan->at(window_line).theta_index]);
				p2(1) = scan->at(window_line).endpoint_ranges[window_gap+1][0]*sin(angle[scan->at(window_line).theta_index]);

				p1 << 5,3;
				p2 << 5-pow(2,.5), 3-pow(2,.5) + 2;


				p << p1(0),p2(0),
		     	     p1(1), p2(1);

				float forward_step = 0.1;


				current_position << 5,1;
				//%current_position = new_position;

				window_center = (p1 + p2)/2;


				slope = p1 - p2;
				slope = slope/pow(pow(slope(0),2) + pow(slope(1),2),0.5);
				float yaw = -asin(slope(1));
	
				perpindicular << -slope(1), slope(0);
				norm_current_position = current_position - window_center;
				projection = perpindicular*(perpindicular.transpose()*perpindicular).inverse()*perpindicular.transpose();
				middle_objective = projection * norm_current_position;
				objective_error = norm_current_position - middle_objective;
				middle_error = pow(pow(objective_error(0),2) + pow(objective_error(1),2),0.5);
	
				if (middle_error < 0.25)
				{
	    				MatrixXd temp0(1,1);
	    				temp0 = (norm_current_position.transpose()*perpindicular);
	    				float temp1 = temp0(0,0)>0? 1:-1;
	    				target = projection*(norm_current_position - temp1*forward_step*perpindicular);
				} else
				{
	    			target = projection*norm_current_position;
				}

				motion = target - norm_current_position;
				new_position = current_position + motion;
				desired_x_translation = motion(0);
				desired_y_translation = motion(1);
				if (perpindicular(0) != 0) {
					desired_rotation = -atan(perpindicular(1)/perpindicular(0))*(180/3.1415);
				} else
				{
					desired_rotation = -atan(perpindicular(1)/0.00001)*(180/3.1415);
				} 
			} else {
				desired_x_translation = 0;
				desired_y_translation = 0;
				desired_rotation = 0;
			}
		}
		


		void lines_callback(const art_lrf::Lines::ConstPtr& msg_lines) {

			if ((msg_lines->theta_index.size() != 0) && (msg_lines->est_rho.size() != 0) && (msg_lines->endpoints.size() != 0) && (msg_lines->lengths.size() != 0) && (msg_lines->endpoint_ranges.size() !=0)) {
				
				scan2.reset(new vector<line> ());
				line temp;
				geometry_msgs::Point32 temp2;
				vector<int> temp3;
				vector<float> temp4;

			// STORE CURRENT SCAN INTO scan2
			for (unsigned int i = 0; i < msg_lines->theta_index.size(); i++) {
				temp.theta_index = msg_lines->theta_index[i];
				temp.est_rho = msg_lines->est_rho[i];

				for (unsigned int j = 0; j < msg_lines->endpoints[i].points.size(); j++) {
					temp2 = msg_lines->endpoints[i].points[j];
					temp3.push_back(temp2.x);
					temp3.push_back(temp2.y);
					temp.endpoints.push_back(temp3);
				}
				temp3.clear();

				for (unsigned int k = 0; k < msg_lines->lengths[i].points.size(); k++) {
					temp2 = msg_lines->lengths[i].points[k];
					temp.lengths.push_back(temp2.x);
				}

				for (unsigned int p = 0; p < msg_lines->endpoint_ranges[i].points.size(); p++) {
					temp2 = msg_lines->endpoint_ranges[i].points[p];
					temp4.push_back(temp2.x);
					temp4.push_back(temp2.y);
					temp.endpoint_ranges.push_back(temp4);
				}
				temp4.clear();
	
				scan2->push_back(temp);
				temp.endpoints.clear();
				temp.lengths.clear();
				temp.endpoint_ranges.clear();
			}

			if (firstRun == true){
				firstRun = false;
				original_scan = scan2;
				previous_translation << 0,0;
			}
			if (doNextRun == false)
			{
				
				//scan1 = scan2;
				scan1 = original_scan;
				doNextRun = true;
			}
			else
			{

			//cout << endl << "Scan 2 start:  " << scan2->at(0).theta_index << endl; 

		 		float min_angle = -2.08621;
 				float max_angle = 2.08621;
    			int tot_scans = 681;
    
    			vector<float> angle1 = linspace(min_angle,max_angle,tot_scans);
    
    			float rho_min = .5;
    			float rho_max = 6;
    			float delta_rho = 0.1;
    			int theta_length = 360;
    			float pi = 3.1415926;
    
    
    			//Initial rotation estimation. This will vary depending on the lines being compared.
    			float rotation_prior_mean = est_rot;
    			float rotation_prior_sd = 10;
       
    			vector<float> rotation_prior (theta_length,0);
    
		    	float dist;
    			float sum1;
    			for (int i=0; i<theta_length; i++)
    			{
        			dist = min(abs(i-rotation_prior_mean),abs(i-theta_length-rotation_prior_mean));
        			rotation_prior[i] = exp(-pow(dist,2)/pow(rotation_prior_sd,2));
        			sum1 += rotation_prior[i];
    			}
    			for (int i=0; i<theta_length; i++)
    			{
        			rotation_prior[i] /= sum1;
    			}
    
    
    			/*
     			for each pair of lines between the two scans, estimate the probability
     			%that they are from the same object. The more likely they are to be
     			%generated by an unmoved object, the more likely that the rotation of the
     			%helicopter was equal to the theta difference between the lines.
     			*/
    
		    	float rotation_posterior_sd = 3;
    			vector<float> rotation_score (theta_length,0);
    
				if (doNextRun == true) {
    				for (unsigned int i=0; i < scan1->size(); i++)
    				{
        				for (unsigned int j=0; j < scan2->size(); j++)
        				{
            				float dist;
            				float theta_diff = (scan1->at(i).theta_index - scan2->at(j).theta_index)%theta_length;
            				for (int k=0; k<theta_length; k++)
            				{
                				dist = min(abs(k-360/theta_length*theta_diff), abs(k-360/theta_length*theta_diff-360));
                				rotation_score[k] = rotation_score[k] + exp(-pow(dist,2)/pow(rotation_posterior_sd,2));
            				}
        				}
    				}
    			}
    			vector<float> rotation_prob(theta_length,0);
    
    			for (unsigned int i=0; i<rotation_prob.size();i++){
        			rotation_prob[i] = rotation_prior[i]* rotation_score[i];
    			}
    
    			float max1 = 0;
    		
    			previous_est_rot = est_rot;
				for (unsigned int i=0; i<rotation_prob.size();i++) {
        			if (rotation_prob[i]> max1){
            			max1 = rotation_prob[i];
            			est_rot = i;
        			}
    			}
    			
				if (est_rot > 180) {
					est_rot -= 360;
				}

    			vector<vector<int> > matched_lines;
				if (doNextRun == true) {    
						
					for (unsigned int i=0; i < scan1->size(); i++)
    				{
        				for (unsigned int j=0; j < scan2->size();j++)
        				{
            				if (min(abs(scan1->at(i).theta_index - scan2->at(j).theta_index - est_rot), abs(scan1->at(i).theta_index - scan2->at(j).theta_index - est_rot + 360))< 4)
            				{
                				vector<int> temp_vector(2,0);
                				temp_vector[0] = i; temp_vector[1] = j;
                				matched_lines.push_back(temp_vector);
                				break;
            				}
        				}
    				}
    		
					if (matched_lines.size() == 1 && abs(est_rot - previous_est_rot) > 10)
					{
						est_rot = previous_est_rot;
						cout << endl << "No Matching Lines" << endl;
						pub_pos.publish(pos_msg_old);
					} else {			

    					MatrixXf rho_change(matched_lines.size(),1);
    					for (unsigned int i=0; i<matched_lines.size(); i++)
    					{   
        					rho_change(i,0) = scan1->at(matched_lines[i][0]).est_rho - scan2->at(matched_lines[i][1]).est_rho;
    					}
    
    					MatrixXf A(matched_lines.size(),2);
    
    					for (unsigned int i=0; i<matched_lines.size(); i++)
    					{
        					A(i,0) = cos(scan2->at(matched_lines[i][1]).theta_index*pi/180-pi);
        					A(i,1) = sin(scan2->at(matched_lines[i][1]).theta_index*pi/180-pi);
    					}
    
    			
						est_translation = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(rho_change)*delta_rho;
   	
			
				float condition_threshold = 0.1;

				Matrix2f C = A.transpose()*A;
    			SelfAdjointEigenSolver<Matrix2f> eigensolver(C);
    			Vector2f Eigenvalues = eigensolver.eigenvalues();
    			Matrix2f Eigenvectors = eigensolver.eigenvectors();
			

    			if (Eigenvalues(0)/Eigenvalues(1) < condition_threshold || Eigenvalues(0)/Eigenvalues(1) > 1/condition_threshold)
    			{
					int principal_eigenvalue = 0;
					if (Eigenvalues(1) > Eigenvalues(0)) {
						principal_eigenvalue = 1;
					}
        			
					MatrixXd unobserved_direction(2,1);				
					unobserved_direction(0) = Eigenvectors(0,1-principal_eigenvalue);
					unobserved_direction(1) = Eigenvectors(1,1-principal_eigenvalue);

					MatrixXd observed_direction(2,1);// = Eigenvectors(0,1,2,1);
					observed_direction(0) = Eigenvectors(0,principal_eigenvalue);
					observed_direction(1) = Eigenvectors(1,principal_eigenvalue);
				
        			
					observed_projection = observed_direction*(observed_direction.transpose()*observed_direction).inverse()*observed_direction.transpose();
        			
    			} else {
					observed_projection << 1, 0,
											0, 1;
				}

				Vector2f current_translation = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(rho_change)*delta_rho;
				Matrix2f observed_projection2f;
				observed_projection2f(0,0) = observed_projection(0,0);
				observed_projection2f(0,1) = observed_projection(0,1);
				observed_projection2f(1,0) = observed_projection(1,0);
				observed_projection2f(1,1) = observed_projection(1,1);

				est_translation = observed_projection2f * current_translation + (est_translation - observed_projection2f*est_translation);

				Matrix2f R;
				R << cos(est_rot*pi/180), -sin (est_rot*pi/180),
				     sin(est_rot*pi/180), cos(est_rot*pi/180);			
			
				Vector2f rotated_translation = R*current_translation; // R*est_translation;

			
			
				/*
				if (rotated_translation(0) - previous_translation(0) > max_translation)
					rotated_translation(0) = previous_translation(1) + max_translation;
				if (rotated_translation(0) - previous_translation(0) < -max_translation)
					 rotated_translation(0) = previous_translation(1) - max_translation;
				if (rotated_translation(1) - previous_translation(1) > max_translation)
					rotated_translation(1) = previous_translation(1) + max_translation;
				if (rotated_translation(1) - previous_translation(1) < -max_translation)
					 rotated_translation(1) = previous_translation(1) - max_translation;
				*/
			
				previous_translation = rotated_translation;

				//cout << endl << "SUCCESS!!" << endl;				
				cout << endl << "Estimated Rotation:  " << est_rot;
				cout<< "   Estimated Translation:  " << rotated_translation(0) << "   " << rotated_translation(1) << "   MatchedLines: " << matched_lines.size(); 
				scan1 = scan2;
				//scan1 = original_scan;

				float desiredX = 0;
				float desiredY = 0;
				int desired_rot = 0;
				find_window_path(scan2, angle1, desiredX, desiredY, desired_rot);

				union PosPacket {
					struct {
						int x;
						int y;
						int theta;
					};
					char data[12];
				} pos;

				pos.x = find_pos_index(desiredX);
				pos.y = find_pos_index(desiredY);
				pos.theta = find_heading_index(desired_rot);

				std_msgs::String pos_msg;
			
				pos_msg.data = pos.data;
				pub_pos.publish(pos_msg);
			
				pos_msg_old.data = pos_msg.data;
			}		
				}
			}
		}
		else {
		cout << endl << "MISSED SCAN" << endl;
		pub_pos.publish(pos_msg_old);
	}

 	} // END lines_callback
};



int main (int argc, char** argv) {
	ros::init(argc, argv, "compare_lines");
	ros::NodeHandle nh;

	Compare compare(nh);

	ros::Rate loop_rate(10);
	while(ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}







