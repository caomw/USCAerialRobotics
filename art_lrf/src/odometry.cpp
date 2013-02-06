#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <boost/shared_ptr.hpp>
#include <art_lrf/Lines.h>
#include "odometry.h"
#include <Eigen/Eigen>
#include <cmath>
#include <vector>
#include <iostream>


#define MIN_VAL(X,Y) ((X < Y) ? (X) : (Y))
#define MAX_VAL(X,Y) ((X > Y) ? (X) : (Y))

using namespace std;
using namespace Eigen;


bool firstRun = true;
int est_rot = 0;
int previous_est_rot = 0;
Vector2f est_translation;
Vector2f previous_translation;

class line {

public:
    int theta_index;
    int rho_index;
    int votes;	
    float est_rho;
    vector<int> pixels;
    vector<int> line_pixels;
    vector<float> lengths;
    vector<bool> partial_observe;
    vector<vector<int> > support_points;
    vector<vector<int> > endpoints;
    vector<vector<float> > endpoint_ranges;
	    
	    
    line()
    {
	theta_index = 0;
	rho_index = 0;
	pixels = vector<int>();
	endpoints = vector<vector<int> >();
	line_pixels = vector<int>();
    }
};

class measurement
{
     vector<line> lines;
     float x;
     float y;
     float yaw;
};

// class waypoints
// {
//     vector<measurement> measurements;
//     void update(measurement newMeasurement)
//     {
// 	if( measurements.size() == 0)
// 	    {
// 		measurements.push_back(newMeasurement);
// 		return;
// 	    }
		

// 	measurement currentMeasurement = measurements(measurements.size()-1);
//     }
			   
	       
// } waypoints;

vector<float> linspace(double min_range, double max_range, int total_no_points)  {
    vector<float> phi = std::vector<float>(total_no_points);
    phi[0] = min_range;
    double delta = (max_range-min_range)/(total_no_points - 1);
    for(int i = 1 ; i < phi.size() ; i++)
        phi[i]=phi[i-1]+delta;

    return phi;
}

void compare_scans(boost::shared_ptr<vector<line> > firstScan,
		   boost::shared_ptr<vector<line> > secondScan,
		   int rotation_prior_mean,
		   int& rotOut,
		   Vector2f translation_prior_mean,
		   Vector2f & translationOut)
{
		   
    int num_theta_entries = (2 * M_PI - delta_theta*angle_increment)/(delta_theta*M_PI/180) + 1;
    vector<float> theta = vector<float>(num_theta_entries);

    for(double i = -M_PI, counter = 0; counter < num_theta_entries; i+= delta_theta*M_PI/180, counter++) {
	theta[counter] = i;
					
    }
				
    			
    			
    
    
    //Initial rotation estimation. This will vary depending on the lines being compared.

//    Vector2f translation_prior_mean = previous_translation;
    float translation_prior_sd = 0.5;

//    float rotation_prior_mean = previous_est_rot;
    float rotation_prior_sd = 15;
    float rotation_posterior_sd = 4;
 
    vector<float> rotation_prior (360,0);
    vector<float> rotation_score (360,0);
    
    float dist;
    float sum1;
    for (int i=0; i<360; i++)
    {
	dist = min(abs(i - rotation_prior_mean),abs(i-360-rotation_prior_mean));
	rotation_prior[i] = exp(-pow(dist,2)/pow(rotation_prior_sd,2));
	sum1 += rotation_prior[i];
    }
    for (int i=0; i<360; i++)
    {
	rotation_prior[i] /= sum1;
    }
    
    

    //for each pair of lines between the two scans, estimate the probability that they are from the same object. The more likely they are to be by an unmoved object, the more likely that the rotation of the was equal to the theta difference between the lines.

    for (int i=0; i < firstScan->size(); i++)
    {
	for (int j=0; j < secondScan->size(); j++)
	{
	    float dist;
	    float theta_diff = (delta_theta*(firstScan->at(i).theta_index - secondScan->at(j).theta_index) );
	    if (theta_diff < 0) {
		theta_diff += 360;
	    }
			    
	    //if the lines are clearly not from the same source because the translation has moved too much for one frame, then do not use their relative angles to estimate the rotation.
	    float expected_rho_change;
	    expected_rho_change = -(translation_prior_mean[0]*cos(theta[firstScan->at(i).theta_index]) + translation_prior_mean(1)*sin(theta[firstScan->at(i).theta_index]) );
	    if(abs(firstScan->at(i).est_rho + expected_rho_change - secondScan->at(j).est_rho) > rho_sanity_tolerance)
	    {
		continue;
	    }

	    for (int k=0; k<360; k++)
	    {
		dist = min(abs(k-theta_diff), abs(k-theta_diff-360));
		rotation_score[k] = rotation_score[k] + exp(-pow(dist,2)/pow(rotation_posterior_sd,2));
	    }
	}
    }

    vector<float> rotation_prob(360,0);

    for (int i=0; i<rotation_prob.size();i++){
	rotation_prob[i] = rotation_prior[i]* rotation_score[i];
	//rotation_prob[i] =  rotation_score[i];
    }
		
    float max1 = 0;
				
				
    for (int i=0; i<rotation_prob.size();i++){
	if (rotation_prob[i]> max1){
	    max1 = rotation_prob[i];
	    rotOut = i;
	}
    }
				
    cout << "est rot: " << rotOut << endl;

    vector<vector<int> > matched_lines;
    for (int i=0; i < firstScan->size(); i++)
    {
	for (int j=0; j < secondScan->size();j++)
	{
	    if (min(abs(delta_theta*(firstScan->at(i).theta_index - secondScan->at(j).theta_index) - rotOut), abs(delta_theta*(firstScan->at(i).theta_index - secondScan->at(j).theta_index) - rotOut + 360))< 10)
	    {

		float expected_rho_change;
		expected_rho_change = -(translation_prior_mean[0]*cos(theta[firstScan->at(i).theta_index]) + translation_prior_mean(1)*sin(theta[firstScan->at(i).theta_index]) );
		if(abs(firstScan->at(i).est_rho + expected_rho_change - secondScan->at(j).est_rho) < rho_sanity_tolerance)
		{
		    vector<int> temp_vector(2,0);
		    temp_vector[0] = i; temp_vector[1] = j;
		    matched_lines.push_back(temp_vector);
		    break;
		}
	    }
	}
    }
		
    if (matched_lines.size() > 0)
    {
			
	
	//compute the change in rho for each pair of matched lines
	MatrixXf rho_change(matched_lines.size(),1);
	for (int i=0; i<matched_lines.size(); i++)
	{   
	    rho_change(i,0) = firstScan->at(matched_lines[i][0]).est_rho - secondScan->at(matched_lines[i][1]).est_rho;
	}


	MatrixXf A(matched_lines.size(),2);
	for (int i=0; i<matched_lines.size(); i++)
	{

	    A(i,0) = cos(theta[firstScan->at(matched_lines[i][0]).theta_index]);
	    A(i,1) = sin(theta[firstScan->at(matched_lines[i][0]).theta_index]);


//	    A(i,0) = cos(theta[secondScan->at(matched_lines[i][1]).theta_index]);
//	    A(i,1) = sin(theta[secondScan->at(matched_lines[i][1]).theta_index]);

	    cout << A(i,0) << "  " << A(i,1) << endl;
	}


	
	
	float condition_threshold = 0.2;

	Matrix2f C = A.transpose()*A;
	SelfAdjointEigenSolver<Matrix2f> eigensolver(C);
	Vector2f Eigenvalues = eigensolver.eigenvalues();
	Matrix2f Eigenvectors = eigensolver.eigenvectors();
	MatrixXd observed_projection(2,2);


	if (Eigenvalues(0) == 0) {
	    Eigenvalues(0) = 0.000001;
	}
	if (Eigenvalues(1) == 0) {
	    Eigenvalues(1) = 0.000001;
	}
	cout << "condition number: " << Eigenvalues(0)/Eigenvalues(1) << endl;
	if (Eigenvalues(0)/Eigenvalues(1) < condition_threshold || Eigenvalues(0)/Eigenvalues(1) > 1/condition_threshold)
	{
	    int principal_eigenvalue = 0;
	    if (Eigenvalues(1) > Eigenvalues(0))
		principal_eigenvalue = 1;
				
	    MatrixXd unobserved_direction(2,1);				
	    unobserved_direction(0) = Eigenvectors(0,1-principal_eigenvalue);
	    unobserved_direction(1) = Eigenvectors(1,1-principal_eigenvalue);

	    MatrixXd observed_direction(2,1);// = Eigenvectors(0,1,2,1);
	    observed_direction(0) = Eigenvectors(0,principal_eigenvalue);
	    observed_direction(1) = Eigenvectors(1,principal_eigenvalue);
						
	    //cout << "principal_eigenvalue: " << principal_eigenvalue << endl;
	    //cout << "Eigenvectors: " << endl;
	    //cout << Eigenvectors;
				
	    observed_projection = observed_direction*(observed_direction.transpose()*observed_direction).inverse()*observed_direction.transpose();
				
	}
	else {
	    observed_projection << 1, 0,
		0, 1;
	}

	Vector2f currentTranslation = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(rho_change);


	Matrix2f observed_projection2f;
	observed_projection2f(0,0) = observed_projection(0,0);
	observed_projection2f(0,1) = observed_projection(0,1);
	observed_projection2f(1,0) = observed_projection(1,0);
	observed_projection2f(1,1) = observed_projection(1,1);

	//translationOut = observed_projection2f * currentTranslation + (previous_translation - observed_projection2f*previous_translation);
	translationOut = currentTranslation;


	if (rotOut > 180) {				
	    rotOut -= 360;
						 
	}
	cout << endl << "Estimated Rotation:  " << rotOut;
	cout<< "   Estimated Translation:  " << translationOut(0) << "   " << translationOut(1) << "   MatchedLines: " << matched_lines.size() << endl; 
		    
    }
}      



class Compare {
public:
    ros::NodeHandle nh;
    ros::Subscriber sub_lines;
    ros::Publisher pub_pos;
    geometry_msgs::Point32 pos_msg_old;

    double counter;
    boost::shared_ptr<vector<line> > scan1, scan2, original_scan;

    Compare(ros::NodeHandle& _nh): nh(_nh) {
	pub_pos = nh.advertise<geometry_msgs::Point32>("/arduino/pos", 1);
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
	
	for (int i=0; i<temp.size(); i++) {
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
	
	for (int i=0; i<temp.size(); i++) {
	    if (angle > temp[i]) {
		current_index = i;
	    } else {
		break;
	    }
	}
	
	return current_index;
    }


    void lines_callback(const art_lrf::Lines::ConstPtr& msg_lines) {

	if ((msg_lines->theta_index.size() != 0) && (msg_lines->est_rho.size() != 0) && (msg_lines->endpoints.size() != 0)) {
			
			
	    scan2.reset(new vector<line> ());
	    line temp;
	    geometry_msgs::Point32 temp2;
	    geometry_msgs::Point32 temp_theta_index;
	    vector<int> temp3;

	    for (int i = 0; i < msg_lines->theta_index.size(); i++) {
		temp.theta_index = msg_lines->theta_index[i];
		temp.est_rho = msg_lines->est_rho[i];
				
		for (int j = 0; j < msg_lines->endpoints[i].points.size(); j++) {
		    temp2 = msg_lines->endpoints[i].points[j];
		    temp3.push_back(temp2.x);
		    temp3.push_back(temp2.y);
		    temp.endpoints.push_back(temp3);
		}
				
				
		scan2->push_back(temp);
		temp.endpoints.clear();
	    }

	    if (firstRun == true){
		firstRun = false;
		scan1 = scan2;
		previous_translation << 0,0;
	    }
	    else
	    {
	
		compare_scans(scan1, scan2, previous_est_rot, est_rot, previous_translation, est_translation );
		previous_translation = est_translation;
		previous_est_rot = est_rot;

		//scan1 = scan2;
	
	    }



	    geometry_msgs::Point32 pos_msg;
	    pos_msg.x = est_translation(0);
	    pos_msg.y = est_translation(1);
	    pos_msg.z = est_rot;

	    pub_pos.publish(pos_msg);
	
	    pos_msg_old.x = pos_msg.x;
	    pos_msg_old.y = pos_msg.y;
	    pos_msg_old.z = pos_msg.z;
	}		


	
	else {
	    cout << endl << "MISSED SCAN" << endl;
	    pub_pos.publish(pos_msg_old);
	}

    }
};



int main (int argc, char** argv) {
    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh;

    Compare compare(nh);

    ros::Rate loop_rate(10);
    while(ros::ok()) {
	ros::spinOnce();
	loop_rate.sleep();
    }
    return 0;

}


