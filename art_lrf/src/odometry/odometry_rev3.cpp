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
#include <deque>
#include <iostream>

#define MIN_VAL(X,Y) ((X < Y) ? (X) : (Y))
#define MAX_VAL(X,Y) ((X > Y) ? (X) : (Y))

using namespace std;
using namespace Eigen;

bool doNextRun = false;
bool firstRun = true;
int est_rot = 0;
int rot_prior = 0;
Vector2f est_translation;
Vector2f previous_translation;

MatrixXd observed_projection(2,2);

class line 
{
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

vector<float> linspace(double min_range, double max_range, int total_no_points)  {
    vector<float> phi = std::vector<float>(total_no_points);
    phi[0] = min_range;
    double delta = (max_range-min_range)/(total_no_points - 1);
    for(int i = 1 ; i < phi.size() ; i++)
        phi[i]=phi[i-1]+delta;

    return phi;
}

class scan
{
public:
    vector<line> lines;
    float x;
    float y;
    float heading;
    scan copy()
    {
	scan copiedScan;
	for(int i=0; i<this->lines.size(); i++)
	{
	    copiedScan.lines.push_back(this->lines[i]);
	}
	copiedScan.x = this->x;
	copiedScan.y = this->y;
	copiedScan.heading = this->heading;
    }
};

class scan_comparison
{
public:
    float est_rot;
    Vector2f est_translation;
    Vector2f Eigenvalues;
    Matrix2f Eigenvectors;
    vector< vector<int> > matched_lines;
};

void calculate_translation(vector<line> *scan1,
			   vector<line> *scan2,
			   vector< vector<int> > matched_lines_values,
			   Vector2f &ReturnEigenvalues,
			   Matrix2f &ReturnEigenvectors,
			   Vector2f &est_translation)
{
        //values used for calculation in line_finder. These need to match.
    float delta_rho = 0.2; //Also redefined in pairwise_correspondence
    float angle_increment =  0.00613592;
    int counter;
    float delta_theta = 1;//Also redefined in pairwise_correspondence

    int num_theta_entries = (2 * M_PI - delta_theta*angle_increment)/(delta_theta*M_PI/180) + 1;

    vector<float> theta(num_theta_entries);

    for(double i = -M_PI, counter = 0; counter < num_theta_entries; i+= delta_theta*M_PI/180, counter++) {
	theta[counter] = i;
    } 

    
    MatrixXf rho_change(matched_lines_values.size(),1);
    for (int i=0; i<matched_lines_values.size(); i++)
    {   
	rho_change(i,0) = scan2->at(matched_lines_values[i][0]).est_rho - scan1->at(matched_lines_values[i][1]).est_rho;
    }

    MatrixXf A(matched_lines_values.size(),2);

    for (int i=0; i<matched_lines_values.size(); i++)
    {
	A(i,0) = cos(theta[scan1->at(matched_lines_values[i][1]).theta_index]);
	A(i,1) = sin(theta[scan1->at(matched_lines_values[i][1]).theta_index]);
		
	cout << A(i,0) << "  " << A(i,1) << endl;
    }


    est_translation = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(rho_change);
    //est_translation << 0,0;

    float condition_threshold = 0.2;

    Matrix2f C = A.transpose()*A;
    SelfAdjointEigenSolver<Matrix2f> eigensolver(C);
    Vector2f Eigenvalues = eigensolver.eigenvalues();
    Matrix2f Eigenvectors = eigensolver.eigenvectors();

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
		
	cout << "principal_eigenvalue: " << principal_eigenvalue << endl;
	cout << "Eigenvectors: " << endl;
	cout << Eigenvectors;

	observed_projection = observed_direction*(observed_direction.transpose()*observed_direction).inverse()*observed_direction.transpose();

    }
    else {
	observed_projection << 1, 0,
	    0, 1;
    }

    Vector2f current_translation = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(rho_change);
	
	
    Matrix2f observed_projection2f;
    observed_projection2f(0,0) = observed_projection(0,0);
    observed_projection2f(0,1) = observed_projection(0,1);
    observed_projection2f(1,0) = observed_projection(1,0);
    observed_projection2f(1,1) = observed_projection(1,1);

    if (est_rot > 180) {				
	est_rot -= 360;
    }
	
    est_translation = observed_projection2f * current_translation + (previous_translation - observed_projection2f*previous_translation);
    ReturnEigenvalues = Eigenvalues;
    ReturnEigenvectors = Eigenvectors;
	
    return;
}


void pairwise_correspondence(vector<line> *scan1,
			     vector<line> *scan2,
			     float rotation_prior_mean,
			     float rotation_prior_sd,
			     float xtranslation_prior_mean,
			     float xtranslation_prior_sd,
			     float ytranslation_prior_mean,
			     float ytranslation_prior_sd,
			     vector<vector<int> >* matched_lines,
			     float &est_rot)
{	

    //values used for calculation in line_finder. These need to match.
    float delta_rho = 0.2; //Also redefined in calculate_translation
    float angle_increment =  0.00613592;
    int counter;
    float delta_theta = 1; //Also redefined in calculate_translation
    int num_theta_entries = (2 * M_PI - delta_theta*angle_increment)/(delta_theta*M_PI/180) + 1;

    vector<float> theta(num_theta_entries);

    for(double i = -M_PI, counter = 0; counter < num_theta_entries; i+= delta_theta*M_PI/180, counter++) {
	theta[counter] = i;
    } 

    //Initial rotation estimation. This will vary depending on the lines being compared.
    //float rotation_prior_mean = previous_est_rot;
    //float rotation_prior_sd = 15;
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


    /*
      for each pair of lines between the two scans, estimate the probability
      %that they are from the same object. The more likely they are to be
      %generated by an unmoved object, the more likely that the rotation of the
      %helicopter was equal to the theta difference between the lines.
    */

	
    for (int i=0; i < scan2->size(); i++)
    {
	for (int j=0; j < scan1->size(); j++)
	{
	    float dist;
	    float theta_diff = (delta_theta*(scan2->at(i).theta_index - scan1->at(j).theta_index) );
	    if (theta_diff < 0) {
		theta_diff += 360;
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
	    est_rot = i;
	}
    }
	
    cout << "est rot: " << est_rot << endl; 

    for (int i=0; i < scan2->size(); i++)
    {
	for (int j=0; j < scan1->size();j++)
	{
	    if (min(abs(delta_theta*(scan2->at(i).theta_index - scan1->at(j).theta_index) - est_rot), abs(delta_theta*(scan2->at(i).theta_index - scan1->at(j).theta_index) - est_rot + 360))< 10)
	    {
		vector<int> temp_vector(2,0);
		temp_vector[0] = i; temp_vector[1] = j;
		(*matched_lines).push_back(temp_vector);
		break;
	    }
	}
    }
    return;
}

class local_scan
{
public:
    deque<scan> scans;
    scan firstScan;
    deque< deque<scan_comparison> > comparisonMatrix;
    int maxScans;

    local_scan(int x)
    {
	maxScans = x;
    }
	
    void add_scan(scan new_scan)
    {
	//----
	//to be later implemented in global_scans class
	if (scans.size() == 0)
	{
	    firstScan = new_scan.copy();
	};
	if (scans.size() >= 10)
	{
	    scans.pop_front();
	    scans.push_back(new_scan);
	    comparisonMatrix.pop_front();
	    for( int i=0; i<comparisonMatrix.size(); i++)
	    {
		comparisonMatrix[i].pop_front();
		comparisonMatrix[i].push_back(*new scan_comparison);
	    }
	    comparisonMatrix.push_back(*new deque<scan_comparison>);
	    for(int i=0; i<comparisonMatrix[0].size(); i++)
	    {
		comparisonMatrix[comparisonMatrix.size()-1].push_back(*new scan_comparison);
	    }
	}
	else
	{
	    scans.push_back(new_scan);
	    for( int i=0; i<comparisonMatrix.size(); i++)
	    {
		comparisonMatrix[i].push_back(*new scan_comparison);
	    }
	    comparisonMatrix.push_back(*new deque<scan_comparison>);
	    for( int i=0; i<comparisonMatrix[0].size(); i++)
	    {
		comparisonMatrix[comparisonMatrix.size()-1].push_back(*new scan_comparison);
	    }
	}
		
	if (scans.size() == 1)
	{
	    scans[0].x = 0;
	    scans[0].y = 0;
	    scans[0].heading = 0;
	}
	else
	{
	    scans[scans.size()-1].x = scans[scans.size()-2].x;
	    scans[scans.size()-1].y = scans[scans.size()-2].y;	
	    scans[scans.size()-1].heading = scans[scans.size()-2].heading;
	}
    }
    void original_comparison(int index2)
    {
	if( index2 <= 2)
	{
	    return;
	}
	
	vector< vector<int> > *matched_lines;
	Vector2f ReturnEigenvalues;
	Matrix2f ReturnEigenvectors;
	Vector2f est_translation;
	float xTranslation_sd = 0.5;
	float yTranslation_sd = 0.5;
	float rotation_sd = 10;
	float est_rot = 0;
	
	pairwise_correspondence(&firstScan.lines,
				&scans[index2].lines,
				scans[index2-1].heading - 0,
				rotation_sd,
				scans[index2-1].x - 0,
				xTranslation_sd,
				scans[index2].y - 0,
				yTranslation_sd,
				matched_lines,
				est_rot);
							 
	calculate_translation(&firstScan.lines,
			      &scans[index2].lines,
			      *matched_lines,
			      ReturnEigenvalues,
			      ReturnEigenvectors,
			      est_translation);
	scans[index2].x = est_translation[0];
	scans[index2].y = est_translation[1];
	scans[index2].heading = est_rot;
    }

    void pairwise_comparison(int index1, int index2)
    {
	vector< vector<int> > *matched_lines;
	Vector2f ReturnEigenvalues;
	Matrix2f ReturnEigenvectors;
	Vector2f est_translation;
	float xTranslation_sd = 0.5;
	float yTranslation_sd = 0.5;
	float rotation_sd = 10;
	float est_rot = 0;
	
	pairwise_correspondence(&scans[index1].lines,
				&scans[index2].lines,
				scans[index2].heading - scans[index1].heading,
				rotation_sd,
				scans[index2].x - scans[index1].x,
				xTranslation_sd,
				scans[index2].y - scans[index1].y,
				yTranslation_sd,
				matched_lines,
				est_rot);
							 
	calculate_translation(&scans[index1].lines,
			      &scans[index2].lines,
			      *matched_lines,
			      ReturnEigenvalues,
			      ReturnEigenvectors,
			      est_translation);
	scans[index2].x = scans[index1].x + est_translation[0];
	scans[index2].y = scans[index1].y + est_translation[1];
	scans[index2].heading = scans[1].heading + est_rot;

    }
} local_scan(2);

class global_scan
{
public:
    vector<scan> scans;
};



class Compare {
public:
    ros::NodeHandle nh;
    ros::Subscriber sub_lines;
    ros::Publisher pub_pos;
    std_msgs::String pos_msg_old;

    double counter;
    boost::shared_ptr<vector<line> > scan1, scan2, originalScan;

    Compare(ros::NodeHandle& _nh): nh(_nh) {
	pub_pos = nh.advertise<std_msgs::String>("/arduino/pos", 1);
	sub_lines = nh.subscribe("/lrfLines", 1, &Compare::lines_callback, this);	
	counter = 0;
    }

    ~Compare() { 
	scan1.reset();
	scan2.reset();
    }

    void lines_callback(const art_lrf::Lines::ConstPtr& msg_lines) {

	if ((msg_lines->theta_index.size() != 0) && (msg_lines->est_rho.size() != 0) && (msg_lines->endpoints.size() != 0)) {
			
	    //Load newest scan into scan2
	    scan2.reset(new vector<line> ());
	    line temp;
	    geometry_msgs::Point32 temp2;
	    geometry_msgs::Point32 temp_theta_index;
	    vector<int> temp3;

	    for (int i = 0; i < msg_lines->theta_index.size(); i++) {
		temp.theta_index = msg_lines->theta_index[i];
		temp.est_rho = msg_lines->est_rho[i];
				
		for (int j = 0; j < msg_lines->endpoints[i].points.size(); j++) 
		{
		    temp2 = msg_lines->endpoints[i].points[j];
		    temp3.push_back(temp2.x);
		    temp3.push_back(temp2.y);
		    temp.endpoints.push_back(temp3);
		}
				
				
		scan2->push_back(temp);
		temp.endpoints.clear();
	    }
	    scan incomingScan;
	    for(int i=0; i<scan2->size(); i++)
	    {
		incomingScan.lines.push_back(scan2->at(i));
	    }
	    local_scan.add_scan(incomingScan);			

	    
	    /*
	pairwise_correspondence(&scans[index1].lines,
				&scans[index2].lines,
				scans[index2].heading - scans[index1].heading,
				rotation_sd,
				scans[index2].x - scans[index1].x,
				xTranslation_sd,
				scans[index2].y - scans[index1].y,
				yTranslation_sd,
				matched_lines);
							 
	calculate_translation(&scans[index1].lines,
			      &scans[index2].lines,
			      *matched_lines,
			      ReturnEigenvalues,
			      ReturnEigenvectors,
			      est_translation);*/


	    cout << endl << "Estimated Rotation:  " << est_rot;
	    //cout<< "   Estimated Translation:  " << est_translation(0) << "   " << est_translation(1) << "   MatchedLines: " << matched_lines.size() << endl; 
	    //scan1 = scan2;
	    scan1 = originalScan;


	    

	    union PosPacket {
		struct {
		    int x;
		    int y;
		    int theta;
		};
		char data[12];
	    } pos;

	    pos.x = 0;
	    pos.y = 0;
	    pos.theta = 0;

	    std_msgs::String pos_msg;
	    pos_msg.data = pos.data;
	    pub_pos.publish(pos_msg);

	    pos_msg_old.data = pos_msg.data;
				
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

    ros::Rate loop_rate(2);
    while(ros::ok()) {
	ros::spinOnce();
	loop_rate.sleep();
    }
    return 0;

/*


  vector<line> scan2;
  vector<float> angle1;

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


  for (int i=0; i<scan2.size(); i++)
  {
  for (int j=1; j<scan2[i].lengths.size(); j=j+2)
  {
  if (abs(scan2[i].lengths[j]-window_size) < window_tolerance){
  window_line = i;
  window_gap = (j-1)/2;
  }
  }	
  }

  vector<int> gap_points(2,0);

  if (window_line >= 0){
  gap_points[0] = scan2[window_line].endpoints[window_gap][1];
  gap_points[1] = scan2[window_line].endpoints[window_gap+1][0];
  p1(0) = scan2[window_line].endpoint_ranges[window_gap][1]*cos(angle1[scan2[window_line].theta_index]);
  p1(1) = scan2[window_line].endpoint_ranges[window_gap][1]*sin(angle1[scan2[window_line].theta_index]);
  p2(0) = scan2[window_line].endpoint_ranges[window_gap+1][0]*cos(angle1[scan2[window_line].theta_index]);
  p2(1) = scan2[window_line].endpoint_ranges[window_gap+1][0]*sin(angle1[scan2[window_line].theta_index]);

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
  float yaw_d = yaw*180/3.1415;

  perpindicular << -slope(1); slope(0);
  norm_current_position = current_position - window_center;
  projection = perpindicular*(perpindicular.transpose()*perpindicular).inverse()*perpindicular.transpose();
  middle_objective = projection * norm_current_position;
  objective_error = norm_current_position - middle_objective;
  middle_error = pow(pow(objective_error(0),2) + pow(objective_error(1),2),0.5);

  if (middle_error  < 0.25)
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
  }
*/
}


