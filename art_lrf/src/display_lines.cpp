class line {

public:
    int theta_index;
    int rho_index;
    float est_rho;
    float est_theta;
    int votes;
    vector<int> pixels;
    vector<int> vote_pixels;
    vector<int> abstain_pixels;
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

vector< vector<float>> display_points;
vector<line> lines;

for (int i=0; i<lines.size(), i++)
{
	float current_theta = lines[i].est_theta;
	float current_rho = liens[i].est_rho;
	for (float t=-100; t < 100; t++)
	{
		vector<float> new_point(2,0);
		new_point[0] = t/100*cos(current_theta-3.1415/2) + rho*cos(current_theta);
		new_point[1] = t/100*sin(current_theta-3.1415/2) + rho*sin(current_theta);
		display_points.push_back(new_point);
	}
}

		
