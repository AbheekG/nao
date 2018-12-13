#include "filter.hh"

// Static variables init.
string Filter::test_path;
std::ifstream Filter::meta_fid;
std::ifstream Filter::motion_fid;
std::ifstream Filter::sensor_fid;
vector<state_t> Filter::domain_points;

int Filter::n_epoch;
int Filter::epoch;
state_t Filter::motion_u;
vector<double> Filter::sensor_pdf;

int Filter::n_dim;
int Filter::total_size;
vector<int> Filter::dim_size;
vector<state_t> Filter::dim_bounds;
state_t Filter::dim_density;

// Init.
void Filter::filter_init  (string _test_path = "data/test1/") {
	
	test_path = _test_path;

	io_init ();

	motion_u.resize (n_dim);
	sensor_pdf.resize (total_size);

	epoch = 0;
}

void Filter::filter_destroy () {
	io_destroy ();
}

void Filter::filter_print () {

	cout << "Test path: " << test_path << endl;
	cout << "# epochs: " << n_epoch << endl;
	cout << "# dimensions: " << n_dim << endl;
	cout << "# total size: " << total_size << endl;
	
	cout << "Dim sizes: ";
	for (const auto &dim : dim_size) cout << dim << ", ";
	
	cout << "\nDim bounds. ";
	cout << "\nLower:\t";
	for (const auto &b : dim_bounds[LOWER]) cout << b << "\t";
	cout << "\nUpper:\t";
	for (const auto &b : dim_bounds[UPPER]) cout << b << "\t";

	cout << "\nPoints in domain along dimensions.";
	for (const auto &dp : domain_points) {
		cout << endl;
		for (const auto &p : dp) {
			cout << p << "\t";
		}
	}

	cout << endl;
}

// Iteration.
void Filter::new_iteration () {
	
	epoch++;

	// Update motion and sensor data.
	// Motion.
	for (auto &u : motion_u) {
		motion_fid >> u;
	}

	// Sensor
	for (auto &p : sensor_pdf) {
		sensor_fid >> p;
	}
}

/*
Clock related.
*/

void Filter::clock_start () {
	time_duration = 0;
	time = std::clock();
}

void Filter::clock_stop () {
	time_duration += std::clock() - time;
}

void Filter::clock_resume () {
	time = std::clock();
}


// Nao
Pose2D Filter::current_state () {
  
  auto mean_ = Pose2D();
  using T = decltype(mean_.translation);
  mean_.translation = T(current.x,current.y);
  mean_.rotation = current.t;
  return mean_;
}

void Filter::generate_particles (const vector<double>& cdf) {

	static std::uniform_real_distribution<double> unif_d(0, cdf[cdf.size()-1]);
	const int n_particles = 100;

	auto &particles = cache_.localization_mem->particles;
	particles.resize(n_particles);

	for (auto &p : particles) {
		int id = std::lower_bound (cdf.begin(), cdf.end(), unif_d(generator)) - cdf.begin();
		if (id < 0 || id >= cdf.size()) cout << "Some problem generating particles.\n";
		else {
			state_t state = index_to_state ( id_to_index (id) );
			p.x = state[0];
			p.y = state[1];
			p.t = state[2];
			p.w = cdf[id] - (id > 0 ? cdf[id-1] : 0);
		}
	}

}