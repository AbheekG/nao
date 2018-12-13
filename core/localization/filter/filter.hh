#pragma once

#include <localization/ParticleFilter.h>
#include <memory/FrameInfoBlock.h>
#include <memory/OdometryBlock.h>
#include <memory/WorldObjectBlock.h>
#include <common/Random.h>
#include <common/WorldObject.h>
#include <assert.h>
#include <math.h>


#include <random>
#include <ctime>
#include <cassert>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <algorithm>

using std::string;
using std::vector;
using std::clock_t;
using std::cin;
using std::cout;
using std::endl;

typedef vector<double> state_t;

// Fast inline functions.
inline int _index_to_id (const vector<int> &, const vector<int> &);
inline vector<int> _id_to_index (int, const vector<int> &);
inline vector<int> _state_to_index (const state_t &, const vector<vector<double>> &);
inline state_t _index_to_state (const vector<int> &, const vector<vector<double>> &);

class Filter {
private:
	// IO	
	static string test_path;
	static std::ifstream meta_fid;
	static std::ifstream motion_fid;
	static std::ifstream sensor_fid;
	static void io_init ();
	static void io_destroy ();

	// Interation related.
	static int n_epoch;
	static int epoch;
	static state_t motion_u;
	static vector<double> sensor_pdf;

	// State related.
	static int n_dim;
	static int total_size;
	static vector<int> dim_size;
	static vector<state_t> dim_bounds; // 0 : lower bound. 1 : upper bound.
	static state_t dim_density;	// Number of points per unit.
	// Points in state space where we have the PDF value.
	static vector<vector<double> > domain_points;

	// Clock related.
	clock_t time, time_duration;

protected:
	// Clock related.
	void clock_start ();
	void clock_stop ();
	void clock_resume ();

	// Randomness related.
	std::default_random_engine generator;

	// Functions.
	// Motion update. Definitons in motion_update.cc
	state_t motion_update (state_t);

	// Sensor update. Returns PDF value for given state. Definitons in sensor_update.cc
	double sensor_update (state_t);

	// Storing PDF, CDF
	std::ofstream cdf_fid;
	std::ofstream pdf_fid;
	vector<double> cdf;

	void io_store_cdf (vector<double> &);
	void io_store_pdf (vector<double> &);
	void pdf_to_cdf (vector<double> &);

	// Fast inline functions.
	inline static int index_to_id (const vector<int> &index) {
		return _index_to_id (index, dim_size);
	}
	inline static vector<int> id_to_index (int id) {
		return _id_to_index (id, dim_size);
	}
	inline static vector<int> state_to_index (const state_t &state) {
		return _state_to_index(state, domain_points);
	}
	inline static state_t index_to_state (const vector<int> &index) {
		return _index_to_state (index, domain_points);
	}

public:
	const static int LOWER = 0;
	const static int UPPER = 1;
	
	// Initiates general filter data and variables.
	static void filter_init (string);
	static void filter_destroy ();
	static void filter_print ();

	// Updates sensor data for new iteration.
	static void new_iteration ();
	
	// Access private data.
	inline clock_t get_clock_time () {return time_duration;};

	inline static int get_total_iteration () {return n_epoch;}
	inline static int get_iteration () {return epoch;}

	inline static string get_test_path () {return test_path;}
	inline static int get_dim () {return n_dim;}
	inline static int get_size (int i_dim) {return dim_size[i_dim];}
	inline static int get_total_size () {return total_size;}
	inline static double get_bound (int lu, int dim) {return dim_bounds[lu][dim];}
	inline static double get_density (int i_dim) {return dim_density[i_dim];}
	inline vector<double>& get_cdf() {return cdf;}

	// Nao
	bool dirty_;
	MemoryCache& cache_;
    TextLogger*& tlogger_;
    Particle current;

    Filter(MemoryCache& cache, TextLogger*& tlogger) : cache_(cache), tlogger_(tlogger), dirty_(true) {};
    Pose2D current_state ();
    void generate_particles (const vector<double>&);

};

// Fast inline functions outside class.
inline int _index_to_id (const vector<int> &index, const vector<int> &dim_size) {
	int id = 0;
	for (int i = 0; i < dim_size.size(); ++i) {
		id = id*dim_size[i] + index[i];
	}
	return id;
}

inline vector<int> _id_to_index (int id, const vector<int> &dim_size) {
	vector<int> index(dim_size.size());
	for (int i = dim_size.size()-1; i >= 0; --i) {
		index[i] = id%dim_size[i];
		id /= dim_size[i];
	}
	return index;
}

inline vector<int> _state_to_index (const state_t &state, 
	const vector<vector<double>> &domain_points) {
	
	vector<int> index(state.size());
	for (int i = 0; i < state.size(); ++i) {
		index[i] = std::lower_bound (domain_points[i].begin()
			, domain_points[i].end(), state[i]) - domain_points[i].begin();
	}
	return index;
}

inline state_t _index_to_state (const vector<int> &index,
	const vector<vector<double>> &domain_points) {
	state_t state(index.size());
	for (int i = 0; i < index.size(); ++i) {
		state[i] = domain_points[i][index[i]];
	}
	return state;
}