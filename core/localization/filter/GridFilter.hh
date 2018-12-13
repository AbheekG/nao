#pragma once

#include "filter.hh"

using std::vector;

class GridFilter : public Filter {
	string name;

	int grid_total_size;
	vector<int> grid_size;
	vector<vector<double>> grid_points;
	vector<double> grid_pdf;

	// Fast inline functions.
	inline int grid_index_to_id (const vector<int> &index) {
		return _index_to_id (index, grid_size);
	}
	inline vector<int> grid_id_to_index (int id) {
		return _id_to_index (id, grid_size);
	}
	inline vector<int> grid_state_to_index (const state_t &state) {
		return _state_to_index(state, grid_points);
	}
	inline state_t grid_index_to_state (const vector<int> &index) {
		return _index_to_state (index, grid_points);
	}

	state_t get_max_state ();

public:
	void init (int);
	void process ();
	// Processes and stores the CDF.
	void store_cdf ();
	void destroy ();

	// Nao
	GridFilter(MemoryCache& cache, TextLogger*& tlogger) : Filter (cache, tlogger) {};
	~GridFilter() {destroy();}
};
