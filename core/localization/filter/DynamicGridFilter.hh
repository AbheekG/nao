#pragma once

#include "filter.hh"

using std::vector;

class DynamicGridFilter : public Filter {

	string name;

	static constexpr double interval_size_lower_limit = 100;

	// Dynamic grid point structure.
	struct node {
		double w;	// Avg. Weight
		double s;	// Size/volume
		// Children
		vector<state_t> dim_bounds;
		vector<node*> ch;
	};

	double thr_l, thr_h;
	node *root, *old_root;

	// NOTE. motion update no erro.
	state_t motion_u;

	// Fast inline functions.

	// Given a bound returns the avg. probabilty density.
	double get_weight (const vector<state_t>&);
	double get_weight_helper (const vector<vector<double>> &, state_t &, int);

	node* create_node (double, vector<state_t>);
	void destroy_node (node*);
	
	// Given a state and node, retuns pdf at state.
	double get_pdf_value (const node*, const state_t &);
	// Checks whether a state is in grid bound.
	bool state_in_range (const vector<state_t> &, const state_t &);
	// Motion/Sensor update
	void update_node (node*);
	// Break/join nodes.
	void restructure_node (node*);
	void restructure_node_helper (node*, vector<state_t> &, int);
	
	// For normalization
	double get_total_weight (const node*);
	void normalize (node*, const double &);

	void store_cdf_helper (const node*);

	// Replicates entire tree below node. Used for update
	node* replicate_node (node*);

	// Returns the state with maximum PDF value.
	double get_max_state (node*, state_t &);

public:
	void init (double, double);
	void process ();
	// Processes and stores the CDF.
	void store_cdf ();
	void destroy ();

	// Nao
	DynamicGridFilter(MemoryCache& cache, TextLogger*& tlogger) : Filter (cache, tlogger) {};
	~DynamicGridFilter() {destroy();}
};
