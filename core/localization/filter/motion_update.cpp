/*
This file contains code for the motion update.
*/
#include "filter.hh"

// state_t Filter::motion_update (state_t x) {
// 	for (int i = 0; i < n_dim; ++i) {
// 		x[i] += motion_u[i];
// 	}
// 	return x;
// }

// Nao
state_t Filter::motion_update (state_t x) {
	const auto& disp = cache_.odometry->displacement;
	// cout << endl << get_dim () << " " << x.size() << endl;
	assert (x.size() == 3);
	x[0] += disp.translation.x;
	x[1] += disp.translation.y;
	x[2] += disp.rotation;
	return x;
}