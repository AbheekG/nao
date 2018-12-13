/*
This file contains code for the sensor update.
And some basic functions for array access.
*/
#include <memory/WorldObjectBlock.h>
#include "filter.hh"

// double Filter::sensor_update (state_t x) {
// 	return sensor_pdf[index_to_id(state_to_index (x))];
// }

void Filter::pdf_to_cdf (vector<double> &cdf) {

	// Summing up to find PDF.
	/*
	TODO.
	Currently supports 1-D, 2-D and 3-D individually.
	Add support for arbritary dimension.
	*/
	vector<int> index(n_dim);

	assert (n_dim == 1 || n_dim == 2 || n_dim == 3);
	double temp = 0;
	if (n_dim == 1) {
		int i = 0;
		for (index[i] = 0; index[i] < dim_size[i]; ++index[i]) {
			cdf[index_to_id(index)] += temp;
			temp = cdf[index_to_id(index)];
		}
	}
	else if (n_dim == 2) {
		int i = 0, j = 1;
		for (index[i] = 0; index[i] < dim_size[i]; ++index[i]) {
			for (index[j] = 0; index[j] < dim_size[j]; ++index[j]) {
				cdf[index_to_id(index)] += temp;
				temp = cdf[index_to_id(index)];
			}
		}
	}
	else if (n_dim == 3) {
		int i = 0, j = 1, k = 2;
		for (index[i] = 0; index[i] < dim_size[i]; ++index[i]) {
			for (index[j] = 0; index[j] < dim_size[j]; ++index[j]) {
				for (index[k] = 0; index[k] < dim_size[k]; ++index[k]) {
					cdf[index_to_id(index)] += temp;
					temp = cdf[index_to_id(index)];
				}
			}
		}
	}
}

// Nao

inline double gaussianPDF( double x, double mu, double sigma = 10) {
  return (1. / sqrt(2*M_PI*sigma*sigma)) * exp(- ((x-mu)*(x-mu)) / (2*sigma*sigma));
}


double Filter::sensor_update (state_t state) {

	// Beacons World Locations
	static map<WorldObjectType, Point2D> beaconLocation = {
		{ WO_BEACON_BLUE_YELLOW,    {1500, 1000} },
		  { WO_BEACON_YELLOW_BLUE,    {1500, -1000} },
		  { WO_BEACON_BLUE_PINK,      {0, 1000} },
		  { WO_BEACON_PINK_BLUE,      {0, -1000} },
		  { WO_BEACON_PINK_YELLOW,    {-1500, 1000} },
		  { WO_BEACON_YELLOW_PINK,    {-1500, -1000} }
	};

  double w = 1.0;

  for (const auto& beacon : beaconLocation) {
    const auto& object = cache_.world_object->objects_[beacon.first];
    if ( object.seen == false )
      continue;

    double dist = sqrt( (beacon.second.x - state[0])*(beacon.second.x - state[0])
                + (beacon.second.y - state[1])*(beacon.second.y - state[1]) );
    w *= gaussianPDF ( object.visionDistance, dist, 500);

    /*
      Setting everything to (-pi, pi].
    */

    double beacon_theta = atan((beacon.second.y - state[1]) / (beacon.second.x - state[0]));

    if (not (beacon_theta >= -M_PI && beacon_theta <= M_PI)) {
    	// cout << "Beacon theta = " << beacon_theta << endl;
    	continue;
    }
    // assert(beacon_theta >= -M_PI && beacon_theta <= M_PI);    

    double beacon_bearing = object.visionBearing;
    if (not (beacon_bearing >= -M_PI && beacon_bearing <= M_PI)) {
    	// cout << "beacon_bearing = " << beacon_bearing << endl;
    	continue;
    }
    // assert(beacon_bearing >= -M_PI && beacon_bearing <= M_PI);

    double theta = beacon_theta - beacon_bearing;

    if (beacon.second.x - state[0] < 0 && beacon.second.y - state[1] < 0) {
      theta -= M_PI;
    }

    if (beacon.second.x - state[0] < 0 && beacon.second.y - state[1] > 0) {
      theta += M_PI;
    }

    if (theta >= M_PI) theta -= M_2PI;
    if (theta <= -M_PI) theta += M_2PI;

    w *= gaussianPDF (state[2], theta, 180/RAD_T_DEG ) * 1e6;

  }

  return w;

}