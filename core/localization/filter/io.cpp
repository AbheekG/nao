#include "filter.hh"

/*
Input/Output.
Plot data and function estimated error related.
*/

void Filter::io_init () {

	string path;

	n_epoch = 10;
	n_dim = 3;

	dim_size.resize(n_dim);
	dim_size[0] = 51;
	dim_size[1] = 51;
	dim_size[2] = 21;
	
	dim_bounds.resize(2);
	dim_bounds[LOWER].resize(n_dim);
	dim_bounds[UPPER].resize(n_dim);
	dim_bounds[LOWER][0] = -2500; dim_bounds[UPPER][0] = 2500;
	dim_bounds[LOWER][1] = -1250; dim_bounds[UPPER][1] = 1250;
	dim_bounds[LOWER][2] = -M_PI; dim_bounds[UPPER][2] = M_PI;

	dim_density.resize(n_dim);
	for (int i = 0; i < n_dim; ++i) {
		dim_density[i] = dim_size[i] / (dim_bounds[UPPER][i] - dim_bounds[LOWER][i]);
	}

	total_size = 1;
	for (auto dim : dim_size) {
		total_size *= dim;
	}
	
	domain_points.resize(n_dim);
	for (int i = 0; i < n_dim; ++i) {
		domain_points[i].resize (dim_size[i]);
		for (int j = 0; j < dim_size[i]; ++j) {
			domain_points[i][j] = dim_bounds[LOWER][i] + (dim_bounds[UPPER][i] - dim_bounds[LOWER][i]) * (1./ (dim_size[i]-1)) * j;
		}
	}

}

void Filter::io_destroy () {
	motion_fid.close ();
	sensor_fid.close ();
}

void Filter::io_store_cdf (vector<double> &cdf) {
	for (int i = 0; i < cdf.size(); i++) {
		cdf_fid << cdf[i];
		if (i < cdf.size() - 1) cdf_fid << " "; else cdf_fid << endl;
	}
}

void Filter::io_store_pdf (vector<double> &pdf) {
	for (int i = 0; i < pdf.size(); i++) {
		pdf_fid << pdf[i];
		if (i < pdf.size() - 1) pdf_fid << " "; else pdf_fid << endl;
	}
}