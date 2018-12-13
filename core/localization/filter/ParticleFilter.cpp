// #include "filter.hh"
// #include "ParticleFilter.hh"
// #include <algorithm>

// // size is the number of particles in each dimension
// void ParticleFilter::init (int n_part) {

//   n_particles = n_part;
//   particles.resize(n_particles);
//   for (auto &particle : particles) {
//     particle.resize(get_dim());
//   }

//   // Init Particles.
//   init_particles ();

//   cdf.resize(get_total_size());
//    // CDF I/O
//   name = "particle-filter-" + std::to_string (n_part) + ".csv";
//   string path = get_test_path() + "cdf-" + name;
//   cdf_fid.open(path);
//   path = get_test_path() + "pdf-" + name;
//   pdf_fid.open(path);
// }

// void ParticleFilter::init_particles () {
//   for (int i_dim = 0; i_dim < get_dim(); ++i_dim) {
//     std::uniform_real_distribution<double> distribution(get_bound(LOWER, i_dim), get_bound(UPPER, i_dim));
//     for (auto &particle : particles) {
//       particle[i_dim] = distribution(generator);
//     }
//   }
// }

// void ParticleFilter::process () {
//   clock_start ();

//   static std::normal_distribution<double> norm_d(0, 1./10);
//   vector<double> weights(n_particles, 0);
//   vector<state_t> old_particles(n_particles);

//   // NEW. Perturbation.
//   // Motion update
//   for (int i = 0; i < n_particles; ++i) {
//     old_particles[i] = motion_update (particles[i]);
//     for (auto &op : old_particles[i]) op *= 1 + norm_d(generator);
//   }

//   // Sensor update
//   // Calculating weights
//   for (int i = 0; i < n_particles; ++i) {
//     weights[i] = sensor_update (old_particles[i]);
//   }
  
//   for (int i = 1; i < n_particles; ++i) {
//     weights[i] += weights[i-1];
//   }

//   // Resampling.
//   std::uniform_real_distribution<double> unif_d(0, weights[n_particles-1]);
//   for (int i = 1; i < n_particles; ++i) {
//     double rand = unif_d(generator);
//     int j = std::lower_bound (weights.begin(), weights.end(), rand) - weights.begin();
//     particles[i] = old_particles[j];
//   }

//   clock_stop ();  
// }

// void ParticleFilter::store_cdf () {
  
//   std::fill(cdf.begin(), cdf.end(), 0);

//   // Construct approximate pdf. here pdf is called cdf.
//   for (const auto &p : particles) {
//     cdf[index_to_id(state_to_index (p))] += 1.0/n_particles;
//   }

//   io_store_pdf (cdf);
//   pdf_to_cdf (cdf);
//   io_store_cdf (cdf);
// }

// void ParticleFilter::destroy () {
//   pdf_fid.close();
//   cdf_fid.close();
// }