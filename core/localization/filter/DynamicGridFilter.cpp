// #include <localization/filter/DynamicGridFilter.hh>
#include "DynamicGridFilter.hh"

void DynamicGridFilter::init (double _thr_l, double _thr_h) {
  
  cout << "\n\n\nDynamic filter initialized. " << _thr_h <<  "\n\n\n";

  thr_l = _thr_l;
  thr_h = _thr_h;

  double sz = 1;
  vector<state_t> bound(2, state_t(get_dim()));
  for (int i = 0; i < get_dim(); ++i) {
    bound[LOWER][i] = get_bound(LOWER, i);
    bound[UPPER][i] = get_bound(UPPER, i);
    sz *= (bound[UPPER][i] - bound[LOWER][i]);
  }
  root = create_node ( 1./sz, bound );


  // cdf.resize(get_total_size());
  // CDF I/O
  // name = "dynamic-grid-" + std::to_string(thr_l) + "-" + std::to_string(thr_h) + ".csv";
  // string path = get_test_path() + "cdf-" + name;
  // cdf_fid.open(path);
  // path = get_test_path() + "pdf-" + name;
  // pdf_fid.open(path);
}


void DynamicGridFilter::process () {
  // clock_start ();

  cache_.localization_mem->particles.resize(100);
  
  // for (int i = 0; i < 50; ++i) cout << "Step 2  \n";
  // Sensor+Motion update
  motion_u = state_t(get_dim(), 0);
  motion_u = motion_update (motion_u);

  // for (int i = 0; i < 50; ++i) cout << "Step 3  \n";
  old_root = replicate_node (root);

  // for (int i = 0; i < 50; ++i) cout << "Step 4  \n";
  update_node (root);

  // for (int i = 0; i < 50; ++i) cout << "Step 5  \n";

  // Normalizing.
  // NOTE normalizing before restructure to make
  // sure that probabilities 1.
  // for (int i = 0; i < 50; ++i) cout << "Step 6  \n";

  double sum = get_total_weight (root);
  cout << "Sum before normalize = " << sum << endl;

  // for (int i = 0; i < 50; ++i) cout << "Step 7  \n";
  normalize (root, sum);

  // for (int i = 0; i < 50; ++i) cout << "Step 8  \n";
  // sum = get_total_weight (root);
  // cout << "Sum after normalize = " << sum << endl;

  // for (int i = 0; i < 50; ++i) cout << "Step 9  \n";

  restructure_node (root);

  // for (int i = 0; i < 50; ++i) cout << "Step 10  \n";
  destroy_node (old_root);

  // for (int i = 0; i < 50; ++i) cout << "Step 11  \n";

  // clock_stop ();

  // TEMP.Just printing.
  // sum = get_total_weight (root);
  // cout << "Sum after restructure = " << sum << endl;

  // for (int i = 0; i < 50; ++i) cout << "Step 12 \n";
  state_t max_state(get_dim());
  double max_wt = get_max_state (root, max_state);

  // for (int i = 0; i < 50; ++i) cout << "Step 13  \n";

  // Nao
  if (max_wt > 1e-7) {
    assert (max_state.size() == 3);
    current.x = max_state[0];
    current.y = max_state[1];
    current.t = max_state[2];
  } else {
    cout << max_wt << endl;
  }
  // store_cdf ();
}



DynamicGridFilter::node* DynamicGridFilter::create_node (
  double wt, vector<state_t> bound) {
  // Should set values, specifically bounds in caller function.
  node* x = new node;
  x->w = wt;
  x->dim_bounds = bound;
  x->s = 1;
  for (int i = 0; i < get_dim(); ++i) {
    x->s *= (bound[UPPER][i] - bound[LOWER][i]);
  }

  // TEMP
  // cout << "New node created.\n";
  // cout << "\tWeight = " << x->w * x->s;
  // cout << "\tBounds: ";
  // for (int i = 0; i < get_dim(); ++i )
  //   cout << " (" << x->dim_bounds[LOWER][i] << " , " << x->dim_bounds[UPPER][i]<< " ) ";

  return x;
}


void DynamicGridFilter::update_node (node* x) {

  // cout << "update_node" << endl;
  x->w = get_weight (x->dim_bounds);

  for (auto &child : x->ch) {
    update_node (child);
  }
}

void DynamicGridFilter::restructure_node (node* x) {

  // bool allow_break = true;
  // for (int i = 0; i < get_dim(); ++i) {
  //   if (x->dim_bounds[UPPER][i] - x->dim_bounds[LOWER][i] 
  //     < interval_size_lower_limit) {
  //     allow_break = false;
  //     break;
  //   }
  // }

  /*
  TODO. Currently using only one threshold thr_h
  else if (x->w < 0 and thr_h > wt and wt >= thr_l) {
    // Maintaining situation.
    x->w = -1;
    for (auto &child : x->ch) {
      restructure_node (child);
    }
  }
  */

  // TEMP
  // cout << "restructure_node.\n";
  // cout << "\tWeight = " << x->w * x->s;
  // cout << "\tBounds: ";
  // for (int i = 0; i < get_dim(); ++i )
  //   cout << " (" << x->dim_bounds[LOWER][i] << " , " << x->dim_bounds[UPPER][i]<< " ) ";

  if (x->w * x->s >= thr_h and x->s > interval_size_lower_limit) {
    // Breaking node.
    // Create children if not present.
    if (x->ch.size() == 0) {
      auto bound = x->dim_bounds;
      restructure_node_helper (x, bound, 0);
    }

    for (auto &child : x->ch) {
      restructure_node (child);
    }
  }
  else {
    // TEMP
    if ( x->s <= interval_size_lower_limit ) {
      cout << "Hit interval size lower limit.";
    }
    // Joining node.
    // Deleting children if any.
    for (auto &child : x->ch) {
      destroy_node (child);
    }
    x->ch.clear ();
  }

}

void DynamicGridFilter::restructure_node_helper (
  node *x, vector<state_t> &bound, int n ) {

  // TEMP
  // cout << "restructure_node_helper.\n";
  // cout << "\tWeight = " << x->w * x->s;
  // cout << "\tBounds: ";
  // for (int i = 0; i < get_dim(); ++i )
  //   cout << " (" << x->dim_bounds[LOWER][i] << " , " << x->dim_bounds[UPPER][i]<< " ) ";

  if (n == get_dim()) {
    node *y = create_node (x->w, bound);
    x->ch.push_back (y);
  }
  else {
    double med = (x->dim_bounds[LOWER][n] + x->dim_bounds[UPPER][n]) / 2;
    bound[LOWER][n] = x->dim_bounds[LOWER][n];
    bound[UPPER][n] = med;
    restructure_node_helper (x, bound, n+1);

    bound[LOWER][n] = med;
    bound[UPPER][n] = x->dim_bounds[UPPER][n];
    restructure_node_helper (x, bound, n+1);
  }
}



double DynamicGridFilter::get_weight (
  const vector<state_t> &bound) {
  
  // cout << "get_weight \n";
  vector<vector<double>> points(get_dim());

  int total_points = 0;

  for (int i = 0; i < get_dim(); ++i) {

    // int n_points = 1 + int(get_density(i) * (bound[UPPER][i] - bound[LOWER][i]));
    int n_points = 5;
    total_points += n_points;

    for (int j = 0; j < n_points; ++j) {
      points[i].push_back ( bound[LOWER][i] + 
        j * (1./n_points) * (bound[UPPER][i] - bound[LOWER][i]) );
    }
  }

  state_t state(get_dim());

  return get_weight_helper (points, state, 0) /total_points;
}

double DynamicGridFilter::get_weight_helper (const vector<vector<double>> &points,
  state_t &state, int n) {

  // cout << "get_weight_helper \n";
  if (n == get_dim()) {
    state_t old_state = state_t(get_dim());
    for (int i = 0; i < get_dim(); ++i) {
      old_state[i] = state[i] - motion_u[i];
    }
    return sensor_update (state) * get_pdf_value (old_root, old_state);
  } else {
    double ret = 0;
    for (const auto &pt : points[n]) {
      state[n] = pt;
      ret += get_weight_helper (points, state, n+1);
    }
    return ret;
  }
}


double DynamicGridFilter::get_pdf_value (
  const node* x, const state_t &state) {

  // TEMP
  // cout << "get_pdf_value.\n";
  // cout << "\tWeight = " << x->w * x->s;
  // cout << "\tBounds: ";
  // for (int i = 0; i < get_dim(); ++i )
  //   cout << " (" << x->dim_bounds[LOWER][i] << " , " << x->dim_bounds[UPPER][i]<< " ) ";

  // TODO. problematic for non 0 motion!
  if (not state_in_range(x->dim_bounds, state)) {
    // TEMP
    // cout << "Missed bound. : " << x->dim_bounds[LOWER][0];
    // cout << x->dim_bounds[UPPER][0] << endl;
    // cout << "State : " << state[0] << endl;
    return 0;
  }

  if (x->ch.size() == 0) {
    // TEMP
    // cout << "Hit bound. : " << x->dim_bounds[LOWER][0];
    // cout << " " << x->dim_bounds[UPPER][0] << endl;
    // cout << "State : " << state[0] << endl;
    return x->w;
  }

  int n = -1;
  for (int i = 0; i < x->ch.size(); ++i) {
    if (state_in_range(x->ch[i]->dim_bounds, state)) {
      n = i;
      break;
    }
  }
  assert (n >= 0);
  return get_pdf_value (x->ch[n], state);
}

bool DynamicGridFilter::state_in_range (
  const vector<state_t> &bound, const state_t &state) {

  // cout << "state_in_range" << endl;
    
    for (int i = 0; i < get_dim(); ++i) {
      // NOTE including both sides of interval.
      if (state[i] < bound[LOWER][i] || state[i] > bound[UPPER][i]) {
        return false;
      }
    }

    return true;
}


void DynamicGridFilter::store_cdf () {

  // cout << "store_cdf" << endl;

  std::fill(cdf.begin(), cdf.end(), 0);

  // Construct approximate pdf. here pdf is called cdf.
  store_cdf_helper (root);

  double sum = 0;

  // io_store_pdf (cdf);
  pdf_to_cdf (cdf);
  // io_store_cdf (cdf);

  // Nao
  generate_particles (cdf);
}

void DynamicGridFilter::store_cdf_helper (const node* x) {

  // cout << "store_cdf_helper" << endl;

  if (x->ch.size() > 0) {
    for (auto &child : x->ch) {
      store_cdf_helper (child);
    }
  } else {
    state_t state(get_dim());
    for (int i = 0; i < get_dim(); ++i) {
      state[i] = (x->dim_bounds[LOWER][i] + x->dim_bounds[UPPER][i])/2;
      // state[i] = x->dim_bounds[LOWER][i];
    }
    cdf[index_to_id( state_to_index( state ))] += x->w * x->s;
  }

}



double DynamicGridFilter::get_total_weight (const node *x) {
  
  // TEMP prints  
  // int temp; cin >> temp;
  // static int iter_depth = 0;
  // iter_depth++;
  // cout << "Iteration depth = " << iter_depth << endl;

  // for (int i = 0; i < get_dim(); ++i )
  //   cout << " (" << x->dim_bounds[LOWER][i] << " , " << x->dim_bounds[UPPER][i]<< " ) ";
  // cout << " ) , " << x->w * x->s << " )\n";

  double sum = 0;

  if (x->ch.size() > 0) {
    for (auto &child : x->ch) {
      sum += get_total_weight (child);
    }
  } else {
    sum += x->w * x->s;    
  }

  // iter_depth--;
  return sum;
}

void DynamicGridFilter::normalize (node *x, const double &sum) {

  // cout << "normalize"<< endl;
  
  x->w /= sum;
  for (auto &child : x->ch) {
    normalize (child, sum);
  }
}

double DynamicGridFilter::get_max_state (node* x, state_t &max_state) {

  const double weight_thr = 0.01;
  // TEMP
  // cout << "get_max_state.\n";
  // cout << "\tWeight = " << x->w * x->s;
  // cout << "\tBounds: ";
  // for (int i = 0; i < get_dim(); ++i )
  //   cout << " (" << x->dim_bounds[LOWER][i] << " , " << x->dim_bounds[UPPER][i]<< " ) ";

  if (x->ch.size() == 0) {
    for (int i = 0; i < get_dim(); ++i) {
      max_state[i] = (x->dim_bounds[LOWER][i] + x->dim_bounds[UPPER][i])/2;
    }
    if (x->w > weight_thr)
      return x->w;
    else
      return x->w;
  } else {
    double max_wt = 0, temp_wt;
    state_t temp_state(get_dim());
    for (const auto &child : x->ch) {
      temp_wt = get_max_state (child, temp_state);
      if (temp_wt > max_wt) {
        max_wt = temp_wt;
        max_state = temp_state;
      }
    }
    return max_wt;
  }
}



void DynamicGridFilter::destroy () {
  // cout << "DynamicGridFilter::destroy " << endl;
  destroy_node (root);
  // pdf_fid.close();
  // cdf_fid.close();
}

void DynamicGridFilter::destroy_node (node* x) {
  
  for (auto &child : x->ch) {
    destroy_node (child);
  }

  // TEMP
  // cout << "Node destroyed.\n";
  // cout << "\tWeight = " << x->w * x->s;
  // cout << "\tBounds: ";
  // for (int i = 0; i < get_dim(); ++i )
  //   cout << " (" << x->dim_bounds[LOWER][i] << " , " << x->dim_bounds[UPPER][i]<< " ) ";

  delete x;
}

DynamicGridFilter::node* DynamicGridFilter::replicate_node (node* x) {

  // TEMP
  // cout << "Replicating node.\n";
  // cout << "\tWeight = " << x->w * x->s;
  // cout << "\tBounds: ";
  // for (int i = 0; i < get_dim(); ++i )
  //   cout << " (" << x->dim_bounds[LOWER][i] << " , " << x->dim_bounds[UPPER][i]<< " ) ";

  node *y = create_node (x->w, x->dim_bounds);
  for (const auto& child : x->ch) {
    y->ch.push_back ( replicate_node(child) );
  }
  return y;
}