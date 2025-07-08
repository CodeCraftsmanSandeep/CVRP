#pragma once

#include "vrp-single-threaded.h"

void tsp_approx(const CVRP &vrp, std::vector<node_t> &cities, std::vector<node_t> &tour, node_t ncities) {
  node_t i, j;
  node_t ClosePt = 0;
  weight_t CloseDist;
  //~ node_t endtour=0;

  for (i = 1; i < ncities; i++)
    tour[i] = cities[i - 1];

  tour[0] = cities[ncities - 1];

  for (i = 1; i < ncities; i++) {
    //~ double ThisX = points.x_coords[tour[i-1]];
    //~ double ThisY = points.y_coords[tour[i-1]];
    weight_t ThisX = vrp.node[tour[i - 1]].x;
    weight_t ThisY = vrp.node[tour[i - 1]].y;
    CloseDist = DBL_MAX;
    for (j = ncities - 1;; j--) {
      weight_t ThisDist = (vrp.node[tour[j]].x - ThisX) * (vrp.node[tour[j]].x - ThisX);
      if (ThisDist <= CloseDist) {
        ThisDist += (vrp.node[tour[j]].y - ThisY) * (vrp.node[tour[j]].y - ThisY);
        if (ThisDist <= CloseDist) {
          if (j < i)
            break;
          CloseDist = ThisDist;
          ClosePt = j;
        }
      }
    }
    /*swapping tour[i] and tour[ClosePt]*/
    unsigned temp = tour[i];
    tour[i] = tour[ClosePt];
    tour[ClosePt] = temp;
  }
}


std::vector<std::vector<node_t>>
postprocess_tsp_approx(const CVRP &vrp, const std::vector<std::vector<node_t>> &solRoutes) {
  std::vector<std::vector<node_t>> modifiedRoutes;

  unsigned nroutes = solRoutes.size();
  for (unsigned i = 0; i < nroutes; ++i) {
    // postprocessing solRoutes[i]
    unsigned sz = solRoutes[i].size();

    std::vector<node_t> cities(sz + 1);
    std::vector<node_t> tour(sz + 1);

    for (unsigned j = 0; j < sz; ++j)
      cities[j] = solRoutes[i][j];

    cities[sz] = 0;  // the last node is the depot.

    tsp_approx(vrp, cities, tour, sz + 1);

    // the first element of the tour is now the depot. So, ignore tour[0] and insert the rest into the vector.

    std::vector<node_t> curr_route;
    for (unsigned kk = 1; kk < sz + 1; ++kk) {
      curr_route.push_back(tour[kk]);
    }

    modifiedRoutes.push_back(curr_route);
  }
  return modifiedRoutes;
}

void tsp_2opt(const CVRP &vrp, std::vector<node_t> &cities, std::vector<node_t> &tour, unsigned ncities) {
  // 'cities' contains the original solution. It is updated during the course of the 2opt-scheme to contain the 2opt soln.
  // 'tour' is an auxillary array.

  // repeat until no improvement is made
  unsigned improve = 0;

  while (improve < 2) {
    double best_distance = 0.0;

    best_distance += vrp.get_distance_on_the_fly(vrp.depot, cities[0]);  // computing distance of the first point in the route with the depot.

    for (unsigned jj = 1; jj < ncities; ++jj) {
      best_distance += vrp.get_distance_on_the_fly(cities[jj - 1], cities[jj]);
    }

    best_distance += vrp.get_distance_on_the_fly(vrp.depot, cities[ncities - 1]);
    // 1x 2x 3x 4 5
    //  1 2  3  4 5
    for (unsigned i = 0; i < ncities - 1; i++) {
      for (unsigned k = i + 1; k < ncities; k++) {
        for (unsigned c = 0; c < i; ++c) {
          tour[c] = cities[c];
        }

        unsigned dec = 0;
        for (unsigned c = i; c < k + 1; ++c) {
          tour[c] = cities[k - dec];
          dec++;
        }

        for (unsigned c = k + 1; c < ncities; ++c) {
          tour[c] = cities[c];
        }
        double new_distance = 0.0;

        new_distance += vrp.get_distance_on_the_fly(vrp.depot, tour[0]);
        for (unsigned jj = 1; jj < ncities; ++jj) {
          new_distance += vrp.get_distance_on_the_fly(tour[jj - 1], tour[jj]);
        }

        new_distance += vrp.get_distance_on_the_fly(vrp.depot, tour[ncities - 1]);

        if (new_distance < best_distance) {
          // Improvement found so reset
          improve = 0;
          for (unsigned jj = 0; jj < ncities; jj++)
            cities[jj] = tour[jj];
          best_distance = new_distance;
        }
      }
    }
    improve++;
  }
}

std::vector<std::vector<node_t>>
postprocess_2OPT(const CVRP &vrp, const std::vector<std::vector<node_t>> &final_routes) {
  std::vector<std::vector<node_t>> postprocessed_final_routes;

  unsigned nroutes = final_routes.size();
  for (unsigned i = 0; i < nroutes; ++i) {
    // postprocessing final_routes[i]
    unsigned sz = final_routes[i].size();

    std::vector<node_t> cities(sz);
    std::vector<node_t> tour(sz);

    for (unsigned j = 0; j < sz; ++j)
      cities[j] = final_routes[i][j];

    std::vector<node_t> curr_route;

    if (sz > 2)                         // for sz <= 1, the cost of the path cannot change. So no point running this.
      tsp_2opt(vrp, cities, tour, sz);  //MAIN

    for (unsigned kk = 0; kk < sz; ++kk) {
      curr_route.push_back(cities[kk]);
    }

    postprocessed_final_routes.push_back(curr_route);
  }
  return postprocessed_final_routes;
}

//
// MAIN POST PROCESS ROUTINE
//
std::vector<std::vector<node_t>>
postProcessIt(const CVRP &vrp, const std::vector<std::vector<node_t>> &final_routes, double& final_cost) {
  std::vector<std::vector<node_t>> postprocessed_final_routes;

  auto postprocessed_final_routes1 = postprocess_tsp_approx(vrp, final_routes);
  auto postprocessed_final_routes2 = postprocess_2OPT(vrp, postprocessed_final_routes1);
  auto postprocessed_final_routes3 = postprocess_2OPT(vrp, final_routes);

  //~ weight_t postprocessed_final_routes_cost;
  double min_cost = 0.0;
  for (unsigned zzz = 0; zzz < final_routes.size(); ++zzz) {
    // include the better route between postprocessed_final_routes2[zzz] and postprocessed_final_routes3[zzz] in the final solution.
    std::vector<node_t> preprocessed_route = final_routes[zzz];
    std::vector<node_t> postprocessed_route2 = postprocessed_final_routes2[zzz];
    std::vector<node_t> postprocessed_route3 = postprocessed_final_routes3[zzz];

    unsigned sz = preprocessed_route.size();
    unsigned sz2 = postprocessed_route2.size();
    unsigned sz3 = postprocessed_route3.size();

    if(sz != sz2 || sz != sz3)
    {
      HANDLE_ERROR("Sizes are not matching for route in prost processing");
    }
    // finding the cost of preprocessed_route
    weight_t preprocessed_route_cost = 0.0;
    preprocessed_route_cost += vrp.get_distance_on_the_fly(vrp.depot, preprocessed_route[0]);
    for(int i = 1; i < sz; i++){
      preprocessed_route_cost += vrp.get_distance_on_the_fly(preprocessed_route[i-1], preprocessed_route[i]);
    }
    preprocessed_route_cost += vrp.get_distance_on_the_fly(preprocessed_route[sz-1], vrp.depot);

    // finding the cost of postprocessed_route2
    weight_t postprocessed_route2_cost = 0.0;
    postprocessed_route2_cost += vrp.get_distance_on_the_fly(vrp.depot, postprocessed_route2[0]);  // computing distance of the first point in the route with the depot.
    for (unsigned jj = 1; jj < sz2; ++jj) {
      postprocessed_route2_cost += vrp.get_distance_on_the_fly(postprocessed_route2[jj - 1], postprocessed_route2[jj]);
    }
    postprocessed_route2_cost += vrp.get_distance_on_the_fly(postprocessed_route2[sz2 - 1], vrp.depot);

    // finding the cost of postprocessed_route3
    weight_t postprocessed_route3_cost = 0.0;
    postprocessed_route3_cost += vrp.get_distance_on_the_fly(vrp.depot, postprocessed_route3[0]);
    for (unsigned jj = 1; jj < sz3; ++jj) {
      postprocessed_route3_cost += vrp.get_distance_on_the_fly(postprocessed_route3[jj - 1], postprocessed_route3[jj]);
    }
    postprocessed_route3_cost += vrp.get_distance_on_the_fly(postprocessed_route3[sz3 - 1], vrp.depot);

    // postprocessed_route2_cost is lower
    if (std::min(postprocessed_route3_cost, preprocessed_route_cost) >= postprocessed_route2_cost) {
      postprocessed_final_routes.push_back(postprocessed_route2);
      min_cost += postprocessed_route2_cost;
    }
    // postprocessed_route3_cost is lower
    else if(std::min(postprocessed_route2_cost, preprocessed_route_cost) >= postprocessed_route3_cost) {
      postprocessed_final_routes.push_back(postprocessed_route3);
      min_cost += postprocessed_route3_cost;
    }else{
      postprocessed_final_routes.push_back(preprocessed_route);
      min_cost += preprocessed_route_cost;
    }
  }
  final_cost = min_cost;
  return postprocessed_final_routes;
}



bool verify_sol(const CVRP &vrp, std::vector<std::vector<node_t>> final_routes, unsigned capacity) {
  /* verifies if the solution is valid or not */
  /**
   * 1. All vertices appear in the solution exactly once.
   * 2. For every route, the capacity constraint is respected.
   **/

  unsigned *hist = (unsigned *)malloc(sizeof(unsigned) * vrp.size);
  memset(hist, 0, sizeof(unsigned) * vrp.size);

  for (unsigned i = 0; i < final_routes.size(); ++i) {
    unsigned route_sum_of_demands = 0;
    for (unsigned j = 0; j < final_routes[i].size(); ++j) {
      //~ route_sum_of_demands += points.demands[final_routes[i][j]];
      route_sum_of_demands += vrp.node[final_routes[i][j]].demand;
      hist[final_routes[i][j]] += 1;
    }
    if (route_sum_of_demands > capacity) {
      return false;
    }
  }

  for (unsigned i = 1; i < vrp.size; ++i) {
    if (hist[i] > 1) {
      return false;
    }
    if (hist[i] == 0) {
      return false;
    }
  }
  return true;
}
