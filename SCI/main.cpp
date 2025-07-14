#include <iostream>
#include <fstream>
#include <cstring>
#include <cstdlib>
#include <assert.h>
#include <cmath>
#include <algorithm>
#include <stack>
#include <vector>
#include <climits>
#include <unordered_map>
#include <set>
#include <cfloat>
#include <random>
#include <getopt.h>
#include <cmath>
#include <chrono>
#define PI 3.1415926535897932384626433832795028841971693993751
using namespace std;
typedef tuple<double,unsigned,unsigned> order_tuple;
class Edge {
  public:
    unsigned to;
    double length;
    Edge(){}
    ~Edge(){}
    Edge(unsigned t, double l){
      to = t; length = l;
    }
    bool operator < (const Edge& e){
      return length < e.length;
    }
};
struct DirVec{
  double x;
  double y;
};
struct Points{
  unsigned dimension;
  double * x_coords;
  double * y_coords;
  unsigned * demands;
  double *pairwise_dist;
  Points (void);
  unsigned read (string filename);
  void cal_pairwise_distances();
  double L2_dist (unsigned node1, unsigned node2);
  double L2_dist_squared (unsigned node1, unsigned node2);
};
Points :: Points () {
  dimension = 0u;
  x_coords = nullptr;
  y_coords = nullptr;
  demands = nullptr;
  pairwise_dist = nullptr;
}
unsigned Points :: read (string filename) {
  ifstream in(filename);
  if(!in.is_open()) {
    std::cerr << "Could not open the file \"" << filename << "\"" << std::endl;
    exit(1);
  }
  string line;
  for(int i=0; i<4; ++i)
    getline (in, line);
  string dim_string = line.substr(line.find(":")+2);
  dimension = atoi(dim_string.c_str());
  getline (in, line);
  getline (in, line);
  string cap_string = line.substr(line.find(":")+2);
  unsigned capacity = atoi(cap_string.c_str());
  x_coords = (double*) malloc (sizeof(double) * dimension);
  y_coords = (double*) malloc (sizeof(double) * dimension);
  demands = (unsigned*) malloc (sizeof(unsigned) * dimension);
  getline (in, line);
  unsigned temp;
  double depot_x_coord, depot_y_coord;
  getline (in, line);
  sscanf(line.c_str(), "%d %lf %lf", &temp, &depot_x_coord, &depot_y_coord);
  x_coords[0] = 0.0;
  y_coords[0] = 0.0;
  double x_temp, y_temp;
  for(unsigned i = 1; i < dimension; ++i) {
    getline (in, line);
    sscanf(line.c_str(), "%d %lf %lf", &temp, &x_temp, &y_temp);
    x_coords[i] = x_temp - depot_x_coord;
    y_coords[i] = y_temp - depot_y_coord;
  }
  getline (in, line);
  for (unsigned i=0; i < dimension; ++i) {
    getline (in, line);
    sscanf(line.c_str(), "%d %d", &temp, &demands[i]);
  }
  return capacity;
}
void Points :: cal_pairwise_distances() {
  pairwise_dist = (double*) malloc(sizeof(double) * (dimension*(dimension-1))/2);
  unsigned k = 0;
  for(unsigned i=0; i < dimension; ++i){
    for(unsigned j=i+1; j < dimension; ++j){
      double w = sqrt( ((x_coords[i] - x_coords[j]) * (x_coords[i] - x_coords[j])) + ((y_coords[i] - y_coords[j]) * (y_coords[i] - y_coords[j])));
      pairwise_dist[k]= w;
      k++;
    }
  }
}
bool compare_tuple1 (const order_tuple &lhs, const order_tuple &rhs){
  bool result = (get<0>(lhs) < get<0>(rhs)) || ((get<0>(lhs) == get<0>(rhs) && (get<1>(lhs) > get<1>(rhs))));
  return result;
}
double Points :: L2_dist (unsigned node1, unsigned node2) {
  if (node1 == node2) return 0.0;
  if (node1 > node2) {
    node1 = node1 ^ node2;
    node2 = node1 ^ node2;
    node1 = node1 ^ node2;
  }
  unsigned myoffset = node1 * dimension - 1 - (node1 * (node1 + 3))/2 + node2;
  return pairwise_dist[myoffset];
}
double L2_dist (double x1, double y1, double x2, double y2) {
  double x_diff = x1 - x2;
  double y_diff = y1 - y2;
  double dist = sqrt ((x_diff * x_diff) + (y_diff * y_diff));
  return dist;
}
double Points :: L2_dist_squared (unsigned node1, unsigned node2) {
  if (node1 == node2) return 0.0;
  if (node1 > node2) {
    node1 = node1 ^ node2;
    node2 = node1 ^ node2;
    node1 = node1 ^ node2;
  }
  unsigned myoffset = node1 * dimension - 1 - (node1 * (node1 + 3))/2 + node2;
  return pairwise_dist[myoffset] * pairwise_dist[myoffset] ;
}
double L2_dist_squared (double x1, double y1, double x2, double y2) {
  double x_diff = x1 - x2;
  double y_diff = y1 - y2;
  double dist_squared = (x_diff * x_diff) + (y_diff * y_diff);
  return dist_squared;
}
void get_distances_from_depot (Points& points, double* distances_from_depot) {
  for(unsigned i=1; i < points.dimension; ++i) {
    distances_from_depot[i-1] = points.L2_dist_squared(0,i);
  }
}
void shuffle (unsigned * V, unsigned N) {
  mt19937_64 prng(0 );
  uniform_real_distribution<> distribution( 0.0, 1.0 );
  for(unsigned i = 0; i < N; ++i) {
    unsigned offset = (unsigned)( distribution( prng ) * (N-i) );
    unsigned x = i + offset;
    unsigned temp = V[i];
    V[i] = V[x];
    V[x] = temp;
  }
}
void reorder_nodes (Points& points, double* distances_from_depot, unsigned* node_order) {
  vector<order_tuple> vec (points.dimension-1);
  for(unsigned i=0; i < points.dimension-1; ++i) {
    vec[i] = make_tuple(distances_from_depot[i],points.demands[i+1],i+1);
  }
  sort(vec.begin(), vec.end(), compare_tuple1);
  for(unsigned i=0; i < points.dimension-1; ++i) {
    node_order[i] = get<2>(vec[i]);
  }
}
bool areClockwise(DirVec& v1, DirVec& v2) {
  return (-v1.x * v2.y + v1.y * v2.x) > 0;
}
bool isInRegion(DirVec& v1, DirVec& v2, DirVec& v3) {
  if(areClockwise(v1, v2))
    return (!areClockwise(v2, v3) && areClockwise(v1, v3));
  else if(areClockwise(v2, v1))
    return (!areClockwise(v1, v3) && areClockwise(v2, v3));
  else return false;
}
double get_cost_of_route (vector <unsigned> curr_route, Points& points) {
  double curr_route_cost = 0;
  curr_route_cost += points.L2_dist(0,curr_route[0]);
  unsigned curr_route_size = curr_route.size();
  for (unsigned jj = 1; jj < curr_route_size; ++jj) {
    curr_route_cost += points.L2_dist(curr_route[jj-1], curr_route[jj]);
  }
  curr_route_cost += points.L2_dist(0,curr_route[curr_route_size-1]);
  return curr_route_cost;
}
void tsp_approx (unsigned* cities, unsigned* tour, Points& points, unsigned ncities) {
  unsigned i,j;
  unsigned ClosePt=0;
  double CloseDist;
  unsigned endtour=0;
  for (i=1; i < ncities; i++)
    tour[i]=cities[i-1];
  tour[0] = cities[ncities-1];
  for (i=1; i < ncities; i++) {
    double ThisX = points.x_coords[tour[i-1]];
    double ThisY = points.y_coords[tour[i-1]];
    CloseDist = DBL_MAX;
    for (j = ncities-1; ; j--) {
      double ThisDist = (points.x_coords[tour[j]]-ThisX) * (points.x_coords[tour[j]]-ThisX);
      if (ThisDist <= CloseDist) {
        ThisDist += (points.y_coords[tour[j]]-ThisY) * (points.y_coords[tour[j]]-ThisY);
        if (ThisDist <= CloseDist) {
          if (j < i)
            break;
          CloseDist = ThisDist;
          ClosePt = j;
        }
      }
    }
    unsigned temp = tour[i];
    tour[i] = tour[ClosePt];
    tour[ClosePt] = temp;
  }
}
vector<unsigned> single_tsp_approx (vector<unsigned>& route, Points& points) {
  unsigned sz = route.size();
  unsigned* cities = (unsigned*) malloc(sizeof(unsigned) * (sz+1));
  unsigned* tour = (unsigned*) malloc(sizeof(unsigned) * (sz+1));
  for(unsigned j = 0; j < sz; ++j)
    cities[j] = route[j];
  cities[sz] = 0;
  tsp_approx (cities, tour, points, sz+1);
  vector<unsigned> curr_route;
  for(unsigned kk = 1; kk < sz+1; ++kk) {
    curr_route.push_back(tour[kk]);
  }
  return curr_route;
}
void tsp_2opt (unsigned* cities, unsigned* tour, Points& points, unsigned ncities) {
#if 1
  unsigned* cities_working_copy = (unsigned*) malloc(sizeof(unsigned) * (ncities+2));
  cities_working_copy[0] = 0;
  cities_working_copy[ncities+1] = 0;
  for (unsigned i = 0; i < ncities; ++i) {
    cities_working_copy[i+1] = cities[i];
  }
  unsigned improve = 0;
  double minchange;
  while ( improve < 1)
  {
    minchange = 0;
    unsigned mini, minj;
    for ( unsigned i = 0; i < ncities; i++ )
    {
      minchange += points.L2_dist(cities_working_copy[i], cities_working_copy[i+1]);
      for ( unsigned j = i + 2; j < ncities+1; j++)
      {
        double change = points.L2_dist(cities_working_copy[i], cities_working_copy[j]) + points.L2_dist(cities_working_copy[i+1], cities_working_copy[j+1]) - points.L2_dist(cities_working_copy[j], cities_working_copy[j+1]);
        if (minchange - change > 0.000001 ) {
          minchange = change;
          mini = i;
          minj = j;
        }
      }
      minchange -= points.L2_dist(cities_working_copy[i], cities_working_copy[i+1]);
    }
    improve++;
    if (minchange < -0.1) {
      improve = 0;
      unsigned start = mini+1;
      unsigned end = minj ;
      while (start < end) {
        unsigned to = cities_working_copy[end];
        cities_working_copy[end] = cities_working_copy[start];
        cities_working_copy[start] = to;
        start++;
        end--;
      }
    }
  }
  for (unsigned i=0; i < ncities; ++i) {
    cities[i] = cities_working_copy[i+1];
  }
#endif
}
vector<unsigned> single_2OPT (vector<unsigned>& route, Points& points) {
  unsigned sz = route.size();
  unsigned* cities = (unsigned*) malloc(sizeof(unsigned) * (sz));
  unsigned* tour = (unsigned*) malloc(sizeof(unsigned) * (sz));
  for(unsigned j = 0; j < sz; ++j)
    cities[j] = route[j];
  vector<unsigned> curr_route;
  if(sz > 2)
    tsp_2opt (cities, tour, points, sz);
  for(unsigned kk = 0; kk < sz; ++kk) {
    curr_route.push_back(cities[kk]);
  }
  return curr_route;
}
double get_best_position_in_route (vector<unsigned> curr_route, Points& points, unsigned& curr_pos, unsigned v) {
  double min_increase_in_cost = DBL_MAX;
  unsigned pos;
  double dist1, dist2, dist_edge_removed;
  unsigned curr_route_size = curr_route.size();
  for(unsigned i=0; i < curr_route_size; ++i) {
    dist1 = points.L2_dist(curr_route[i], v);
    if (i == (curr_route_size - 1)) {
      dist2 = points.L2_dist(0, v);
      dist_edge_removed = points.L2_dist(0,curr_route[i]);
    }
    else {
      dist2 = points.L2_dist(v, curr_route[i+1]);
      dist_edge_removed = points.L2_dist(curr_route[i],curr_route[i+1]);
    }
    double new_increase_in_cost = dist1 + dist2 - dist_edge_removed;
    if (new_increase_in_cost < min_increase_in_cost) {
      min_increase_in_cost = new_increase_in_cost;
      pos = i;
    }
  }
  curr_pos = pos;
  return min_increase_in_cost;
}
unsigned get_best_position_in_route (vector<unsigned> curr_route, Points& points, unsigned& curr_pos, unsigned v, const unsigned cost_func_id ) {
  unsigned min_increase_in_cost = UINT_MAX;
  unsigned pos;
  double dist1, dist2, dist_edge_removed;
  unsigned curr_route_size = curr_route.size();
  for(unsigned i=0; i < curr_route_size; ++i) {
    dist1 = points.L2_dist_squared(curr_route[i], v);
    if (i == (curr_route_size - 1)) {
      dist2 = points.L2_dist_squared(0,v);
      dist_edge_removed = points.L2_dist_squared(0,curr_route[i]);
    }
    else {
      dist2 = points.L2_dist_squared(v,curr_route[i+1]);
      dist_edge_removed = points.L2_dist_squared(curr_route[i],curr_route[i+1]);
    }
    unsigned new_increase_in_cost;
    switch (cost_func_id) {
      case 1:
        new_increase_in_cost = (unsigned) dist1;
        break;
      case 2:
        new_increase_in_cost = (unsigned) dist2;
        break;
      case 3:
        new_increase_in_cost = (unsigned) max(dist1,dist2);
        break;
      case 4:
        new_increase_in_cost = (unsigned) min(dist1,dist2);
        break;
      case 5:
        new_increase_in_cost = (unsigned) (dist1 + dist2);
        break;
      case 6:
        new_increase_in_cost = (unsigned) (dist1*dist1 + dist2*dist2);
        break;
      case 7:
        new_increase_in_cost = (unsigned) (dist1 + dist2 - dist_edge_removed);
        break;
    }
    if (new_increase_in_cost < min_increase_in_cost) {
      min_increase_in_cost = new_increase_in_cost;
      pos = i;
    }
  }
  curr_pos = pos;
  return min_increase_in_cost;
}
vector <unsigned> get_route (Points &points, unsigned* node_order, unsigned* shuffled_order, const unsigned capacity, unsigned& num_nodes_included, bool * isIncluded, const unsigned cost_func_id, unsigned cone_angle) {
  vector<unsigned> curr_route;
  unsigned route_capacity_remaining = capacity;
  for(unsigned i=0; i < points.dimension-1; ++i) {
    unsigned v = node_order[i];
    if (!isIncluded[v]) {
      curr_route.push_back(v);
      route_capacity_remaining -= points.demands[v];
      isIncluded[v] = true;
      num_nodes_included += 1;
      break;
    }
  }
  unsigned first_node_of_route = curr_route[0];
  DirVec dir_vec1 ;
  double v_magnitude = sqrt((points.x_coords[first_node_of_route] * points.x_coords[first_node_of_route]) + (points.y_coords[first_node_of_route] * points.y_coords[first_node_of_route]));
  dir_vec1.x = points.x_coords[first_node_of_route]/v_magnitude;
  dir_vec1.y = points.y_coords[first_node_of_route]/v_magnitude;
  double theta1_degree = cone_angle;
  double theta2_degree = cone_angle;
  double theta1_rad = theta1_degree * (PI/180);
  double theta2_rad = theta2_degree * (PI/180);
  DirVec dir_vec2;
  double x_temp = dir_vec1.x * cos(theta1_rad) - dir_vec1.y * sin(theta1_rad);
  double y_temp = dir_vec1.x * sin(theta1_rad) + dir_vec1.y * cos(theta1_rad);
  v_magnitude = sqrt((x_temp * x_temp) + (y_temp * y_temp));
  dir_vec2.x = x_temp / v_magnitude;
  dir_vec2.y = y_temp / v_magnitude;
  DirVec dir_vec3;
  x_temp = dir_vec1.x * cos(theta2_rad) + dir_vec1.y * sin(theta2_rad);
  y_temp = -1.0 * dir_vec1.x * sin(theta2_rad) + dir_vec1.y * cos(theta2_rad);
  v_magnitude = sqrt((x_temp * x_temp) + (y_temp * y_temp));
  dir_vec3.x = x_temp / v_magnitude;
  dir_vec3.y = y_temp / v_magnitude;
  vector<unsigned> nodes_in_region;
  for(unsigned i=0; i < points.dimension-1; ++i) {
    unsigned p1 = shuffled_order[i];
    DirVec dir_vec4 ;
    v_magnitude = sqrt((points.x_coords[p1] * points.x_coords[p1]) + (points.y_coords[p1] * points.y_coords[p1]));
    dir_vec4.x = points.x_coords[p1]/v_magnitude;
    dir_vec4.y = points.y_coords[p1]/v_magnitude;
    bool region_flag1 = isInRegion(dir_vec1, dir_vec2, dir_vec4);
    bool region_flag2 = isInRegion(dir_vec1, dir_vec3, dir_vec4);
    if (region_flag1 || region_flag2) {
      nodes_in_region.push_back(p1);
    }
  }
  while(true) {
    unsigned least_inc = UINT_MAX;
    unsigned best_pos;
    unsigned best_node;
    unsigned flag = false;
    unsigned nodes_in_region_sz = nodes_in_region.size();
    for(unsigned ii=0; ii < nodes_in_region_sz; ++ii) {
      unsigned v = nodes_in_region[ii];
      if(isIncluded[v]) continue;
      if(route_capacity_remaining < points.demands[v]) continue;
      flag = true;
      unsigned curr_pos;
      unsigned curr_inc = get_best_position_in_route (curr_route, points, curr_pos, v, cost_func_id);
      if (curr_inc < least_inc) {
        least_inc = curr_inc;
        best_pos = curr_pos;
        best_node = v;
      }
    }
    if (!flag) break;
    auto it = curr_route.begin() + best_pos + 1;
    curr_route.insert(it,best_node);
    route_capacity_remaining -= points.demands[best_node];
    isIncluded[best_node] = true;
    num_nodes_included += 1;
    if(curr_route.size() > 2 && curr_route.size() < 30) {
      vector<unsigned> curr_route1 = single_2OPT(curr_route, points);
      vector<unsigned> curr_route2 = single_tsp_approx(curr_route, points);
      vector<unsigned> curr_route3 = single_2OPT(curr_route2, points);
      double curr_length = get_cost_of_route (curr_route, points);
      double curr1_length = get_cost_of_route (curr_route1, points);
      double curr3_length = get_cost_of_route (curr_route3, points);
      double curr13_length = DBL_MAX;
      vector<unsigned> curr13_route;
      if (curr3_length < curr1_length) {
        curr13_length = curr3_length;
        curr13_route = curr_route3;
      }
      else {
        curr13_length = curr1_length;
        curr13_route = curr_route1;
      }
      if (curr13_length < curr_length)
        curr_route = curr13_route;
    }
  }
  return curr_route;
}
vector <unsigned> get_route_nearest_neighbor (Points &points, unsigned* node_order, unsigned* shuffled_order, const unsigned capacity, unsigned& num_nodes_included, bool * isIncluded, unsigned cone_angle) {
  vector<unsigned> curr_route;
  unsigned first_node_in_region;
  unsigned route_capacity_remaining = capacity;
  vector<unsigned> nodes_in_region;
  if(cone_angle == 360) {
    for(unsigned i=0; i < points.dimension-1; ++i)
      nodes_in_region.push_back(shuffled_order[i]);
  }
  else {
    for(unsigned i=0; i < points.dimension-1; ++i) {
      unsigned v = node_order[i];
      if (!isIncluded[v]) {
        first_node_in_region = v;
        break;
      }
    }
    nodes_in_region.push_back(first_node_in_region);
    DirVec dir_vec1 ;
    double v_magnitude = sqrt((points.x_coords[first_node_in_region] * points.x_coords[first_node_in_region]) + (points.y_coords[first_node_in_region] * points.y_coords[first_node_in_region]));
    dir_vec1.x = points.x_coords[first_node_in_region]/v_magnitude;
    dir_vec1.y = points.y_coords[first_node_in_region]/v_magnitude;
    double theta1_degree = cone_angle;
    double theta2_degree = cone_angle;
    double theta1_rad = theta1_degree * (PI/180);
    double theta2_rad = theta2_degree * (PI/180);
    DirVec dir_vec2;
    double x_temp = dir_vec1.x * cos(theta1_rad) - dir_vec1.y * sin(theta1_rad);
    double y_temp = dir_vec1.x * sin(theta1_rad) + dir_vec1.y * cos(theta1_rad);
    v_magnitude = sqrt((x_temp * x_temp) + (y_temp * y_temp));
    dir_vec2.x = x_temp / v_magnitude;
    dir_vec2.y = y_temp / v_magnitude;
    DirVec dir_vec3;
    x_temp = dir_vec1.x * cos(theta2_rad) + dir_vec1.y * sin(theta2_rad);
    y_temp = -1.0 * dir_vec1.x * sin(theta2_rad) + dir_vec1.y * cos(theta2_rad);
    v_magnitude = sqrt((x_temp * x_temp) + (y_temp * y_temp));
    dir_vec3.x = x_temp / v_magnitude;
    dir_vec3.y = y_temp / v_magnitude;
    for(unsigned i=0; i < points.dimension-1; ++i) {
      unsigned p1 = shuffled_order[i];
      DirVec dir_vec4 ;
      v_magnitude = sqrt((points.x_coords[p1] * points.x_coords[p1]) + (points.y_coords[p1] * points.y_coords[p1]));
      dir_vec4.x = points.x_coords[p1]/v_magnitude;
      dir_vec4.y = points.y_coords[p1]/v_magnitude;
      bool region_flag1 = isInRegion(dir_vec1, dir_vec2, dir_vec4);
      bool region_flag2 = isInRegion(dir_vec1, dir_vec3, dir_vec4);
      if (region_flag1 || region_flag2) {
        nodes_in_region.push_back(p1);
      }
    }
  }
  if(curr_route.size() == 0) {
    unsigned v = 0;
    unsigned closestPt = UINT_MAX;
    double min_dist = DBL_MAX;
    unsigned nodes_in_region_sz = nodes_in_region.size();
    for(unsigned ii=0; ii < nodes_in_region_sz; ++ii) {
      unsigned w = nodes_in_region[ii];
      if(isIncluded[w]) continue;
      if(route_capacity_remaining < points.demands[w]) continue;
      double dist_vw = DBL_MAX;
      dist_vw = points.L2_dist_squared(v, w);
      if(dist_vw < min_dist) {
        min_dist = dist_vw;
        closestPt = w;
      }
    }
    curr_route.push_back(closestPt);
    route_capacity_remaining -= points.demands[closestPt];
    isIncluded[closestPt] = true;
    num_nodes_included += 1;
  }
  while(true) {
    unsigned v = curr_route[curr_route.size()-1];
    unsigned closestPt = UINT_MAX;
    bool flag = false;
    double min_dist = DBL_MAX;
    unsigned nodes_in_region_sz = nodes_in_region.size();
    for(unsigned ii=0; ii < nodes_in_region_sz; ++ii) {
      unsigned w = nodes_in_region[ii];
      if(isIncluded[w]) continue;
      if(route_capacity_remaining < points.demands[w]) continue;
      double dist_vw = DBL_MAX;
      dist_vw = points.L2_dist_squared(v,w);
      if(dist_vw < min_dist) {
        flag = true;
        min_dist = dist_vw;
        closestPt = w;
      }
    }
    if (!flag) break;
    curr_route.push_back(closestPt);
    route_capacity_remaining -= points.demands[closestPt];
    isIncluded[closestPt] = true;
    num_nodes_included += 1;
    if(curr_route.size() > 2 && curr_route.size() < 30) {
      vector<unsigned> curr_route1 = single_2OPT(curr_route, points);
      vector<unsigned> curr_route2 = single_tsp_approx(curr_route, points);
      vector<unsigned> curr_route3 = single_2OPT(curr_route2, points);
      double curr_length = get_cost_of_route (curr_route, points);
      double curr1_length = get_cost_of_route (curr_route1, points);
      double curr3_length = get_cost_of_route (curr_route3, points);
      double curr13_length = DBL_MAX;
      vector<unsigned> curr13_route;
      if (curr3_length < curr1_length) {
        curr13_length = curr3_length;
        curr13_route = curr_route3;
      }
      else {
        curr13_length = curr1_length;
        curr13_route = curr_route1;
      }
      if (curr13_length < curr_length)
        curr_route = curr13_route;
    }
  }
  return curr_route;
}
void populate_routes (Points& points, unsigned * node_order, unsigned * shuffled_order, unsigned capacity, vector<vector <unsigned> >& final_routes, const unsigned cost_func_id, unsigned cone_angle) {
  bool* isIncluded = (bool*) malloc(sizeof(bool) * points.dimension);
  memset(isIncluded, 0, sizeof(bool) * (points.dimension));
  isIncluded[0] = true;
  unsigned num_nodes_included = 1;
  unsigned count = 0;
  unsigned cone_angle1 = cone_angle;
  while (num_nodes_included < points.dimension) {
    if (count > 0 && cone_angle < 85) {cone_angle += 5;}
    vector<unsigned> curr_route = get_route (points, node_order, shuffled_order, capacity, num_nodes_included, isIncluded, cost_func_id, cone_angle);
    unsigned sum_of_demands = 0;
    for(unsigned xyz = 0; xyz < curr_route.size(); ++xyz) {
      sum_of_demands += points.demands[curr_route[xyz]];
    }
    if(count < 3 && sum_of_demands < capacity) {
      for(unsigned xyz = 0; xyz < curr_route.size(); ++xyz) {
        isIncluded[curr_route[xyz]] = false;
      }
      num_nodes_included -= curr_route.size();
      count++;
      continue;
    }
    final_routes.push_back(curr_route);
    count = 0;
    cone_angle = cone_angle1;
  }
}
bool verify_sol (vector<vector<unsigned> > final_routes, unsigned capacity, Points &points) {
  unsigned * hist = (unsigned*) malloc(sizeof(unsigned) * points.dimension);
  memset(hist, 0, sizeof(unsigned) * points.dimension);
  for(unsigned i=0; i < final_routes.size(); ++i) {
    unsigned route_sum_of_demands = 0;
    for(unsigned j=0; j < final_routes[i].size(); ++j) {
      route_sum_of_demands += points.demands[final_routes[i][j]];
      hist[final_routes[i][j]] += 1;
    }
    if(route_sum_of_demands > capacity) { return false;}
  }
  for(unsigned i=1; i < points.dimension; ++i){
    if(hist[i] > 1) { return false; }
    if(hist[i] == 0) { return false; }
  }
  return true;
}
double get_total_cost_of_routes_rounded (vector<vector <unsigned> > final_routes, Points& points) {
  double total_cost = 0.0;
  unsigned final_routes_size = final_routes.size();
  for (unsigned ii=0; ii < final_routes_size; ++ii) {
    double curr_route_cost = 0.0;
    curr_route_cost += round(points.L2_dist(0,final_routes[ii][0]));
    unsigned curr_route_size = final_routes[ii].size();
    for (unsigned jj = 1; jj < curr_route_size; ++jj) {
      curr_route_cost += round(points.L2_dist(final_routes[ii][jj-1],final_routes[ii][jj]));
    }
    curr_route_cost += round(points.L2_dist(0,final_routes[ii][curr_route_size-1]));
    total_cost += curr_route_cost;
  }
  return total_cost;
}
double get_total_cost_of_routes (vector<vector <unsigned> > final_routes, Points& points) {
  double total_cost = 0.0;
  unsigned final_routes_size = final_routes.size();
  for (unsigned ii=0; ii < final_routes_size; ++ii) {
    double curr_route_cost = 0;
    curr_route_cost += points.L2_dist(0,final_routes[ii][0]);
    unsigned curr_route_size = final_routes[ii].size();
    for (unsigned jj = 1; jj < curr_route_size; ++jj) {
      curr_route_cost += points.L2_dist(final_routes[ii][jj-1],final_routes[ii][jj]);
    }
    curr_route_cost += points.L2_dist(0,final_routes[ii][curr_route_size-1]);
    total_cost += curr_route_cost;
  }
  return total_cost;
}
vector<vector<unsigned> > postprocess_tsp_approx (vector<vector<unsigned> >& final_routes, Points& points) {
  vector<vector<unsigned> > postprocessed_final_routes;
  unsigned nroutes = final_routes.size();
  for(unsigned i=0; i < nroutes; ++i ) {
    unsigned sz = final_routes[i].size();
    unsigned* cities = (unsigned*) malloc(sizeof(unsigned) * (sz+1));
    unsigned* tour = (unsigned*) malloc(sizeof(unsigned) * (sz+1));
    for(unsigned j = 0; j < sz; ++j)
      cities[j] = final_routes[i][j];
    cities[sz] = 0;
    tsp_approx (cities, tour, points, sz+1);
    vector<unsigned> curr_route;
    for(unsigned kk = 1; kk < sz+1; ++kk) {
      curr_route.push_back(tour[kk]);
    }
    postprocessed_final_routes.push_back(curr_route);
  }
  return postprocessed_final_routes;
}
vector<vector<unsigned> > postprocess_2OPT (vector<vector<unsigned> >& final_routes, Points& points) {
  vector<vector<unsigned> > postprocessed_final_routes;
  unsigned nroutes = final_routes.size();
  for(unsigned i=0; i < nroutes; ++i ) {
    unsigned sz = final_routes[i].size();
    unsigned* cities = (unsigned*) malloc(sizeof(unsigned) * (sz));
    unsigned* tour = (unsigned*) malloc(sizeof(unsigned) * (sz));
    for(unsigned j = 0; j < sz; ++j)
      cities[j] = final_routes[i][j];
    vector<unsigned> curr_route;
    if(sz > 2)
      tsp_2opt (cities, tour, points, sz);
    for(unsigned kk = 0; kk < sz; ++kk) {
      curr_route.push_back(cities[kk]);
    }
    postprocessed_final_routes.push_back(curr_route);
  }
  return postprocessed_final_routes;
}
bool isFeasible(vector<unsigned>& route1, vector<unsigned>& route2, unsigned capacity, Points& points) {
  unsigned sum_demands1 = 0;
  for(unsigned i = 0; i < route1.size(); ++i) {
    sum_demands1 += points.demands[route1[i]];
  }
  unsigned sum_demands2 = 0;
  for(unsigned i = 0; i < route2.size(); ++i) {
    sum_demands2 += points.demands[route2[i]];
  }
  if (sum_demands1 <= capacity && sum_demands2 <= capacity)
    return true;
  else return false;
}
bool isFeasible(vector<unsigned>& route, unsigned curr_node, unsigned capacity, Points& points) {
  unsigned sum_demands = 0;
  for(unsigned i = 0; i < route.size(); ++i) {
    sum_demands += points.demands[route[i]];
  }
  unsigned new_sum_demands = sum_demands + points.demands[curr_node];
  if (new_sum_demands <= capacity)
    return true;
  else return false;
}
bool isFeasible(vector<unsigned>& final_routes_i, vector<unsigned>& final_routes_k, unsigned curr_node, unsigned candidate_node, unsigned capacity, Points& points) {
  unsigned sum_demands_i = 0;
  for(unsigned i = 0; i < final_routes_i.size(); ++i) {
    sum_demands_i += points.demands[final_routes_i[i]];
  }
  unsigned sum_demands_k = 0;
  for(unsigned i = 0; i < final_routes_k.size(); ++i) {
    sum_demands_k += points.demands[final_routes_k[i]];
  }
  unsigned new_sum_demands_i = sum_demands_i - points.demands[curr_node] + points.demands[candidate_node];
  unsigned new_sum_demands_k = sum_demands_k - points.demands[candidate_node] + points.demands[curr_node];
  if ((new_sum_demands_i <= capacity) && (new_sum_demands_k <= capacity) )
    return true;
  else return false;
}
void inter_route_2opt (vector<unsigned>& cities_i, vector<unsigned>& cities_j, Points& points, unsigned capacity) {
#if 1
  unsigned improve = 0;
  double minchange;
  while ( improve < 1)
  {
    minchange = 0;
    unsigned minii, minjj;
    unsigned cities_i_sz = cities_i.size();
    unsigned cities_j_sz = cities_j.size();
    unsigned * prefix_sum_demands_cities_i = (unsigned*)malloc(sizeof(unsigned) * cities_i_sz);
    unsigned * prefix_sum_demands_cities_j = (unsigned*)malloc(sizeof(unsigned) * cities_j_sz);
    prefix_sum_demands_cities_i[0] = points.demands[cities_i[0]];
    for(unsigned x = 1; x < cities_i_sz; ++x) {
      prefix_sum_demands_cities_i[x] = prefix_sum_demands_cities_i[x-1] + points.demands[cities_i[x]];
    }
    prefix_sum_demands_cities_j[0] = points.demands[cities_j[0]];
    for(unsigned x = 1; x < cities_j_sz; ++x) {
      prefix_sum_demands_cities_j[x] = prefix_sum_demands_cities_j[x-1] + points.demands[cities_j[x]];
    }
    for(unsigned ii = 0; ii < cities_i_sz - 1; ++ii) {
      for(unsigned jj = 0; jj < cities_j_sz - 1; ++jj) {
L1:
        double change = points.L2_dist(cities_i[ii], cities_j[jj+1]) + points.L2_dist(cities_j[jj], cities_i[ii+1]) - points.L2_dist(cities_i[ii], cities_i[ii+1]) - points.L2_dist(cities_j[jj], cities_j[jj+1]);
        if (minchange - change > 0.000001 ) {
          unsigned sum_of_demands_cities_i = prefix_sum_demands_cities_i[ii] + prefix_sum_demands_cities_j[cities_j_sz - 1] - prefix_sum_demands_cities_j[jj];
          unsigned sum_of_demands_cities_j = prefix_sum_demands_cities_j[jj] + prefix_sum_demands_cities_i[cities_i_sz - 1] - prefix_sum_demands_cities_i[ii];
          if (sum_of_demands_cities_i > capacity || sum_of_demands_cities_j > capacity) continue;
          minchange = change;
          minii = ii;
          minjj = jj;
        }
      }
    }
    improve++;
    if (minchange < -0.000001) {
      improve = 0;
      unsigned new_size_of_cities_i = minii - minjj + cities_j_sz;
      unsigned new_size_of_cities_j = minjj - minii + cities_i_sz;
      vector<unsigned> tour_i (new_size_of_cities_i);
      vector<unsigned> tour_j (new_size_of_cities_j);
      unsigned k = 0;
      for(unsigned c = 0; c <= minii; ++c) {
        tour_i[k] = cities_i[c];
        ++k;
      }
      for(unsigned c = minjj+1; c < cities_j_sz; ++c) {
        tour_i[k] = cities_j[c];
        ++k;
      }
      k = 0;
      for(unsigned c = 0; c <= minjj; ++c) {
        tour_j[k] = cities_j[c];
        k++;
      }
      for(unsigned c = minii+1; c < cities_i_sz; ++c) {
        tour_j[k] = cities_i[c];
        k++;
      }
      cities_i = tour_i;
      cities_j = tour_j;
    }
  }
#endif
}
vector<vector<unsigned> > intra_route_TSP (vector<vector<unsigned> >& final_routes, Points& points) {
  vector<vector<unsigned> > postprocessed_final_routes1 = postprocess_tsp_approx (final_routes, points);
  vector<vector<unsigned> > postprocessed_final_routes2 = postprocess_2OPT (postprocessed_final_routes1, points);
  vector<vector<unsigned> > postprocessed_final_routes3 = postprocess_2OPT (final_routes, points);
  vector<vector<unsigned> > postprocessed_final_routes;
  double postprocessed_final_routes_cost;
  for (unsigned zzz = 0; zzz < final_routes.size(); ++zzz) {
    vector<unsigned> postprocessed_route2 = postprocessed_final_routes2[zzz];
    vector<unsigned> postprocessed_route3 = postprocessed_final_routes3[zzz];
    unsigned sz2 = postprocessed_route2.size();
    unsigned sz3 = postprocessed_route3.size();
    double postprocessed_route2_cost = get_cost_of_route(postprocessed_route2, points);
    double postprocessed_route3_cost = get_cost_of_route(postprocessed_route3, points);
    if (postprocessed_route3_cost > postprocessed_route2_cost) {
      postprocessed_final_routes.push_back(postprocessed_route2);
    }
    else {
      postprocessed_final_routes.push_back(postprocessed_route3);
    }
  }
  return postprocessed_final_routes;
}
void populate_routes_nearest_neighbor (Points& points, unsigned * node_order, unsigned * shuffled_order, unsigned capacity, vector<vector <unsigned> >& final_routes, unsigned cone_angle) {
  bool* isIncluded = (bool*) malloc(sizeof(bool) * points.dimension);
  memset(isIncluded, 0, sizeof(bool) * (points.dimension));
  isIncluded[0] = true;
  unsigned num_nodes_included = 1;
  unsigned count = 0;
  unsigned cone_angle1 = cone_angle;
  while (num_nodes_included < points.dimension) {
    if (count > 0 && cone_angle < 85) {cone_angle += 5;}
    vector<unsigned> curr_route = get_route_nearest_neighbor (points, node_order, shuffled_order, capacity, num_nodes_included, isIncluded, cone_angle);
    unsigned sum_of_demands = 0;
    for(unsigned xyz = 0; xyz < curr_route.size(); ++xyz) {
      sum_of_demands += points.demands[curr_route[xyz]];
    }
    if(count < 3 && sum_of_demands < capacity) {
      for(unsigned xyz = 0; xyz < curr_route.size(); ++xyz) {
        isIncluded[curr_route[xyz]] = false;
      }
      num_nodes_included -= curr_route.size();
      count++;
      continue;
    }
    final_routes.push_back(curr_route);
    count = 0;
    cone_angle = cone_angle1;
  }
}
vector<vector<unsigned> > sci_heuristic (Points& points, unsigned capacity, unsigned* node_order) {
  vector<vector <unsigned> > final_routes;
  double final_total_cost = DBL_MAX;
  unsigned dimension = points.dimension;
  unsigned * shuffled_order = (unsigned*) malloc(sizeof(unsigned) * (dimension-1));
  for(unsigned i=1; i < dimension; ++i)
    shuffled_order[i-1] = i;
  vector<unsigned> theta_vec = {10,15,20,25,30,35,40,45,50,55,60,65,70,75,89};
  unsigned numIter = 5;
  if (dimension-1 < 500) numIter = 25;
  else if (dimension - 1 <= 1000) numIter = 25;
  else if (dimension - 1 <= 1500) numIter = 15;
  else if ((dimension - 1 > 1500) && (dimension-1 <= 12000)) numIter = 15;
  else if ((dimension - 1 > 12000) && (dimension-1 < 20000)) numIter = 10;
  unsigned best_cost_function = UINT_MAX;
  unsigned best_ordering = UINT_MAX;
  for(unsigned numTry = 1; numTry <=numIter; ++numTry) {
    vector<vector <unsigned> > semi_final_routes;
    double semi_final_total_cost = DBL_MAX;
    unsigned semi_best_cost_function = UINT_MAX;
    for(unsigned ii=1; ii <= 7; ++ii) {
      vector<vector <unsigned> > final_routes_temp;
      double total_cost_temp = DBL_MAX;
      for(unsigned kk = 0; kk < theta_vec.size(); ++kk) {
        vector<vector <unsigned> > final_routes_theta_temp;
        unsigned ca = theta_vec[kk];
        populate_routes (points, node_order, shuffled_order, capacity, final_routes_theta_temp, ii, ca);
        double total_cost_theta_temp = get_total_cost_of_routes (final_routes_theta_temp, points);
        if(total_cost_theta_temp < total_cost_temp) {
          total_cost_temp = total_cost_theta_temp;
          final_routes_temp = final_routes_theta_temp;
        }
      }
      if(total_cost_temp < semi_final_total_cost) {
        semi_final_total_cost = total_cost_temp;
        semi_final_routes = final_routes_temp;
        semi_best_cost_function = ii;
      }
    }
    if(semi_final_total_cost < final_total_cost) {
      final_total_cost = semi_final_total_cost;
      final_routes = semi_final_routes;
      best_cost_function = semi_best_cost_function;
      best_ordering = numTry;
    }
    if(numTry == 2) {
      for(unsigned i=0; i < dimension-1; ++i)
        shuffled_order[i] = node_order[dimension-2-i];
    }
    if(numTry == 3) {
      for(unsigned i=0; i < dimension-1; ++i)
        shuffled_order[i] = node_order[i];
    }
    if(numTry >= 4)
      shuffle(shuffled_order, dimension-1);
  }
  cout << "(Cost-function, node_ordering) that gave the best result for SCI = (" << best_cost_function << ", " << best_ordering << ")" << endl;
  vector<vector<unsigned> > postprocessed_final_routes = intra_route_TSP(final_routes, points);
  return postprocessed_final_routes;
}

vector<vector<unsigned> > nearest_neighbor_heuristic (Points& points, unsigned capacity, unsigned* node_order) {
  vector<vector <unsigned> > final_routes_nn;
  double final_total_nn_cost = DBL_MAX;
  unsigned dimension = points.dimension;
  unsigned * shuffled_order = (unsigned*) malloc(sizeof(unsigned) * (dimension-1));
  for(unsigned i=1; i < dimension; ++i)
    shuffled_order[i-1] = i;
  vector<unsigned> theta_vec = {10,15,20,25,30,35,40,45,50,55,60,65,75,80,85,89, 360};
  unsigned numIter = 5;
  if (dimension-1 < 500) numIter = 25;
  else if (dimension - 1 <= 1000) numIter = 25;
  else if (dimension - 1 <= 1500) numIter = 15;
  else if ((dimension - 1 > 1500) && (dimension-1 <= 12000)) numIter = 15;
  else if ((dimension - 1 > 12000) && (dimension-1 < 20000)) numIter = 10;
  for(unsigned numTry = 1; numTry <=numIter; ++numTry) {
    vector<vector <unsigned> > semi_final_routes_nearest_neighbor;
    double semi_final_total_cost_nearest_neighbor = DBL_MAX;
    for(unsigned kk = 0; kk < theta_vec.size(); ++kk) {
      vector<vector <unsigned> > final_routes_nearest_neighbor_temp;
      unsigned ca = theta_vec[kk];
      populate_routes_nearest_neighbor (points, node_order, shuffled_order, capacity, final_routes_nearest_neighbor_temp, ca);
      double total_cost_nearest_neighbor_temp = get_total_cost_of_routes (final_routes_nearest_neighbor_temp, points);
      if(total_cost_nearest_neighbor_temp < semi_final_total_cost_nearest_neighbor) {
        semi_final_total_cost_nearest_neighbor = total_cost_nearest_neighbor_temp;
        semi_final_routes_nearest_neighbor = final_routes_nearest_neighbor_temp;
      }
    }
    if ( semi_final_total_cost_nearest_neighbor < final_total_nn_cost ) {
      final_total_nn_cost = semi_final_total_cost_nearest_neighbor;
      final_routes_nn = semi_final_routes_nearest_neighbor;
    }
    if(numTry == 2) {
      for(unsigned i=0; i < dimension-1; ++i)
        shuffled_order[i] = node_order[dimension-2-i];
    }
    if(numTry == 3) {
      for(unsigned i=0; i < dimension-1; ++i)
        shuffled_order[i] = node_order[i];
    }
    if(numTry >= 4)
      shuffle(shuffled_order, dimension-1);
  }
  vector<vector<unsigned> > postprocessed_final_routes_nn = intra_route_TSP (final_routes_nn, points);
  return postprocessed_final_routes_nn;
}
vector<vector<Edge> > PrimsMST(Points &points, vector<vector<Edge> > & graph, unsigned capacity) {
  unsigned N = points.dimension;
  const unsigned INIT = UINT_MAX;
  vector <double> key(N, DBL_MAX);
  vector <unsigned> toEdges(N, UINT_MAX);
  vector <bool> visited(N, false);
  set< pair<double, unsigned> > active;
  vector<vector<Edge> > nG(N);
  unsigned src = 0;
  key[src] = 0.0;
  active.insert( { 0.0, src});
  while( active.size() > 0 ){
    unsigned where = active.begin()->second;
    active.erase(active.begin());
    if(visited[where]) {
      continue;
    }
    visited[where] = true;
    for(Edge E : graph[where]){
      if(!visited[E.to] && E.length < key[E.to]) {
        key[E.to] = E.length;
        active.insert( { key[E.to], E.to});
        toEdges[E.to]=where;
      }
    }
  }
  unsigned u=0;
  for(unsigned v : toEdges){
    if(v != INIT ){
      double wt = points.L2_dist(u,v);
      nG[u].push_back(Edge(v,wt));
      nG[v].push_back(Edge(u,wt));
    }
    u++;
  }
  return nG;
}
void ShortCircutTour(vector< vector<Edge> > &g, vector <bool> &visited, unsigned u, vector<unsigned> &out){
  visited [u] = true;
  out.push_back(u);
  for( auto e: g[u]){
    unsigned v = e.to;
    if(!visited [v]){
      ShortCircutTour(g,visited,v, out);
    }
  }
}
vector<vector<unsigned> >
convertToVrpRoutes(Points &points, vector <unsigned> &singleRoute, unsigned capacity) {
  vector< vector<unsigned> > routes;
  unsigned residueCap = capacity;
  std::vector<unsigned> aRoute;
  for(unsigned ii = 0; ii < singleRoute.size(); ++ii ) {
    unsigned v = singleRoute[ii];
    if (v == 0) continue;
    if(residueCap >= points.demands[v]){
      aRoute.push_back(v);
      residueCap -= points.demands[v];
    }
    else{
      routes.push_back(aRoute);
      aRoute.clear();
      aRoute.push_back(v);
      residueCap = capacity - points.demands[v];
    }
  }
  routes.push_back(aRoute);
  return routes;
}
vector<vector<unsigned> > mst_dfs_approach (Points& points, unsigned capacity) {
  unsigned dimension = points.dimension;
  vector<vector<Edge> > G (dimension);
  for(size_t i=0; i < dimension; ++i){
    for(size_t j=i+1; j < dimension; ++j){
      double wt = points.L2_dist(i,j);
      G[i].push_back(Edge(j,wt));
      G[j].push_back(Edge(i,wt));
    }
  }
  vector<vector<Edge> > mstG = PrimsMST(points, G, capacity);
  vector <bool> visited(mstG.size(), false);
  visited[0] = true;
  vector <unsigned> singleRoute;
  double minCost = DBL_MAX;
  vector< vector<unsigned> > minRoutes;
  srand(0);
  for(int i=0; i < 1000000; ++i) {
    for(auto &list : mstG){
      std::shuffle(list.begin(),list.end(),std::default_random_engine(rand()));
    }
    singleRoute.clear();
    vector <bool> visited(mstG.size(), false);
    visited [0] = true;
    ShortCircutTour(mstG,visited,0, singleRoute);
    vector< vector<unsigned> > aRoutes = convertToVrpRoutes(points, singleRoute, capacity);
    double aCostRoute = get_total_cost_of_routes(aRoutes,points);
    if(aCostRoute < minCost){
      minCost = aCostRoute;
      minRoutes = aRoutes;
    }
  }
  vector<vector<unsigned> > postprocessed_final_routes = intra_route_TSP(minRoutes, points);
  return postprocessed_final_routes;
}
int main (int argc, char** argv) {
  int opt;
  string filename = "";
  bool round = false;
  while ((opt = getopt(argc, argv, "f:r")) != -1)
  {
    switch (opt)
    {
      case 'f':
        filename = string(optarg);
        break;
      case 'r':
        round = true;
        break;
      case 'h' :
      case '?' :
      default:
        cerr << "Usage: " << argv[0] << "\n"
          " -f : .vrp instance filename\n"
          " -r : use distance values rounded to integers\n";
        exit(1);
    }
  }
  if(filename.compare("") == 0) {
    cerr << "Input filename not specified!" << endl;
    cerr << "Usage: " << argv[0] << "\n"
      "\t-f : .vrp instance filename\n"
      "\t-r : round distance to the nearest integer\n";
    exit(1);
  }
  Points points;
  unsigned capacity = points.read (filename );
  points.cal_pairwise_distances();
  unsigned dimension = points.dimension;
  chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
  double * distances_from_depot = (double*) malloc ((dimension-1) * sizeof(double));
  get_distances_from_depot (points, distances_from_depot);
  unsigned * node_order = (unsigned*) malloc ((dimension-1) * sizeof(unsigned));
  reorder_nodes (points, distances_from_depot, node_order);
  vector<vector<unsigned> > postprocessed_final_routes_mst_dfs = mst_dfs_approach (points, capacity);
  double postprocessed_final_routes_mst_dfs_cost = get_total_cost_of_routes (postprocessed_final_routes_mst_dfs, points);
  vector<vector<unsigned> > postprocessed_final_routes_sci = sci_heuristic (points, capacity, node_order);
  double postprocessed_final_routes_sci_cost = get_total_cost_of_routes (postprocessed_final_routes_sci, points);
  vector<vector<unsigned> > postprocessed_final_routes;
  double postprocessed_final_routes_cost;
  if(postprocessed_final_routes_mst_dfs_cost < postprocessed_final_routes_sci_cost) {
    cout << "MST" << endl;
    postprocessed_final_routes = postprocessed_final_routes_mst_dfs;
    postprocessed_final_routes_cost = postprocessed_final_routes_mst_dfs_cost;
  }
  else {
    cout << "SCI" << endl;
    postprocessed_final_routes = postprocessed_final_routes_sci;
    postprocessed_final_routes_cost = postprocessed_final_routes_sci_cost;
  }
  if (round) {
    postprocessed_final_routes_cost = get_total_cost_of_routes_rounded (postprocessed_final_routes,points);
  }
  chrono::high_resolution_clock::time_point end = chrono::high_resolution_clock::now();
  uint64_t elapsed = chrono::duration_cast<chrono::nanoseconds>(end - start).count();
  double total_time = (double)(elapsed * 1.E-9 );
#if 1
  for(unsigned i=0; i < postprocessed_final_routes.size(); ++i) {
    cout << "Route #" << i+1 << ": ";
    for(unsigned j=0; j < postprocessed_final_routes[i].size(); ++j) {
      cout << postprocessed_final_routes[i][j] << " \n"[j+1 == postprocessed_final_routes[i].size()];
    }
  }
#endif
  cout << "Cost "<< postprocessed_final_routes_cost << endl;
  bool verified = false;
  verified = verify_sol (postprocessed_final_routes, capacity, points);
  if(verified) cout << "VALID solution" << endl;
  else cout << "INVALID solution" << endl;
  cout << "Total execution time = "<< total_time << " s" << endl;
  return 0;
}