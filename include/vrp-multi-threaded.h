#pragma once

/*
DEPOT is 0
Customers: 1 to size-1 inclusive
*/

#include <iostream>
#include <vector>
#include <fstream>
#include <cstring>
#include <iomanip>
#include <sstream>  //stringstream
#include <random>
#include <algorithm>
#include <stack>
#include <cfloat>
#include <queue>
#include <chrono>  //timing CPU
#include <functional>
#include <climits>
#include <omp.h>

constexpr bool DEBUG_MODE = false; // Set to true for debugging

// Error handling
#define ERROR_FILE std::cerr
#define HANDLE_ERROR(message) \
    do { \
        ERROR_FILE << "❌ Error: " << (message) << "\n" \
                   << "   In file: " << __FILE__ << "\n" \
                   << "   At line: " << __LINE__ << std::endl; \
        std::exit(EXIT_FAILURE); \
    } while (0)

// Output file
#define OUTPUT_FILE std::cout

// Constants
constexpr double PI = 3.14159265358979323846;

// Types
using cord_t = double;       // Type for x and y co-ordinates
using weight_t = double;     // Type of weight between two 
using demand_t = double;
using capacity_t = double;
using node_t = int32_t;

// Point in the 2-D plane: (customer (or) depot)
class Point
{
public:
    cord_t x;
    cord_t y;
    demand_t demand;

    Point() {}
    Point(cord_t _x, cord_t _y, demand_t _demand): x(_x), y(_y), demand(_demand) {}
    ~Point() {}
};

class CVRP
{
    void read (const std::string filename);
public:
    capacity_t capacity;
    size_t size;
    std::vector <Point> node;
    std::string type;
    const node_t depot = 0; // this is must
    std::vector<weight_t> dist; // distance matrix

    CVRP (const std::string filename);
    void print ();
    double get_distance_on_the_fly(node_t, node_t) const;
    ~CVRP () {}
};

weight_t CVRP::get_distance_on_the_fly(node_t u, node_t v) const
{
  if (u < 0 || u >= size || v < 0 || v >= size) {
    HANDLE_ERROR("Node index out of bounds");
  }
  if (u == v) {
    return 0.0; // Distance to itself is zero
  }

  // Euclidian distance calculating on the fly
  return sqrt((node[u].x - node[v].x) * (node[u].x - node[v].x) + (node[u].y - node[v].y) * (node[u].y - node[v].y));
}

CVRP::CVRP(const std::string filename)
{
    read(filename);
}

void CVRP::read(const std::string filename) 
{
  std::ifstream in(filename);
  if (!in.is_open()) {
    std::cerr << "Could not open the file \"" << filename << "\"" << std::endl;
    exit(1);
  }
  std::string line;
  for (int i = 0; i < 3; ++i)
    getline(in, line);

  // DIMENSION
  getline(in, line);
  size = stof(line.substr(line.find(":") + 2));


  // DISTANCE TYPE
  getline(in, line);
  type = line.find(":");

  // CAPACITY
  getline(in, line);
  capacity = stof(line.substr(line.find(":") + 2));

  //skip NODE_COORD_SECTION
  getline(in, line);

  // Allocate
  node.resize(size);

  //~ 1  x1  y1
  //~ 2  x2  y2
  //~ 3  x3  y3
  //~ ...
  //~ n  xn  yn

  for (size_t i = 0; i < size; ++i) {
    getline(in, line);

    std::stringstream iss(line);
    size_t id;
    std::string xStr, yStr;

    iss >> id >> xStr >> yStr;
    node[i].x = stof(xStr);
    node[i].y = stof(yStr);
  }

  // skip DEMAND_SECTION
  getline(in, line);

  for (size_t i = 0; i < size; ++i) {
    getline(in, line);
    std::stringstream iss(line);

    size_t id;
    std::string dStr;
    iss >> id >> dStr;

    node[i].demand = stof(dStr);
  }
  in.close();

  return;
}

void CVRP::print()
{
    OUTPUT_FILE << "SIZE: " << size << "\n";
    OUTPUT_FILE << "Capacity: " << capacity << "\n";
    OUTPUT_FILE << std::setw(6) << "NODE" 
                << std::setw(10) << "X"  
                << std::setw(10) << "Y" 
                << std::setw(10) << "DEMAND" << "\n";

    for (size_t i = 0; i < size; ++i) {
        OUTPUT_FILE << std::setw(6) << i
                    << std::setw(10) << node[i].x 
                    << std::setw(10) << node[i].y 
                    << std::setw(10) << node[i].demand 
                    << "\n";
    }
    return;
}


// Print in DIMACS output format http://dimacs.rutgers.edu/programs/challenge/vrp/cvrp/
// Depot is 0
// Route #1: 1 2 3
// Route #2: 4 5
// ...
// Route #k: n-1 n
//
void print_routes(const std::vector<std::vector<node_t>> &final_routes, const weight_t final_cost) 
{

  for (unsigned ii = 0; ii < final_routes.size(); ++ii) {
    OUTPUT_FILE << "Route #" << ii + 1 << ":";
    for (unsigned jj = 0; jj < final_routes[ii].size(); ++jj) {
      OUTPUT_FILE << " " << final_routes[ii][jj];
    }
    OUTPUT_FILE << '\n';
  }

  OUTPUT_FILE << "Cost " << final_cost << std::endl;
}

class Edge
{
public:
    node_t v;
    weight_t w;

    Edge(node_t _v, weight_t _w): v(_v), w(_w) {}
};

void print_graph(const std::vector<std::vector<Edge>>& G)
{
    for(node_t u = 0; u < G.size(); u++)
    {
        OUTPUT_FILE << "Node " << u << ": ";
        for(const auto& e : G[u])
        {
            OUTPUT_FILE << "(" << e.v << ", " << e.w << ") ";
        }
        OUTPUT_FILE << "\n";
    }
}

class Vector {
public:
    cord_t x, y; // x i + y j vector in 2-D plane
    Vector() : x(0), y(0) {} // Default constructor for a zero vector
    Vector(cord_t _x, cord_t _y) : x(_x), y(_y) {} // Constructor for a vector with given coordinates
    // Constructor: Vector pointing towards (x2, y2) from (x1, y1)
    Vector(cord_t x1, cord_t y1, cord_t x2, cord_t y2) {
        x = x2 - x1;
        y = y2 - y1;
    }

    // Constructor: new vector by rotating vector `v` by `theta` radians
    Vector(const Vector& v, cord_t theta_rad) {
        x = v.x * std::cos(theta_rad) - v.y * std::sin(theta_rad);
        y = v.x * std::sin(theta_rad) + v.y * std::cos(theta_rad);
    }

    // For checking whether this vector is in between two given vectors
    bool is_in_between(const Vector& vec1, const Vector& vec2) const {
        // Compute cross products
        cord_t cross12 = vec1.x * vec2.y - vec1.y * vec2.x; // vec1 × vec2
        cord_t cross1p = vec1.x * y - vec1.y * x; // vec1 × vecp
        cord_t crossp2 = x * vec2.y - y * vec2.x; // vecp × vec2
        if (cross12 == 0) {
            // vec1 and vec2 are collinear
            if (vec1.x * vec2.x + vec1.y * vec2.y >= 0) { // Same direction or one is zero vector
                // If vecp is also collinear with them, and in the same general direction:
                return cross1p == 0 && (vec1.x * x + vec1.y * y > 0); // Check if vecp is also in the same direction as vec1
            } else { // Opposite directions (180 degrees apart)
                // ambigious case
                return cross1p < 0 && crossp2 < 0; 
            }
        }

        if (cross12 > 0) {
          return cross1p > 0 && crossp2 >= 0;  // vecp lies between vec1 and vec2 (angle < 180°)
        } else {
          return cross1p < 0 && crossp2 <= 0;  // reflex angle case   
        }
    }
};

weight_t get_total_cost_of_routes(const CVRP& cvrp, const std::vector<std::vector<node_t>>& final_routes)
{
    weight_t total_cost = 0.0;
    for(const auto& route : final_routes)
    {
        // if(route.empty()) continue; // Skip empty routes
        weight_t curr_route_cost = cvrp.get_distance_on_the_fly(cvrp.depot, route[0]);
        for(size_t j = 1; j < route.size(); ++j)
        {
            curr_route_cost += cvrp.get_distance_on_the_fly(route[j - 1], route[j]);
        }
        curr_route_cost += cvrp.get_distance_on_the_fly(route.back(), cvrp.depot);
        total_cost += curr_route_cost;
    }
    return total_cost;
}

template <typename T>
class MinHeap {
  int   last_index;
  T*    vec;
  int*  vertex_to_index_map;
  
  void swap(T& left, T& right) {
    T storage = left; 
    left = right;
    right = storage;
  }
public:
  MinHeap(const int num_nodes){
    last_index          = num_nodes - 1;
    vec                 = new T[num_nodes];
    vertex_to_index_map = new int[num_nodes];

    for(int i = 0; i < num_nodes; i++)
    {
      vertex_to_index_map[i] = i;
      vec[i].u      = -1;
      vec[i].v      = i;
      vec[i].weight = INT_MAX;
    }
  }

  void heapify_up(int index) {
    while(index > 0) {
      int parent = (index - 1) >> 1;
      if(vec[parent].weight > vec[index].weight) {
        T storage = vec[parent];
        vec[parent] = vec[index];
        vec[index] = storage;

        vertex_to_index_map[vec[index].v] = index;
        index = parent;
      } else {
        vertex_to_index_map[vec[index].v] = index;
        break;
      }
    }
  }

  void DecreaseKey(const T& node) {
    int vertex = node.v;
    int index  = vertex_to_index_map[vertex];
    if(index > last_index) {
      HANDLE_ERROR("The vertex is not there in heap");
    }
    if(vec[index].weight <= node.weight) return;
    vec[index].weight = node.weight;
    vec[index].u      = node.u;

    heapify_up(index);
  }

  void heapify_down(int index) {
    int left, right, small_weight_index;
    while (index <= last_index) {
      left                = (index << 1) + 1;
      right               = (left) + 1;
      small_weight_index  = index;

      if (left <= last_index && vec[left].weight < vec[small_weight_index].weight)   small_weight_index = left;
      if (right <= last_index && vec[right].weight < vec[small_weight_index].weight) small_weight_index = right;

      if (small_weight_index != index) {
        T storage                         = vec[small_weight_index];
        vec[small_weight_index]           = vec[index];
        vec[index]                        = storage;

        vertex_to_index_map[vec[index].v] = index;
        index                             = small_weight_index;
      } else {
        vertex_to_index_map[vec[index].v] = index;
        break;
      }
    }
  }

  T pop() {
    int vertex = vec[0].v;
    swap(vec[0], vec[last_index]);
    vertex_to_index_map[vec[last_index].v] = last_index;
    last_index--;

    heapify_down(0);
    return vec[last_index + 1];
  }

  bool empty() {
    return last_index < 0;
  }

  ~MinHeap() {
    delete[] vec;
    delete[] vertex_to_index_map;
  }
};