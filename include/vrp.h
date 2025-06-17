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

// Types
using cord_t = double;       // Type for x and y co-ordinates
using weight_t = double;     // Type of weight between two 
using demand_t = double;
using capacity_t = double;
using node_t = int32;

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

    CVRP (const std::string filename) {};
    void print ();
    ~CVRP () {}
};

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
    for (size_t i = 0; i < size; ++i) {
        OUTPUT_FILE << i << ':' << std::setw(6) << node[i].x << ' ' << std::setw(6) << node[i].y << ' ' << std::setw(6) << node[i].demand << std::endl;
    }
}