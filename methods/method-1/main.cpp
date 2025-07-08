#include "vrp.h"
#include "rajesh_codes.h"

class CommandLineArgs
{
public:
    std::string input_file_name;
};


class CommandLineArgs get_command_line_args(int argc, char* argv[])
{
    if(argc != 2)
    {
        HANDLE_ERROR(std::string("Usage: ./") + argv[0] + " input_file_path");
    }
    struct CommandLineArgs command_line_args;
    command_line_args.input_file_name = argv[1];
    
    return command_line_args;
}

class CVRP get_cvrp(class CommandLineArgs command_line_args)
{
    return CVRP(command_line_args.input_file_name);
}

// Parametrs are the ones on which we have complete control of.
class Parameters
{
public:
    double theta; // This is in degrees
    int D;
    int rho;

    Parameters() {}
    void set_theta_in_degrees(double _theta)
    {
        theta = _theta;
    }
    double get_theta_in_radians()
    {
        return theta * PI / 180.0;
    }
    double get_theta_in_degrees() 
    {
        return theta;
    }
    void set_D(int _D)
    {
        D = _D;
    }
    void set_rho(int _rho)
    {
        rho = _rho;
    }
    ~Parameters() {}
};

Parameters get_tunable_parameters()
{
    Parameters par;
    par.set_theta_in_degrees(25);  // try: 5, 12, 25
    par.set_D(12);                  // try: 3, 5, 7, 12
    par.set_rho(1e4);               // try: 1e4
    return par;
}

class Vector {
public:
    cord_t x, y; // x i + y j vector in 2-D plane

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
};

bool is_in_between(const Vector& vec1, const Vector& vec2, const Vector& vecp) {
    // Compute cross products
    cord_t cross12 = vec1.x * vec2.y - vec1.y * vec2.x; // vec1 × vec2
    cord_t cross1p = vec1.x * vecp.y - vec1.y * vecp.x; // vec1 × vecp
    cord_t crossp2 = vecp.x * vec2.y - vecp.y * vec2.x; // vecp × vec2
    if (cross12 == 0) {
        // vec1 and vec2 are collinear
        if (vec1.x * vec2.x + vec1.y * vec2.y >= 0) { // Same direction or one is zero vector
            // If vecp is also collinear with them, and in the same general direction:
            return cross1p == 0 && (vec1.x * vecp.x + vec1.y * vecp.y >= 0); // Check if vecp is also in the same direction as vec1
        } else { // Opposite directions (180 degrees apart)
        // ambiguis case
            return cross1p <= 0 && crossp2 <= 0; // This is what the current code would do.
        }
    }

    if (cross12 > 0) 
        return cross1p >= 0 && crossp2 >= 0;  // vecp lies between vec1 and vec2 (angle < 180°)
    else
        return cross1p <= 0 && crossp2 <= 0;  // reflex angle case
}

weight_t get_total_cost_of_routes(const CVRP& cvrp, const std::vector<std::vector<node_t>>& final_routes)
{
    weight_t total_cost = 0.0;
    for(const auto& route : final_routes)
    {
        if(route.empty()) continue; // Skip empty routes
        weight_t curr_route_cost = cvrp.get_distance(cvrp.depot, route[0]);
        for(size_t j = 1; j < route.size(); ++j)
        {
            curr_route_cost += cvrp.get_distance(route[j - 1], route[j]);
        }
        curr_route_cost += cvrp.get_distance(route.back(), cvrp.depot);
        total_cost += curr_route_cost;
    }
    return total_cost;
}

void run_our_method(const CVRP& cvrp, Parameters& par, const CommandLineArgs& command_line_args)
{
    size_t N = cvrp.size;
    node_t depot = cvrp.depot;

    std::vector <std::vector <Edge>> G(N);
    // Construct auxilary graph G
    {
        std::priority_queue<
            Edge,
            std::vector<Edge>,
            std::function<bool(const Edge&, const Edge&)>
        > pq([](const Edge& e1, const Edge& e2) {
            return e1.w < e2.w;
        });

        for(node_t u = 0; u < N; u++)
        {
            if(u == cvrp.depot)
            {
                // For the depot every customer is neighbour
                for(node_t v = 0; v < N; v++)
                {
                    if(v == depot) continue; // Skip self-loops
                    weight_t distance = cvrp.get_distance(depot, v);
                    G[u].push_back(Edge(v, distance));
                }
                continue; // Skip depot
            }

            Vector vec(cvrp.node[depot].x, cvrp.node[depot].y, cvrp.node[u].x, cvrp.node[u].y);
            Vector vec1(vec, par.get_theta_in_radians());
            Vector vec2(vec, -par.get_theta_in_radians());
            // pq is empty here
            
            for(node_t v = 0; v < N; v++)
            {
                if(v == depot || v == u) continue; // Skip depot and self-loops
                Vector vecp(cvrp.node[depot].x, cvrp.node[depot].y, cvrp.node[v].x, cvrp.node[v].y);

                // Checking whether vecp is inside angle made between vec1 and vec2 at depot
                if(!is_in_between(vec1, vec2, vecp))
                {
                    continue;
                }
                weight_t distance = cvrp.get_distance(depot, v);
                // Each vertex is permiited to have at most D neighbours
                // And these D neighbours are the closest ones
                if(pq.size() < par.D)
                {
                    pq.push(Edge(v, distance));
                }else if(pq.top().w > distance)
                {
                    pq.pop();
                    pq.push(Edge(v, distance));
                }
            }
            // Adding the edges to the auxilary graph G
            // The edges are directed from u to v
            while(!pq.empty())
            {
                Edge e = pq.top();
                pq.pop();
                G[u].push_back(e);
                // G[e.v].push_back(Edge(u, e.w)); // Undirected graph
            }
        }
    }
    // // Print the graph for debugging
    // {
    //     OUTPUT_FILE << "----------------------------------------------\n";
    //     OUTPUT_FILE << "GRAPH_G_AFTER_CONSTRUCTION:\n";
    //     print_graph(G);
    //     OUTPUT_FILE << "----------------------------------------------\n";
    // }
    
    std::vector <bool> visited(N);
    // Exploring solution space
    std::vector <std::vector <node_t>> final_routes;
    weight_t final_cost = INT_MAX;  // Initialize to a large value
    std::random_device rd;
    std::mt19937 rng(rd());  // Initialize once
    for(int iter = 1; iter <= par.rho; iter++) // 3 steps per iteration i) randomize ii) create routes ii) update the running total cost
    {
        // Step i) Randomizing the neighbours for each vertex   
        for(node_t u = 0; u < N; u++)
        {
            std::shuffle(G[u].begin(), G[u].end(), rng);
        }

        // Step ii) Create routes
        std::fill(visited.begin(), visited.end(), false);
        std::vector<std::vector<node_t>> curr_routes;
        weight_t curr_total_cost = 0.0;
        {
            std::vector<node_t> current_route;
            capacity_t residue_capacity = cvrp.capacity;
            node_t prev_node = depot;
            weight_t curr_route_cost = 0.0;

            // DFS iterative
            std::stack< std::pair <node_t, int>> rec;
            rec.push({depot, 0}); // Start from depot
            visited[depot] = true;
            while(!rec.empty())
            {
                auto [u, index] = rec.top();
                while(index < G[u].size())
                {
                    Edge e = G[u][index];
                    if(!visited[e.v])
                    {
                        if(residue_capacity >= cvrp.node[e.v].demand)
                        {
                            current_route.push_back(e.v);
                            curr_route_cost += cvrp.get_distance(prev_node, e.v);
                            residue_capacity -= cvrp.node[e.v].demand;
                            prev_node = e.v; // Update previous node to current vertex
                        }else
                        {
                            curr_routes.push_back(current_route);
                            curr_route_cost += cvrp.get_distance(prev_node, depot);
                            curr_total_cost += curr_route_cost;
                            current_route.clear();
                            prev_node = depot; // Reset previous node to depot
                            curr_route_cost = 0.0; // Reset current route cost
                            residue_capacity = cvrp.capacity; // Reset residue capacity
            
                            // Start a new route
                            current_route.push_back(e.v);
                            residue_capacity -= cvrp.node[e.v].demand;
                            curr_route_cost += cvrp.get_distance(prev_node, e.v);
                            prev_node = e.v; // Update previous node to current vertex
                        }
                        visited[e.v] = true;
                        rec.top().second = index + 1; // Update index for next iteration
                        rec.push({e.v, 0});           // Push next vertex to stack
                        break;
                    }
                    index++;
                }
                if(index == G[u].size())
                {
                    rec.pop(); // All neighbours of u are visited
                }
            }

            // If there are any remaining nodes in the current route, add it to routes
            if(!current_route.empty())
            {
                curr_routes.push_back(current_route);
                curr_route_cost += cvrp.get_distance(prev_node, depot); // Add cost to return to depot
                curr_total_cost += curr_route_cost; // Add the cost of the last route
            }
        }
        // One question to address: Is order within the route good enough??

        // Step iii) Update the running total cost
        if(curr_total_cost < final_cost)
        {
            final_cost = curr_total_cost;
            final_routes = curr_routes; // Update the best routes found so far
        }

        // Print the routes for debugging
        if (iter == 1 || iter == par.rho/2){
            OUTPUT_FILE << "----------------------------------------------\n";
            OUTPUT_FILE << "ROUTES_AFTER_ITERATION_" << iter << ":\n";
            print_routes(final_routes, final_cost);
            OUTPUT_FILE << "----------------------------------------------\n";
        }
    }
    if (std::abs(final_cost - get_total_cost_of_routes(cvrp, final_routes)) > 1e-3) {
        HANDLE_ERROR("Final cost != calculated cost in loop");
    }
    {
            OUTPUT_FILE << "----------------------------------------------\n";
            OUTPUT_FILE << "ROUTES_AFTER_LOOP" << ":\n";
            print_routes(final_routes, final_cost);
            OUTPUT_FILE << "----------------------------------------------\n";
    }
    // Refining routes using optimizations
    {
      // using rajesh code
      final_routes = postProcessIt(cvrp, final_routes);
      final_cost = get_total_cost_of_routes(cvrp, final_routes);   
    }
    {
        OUTPUT_FILE << "----------------------------------------------\n";
        OUTPUT_FILE << "ROUTES_AFTER_REFINEMENT\n";
        print_routes(final_routes, final_cost);
        OUTPUT_FILE << "----------------------------------------------\n";
    }

    // Validate the solution
    {
        // this is also rajesh code for verification
        if(!verify_sol(cvrp, final_routes, cvrp.capacity))
        {
            HANDLE_ERROR("Solution is not valid!");
        }
    }

    OUTPUT_FILE << "----------------------------------------------\n";
    OUTPUT_FILE << "FINAL_OUTPUT:\n";
    OUTPUT_FILE << "file-name,minCost,correctness\n";
    OUTPUT_FILE << command_line_args.input_file_name << "," << final_cost << "," << "VALID\n";
    // Print output
    print_routes(final_routes, final_cost);
    OUTPUT_FILE << "----------------------------------------------\n";
}

int main(int argc, char* argv[])
{
    auto command_line_args = get_command_line_args(argc, argv);
    auto cvrp = get_cvrp(command_line_args);
    auto parameters = get_tunable_parameters();
    run_our_method(cvrp, parameters, command_line_args);
}