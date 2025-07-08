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
    double alpha; // This is in degrees, need not be a multiple of 360
    int rho;
    int D; // D is the maximum number of neighbours for each vertex in the auxilary graph G

    Parameters() {}
    void set_alpha_in_degrees(double _alpha)
    {
        alpha = _alpha;
    }
    double get_alpha_in_radians() const
    {
        return alpha * PI / 180.0;
    }
    double get_alpha_in_degrees() const
    {
        return alpha;
    }
    void set_rho(int _rho)
    {
        rho = _rho;
    }
    void set_D(int _D)
    {
        D = _D;
    }
    ~Parameters() {}
};

Parameters get_tunable_parameters()
{
    Parameters par;
    par.set_alpha_in_degrees(50); // 5, 12, 25, 50
    par.set_D(12);                 // 3, 5, 7, 12
    par.set_rho(1e4);
    return par;
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


std::vector <std::vector<node_t>> make_partitions(const Parameters& par, const CVRP& cvrp)
{
    size_t N = cvrp.size;
    node_t depot = cvrp.depot;

    // Diving 2-D plane into ceil of 360/alpha partitions
    int num_partitions = static_cast<int>(std::ceil(360.0 / par.get_alpha_in_degrees()));

    std::vector<Vector> seperating_vectors(num_partitions + 1);
    Vector xaxis(1, 0); // Vector along x-axis
    seperating_vectors[0] = seperating_vectors[num_partitions] = xaxis;
    for(int i = 1; i < num_partitions; i++)
    {
        // Create a vector by rotating x-axis by i * alpha degrees
        seperating_vectors[i] = Vector(xaxis, i * par.get_alpha_in_radians());
    }

    std::vector<std::vector<node_t>> buckets(num_partitions);
    // Distributing nodes into buckets based on their angles
    for(node_t u = 0; u < N; u++)
    {
        if(u == depot)
        {
            for(int i = 0; i < num_partitions; i++)
            {
                buckets[i].push_back(u); // Depot goes into all buckets
            }
            continue;
        }
        Vector vec(cvrp.node[cvrp.depot].x, cvrp.node[cvrp.depot].y, cvrp.node[u].x, cvrp.node[u].y);
        bool covered = false;
        // Find the bucket for this node
        for(int i = 0; i < num_partitions; i++)
        {
            if(is_in_between(seperating_vectors[i], seperating_vectors[i + 1], vec))
            {
                buckets[i].push_back(u);
                // std::cout << "Node " << u << " is in bucket " << i << "\n";
                covered = true;
                break;
            }
        }
        if(covered == false)
        {
            HANDLE_ERROR("Node " + std::to_string(u) + " is not covered by any partition!");
        }

    }
    return buckets;
}

void construct_auxilary_graph(const CVRP& cvrp, const std::vector<node_t>& bucket, std::vector<std::vector<Edge>>& graph, const Parameters& par)
{
    // Here we will construct D-generate graph with each vertex having at most D neighbours
    size_t num_nodes = bucket.size();
    graph.resize(num_nodes);
    if(num_nodes == 1) return;

    std::priority_queue<
        std::pair<int, std::pair<int, weight_t>>, // {u_index, {v_index, weight}}
        std::vector<std::pair<int, std::pair<int, weight_t>>>,
        std::function<bool(const std::pair<int, std::pair<int, weight_t>>&,
                           const std::pair<int, std::pair<int, weight_t>>&)>>
        pq([](const std::pair<int, std::pair<int, weight_t>>& e1, const std::pair<int, std::pair<int, weight_t>>& e2) {
            return e1.second.second < e2.second.second; // Max-heap based on weight
        });

    // for depot every vertex is neighbour
    {
        int depot_index = 0; // Assuming depot is at index 0 in the bucket
        for(int v_index = 1; v_index < num_nodes; v_index++)
        {
            node_t v = bucket[v_index]; // Get the actual node from the bucket
            weight_t weight = cvrp.get_distance(bucket[depot_index], v);
            graph[depot_index].push_back(Edge(v_index, weight)); // Add edge from depot to v
        }
    }
    for(int u_index = 1; u_index < num_nodes; u_index++)
    {
        node_t u = bucket[u_index]; // Get the actual node from the bucket
        for(int v_index = 0; v_index < num_nodes; v_index++)
        {
            if(u_index == v_index) continue; // Skip self-loops
            node_t v = bucket[v_index]; // Get the actual node from the bucket
            weight_t weight = cvrp.get_distance(u, v);
            if(pq.size() < par.D)
            {
                pq.push({u_index, {v_index, weight}}); // Push the edge to the priority queue
            }else if(pq.top().second.second > weight)
            {
                pq.pop(); // Remove the heaviest edge
                pq.push({u_index, {v_index, weight}}); // Push the new edge
            }
        }
        while(!pq.empty())
        {
            auto e = pq.top(); 
            pq.pop();
            int u_index = e.first; // Index of the node in the bucket
            int v_index = e.second.first; // Index of the adjacent node in the bucket
            weight_t weight = e.second.second; // Weight of the edge
            // Add the edge to the graph
            graph[u_index].push_back(Edge(v_index, weight)); // Add edge u -> v
        }
    }

    return;
}

void run_our_method(const CVRP& cvrp, const Parameters& par, const CommandLineArgs& command_line_args)
{
    size_t N = cvrp.size;
    node_t depot = cvrp.depot;

    // Make buckets
    std::vector<std::vector<node_t>> buckets = make_partitions(par, cvrp);

    // For each bucket, construct auxiliary graph G 
    std::vector<std::vector<std::vector<Edge>>> graph(buckets.size());
    for(int b = 0; b < buckets.size(); b++)
    {
        construct_auxilary_graph(cvrp, buckets[b], graph[b], par);
    }

    std::random_device rd;
    std::mt19937 rng(rd());  // Initialize once
    // For each bucket, find the minimum possible routes

    weight_t final_cost =  0.0;
    std::vector<std::vector<int>> final_routes;
    for(int b = 0; b < buckets.size(); b++)
    {
        weight_t min_cost = INT_MAX; // taking INT_MAX as infinity
        std::vector <std::vector<int>> min_routes;
        int num_nodes = buckets[b].size();
        std::vector <bool> visited(num_nodes, false);

        // Search in solution space using randomization
        for(int iter = 1; iter <= par.rho; iter++)
        {
            // i) Randomize adjacency list
            for(int u = 0; u < num_nodes; u++)
            {
                std::shuffle(graph[b][u].begin(), graph[b][u].end(), rng);
            }

            // Step ii) Create routes
            std::vector<std::vector<node_t>> curr_routes;
            weight_t curr_total_cost = 0.0;
            int covered = 1; // Start with depot covered
            {   
                std::fill(visited.begin(), visited.end(), false);
                std::vector<node_t> current_route;
                capacity_t residue_capacity = cvrp.capacity;
                node_t prev_node = depot;                       // Assuming local id of depot is also depot which is 0 
                weight_t curr_route_cost = 0.0;

                // DFS iterative
                std::stack< std::pair <node_t, int>> rec;
                rec.push({depot, 0}); // Start from depot
                visited[depot] = true;
                while(!rec.empty())
                {
                    auto [u, index] = rec.top();
                    while(index < graph[b][u].size())
                    {
                        Edge e = graph[b][u][index];
                        if(!visited[e.v])
                        {
                            if(residue_capacity >= cvrp.node[buckets[b][e.v]].demand)
                            {
                                current_route.push_back(e.v);
                                curr_route_cost += cvrp.get_distance(buckets[b][prev_node], buckets[b][e.v]);
                                residue_capacity -= cvrp.node[buckets[b][e.v]].demand;
                                prev_node = e.v; // Update previous node to current vertex
                            }else
                            {
                                covered += current_route.size(); // Count the number of nodes in the current route
                                curr_routes.push_back(current_route);
                                curr_route_cost += cvrp.get_distance(buckets[b][prev_node], depot); // Add cost to return to depot
                                curr_total_cost += curr_route_cost;
                                current_route.clear();
                                prev_node = depot; // Reset previous node to depot
                                curr_route_cost = 0.0; // Reset current route cost
                                residue_capacity = cvrp.capacity; // Reset residue capacity
                
                                // Start a new route
                                current_route.push_back(e.v);
                                residue_capacity -= cvrp.node[buckets[b][e.v]].demand;
                                curr_route_cost += cvrp.get_distance(buckets[b][prev_node], buckets[b][e.v]);
                                prev_node = e.v; // Update previous node to current vertex
                            }
                            visited[e.v] = true;
                            rec.top().second = index + 1; // Update index for next iteration
                            rec.push({e.v, 0});           // Push next vertex to stack
                            break;
                        }
                        index++;
                    }
                    if(index == graph[b][u].size())
                    {
                        rec.pop(); // All neighbours of u are visited
                    }
                }

                // If there are any remaining nodes in the current route, add it to routes
                if(!current_route.empty())
                { 
                    covered += current_route.size(); // Count the number of nodes in the last route
                    curr_routes.push_back(current_route);
                    curr_route_cost += cvrp.get_distance(buckets[b][prev_node], depot); // Add cost to return to depot
                    curr_total_cost += curr_route_cost; // Add the cost of the last route
                }
            }

            if(covered != num_nodes)
            {
                HANDLE_ERROR("Not all nodes are covered in the bucket " + std::to_string(b) + "! Covered: " + std::to_string(covered) + ", Expected: " + std::to_string(num_nodes));
            }
            // Step iii) Update the running total cost
            if(curr_total_cost < min_cost)
            {
                min_cost = curr_total_cost;
                min_routes = curr_routes; // Update the best routes found so far
            }
        }

        if(min_routes.size() != 0)
        {
            final_cost += min_cost;
            for(auto& route: min_routes)
            {
                auto renamed_route = route;
                for(int i = 0; i < route.size(); i++)
                {
                    renamed_route[i] = buckets[b][route[i]];
                }
                final_routes.push_back(renamed_route);
            }
        }else{
            // This is case where ther are no vertices in the bucket other than depot
        }
    }
    if (std::abs(final_cost - get_total_cost_of_routes(cvrp, final_routes)) > 1e-3) {
        HANDLE_ERROR("Final cost != calculated cost in loop");
    }
    {
        OUTPUT_FILE << "----------------------------------------------\n";
        OUTPUT_FILE << "ROUTES_AFTER_LOOP\n";
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

// void run_our_method(const CVRP& cvrp, Parameters& par, const CommandLineArgs& command_line_args)
// {
//     make_partitions(par);
    
//     return;
//     size_t N = cvrp.size;
//     node_t depot = cvrp.depot;

//     std::vector <std::vector <Edge>> G(N);
//     // Construct auxilary graph G
//     {
//         std::priority_queue<
//             Edge,
//             std::vector<Edge>,
//             std::function<bool(const Edge&, const Edge&)>
//         > pq([](const Edge& e1, const Edge& e2) {
//             return e1.w < e2.w;
//         });

//         for(node_t u = 0; u < N; u++)
//         {
//             if(u == cvrp.depot)
//             {
//                 // For the depot every customer is neighbour
//                 for(node_t v = 0; v < N; v++)
//                 {
//                     if(v == depot) continue; // Skip self-loops
//                     weight_t distance = cvrp.get_distance(depot, v);
//                     G[u].push_back(Edge(v, distance));
//                 }
//                 continue; // Skip depot
//             }

//             Vector vec(cvrp.node[depot].x, cvrp.node[depot].y, cvrp.node[u].x, cvrp.node[u].y);
//             Vector vec1(vec, par.get_theta_in_radians());
//             Vector vec2(vec, -par.get_theta_in_radians());
//             // pq is empty here
            
//             for(node_t v = 0; v < N; v++)
//             {
//                 if(v == depot || v == u) continue; // Skip depot and self-loops
//                 Vector vecp(cvrp.node[depot].x, cvrp.node[depot].y, cvrp.node[v].x, cvrp.node[v].y);

//                 // Checking whether vecp is inside angle made between vec1 and vec2 at depot
//                 if(!is_in_between(vec1, vec2, vecp))
//                 {
//                     continue;
//                 }
//                 weight_t distance = cvrp.get_distance(depot, v);
//                 // Each vertex is permiited to have at most D neighbours
//                 // And these D neighbours are the closest ones
//                 if(pq.size() < par.D)
//                 {
//                     pq.push(Edge(v, distance));
//                 }else if(pq.top().w > distance)
//                 {
//                     pq.pop();
//                     pq.push(Edge(v, distance));
//                 }
//             }
//             // Adding the edges to the auxilary graph G
//             // The edges are directed from u to v
//             while(!pq.empty())
//             {
//                 Edge e = pq.top();
//                 pq.pop();
//                 G[u].push_back(e);
//                 // G[e.v].push_back(Edge(u, e.w)); // Undirected graph
//             }
//         }
//     }
//     // // Print the graph for debugging
//     // {
//     //     OUTPUT_FILE << "----------------------------------------------\n";
//     //     OUTPUT_FILE << "GRAPH_G_AFTER_CONSTRUCTION:\n";
//     //     print_graph(G);
//     //     OUTPUT_FILE << "----------------------------------------------\n";
//     // }
    
//     std::vector <bool> visited(N);
//     // Exploring solution space
//     std::vector <std::vector <node_t>> final_routes;
//     weight_t final_cost = INT_MAX;  // Initialize to a large value
//     std::random_device rd;
//     std::mt19937 rng(rd());  // Initialize once
//     for(int iter = 1; iter <= par.rho; iter++) // 3 steps per iteration i) randomize ii) create routes ii) update the running total cost
//     {
//         // Step i) Randomizing the neighbours for each vertex   
//         for(node_t u = 0; u < N; u++)
//         {
//             std::shuffle(G[u].begin(), G[u].end(), rng);
//         }



//         // Print the routes for debugging
//         if (iter == 1 || iter == par.rho/2){
//             OUTPUT_FILE << "----------------------------------------------\n";
//             OUTPUT_FILE << "ROUTES_AFTER_ITERATION_" << iter << ":\n";
//             print_routes(final_routes, final_cost);
//             OUTPUT_FILE << "----------------------------------------------\n";
//         }
//     }
//     {
//             OUTPUT_FILE << "----------------------------------------------\n";
//             OUTPUT_FILE << "ROUTES_AFTER_LOOP" << ":\n";
//             print_routes(final_routes, final_cost);
//             OUTPUT_FILE << "----------------------------------------------\n";
//     }
//     // Refining routes using optimizations
//     {
//       // using rajesh code
//       final_routes = postProcessIt(cvrp, final_routes);
//       final_cost = get_total_cost_of_routes(cvrp, final_routes);   
//     }
//     {
//         OUTPUT_FILE << "----------------------------------------------\n";
//         OUTPUT_FILE << "ROUTES_AFTER_REFINEMENT\n";
//         print_routes(final_routes, final_cost);
//         OUTPUT_FILE << "----------------------------------------------\n";
//     }

//     // Validate the solution
//     {
//         // this is also rajesh code for verification
//         if(!verify_sol(cvrp, final_routes, cvrp.capacity))
//         {
//             HANDLE_ERROR("Solution is not valid!");
//         }
//     }

//     OUTPUT_FILE << "----------------------------------------------\n";
//     OUTPUT_FILE << "FINAL_OUTPUT:\n";
//     OUTPUT_FILE << "file-name,minCost,correctness\n";
//     OUTPUT_FILE << command_line_args.input_file_name << "," << final_cost << "," << "VALID\n";
//     // Print output
//     print_routes(final_routes, final_cost);
//     OUTPUT_FILE << "----------------------------------------------\n";
// }

int main(int argc, char* argv[])
{
    OUTPUT_FILE << std::fixed << std::setprecision(2);
    auto command_line_args = get_command_line_args(argc, argv);
    auto cvrp = get_cvrp(command_line_args);
    auto parameters = get_tunable_parameters();
    run_our_method(cvrp, parameters, command_line_args);
}
