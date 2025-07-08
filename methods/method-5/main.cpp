#include "vrp-single-threaded.h"
#include "rajesh_codes-single-threaded.h"

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
    int lambda;

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
    void set_lambda(int _lambda)
    {
        lambda = _lambda;
    }

    ~Parameters() {}
};

Parameters get_tunable_parameters()
{
    Parameters par;
    par.set_alpha_in_degrees(75);     // 5, 12, 25, 50, 75
    par.set_lambda(12);                // 1, 6, 12
    par.set_rho(1e3);
    return par;
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
            if(vec.is_in_between(seperating_vectors[i], seperating_vectors[i + 1]))
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

void construct_auxilary_graph(const CVRP& cvrp, const std::vector<node_t>& bucket, std::vector<std::vector<Edge>>& graph, const Parameters& par, int random_start_index)
{
    // Here we will construct MST
    size_t num_nodes = bucket.size();
    graph.clear();
    graph.resize(num_nodes);
    if(num_nodes == 1) return;

    std::priority_queue<
        std::pair<int, std::pair<int, weight_t>>, // {u_index, {v_index, weight}}
        std::vector<std::pair<int, std::pair<int, weight_t>>>,
        std::function<bool(const std::pair<int, std::pair<int, weight_t>>&,
                           const std::pair<int, std::pair<int, weight_t>>&)>>
        pq([](const std::pair<int, std::pair<int, weight_t>>& e1, const std::pair<int, std::pair<int, weight_t>>& e2) {
            return e1.second.second > e2.second.second; // Min-heap based on weight
        });

    std::vector<bool> in_mst(num_nodes, false);
    
    
    // Start Prim's algorithm from the arbitary node
    int u_index = random_start_index;

    for(int v_index = 0; v_index < num_nodes; v_index++)
    {
        if(u_index == v_index) continue;
        weight_t weight = cvrp.get_distance_on_the_fly(bucket[u_index], bucket[v_index]);
        pq.push({u_index, {v_index, weight}}); // Push the edge to the priority queue
    }
    in_mst[u_index] = true; // Mark the first node as included in MST

    int completed = 1; // Number of nodes included in the MST
    // Prim's algorithm to construct the MST
    while(!pq.empty())
    {
        auto e = pq.top(); // min-weight edge
        pq.pop();

        int u_index = e.first; // Index of the node in the bucket
        int v_index = e.second.first; // Index of the adjacent node in the bucket
        weight_t weight = e.second.second; // Weight of the edge
        if(in_mst[v_index]) continue; // Skip if already in MST
        
        in_mst[v_index] = true; // Mark this node as included in MST
        completed++;

        // Add the edge to the graph
        node_t u = bucket[u_index]; // Get the actual node from the bucket
        node_t v = bucket[v_index]; // Get the actual node from the bucket
        graph[u_index].push_back(Edge(v_index, weight)); // Add edge u -> v
        graph[v_index].push_back(Edge(u_index, weight)); // Add edge v -> u (undirected graph)
        // Add all edges from u to other nodes in the bucket
        for(int w_index = 0; w_index < num_nodes; w_index++)
        {
            if(in_mst[w_index]) continue; // Skip already included nodes
            weight_t weight = cvrp.get_distance_on_the_fly(v, bucket[w_index]);
            pq.push({v_index, {w_index, weight}}); // Push the edge to the priority queue
        }
    }

    if(completed != num_nodes)
    {
        HANDLE_ERROR("Not all nodes are included in the MST! Completed: " + std::to_string(completed) + ", Expected: " + std::to_string(num_nodes));
    }
    return;
}

void run_our_method(const CVRP& cvrp, const Parameters& par, const CommandLineArgs& command_line_args)
{
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    size_t N = cvrp.size;
    node_t depot = cvrp.depot;

    // Make buckets
    std::vector<std::vector<node_t>> buckets = make_partitions(par, cvrp);

    // For each bucket, construct auxiliary graph G 
    std::vector<std::vector<std::vector<Edge>>> graph(buckets.size());
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
        std::uniform_int_distribution<> distrib(0, num_nodes - 1);

        for(int _ = 1; _ <= par.lambda; _++)
        {
            // Construct auxilar graph
            {
                construct_auxilary_graph(cvrp, buckets[b], graph[b], par, distrib(rng));
            }
            // Explore in solution space
            {
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
                                        curr_route_cost += cvrp.get_distance_on_the_fly(buckets[b][prev_node], buckets[b][e.v]);
                                        residue_capacity -= cvrp.node[buckets[b][e.v]].demand;
                                        prev_node = e.v; // Update previous node to current vertex
                                    }else
                                    {
                                        covered += current_route.size(); // Count the number of nodes in the current route
                                        curr_routes.push_back(current_route);
                                        curr_route_cost += cvrp.get_distance_on_the_fly(buckets[b][prev_node], depot); // Add cost to return to depot
                                        curr_total_cost += curr_route_cost;
                                        current_route.clear();
                                        prev_node = depot; // Reset previous node to depot
                                        curr_route_cost = 0.0; // Reset current route cost
                                        residue_capacity = cvrp.capacity; // Reset residue capacity
                        
                                        // Start a new route
                                        current_route.push_back(e.v);
                                        residue_capacity -= cvrp.node[buckets[b][e.v]].demand;
                                        curr_route_cost += cvrp.get_distance_on_the_fly(buckets[b][prev_node], buckets[b][e.v]);
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
                            curr_route_cost += cvrp.get_distance_on_the_fly(buckets[b][prev_node], depot); // Add cost to return to depot
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
            }
        }

        if(min_routes.size() != 0)
        {
            final_cost += min_cost;
            for(auto& route: min_routes)
            {
                for(int i = 0; i < route.size(); i++)
                {
                    route[i] = buckets[b][route[i]];
                }
                final_routes.push_back(route);
            }
        }else{
            // This is case where ther are no vertices in the bucket other than depot
        }
    }
    if constexpr (DEBUG_MODE){
        if (std::abs(final_cost - get_total_cost_of_routes(cvrp, final_routes)) > 1e-3) {
            HANDLE_ERROR("Final cost != calculated cost in loop");
        }
        // {
        //     OUTPUT_FILE << "----------------------------------------------\n";
        //     OUTPUT_FILE << "ROUTES_AFTER_LOOP\n";
        //     print_routes(final_routes, final_cost);
        //     OUTPUT_FILE << "----------------------------------------------\n";
        // }
    }
    double time_till_loop = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() / 1000.0;
    // Refining routes using optimizations
    {
      // using rajesh code
      final_routes = postProcessIt(cvrp, final_routes, final_cost);
    }
    auto end = std::chrono::high_resolution_clock::now();
    double elapsed_time = std::chrono::duration<double>(end - start).count();

    // {
    //     OUTPUT_FILE << "----------------------------------------------\n";
    //     OUTPUT_FILE << "ROUTES_AFTER_REFINEMENT\n";
    //     print_routes(final_routes, final_cost);
    //     OUTPUT_FILE << "----------------------------------------------\n";
    // }

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
    OUTPUT_FILE << "file-name,time_till_loop,total_elapsed_time,minCost,correctness\n";

    OUTPUT_FILE << std::fixed << std::setprecision(6)
                << command_line_args.input_file_name << ","
                << time_till_loop << ","
                << elapsed_time << ","
                << final_cost << ","
                << "VALID\n";

    // Print output
    print_routes(final_routes, final_cost);

    OUTPUT_FILE << "----------------------------------------------\n";
}

int main(int argc, char* argv[])
{
    OUTPUT_FILE << std::fixed << std::setprecision(2);
    auto command_line_args = get_command_line_args(argc, argv);
    auto cvrp = get_cvrp(command_line_args);
    auto parameters = get_tunable_parameters();
    run_our_method(cvrp, parameters, command_line_args);
}
