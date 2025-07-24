#include "vrp-multi-threaded.h"
#include "rajesh_codes-multi-threaded.h"
// #include <tbb/concurrent_vector.h> 

class CommandLineArgs
{
public:
    std::string input_file_name;
    double alpha;
    int rho;
    CommandLineArgs(const std::string& file_name, double _alpha, int _rho)
        : input_file_name(file_name), alpha(_alpha), rho(_rho) {}
};

class CommandLineArgs get_command_line_args(int argc, char* argv[])
{ 
    if(argc != 4)
    {
        HANDLE_ERROR(std::string("Usage: ") + argv[0] + " input_file_path --alpha=<alpha> --rho=<rho>");
    }
    std::string input_file_name = argv[1];
    if(input_file_name.empty()) HANDLE_ERROR("Input file name cannot be empty.");

    double alpha;
    int rho;
    for(int i = 2; i < argc; i++)
    {
        std::string arg = argv[i];
        if(arg.find("--alpha=") == 0)
        {
            alpha = std::stod(arg.substr(8)); // Extract the value after "--alpha="
            if(alpha <= 0 || alpha >= 360) HANDLE_ERROR("Alpha must be in the range (0, 360).");
        }
        else if(arg.find("--rho=") == 0)
        {
            rho = std::stoi(arg.substr(6)); // Extract the value after "--rho="
            if(rho <= 0) HANDLE_ERROR("Rho must be a positive integer.");
        }
        else HANDLE_ERROR("Unknown argument: " + arg);
    }

    // Create and return the CommandLineArgs object
    return CommandLineArgs(input_file_name, alpha, rho);
}

class CVRP get_cvrp(class CommandLineArgs command_line_args) {
    return CVRP(command_line_args.input_file_name);
}

// Parametrs are the ones on which we have complete control of.
class Parameters
{
public:
    double alpha; // This is in degrees, need not be a multiple of 360
    int rho;

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
    ~Parameters() {}
};

Parameters get_tunable_parameters(const CommandLineArgs& command_line_args)
{
    Parameters par;
    par.set_alpha_in_degrees(command_line_args.alpha);  // 5, 10, 25, 50, 75
    par.set_rho(command_line_args.rho);                 // 1e3, 1e4
    return par;
}


// You may look into (a + b - 1) / b in ceil when a and b are integers
int my_ceil(double a, double b)
{
    if(b <= 0)
    {
        HANDLE_ERROR("Division by zero or negative value in my_ceil");
    }
    return static_cast<int>(std::ceil(a / b));
}

std::vector <std::vector<node_t>> make_partitions(const Parameters& par, const CVRP& cvrp, std::vector <int>& reverse_map)
{
    size_t N = cvrp.size;
    const node_t depot = cvrp.depot;

    // Diving 2-D plane into ceil of 360/alpha partitions
    int num_partitions = my_ceil(360.0, par.get_alpha_in_degrees());

    std::vector<Vector> seperating_vectors(num_partitions + 1);
    Vector xaxis(1, 0); // Vector along x-axis
    seperating_vectors[0] = seperating_vectors[num_partitions] = xaxis;
    for(int i = 1; i < num_partitions; i++)
    {
        // Create a vector by rotating x-axis by i * alpha degrees
        seperating_vectors[i] = Vector(xaxis, i * par.get_alpha_in_radians());

        // // Print the current angle in radians and/or degrees
        // std::cout << "Partition " << i 
        //         << ": angle = " << i * par.get_alpha_in_radians() 
        //         << " radians (" << (i * par.get_alpha_in_radians() * 180.0 / M_PI) 
        //         << " degrees)" << std::endl;
    }


    std::vector<std::vector<node_t>> buckets(num_partitions);
    // Distributing nodes into buckets based on their angles
    { // Depot goes into all buckets
        node_t u = depot;
        for(int i = 0; i < num_partitions; i++)
        {
            buckets[i].push_back(u); 
        }
    }

    for(node_t u = 1; u < N; u++)
    {
        Vector vec(cvrp.node[depot].x, cvrp.node[depot].y, cvrp.node[u].x, cvrp.node[u].y);
        bool covered = false;
        // Find the bucket for this node
        for(int i = 0; i < num_partitions; i++)
        {
            if(vec.is_in_between(seperating_vectors[i], seperating_vectors[i + 1]))
            {
                buckets[i].push_back(u);
                // std::cout << "Node " << u << " is in bucket " << i << "\n";
                reverse_map[u] = buckets[i].size() - 1;
                covered = true;
                break;
            }
        }
        if(covered == false)
        {
            HANDLE_ERROR("Node " + std::to_string(u) + " is not covered by any partition!");
        }
    }
    // #pragma omp parallel for
    // for(int i = 0; i < num_partitions; i++){
    //     for(node_t u = 1; u < N; u++) 
    //     {
    //         Vector vec(cvrp.node[depot].x, cvrp.node[depot].y, cvrp.node[u].x, cvrp.node[u].y);   
    //         if(vec.is_in_between(seperating_vectors[i], seperating_vectors[i + 1]))
    //         {
    //             buckets[i].push_back(u);
    //         }
    //     }
    // }
    return buckets;
}

struct MinHeapNode {
    int u;      // already present in MST
    int v;      // going to be present in MST
    int weight; // weight of the edge

    MinHeapNode (int _u, int _v, int _weight): u(_u), v(_v), weight(_weight) {}
    MinHeapNode () {}
    ~MinHeapNode() {}
};

void create_aux_graph(std::vector <std::vector<int>>& adj, std::vector <int>& depot_neighbours, const std::vector <node_t>& bucket, const CVRP& cvrp, const Parameters& par) {
    const int num_nodes = bucket.size();
    if(num_nodes == 1) return;

    MinHeap <MinHeapNode> min_heap(num_nodes);
    std::vector <bool> in_mst(num_nodes, false); // TODO: this can be removed and optimized, we should not create vectors each time you need

    const int depot         = 0;
    const int depot_index   = 0;
    int u_index             = depot_index;              // Start from the first node in the bucket (which is depot)
    in_mst[u_index]         = true;                     // Mark the first node as included in MST
    int completed           = 1;                        // Number of nodes included in the MST
    min_heap.DecreaseKey(MinHeapNode(-1, depot, 0));    
    MinHeapNode min_node    = min_heap.pop(); 
    int v_index;
    weight_t weight;

    for(v_index = 0; v_index < num_nodes; v_index++) {
        if(u_index == v_index) continue;
        min_heap.DecreaseKey(MinHeapNode(u_index, v_index, cvrp.get_distance_on_the_fly(bucket[u_index], bucket[v_index]))); 
    }

    node_t u, v;
    while(!min_heap.empty()) { 
        min_node    = min_heap.pop();
        u_index     = min_node.u;                                                   // Index of the node in the bucket
        v_index     = min_node.v;                                                   // Index of the adjacent node in the bucket
        weight      = min_node.weight;                                              // Weight of the edge
        if(in_mst[v_index]) continue;                                               // Skip if already in MST
        
        in_mst[v_index] = true;                                                     // Mark this node as included in MST
        completed++;

        // Add the edge to the graph
        u = bucket[u_index];                                                        // Get the actual node from the bucket
        v = bucket[v_index];                                                        // Get the actual node from the bucket
        if(u != depot) adj[u].push_back(v);                                         // Add edge u -> v
        else           depot_neighbours.push_back(v);
 
        if(v != depot) adj[v].push_back(u);                                         // Add edge v -> u (undirected graph)
        else           depot_neighbours.push_back(v);                               

        for(int w_index = 0; w_index < num_nodes; w_index++) {
            int w = bucket[w_index];
            if(in_mst[w_index]) continue;                                           // Skip already included nodes 
            min_heap.DecreaseKey(MinHeapNode(v_index, w_index, cvrp.get_distance_on_the_fly(v, w)));
        }
    }

    if(completed != num_nodes) {
        HANDLE_ERROR("Not all nodes are included in the MST! Completed: " + std::to_string(completed) + ", Expected: " + std::to_string(num_nodes));
    }
    return;
}

void run_our_method(const CVRP& cvrp, const Parameters& par, const CommandLineArgs& command_line_args)
{
    omp_set_nested(2); // Enable nested parallelism

    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    const size_t N = cvrp.size;
    node_t depot = cvrp.depot;

    // Make buckets 
    // Depot is present in first location in every bucket
    // Every customer has its own unique id in bucket
    std::vector <int> reverse_map(N);
    std::vector <std::vector <node_t>> buckets = make_partitions(par, cvrp, reverse_map);

    // For each bucket, find the minimum possible routes
    weight_t final_cost = 0.0;
    std::vector <std::vector<int>> final_routes;
    std::vector <std::vector <int>> shared_adj(N);

    #pragma omp parallel for 
    for(int b = 0; b < buckets.size(); b++) {
        std::vector <node_t> depot_neighbours;
        const int num_nodes = buckets[b].size();
        create_aux_graph(shared_adj, depot_neighbours, buckets[b], cvrp, par);

        weight_t min_cost = INT_MAX;                                                            // taking INT_MAX as infinity
        std::vector <std::vector<int>> min_routes;
        
        // Search in solution space using randomization
        // You should enable open mp nested parallelsim
        // You should check whether rng is thread safe
        // do you want to use intel vectors which has thread safe push_back???
        #pragma omp parallel for
        for(int iter = 1; iter <= par.rho; iter++) {
            std::random_device rd;
            std::mt19937 rng(rd());  
            std::vector <bool> visited(num_nodes, false);

            std::vector<std::vector<node_t>>    curr_routes;
            weight_t                            curr_total_cost     = 0.0;
            unsigned int                        covered             = 1;                            // Start with depot covered
            std::vector<node_t>                 current_route;
            capacity_t                          residue_capacity    = cvrp.capacity;
            node_t                              prev_node           = depot;                       // Assuming local id of depot is also depot which is 0 
            weight_t                            curr_route_cost     = 0.0;

            // DFS iterative
            std::stack< std::pair <int, int*>> rec;
            uint32_t                            neigh_size          = depot_neighbours.size();
            int32_t*                            neigh               = new int32_t[neigh_size];
            std::copy(depot_neighbours.begin(), depot_neighbours.end(), neigh);
            std::shuffle(neigh, neigh + neigh_size, rng);
            rec.push({neigh_size - 1, neigh});
            visited[depot]      = true;
            int index;

            while(!rec.empty()) {
                index = rec.top().first; 
                neigh = rec.top().second;

                bool pushed = false;
                while(index >= 0) {
                    node_t v        = neigh[index];
                    node_t v_index  = reverse_map[v];
                    if(!visited[v_index]) {
                        if(residue_capacity >= cvrp.node[v].demand) {
                            current_route.push_back(v);
                            curr_route_cost     += cvrp.get_distance_on_the_fly(prev_node, v);
                            residue_capacity    -= cvrp.node[v].demand;
                            prev_node           = v;                                                            // Update previous node to current vertex
                        } else {
                            covered             += current_route.size(); 
                            curr_routes.push_back(std::move(current_route));
                            curr_route_cost     += cvrp.get_distance_on_the_fly(prev_node, depot); 
                            curr_total_cost     += curr_route_cost;
                            current_route.clear();
                            prev_node           = depot; 
                            curr_route_cost     = 0.0; 
                            residue_capacity    = cvrp.capacity; 
            
                            // Start a new route
                            current_route.push_back(v);
                            residue_capacity    -= cvrp.node[v].demand;
                            curr_route_cost     += cvrp.get_distance_on_the_fly(prev_node, v);
                            prev_node           = v; 
                        }
                        visited[v_index]        = true;
                        rec.top().first         = index - 1; 

                        neigh_size  = shared_adj[v].size();
                        neigh       = new int[neigh_size];
                        std::copy(shared_adj[v].begin(), shared_adj[v].end(), neigh);
                        std::shuffle(neigh, neigh + neigh_size, rng);
                        rec.push({neigh_size - 1, neigh});
                        pushed      = true;         
                        break;
                    }
                    index--;
                }
                if(!pushed) {
                    free(rec.top().second);
                    rec.pop();
                }
            }
            
            // If there are any remaining nodes in the current route, add it to routes
            if(!current_route.empty()) { 
                covered         += current_route.size(); 
                curr_routes.push_back(std::move(current_route));
                curr_route_cost += cvrp.get_distance_on_the_fly(prev_node, depot); 
                curr_total_cost += curr_route_cost; 
            }

            // if(covered != num_nodes)
            // {
            //     HANDLE_ERROR("Not all nodes are covered in the bucket " + std::to_string(b) + "! Covered: " + std::to_string(covered) + ", Expected: " + std::to_string(num_nodes));
            // }

            // Step: Update the running total cost
            #pragma omp critical
            {
                if(curr_total_cost < min_cost) {
                    min_cost   = curr_total_cost;
                    min_routes = curr_routes; // Update the best routes found so far
                }
            }
        }

        if(min_routes.size() != 0)
        {
            #pragma omp critical
            {
                final_cost += min_cost;                                 // Update the final cost
                for(auto& route: min_routes) {
                    final_routes.push_back(std::move(route));
                }
            }
        }else{
            // This is case where ther are no vertices in the bucket other than depot
        }
    }
    // {
    //     OUTPUT_FILE << "----------------------------------------------\n";
    //     OUTPUT_FILE << "ROUTES_AFTER_LOOP\n";
    //     print_routes(final_routes, final_cost);
    //     OUTPUT_FILE << "----------------------------------------------\n";
    // }
    // if (std::abs(final_cost - get_total_cost_of_routes(cvrp, final_routes)) > 1e-3) {
    //     HANDLE_ERROR("Final cost != calculated cost in loop! Final cost: " + std::to_string(final_cost) + ", Calculated cost: " + std::to_string(get_total_cost_of_routes(cvrp, final_routes)));
    // }

    double time_till_loop = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() / 1000.0;
    // Refining routes using optimizations
    {
      // using rajesh code
      final_routes = postProcessIt(cvrp, final_routes, final_cost);  
    }

    auto end            = std::chrono::high_resolution_clock::now();
    double elapsed_time = std::chrono::duration<double>(end - start).count();

    // Validate the solution
    {
        // this is also rajesh code for verification
        if(!verify_sol(cvrp, final_routes, cvrp.capacity))
        {
            HANDLE_ERROR("Solution is not valid!");
        }
    }

    // {
    //     OUTPUT_FILE << "----------------------------------------------\n";
    //     OUTPUT_FILE << "ROUTES_AFTER_REFINEMENT\n";
    //     print_routes(final_routes, final_cost);
    //     OUTPUT_FILE << "----------------------------------------------\n";
    // }

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
    auto parameters = get_tunable_parameters(command_line_args);
    run_our_method(cvrp, parameters, command_line_args);
}
