#include <iostream>
#include <vector>
#include <chrono>
#include <algorithm>

using namespace std;

struct TwoWayStreet {
    int u, v;
};

static double elapsed_seconds(const chrono::steady_clock::time_point& start_time) {
    auto now = chrono::steady_clock::now();
    return chrono::duration<double>(now - start_time).count();
}

static vector<vector<int>> find_shortest_paths(vector<vector<int>> shortest_mat) {
    int n = static_cast<int>(shortest_mat.size());
    for (int k = 0; k < n; ++k) {
        for (int i = 0; i < n; ++i) {
            if (shortest_mat[i][k] == -1) continue;
            for (int j = 0; j < n; ++j) {
                if (shortest_mat[k][j] == -1) continue;
                int cand = shortest_mat[i][k] + shortest_mat[k][j];
                if (shortest_mat[i][j] == -1 || cand < shortest_mat[i][j]) shortest_mat[i][j] = cand;
            }
        }
    }
    return shortest_mat;
}

static bool is_arc_open(const vector<int>& street_state,
                        int pair_index,
                        int from,
                        int to,
                        const vector<vector<int>>& streets_graph,
                        const vector<TwoWayStreet>& two_way_streets) {
    if (pair_index < 0) return streets_graph[from][to] != -1;
    int u = two_way_streets[pair_index].u;
    int v = two_way_streets[pair_index].v;
    if (street_state[pair_index] == 0) return true;
    if (street_state[pair_index] == 1) return (from == u && to == v);
    return (from == v && to == u);
}

static vector<vector<int>> build_directed_graph(const vector<int>& street_state,
                                                const vector<vector<int>>& streets_graph,
                                                const vector<TwoWayStreet>& two_way_streets) {
    // Street Integrity constraint:
    // Build a feasible directed graph from the original while only modifying directions of original two-way streets
    int n = static_cast<int>(streets_graph.size());
    vector<vector<int>> directed_graph = streets_graph;
    for (int i = 0; i < n; ++i) {
        directed_graph[i][i] = 0;
    }
    for (int pair_index = 0; pair_index < static_cast<int>(two_way_streets.size()); ++pair_index) {
        int u = two_way_streets[pair_index].u;
        int v = two_way_streets[pair_index].v;
        if (street_state[pair_index] == 1) {
            directed_graph[v][u] = -1;
        } else if (street_state[pair_index] == 2) {
            directed_graph[u][v] = -1;
        }
    }
    return directed_graph;
}

static int count_converted_streets(const vector<int>& street_state) {
    // Objective variable (sum of C_m)
    int converted = 0;
    for (int value : street_state) {
        if (value != 0) ++converted;
    }
    return converted;
}

static vector<pair<int, int>> collect_qos_violations(const vector<vector<int>>& candidate_shortest_mat,
                                                     const vector<vector<int>>& original_shortest_mat,
                                                     const vector<vector<int>>& max_time_allowed) {
    // Travel Time Limit constraint:
    // detect all (source, destination) pairs that violate the allowed degradation bound
    int n = static_cast<int>(candidate_shortest_mat.size());
    vector<pair<int, int>> violations;
    for (int source = 0; source < n; ++source) {
        for (int destination = 0; destination < n; ++destination) {
            if (original_shortest_mat[source][destination] == -1) continue;
            if (candidate_shortest_mat[source][destination] == -1 ||
                candidate_shortest_mat[source][destination] > max_time_allowed[source][destination]) {
                violations.push_back({source, destination});
            }
        }
    }
    return violations;
}

static vector<int> repair_state(vector<int> street_state,
                                const vector<vector<int>>& streets_graph,
                                const vector<vector<int>>& original_shortest_mat,
                                const vector<vector<int>>& max_time_allowed,
                                const vector<TwoWayStreet>& two_way_streets,
                                const vector<vector<int>>& pair_index_by_nodes,
                                const chrono::steady_clock::time_point& start_time,
                                double time_out_bound) {
    // Constraint-repair loop: keeps applying corrections until there are no violations or the time bound is exceeded
    int n = static_cast<int>(streets_graph.size());
    int total_two_way = static_cast<int>(two_way_streets.size());
    int max_rounds = max(20, 2 * n + total_two_way / 2);

    for (int round = 0; round < max_rounds; ++round) {
        if (elapsed_seconds(start_time) > time_out_bound) break;

        vector<vector<int>> directed_graph = build_directed_graph(street_state, streets_graph, two_way_streets);
        vector<vector<int>> candidate_shortest_mat = find_shortest_paths(directed_graph);
        vector<pair<int, int>> violations = collect_qos_violations(candidate_shortest_mat, original_shortest_mat, max_time_allowed);
        if (violations.empty()) {
            return street_state;
        }

        bool changed = false;

        for (const auto& source_destination : violations) {
            int source = source_destination.first;
            int destination = source_destination.second;
            int current = source;

            // Reachability/propagation idea:
            // follow an original shortest-path chain and reopen blocked arcs when needed
            while (current != destination) {
                int next_closed = -1;
                int next_any = -1;

                for (int next = 0; next < n; ++next) {
                    if (next == current) continue;
                    if (streets_graph[current][next] == -1 || original_shortest_mat[next][destination] == -1) continue;
                    if (streets_graph[current][next] + original_shortest_mat[next][destination] != original_shortest_mat[current][destination]) continue;

                    if (next_any == -1) next_any = next;

                    int pair_index = pair_index_by_nodes[current][next];
                    if (pair_index != -1 && !is_arc_open(street_state, pair_index, current, next, streets_graph, two_way_streets)) {
                        next_closed = next;
                        break;
                    }
                }

                if (next_closed != -1) {
                    int pair_index = pair_index_by_nodes[current][next_closed];
                    if (street_state[pair_index] != 0) {
                        street_state[pair_index] = 0;
                        changed = true;
                    }
                    current = next_closed;
                } else if (next_any != -1) {
                    current = next_any;
                } else {
                    break;
                }
            }
        }

        if (!changed) {
            // Fallback move to keep progress: reopen one converted pair
            for (int pair_index = 0; pair_index < total_two_way; ++pair_index) {
                if (street_state[pair_index] != 0) {
                    street_state[pair_index] = 0;
                    changed = true;
                    break;
                }
            }
            if (!changed) break;
        }
    }

    return street_state;
}

int main() {
    int n;
    if (!(cin >> n)) return 0;

    vector<vector<int>> streets_graph(n, vector<int>(n));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            cin >> streets_graph[i][j];
        }
    }

    double P;
    cin >> P;
    
    // Start Timer
    auto start_time = chrono::steady_clock::now();
    const double TIME_OUT_BOUND = 60.0;

    // To calculate the maximum time allowed for each path i->j, first I need to know the current shortest path:
    // I used the loop from the CHECKER.cc that iterates over all possible paths, and I added a checker to take the best option.
    // later in the model, this will be used as the upper bound that the model has to respect in each path
    vector<vector<int>> orig_shortest_mat = find_shortest_paths(streets_graph);
    vector<vector<int>> max_time_allowed(n, vector<int>(n, -1));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (orig_shortest_mat[i][j] != -1) {
                max_time_allowed[i][j] = static_cast<int>(orig_shortest_mat[i][j] * (1.0 + P / 100.0));
            }
        }
    }

    vector<TwoWayStreet> two_way_streets;
    vector<vector<int>> pair_idx(n, vector<int>(n, -1));
    // Street Integrity preprocessing:
    // index only original two-way streets, because those are the only streets that can be converted
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (streets_graph[i][j] != -1 && streets_graph[j][i] != -1) {
                int idx = static_cast<int>(two_way_streets.size());
                two_way_streets.push_back({i, j});
                pair_idx[i][j] = idx;
                pair_idx[j][i] = idx;
            }
        }
    }

    int total_two_way = static_cast<int>(two_way_streets.size());

    // best_k is the optimization target objective (maximize converted streets)
    vector<int> best_state(total_two_way, 0);
    int best_k = 0;

    vector<long long> out_score(n, 0);
    for (int i = 0; i < n; ++i) {
        long long acc = 0;
        for (int j = 0; j < n; ++j) {
            if (orig_shortest_mat[i][j] != -1) acc += orig_shortest_mat[i][j];
        }
        out_score[i] = acc;
    }

    vector<vector<int>> seeds;
    // Initial direction seeds emulate the SAT search exploration with different orientation biases
    seeds.push_back(vector<int>(total_two_way, 1));
    seeds.push_back(vector<int>(total_two_way, 2));
    seeds.push_back(vector<int>(total_two_way, 1));
    for (int idx = 0; idx < total_two_way; ++idx) {
        int u = two_way_streets[idx].u;
        int v = two_way_streets[idx].v;
        seeds[2][idx] = (out_score[u] <= out_score[v]) ? 1 : 2;
    }

    for (const auto& seed : seeds) {
        if (elapsed_seconds(start_time) > TIME_OUT_BOUND) break;
        vector<int> candidate = repair_state(seed,
                                             streets_graph,
                                             orig_shortest_mat,
                                             max_time_allowed,
                                             two_way_streets,
                                             pair_idx,
                                             start_time,
                                             TIME_OUT_BOUND);
        vector<vector<int>> candidate_graph = build_directed_graph(candidate, streets_graph, two_way_streets);
        vector<vector<int>> candidate_shortest_mat = find_shortest_paths(candidate_graph);

        // Travel Time constraint: every pair must stay within max_time_allowed.
        if (collect_qos_violations(candidate_shortest_mat, orig_shortest_mat, max_time_allowed).empty()) {
            int cand_k = count_converted_streets(candidate);
            if (cand_k > best_k) {
                best_k = cand_k;
                best_state = candidate;
            }
        }
    }

    if (best_k == 0 && total_two_way > 0) {
        // Safety fallback: keep every two-way street open if no converted
        vector<int> zero_state(total_two_way, 0);
        vector<vector<int>> zero_graph = build_directed_graph(zero_state, streets_graph, two_way_streets);
        vector<vector<int>> zero_shortest_mat = find_shortest_paths(zero_graph);
        if (collect_qos_violations(zero_shortest_mat, orig_shortest_mat, max_time_allowed).empty()) {
            best_state = zero_state;
            best_k = 0;
        }
    }

    cout << n << endl;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            cout << streets_graph[i][j] << (j + 1 == n ? "" : " ");
        }
        cout << endl;
    }
    cout << static_cast<int>(P) << endl;

    for (int idx = 0; idx < total_two_way; ++idx) {
        int u = two_way_streets[idx].u;
        int v = two_way_streets[idx].v;
        if (best_state[idx] == 1) cout << u << " " << v << endl;
        else if (best_state[idx] == 2) cout << v << " " << u << endl;
    }
    cout << best_k << endl;

    auto end_time = chrono::steady_clock::now();
    chrono::duration<double> diff = end_time - start_time;
    cerr << "SAT Execution Time: " << diff.count() << " s\n";

    return 0;
}
