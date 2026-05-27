#include <ilcplex/ilocplex.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <algorithm>

ILOSTLBEGIN
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
    auto start_time = chrono::high_resolution_clock::now();

    // Once the input is readed/imported, we have T as the input 'graph'
    // the idea is to pre-proces the input so the model can be more efficient:

    vector<vector<int>> orig_shortest_mat = streets_graph;
    vector<vector<int>> max_time_allowed_matrix(n, vector<int>(n, -1)); // matrix with the maximum traver time allowed by the statement (the %P function)
    
    // To calculate the maximum time allowed for each path i->j, first I need to know the current shortest path:
    // I used the loop from the CHECKER.cc that iterates over all possible paths, and I added a checker to take the best option.
    // later in the model, this will be used as the upper bound that the model has to respect in each path
    for (int k = 0; k < n; ++ k)
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j) {
                if (orig_shortest_mat[i][k] != -1 && orig_shortest_mat[k][j] != -1) {
                    if (orig_shortest_mat[i][j] != -1) orig_shortest_mat[i][j] = min(orig_shortest_mat[i][j], orig_shortest_mat[i][k] + orig_shortest_mat[k][j]);
                    else orig_shortest_mat[i][j] = orig_shortest_mat[i][k] + orig_shortest_mat[k][j];
                }
            }
            
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (orig_shortest_mat[i][j] != -1) {
                int limit = orig_shortest_mat[i][j] + (orig_shortest_mat[i][j] * (P / 100.0));
                max_time_allowed_matrix[i][j] = limit;
                // max_time_allowed_matrix[i][j] = INT_MAX;
            }
        }
    }

    // Also with the input, the maximum of possible converted streets can be computed (just the streets with both i->j and j->i different to -1)
    int total_two_way = 0;
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) { 
            if (streets_graph[i][j] != -1 && streets_graph[j][i] != -1) total_two_way++;
        }
    }
    // total_two_way = n * (n - 1) / 2;
    // here ends the input pre proces and the model is initialized

    IloEnv env;
    try {
        IloModel model(env);

        // --- VARIABLES ---
        // binary variable, 1 if street i -> j is open
        // represented as an array of size n*n, where the index is i*n + j
        IloBoolVarArray streets_state(env, n * n);

        // current_paths[s][d][i][j]: int variable. 1 if path from s to d uses street i -> j.
        // the first two dimensions represents the source and destination, 
        // and the last two dimensions represents if the street from i to j is used in the path
        vector<vector<vector<vector<IloNumVar>>>> current_paths(n, vector<vector<vector<IloNumVar>>>(n, 
               vector<vector<IloNumVar>>(n, vector<IloNumVar>(n))));

        for (int s = 0; s < n; ++s) {
            for (int d = 0; d < n; ++d) {
                if (s == d || max_time_allowed_matrix[s][d] == -1) continue;
                for (int i = 0; i < n; ++i) {
                    for (int j = 0; j < n; ++j) {
                        if (streets_graph[i][j] != -1) {
                            current_paths[s][d][i][j] = IloNumVar(env, 0, 1, ILOINT);
                        }
                    }
                }
            }
        }

        IloExpr converted_expr(env);
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                if (streets_graph[i][j] != -1 && streets_graph[j][i] != -1) {
                    // in case there is a two-way street:
                    // one must stick open
                    model.add(streets_state[i * n + j] + streets_state[j * n + i] >= 1);
                    // and this expresion tells to the objective functions that is better to convert one of them,
                    // if both are open, the contribution to the objective function is 0, 
                    // if one of them is closed, the contribution is 1
                    converted_expr += (2 - (streets_state[i * n + j] + streets_state[j * n + i]));
                } else if (streets_graph[i][j] != -1) {
                    // one-way street i -> j
                    model.add(streets_state[i * n + j] == 1);
                    model.add(streets_state[j * n + i] == 0);
                } else if (streets_graph[j][i] != -1) {
                    // one-way street j -> i
                    model.add(streets_state[i * n + j] == 0);
                    model.add(streets_state[j * n + i] == 1);
                } else { 
                    model.add(streets_state[i * n + j] == 0);
                    model.add(streets_state[j * n + i] == 0);
                }
            }
        }

        // Maximize number of converted streets
        model.add(IloMaximize(env, converted_expr));
        converted_expr.end();

        // --- FLOW AND TIME CONSTRAINTS ---
        for (int s = 0; s < n; ++s) {
            for (int d = 0; d < n; ++d) {
                if (s == d || max_time_allowed_matrix[s][d] == -1) continue;

                IloExpr path_length(env);

                for (int i = 0; i < n; ++i) {
                    // in here I set the flow constraint and the max time constraint
                    IloExpr flow_out(env);
                    IloExpr flow_in(env);

                    for (int j = 0; j < n; ++j) {
                        if (streets_graph[i][j] != -1) {
                            // Path check: Cannot send flow through a closed street
                            model.add(current_paths[s][d][i][j] <= streets_state[i * n + j]);
                            flow_out += current_paths[s][d][i][j];
                            path_length += (streets_graph[i][j] * current_paths[s][d][i][j]);
                        }
                        if (streets_graph[j][i] != -1) {
                            flow_in += current_paths[s][d][j][i];
                        }
                    }

                    // to know if there is a continious path from s to d:
                    // the flow that goes out of a node must be the same that goes in
                    
                    // !! except for the source and destination !!
                    if (i == s) {
                        model.add(flow_out - flow_in == 1); // Source sends 1 unit
                    } else if (i == d) {
                        model.add(flow_out - flow_in == -1); // Destination receives 1 unit
                    } else {
                        model.add(flow_out - flow_in == 0); // Intermediate nodes pass flow cleanly
                    }
                    flow_out.end();
                    flow_in.end();
                }

                // As in CP, I used the precomputed max_time allowed for each path, so I can easily add the constraint
                model.add(path_length <= max_time_allowed_matrix[s][d]);
                path_length.end();
            }
        }
        IloCplex cplex(model);
        // THIS NEEDS TO BE SET!! CPLEX prints a lot of strange logs (good for debugging, but the checker fails ;( )
        cplex.setOut(env.getNullStream());
        cplex.setWarning(env.getNullStream());

        if (cplex.solve()) {
            auto end_time = chrono::high_resolution_clock::now();
            chrono::duration<double> diff = end_time - start_time;

            cout << n << endl;
            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n; ++j) {
                    cout << streets_graph[i][j] << (j == n - 1 ? "" : " ");
                }
                cout << endl;
            }
            cout << (int)P << endl; // as the statement says "only contain integer values"
            for (int i = 0; i < n; ++i) {
                for (int j = i + 1; j < n; ++j) {
                    if (streets_graph[i][j] != -1 && streets_graph[j][i] != -1) {
                        bool open_ij = (cplex.getValue(streets_state[i * n + j]) > 0.5);
                        bool open_ji = (cplex.getValue(streets_state[j * n + i]) > 0.5);

                        // If it was converted to one-way i -> j
                        if (open_ij && !open_ji) {
                            cout << i << " " << j << endl;
                        }
                        // If it was converted to one-way j -> i
                        else if (!open_ij && open_ji) {
                            cout << j << " " << i << endl;
                        }
                    }
                }
            }
            cout << (int)(cplex.getObjValue() + 0.5) << endl;
            
            // checker ignores cerr
            cerr << "ILP Execution Time: " << diff.count() << " s\n";
            
        } else {
            cout << 0 << endl;
            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n; ++j) {
                    cout << streets_graph[i][j] << (j == n - 1 ? "" : " ");
                }
                cout << endl;
            }
        }

    } catch (IloException& e) {
        cerr << "Concert exception caught: " << e << endl;
    } 

    env.end();
    return 0;
}