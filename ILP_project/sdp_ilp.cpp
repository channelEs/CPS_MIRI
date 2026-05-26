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
        // E[i*n + j]: Binary variable, 1 if street i -> j is kept open
        IloBoolVarArray E(env, n * n);
        auto idx2 = [&](int i, int j) { return i * n + j; };

        // X[s][d][i][j]: Flow variable. 1 if path from s to d uses street i -> j.
        // We use a 4D vector of IloNumVar to only allocate memory for paths that exist.
        // Using ILOFLOAT (continuous 0.0 to 1.0) is much faster for flow conservation than ILOINT!
        vector<vector<vector<vector<IloNumVar>>>> X(n, vector<vector<vector<IloNumVar>>>(n, 
               vector<vector<IloNumVar>>(n, vector<IloNumVar>(n))));

        for (int s = 0; s < n; ++s) {
            for (int d = 0; d < n; ++d) {
                if (s == d || max_time_allowed_matrix[s][d] == -1) continue;
                for (int i = 0; i < n; ++i) {
                    for (int j = 0; j < n; ++j) {
                        if (streets_graph[i][j] != -1) {
                            X[s][d][i][j] = IloNumVar(env, 0, 1, ILOFLOAT);
                        }
                    }
                }
            }
        }

        IloExpr converted_expr(env);
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                if (streets_graph[i][j] != -1 && streets_graph[j][i] != -1) {
                    // At least one direction must be open
                    model.add(E[idx2(i, j)] + E[idx2(j, i)] >= 1);
                    // Add to objective sum
                    converted_expr += (2 - (E[idx2(i, j)] + E[idx2(j, i)]));
                } else if (streets_graph[i][j] != -1) {
                    // One-way street i -> j
                    model.add(E[idx2(i, j)] == 1);
                    model.add(E[idx2(j, i)] == 0);
                } else if (streets_graph[j][i] != -1) {
                    // One-way street j -> i
                    model.add(E[idx2(i, j)] == 0);
                    model.add(E[idx2(j, i)] == 1);
                } else {
                    // No street 
                    model.add(E[idx2(i, j)] == 0);
                    model.add(E[idx2(j, i)] == 0);
                }
            }
        }

        // Maximize converted streets
        model.add(IloMaximize(env, converted_expr));
        converted_expr.end();

        // --- FLOW AND TIME CONSTRAINTS ---
        for (int s = 0; s < n; ++s) {
            for (int d = 0; d < n; ++d) {
                if (s == d || max_time_allowed_matrix[s][d] == -1) continue;

                IloExpr path_length(env);

                for (int i = 0; i < n; ++i) {
                    IloExpr flow_out(env);
                    IloExpr flow_in(env);

                    for (int j = 0; j < n; ++j) {
                        if (streets_graph[i][j] != -1) {
                            // Path validity: Cannot send flow through a closed street
                            model.add(X[s][d][i][j] <= E[idx2(i, j)]);
                            flow_out += X[s][d][i][j];
                            path_length += (streets_graph[i][j] * X[s][d][i][j]);
                        }
                        if (streets_graph[j][i] != -1) {
                            flow_in += X[s][d][j][i];
                        }
                    }

                    // Flow Conservation
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
                model.add(path_length <= max_time_allowed_matrix[s][d]);
                path_length.end();
            }
        }
        IloCplex cplex(model);
        // THIS NEEDS TO BE SET!! CPLEX prints a lot of strange logs (good for debugging)
        cplex.setOut(env.getNullStream());
        cplex.setWarning(env.getNullStream());

        if (cplex.solve()) {
            auto end_time = chrono::high_resolution_clock::now();
            chrono::duration<double> diff = end_time - start_time;

            cout << (int)(cplex.getObjValue() + 0.5) << endl;
            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n; ++j) {
                    if (i == j) {
                        cout << 0;
                    } else if (cplex.getValue(E[idx2(i, j)]) > 0.5) {
                        cout << streets_graph[i][j];
                    } else {
                        cout << -1;
                    }
                    cout << (j == n - 1 ? "" : " ");
                }
                cout << endl;
            }
            
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