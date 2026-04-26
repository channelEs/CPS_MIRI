#include <gecode/int.hh>
#include <gecode/search.hh>
#include <gecode/minimodel.hh>
#include <iostream>
#include <vector>
#include <limits>
using namespace Gecode;
using namespace std;

// limit vlue hardcoded (used as the upper bound in distance)
const int INF_INT = 1000; // std::numeric_limits<int>::max();

class StreetDirectionalProblem_CP : public Space {
protected:
    int n;

    IntVarArray open_streets_mat; // 'boolean' matrix which each index tells if i->j is open able
    
    // main custom data structure
    // is a 3 dimension matrix and it is used to iterate trying to optimize the solution
    // the base is the n * n matrix with the time value going i->j, the third dimension (k) is used like a iteration history
    // in other words, is the 'space' needed to compute/try all the possibilities of using new paths of T
    // each dimension of K saves the improved time from i->j (if there is no improvement it will stay the same value as the k=0 dimension)
    IntVarArray travel_times;
    IntVar converted_streets; // counter for the number of converted two way streets
        
    // this is an axiliar function to get the specific index
    // the code was a bit messy to follow having to use this formula all the time to access an specific index :)
    int idx(int k, int i, int j) const {
        return k * n * n + i * n + j;
    }

public:
    StreetDirectionalProblem_CP(int n, const vector<vector<int>>& T, const vector<vector<int>>& max_time_allowed, int total_two_way) 
        : n(n), 
          open_streets_mat(*this, n * n, 0, 1), 
          travel_times(*this, (n + 1) * n * n, 0, INF_INT), //value domain: from 0 to large number 
          converted_streets(*this, 0, total_two_way) 
    {
        // constraints for the base of the matrix times
        // I just constraint the values of the k=0 dimension, making sure that they are 0, INF_INT or an specific value depending on the case
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                if (i == j) {
                    rel(*this, travel_times[idx(0, i, j)] == 0);
                } else if (T[i][j] != -1) {
                    IntVar time_value = expr(*this, ite(open_streets_mat[i * n + j] == 1, T[i][j], INF_INT));
                    rel(*this, travel_times[idx(0, i, j)] == time_value);
                } else {
                    rel(*this, travel_times[idx(0, i, j)] == INF_INT);
                }
            }
        }

        // now the constraints on how to compute the solution
        for (int k = 0; k < n; ++k) {
            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n; ++j) {
                    // for each k dimension ('iterations') add:

                    IntVar opt1 = travel_times[idx(k, i, j)]; // the value from the last iteration
                    // or
                    IntVar opt2 = expr(*this, travel_times[idx(k, i, k)] + travel_times[idx(k, k, j)]); // the value of adding the street k as intersaction
                    
                    // it is just: new time value = (i->j OR i->j->k)
                    // IntVar opt2_capped = expr(*this, Gecode::min(opt2, INF_INT));
                    // rel(*this, travel_times[idx(k + 1, i, j)] == Gecode::min(opt1, opt2_capped));
                    rel(*this, travel_times[idx(k + 1, i, j)] == Gecode::min(opt1, opt2));
                }
            }
        }
        // final time constraint
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                // it cannot pass the upper bound calculated in the main (%P formula)
                if (max_time_allowed[i][j] != -1) {
                    rel(*this, travel_times[idx(n, i, j)] <= max_time_allowed[i][j]);
                } else {
                    rel(*this, travel_times[idx(n, i, j)] == INF_INT);
                }
            }
        }

        IntVarArgs active_two_way_edges; 
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                if (T[i][j] != -1 && T[j][i] != -1) {
                    // if is a two way street, it can be converted but I added this constraint to respect that the street cannot be closed:
                    // either i->j is closed or j->i is closed (with the >= 1)
                    rel(*this, open_streets_mat[i * n + j] + open_streets_mat[j * n + i] >= 1);
                    // appends the state of both ways from i to j (index: i * n + j | j * n + i)
                    active_two_way_edges << open_streets_mat[i * n + j] << open_streets_mat[j * n + i];
                } else if (T[i][j] != -1) {
                    // constraint to keep the street as it is (cannot be deleted)
                    rel(*this, open_streets_mat[i * n + j] == 1); rel(*this, open_streets_mat[j * n + i] == 0); 
                } else if (T[j][i] != -1) {
                    // same but for other way
                    rel(*this, open_streets_mat[i * n + j] == 0); rel(*this, open_streets_mat[j * n + i] == 1);
                } else {
                    // same, this is when there is no direction for both streets
                    rel(*this, open_streets_mat[i * n + j] == 0); rel(*this, open_streets_mat[j * n + i] == 0);
                }
            }
        }

        if (total_two_way > 0) {
            // the idea is to sum all the 1s from total_two_way to know how many streets have been converted
            IntVar sum_possible_directions(*this, total_two_way, 2 * total_two_way); // note: the (total_two_way, 2 * total_two_way) is the possible range 
            linear(*this, active_two_way_edges, IRT_EQ, sum_possible_directions); // active_two_ways == sum_possible_directions
            rel(*this, converted_streets == (2 * total_two_way) - sum_possible_directions);
            // with the substraction I get how many streets have converted
        } else {
            rel(*this, converted_streets == 0);
        }

        branch(*this, open_streets_mat, INT_VAR_DEGREE_MAX(), INT_VAL_MIN());
    }

    StreetDirectionalProblem_CP(StreetDirectionalProblem_CP& s) : Space(s), n(s.n) {
        open_streets_mat.update(*this, s.open_streets_mat);
        travel_times.update(*this, s.travel_times);
        converted_streets.update(*this, s.converted_streets);
    }
    virtual Space* copy() { 
        return new StreetDirectionalProblem_CP(*this); 
    }

    virtual void constrain(const Space& _best) override {
        const StreetDirectionalProblem_CP& best = 
            static_cast<const StreetDirectionalProblem_CP&>(_best);
        
        int best_score = best.converted_streets.val();
        
        // Check if the new solution is better than the current best
        // this constraint ensures that the nexts solutions will be better
        rel(*this, converted_streets > best_score);
    }

    // print for debugging
    // to use the 'debug version' just uncomment in the main() part
    void print() const {
        cout << "---------------------" << endl;
        cout << "Converted streets: " << converted_streets.val() << endl;
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                if (open_streets_mat[i * n + j].assigned() && open_streets_mat[i * n + j].val() == 1) {
                    // cout << i << " " << j << endl;
                }
            }
        }
        cout << "Initial Decisions (Layer 0):" << endl;
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                cout << open_streets_mat[i*n + j].val() << " "; 
            }
            cout << endl;
        }

        cout << "Final Distance Matrix (Layer " << n << "):" << endl;
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                int val = travel_times[idx(n, i, j)].val();
                cout << (val >= 1000 ? -1 : val) << "\t";
            }
            cout << endl;
        }
    }
    
    void final_print(const vector<vector<int>>& T) const {
        // cout << "BEST SOLUTION FOUND || Final Distance Matrix:" << endl;
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                if (T[i][j] != -1 && T[j][i] != -1) {
                    if (open_streets_mat[i * n + j].val() == 1 && open_streets_mat[j * n + i].val() == 0) {
                        cout << i << " " << j << endl;
                    } else if (open_streets_mat[i * n + j].val() == 0 && open_streets_mat[j * n + i].val() == 1) {
                        cout << j << " " << i << endl;
                    }
                }
            }
        }
        cout << converted_streets.val() << endl;
    }

    int getConvertedStreets() {
        return converted_streets.val();
    }
};

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
    // Once the input is readed/imported, we have T as the input 'graph'
    // the idea is to pre-proces the input so the model can be more efficient:

    vector<vector<int>> max_time_allowed_matrix(n, vector<int>(n, -1)); // matrix with the maximum traver time allowed by the statement (the %P function)
    // this will is used as the upper bound that the model has to respect in each path
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (streets_graph[i][j] != -1) {
                int limit = streets_graph[i][j] + (streets_graph[i][j] * (P / 100.0));
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
    total_two_way = n * (n - 1) / 2;

    // here ends the input pre proces and the model is initialized

    StreetDirectionalProblem_CP* m = new StreetDirectionalProblem_CP(n, streets_graph, max_time_allowed_matrix, total_two_way);
    BAB<StreetDirectionalProblem_CP> search(m);
    StreetDirectionalProblem_CP* best_solution = nullptr;
    // int best_count_converted = 0;

    while (StreetDirectionalProblem_CP* s = search.next()) {
        // s.print(); // debug print 
        delete best_solution;
        best_solution = s; 
    }
    
    if (best_solution) {
        best_solution->final_print(streets_graph);
        delete best_solution;
    } else {
        cout << "0 || not best solution" << endl; 
    }
}