#include <iostream>
#include <vector>
#include <chrono>
#include <algorithm>

using namespace std;

struct TwoWayStreet {
    int u, v;
};

static vector<vector<int>> floyd(vector<vector<int>> F) {
    int n = static_cast<int>(F.size());
    for (int k = 0; k < n; ++k) {
        for (int i = 0; i < n; ++i) {
            if (F[i][k] == -1) continue;
            for (int j = 0; j < n; ++j) {
                if (F[k][j] == -1) continue;
                int cand = F[i][k] + F[k][j];
                if (F[i][j] == -1 || cand < F[i][j]) F[i][j] = cand;
            }
        }
    }
    return F;
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

    auto start_time = chrono::steady_clock::now();
    const double TIME_BUDGET_SEC = 58.0;

    auto elapsed_sec = [&]() {
        auto now = chrono::steady_clock::now();
        return chrono::duration<double>(now - start_time).count();
    };

    vector<vector<int>> orig_shortest_mat = floyd(streets_graph);
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

    auto is_arc_open = [&](const vector<int>& state, int idx, int from, int to) {
        if (idx < 0) return streets_graph[from][to] != -1;
        int u = two_way_streets[idx].u;
        int v = two_way_streets[idx].v;
        if (state[idx] == 0) return true;
        if (state[idx] == 1) return (from == u && to == v);
        return (from == v && to == u);
    };

    auto build_graph = [&](const vector<int>& state) {
        vector<vector<int>> U = streets_graph;
        for (int i = 0; i < n; ++i) {
            U[i][i] = 0;
        }
        for (int idx = 0; idx < total_two_way; ++idx) {
            int u = two_way_streets[idx].u;
            int v = two_way_streets[idx].v;
            if (state[idx] == 1) {
                U[v][u] = -1;
            } else if (state[idx] == 2) {
                U[u][v] = -1;
            }
        }
        return U;
    };

    auto count_converted = [&](const vector<int>& state) {
        int k = 0;
        for (int x : state) {
            if (x != 0) ++k;
        }
        return k;
    };

    auto collect_violations = [&](const vector<vector<int>>& FU) {
        vector<pair<int, int>> bad;
        for (int s = 0; s < n; ++s) {
            for (int d = 0; d < n; ++d) {
                if (orig_shortest_mat[s][d] == -1) continue;
                if (FU[s][d] == -1 || FU[s][d] > max_time_allowed[s][d]) {
                    bad.push_back({s, d});
                }
            }
        }
        return bad;
    };

    auto repair_state = [&](vector<int> state) {
        int max_rounds = max(20, 2 * n + total_two_way / 2);

        for (int round = 0; round < max_rounds; ++round) {
            if (elapsed_sec() > TIME_BUDGET_SEC) break;

            vector<vector<int>> U = build_graph(state);
            vector<vector<int>> FU = floyd(U);
            vector<pair<int, int>> bad = collect_violations(FU);
            if (bad.empty()) {
                return state;
            }

            bool changed = false;

            for (const auto& sd : bad) {
                int s = sd.first;
                int d = sd.second;
                int cur = s;

                while (cur != d) {
                    int next_closed = -1;
                    int next_any = -1;

                    for (int nxt = 0; nxt < n; ++nxt) {
                        if (nxt == cur) continue;
                        if (streets_graph[cur][nxt] == -1 || orig_shortest_mat[nxt][d] == -1) continue;
                        if (streets_graph[cur][nxt] + orig_shortest_mat[nxt][d] != orig_shortest_mat[cur][d]) continue;

                        if (next_any == -1) next_any = nxt;

                        int idx = pair_idx[cur][nxt];
                        if (idx != -1 && !is_arc_open(state, idx, cur, nxt)) {
                            next_closed = nxt;
                            break;
                        }
                    }

                    if (next_closed != -1) {
                        int idx = pair_idx[cur][next_closed];
                        if (state[idx] != 0) {
                            state[idx] = 0;
                            changed = true;
                        }
                        cur = next_closed;
                    } else if (next_any != -1) {
                        cur = next_any;
                    } else {
                        break;
                    }
                }
            }

            if (!changed) {
                for (int idx = 0; idx < total_two_way; ++idx) {
                    if (state[idx] != 0) {
                        state[idx] = 0;
                        changed = true;
                        break;
                    }
                }
                if (!changed) break;
            }
        }

        return state;
    };

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
    seeds.push_back(vector<int>(total_two_way, 1));
    seeds.push_back(vector<int>(total_two_way, 2));
    seeds.push_back(vector<int>(total_two_way, 1));
    for (int idx = 0; idx < total_two_way; ++idx) {
        int u = two_way_streets[idx].u;
        int v = two_way_streets[idx].v;
        seeds[2][idx] = (out_score[u] <= out_score[v]) ? 1 : 2;
    }

    for (const auto& seed : seeds) {
        if (elapsed_sec() > TIME_BUDGET_SEC) break;
        vector<int> candidate = repair_state(seed);
        vector<vector<int>> FU = floyd(build_graph(candidate));
        if (collect_violations(FU).empty()) {
            int cand_k = count_converted(candidate);
            if (cand_k > best_k) {
                best_k = cand_k;
                best_state = candidate;
            }
        }
    }

    // Safety fallback: always valid (no conversions).
    if (best_k == 0 && total_two_way > 0) {
        vector<int> zero_state(total_two_way, 0);
        vector<vector<int>> FU = floyd(build_graph(zero_state));
        if (collect_violations(FU).empty()) {
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
