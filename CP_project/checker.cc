#include <iostream>
#include <vector>
#include <set>
#include <assert.h>

using namespace std;

vector<vector<int>> T, U;
double P;
vector<pair<int,int>> res;

vector<vector<int>> floyd(vector<vector<int>> F) {
  int n = F.size();
  for (int x = 0; x < n; ++x) assert(F[x][x] == 0);

  for (int k = 0; k < n; ++ k)
    for (int i = 0; i < n; ++i)
      for (int j = 0; j < n; ++j) {
        if (F[i][k] != -1 and F[k][j] != -1) {
          if (F[i][j] != -1)
            F[i][j] = min(F[i][j], F[i][k] + F[k][j]);
          else
            F[i][j] = F[i][k] + F[k][j];
        }
      }

  return F;
}


void read_input() {
  int n;
  cin >> n;
  T = vector<vector<int>>(n, vector<int>(n));
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j)
      cin >> T[i][j];
  
  cin >> P;
}


bool check_input() {

  int n = T.size();

  for (int i = 0; i < n; ++i) {
    if (T[i][i] != 0) {
      cout << "Error: input has non-zeroes in the diagonal of the matrix" << endl;
      return false;
    }
    for (int j = 0; j < n; ++j) {
      if (T[i][j] != -1 and T[j][i] != -1 and T[i][j] != T[j][i]) {
	cout << "Error: "
	     << "T[" << i << "][" << j << "] = " << T[i][j] << " and "
	     << "T[" << j << "][" << i << "] = " << T[j][i] << " do not meet the precondition" << endl;
	return false;
      }
    }
  }

  if (P <= 0) {
    cout << "Error: P = " << P << " does not meet the precondition" << endl;
    return false;
  }

  return true;
}


bool read_output() {

  int n = T.size();

  U = T;

  int i, j;
  set<pair<int,int>> s;
  while (cin >> i >> j) {
    if (not (i >= 0 and i < n)) {
      cout << "Error: " << i << " is not a valid crossing" << endl;
      return false;
    }
    if (not (j >= 0 and j < n)) {
      cout << "Error: " << j << " is not a valid crossing" << endl;
      return false;
    }
    if (T[i][j] == -1 or T[j][i] == -1) {
      cout << "Error: there is no two-way street connecting " << i << " and " << j << endl;
      return false;
    }
    U[j][i] = -1; // if we can only go from i to j, then j -> i has now +oo cost
    s.insert({i, j});
    
  }
  int     opt = s.size();
  int exp_opt = i;
  
  if (exp_opt != opt) {
    cout << "Error: reported objective value (" << exp_opt << ") does not match"
	    << " the objective value of the reported solution (" << opt << ")" << endl;
    return false;
  }
  else return true;
}


bool check_output() {

  auto FT = floyd(T);
  auto FU = floyd(U);

  int n = T.size();
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j)
      if (FT[i][j] != -1 and (FU[i][j] == -1 or (1 + P/100) * FT[i][j] < FU[i][j])) {
	cout << "Error: "
	     << "distance from " << i << " to " << j << " was "
	     << (FT[i][j] == -1 ? "+oo" : to_string(FT[i][j]))
	     << " but is now "
	     << (FU[i][j] == -1 ? "+oo" : to_string(FU[i][j])) << endl;
	return false;
      }

  return true;
}


void check() {
  read_input();

  bool ok = check_input();
  if (not ok) return;

  ok = read_output();
  if (not ok) return;

  ok = check_output();
  if (not ok) return;

  cout << "OK" << endl;
}


int main(int argc, char *argv[]) {

  // Write help message.
  if (argc != 1) {
    cout << "Makes a sanity check of a solution to the Street Directionality Problem" << endl;
    cout << "Usage: " << argv[0] << " < output_file" << endl;
    exit(0);
  }

  check();
}
