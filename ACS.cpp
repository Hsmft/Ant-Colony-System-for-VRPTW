#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <random>
#include <ctime>
#include <iomanip>
#include <chrono>
#include <unordered_set>
#include <queue>
#include <map>
#include <unordered_map>
#include <cstdlib>
#include <atomic>
#include <tuple>

using namespace std;

// Forward declarations
struct Customer;
struct Route;
struct ProblemData;

// Prototypes for functions that will be used before their definitions
double evaluateSolution(const vector<Route>& routes, const ProblemData& data, long long& evalCount, int maxEvaluations, bool isFinal = false);
vector<Route> constructSolution(
    vector<vector<double>>& tau,
    const vector<vector<double>>& eta,
    const ProblemData& data,
    mt19937& rng,
    chrono::steady_clock::time_point startTime,
    int maxTime,
    double expoAlpha,
    double beta,
    double phi,
    double q0,
    double tau_0 );

vector<Route> antColonySystem(
    const ProblemData& data,
    mt19937& rng,
    int maxTime,
    int maxEvaluations,
    int maxiteration,  // پارامتر جدید اضافه شده
    chrono::steady_clock::time_point startTime,
    bool useLocalSearch = true
);

// Global variables
atomic<bool> stopAlgorithm(false);
bool timeLimitMessagePrinted = false;

// Customer structure
struct Customer {
    int id;
    double x, y;
    int demand;
    double serviceTime;
    double earliest;
    double latest;
};

// Route structure
struct Route {
    vector<int> customerIds; // Indices in the customers vector
    double totalDistance;
    vector<double> arrivalTimes;
    vector<double> departureTimes;
    int load;
    vector<double> min_slacks; // Minimum slack from each position to the end
    Route() : totalDistance(0.0), load(0) {}  };

// Problem data structure
struct ProblemData {
    vector<Customer> customers;
    int vehicleCapacity;
    int maxVehicles;
    double maxRouteDuration;
    unordered_map<int, int> idToIndex;
    vector<int> indexToId;
    double* distanceMatrix; // Changed to 1D array
    size_t numCustomers;};

// Helper function to print solution information
void printSolutionInfo(const vector<Route>& routes, const ProblemData& data, const string& label) {
    int vehicles = 0;
    double totalDistance = 0.0;
    for (const auto& route : routes) {
        if (!route.customerIds.empty()) {
            vehicles++;
            totalDistance += route.totalDistance;}}
    cout << label << ": Vehicles = " << vehicles << ", Total Distance = " << fixed << setprecision(2) << totalDistance << endl;}

// Function to print routes
void printSolution(const vector<Route>& routes, const ProblemData& data, const string& label) {
    cout << "Starting to print solution...\n";
    cout << label << ":\n";
    int routeNumber = 1;
    int vehicles = 0;
    double totalDistance = 0.0;
    cout << fixed << setprecision(2);
    for (size_t i = 0; i < routes.size(); ++i) {
        if (!routes[i].customerIds.empty()) {
            cout << "Route " << routeNumber++ << ":";
            for (int custIndex : routes[i].customerIds) {
                cout << " " << data.indexToId[custIndex];}
            cout << "\n";
            vehicles++;
            totalDistance += routes[i].totalDistance;}}
    cout << "Vehicles: " << vehicles << "\n";
    cout << "Distance: " << totalDistance << "\n";
    if (routes.empty()) {
        cout << "No solution found.\n";}}

// Inline function to check time limit
inline bool checkTimeLimit(chrono::steady_clock::time_point startTime, int maxTime) {
    if (stopAlgorithm) return false;
    auto currentTime = chrono::steady_clock::now();
    double elapsedTime = chrono::duration_cast<chrono::seconds>(currentTime - startTime).count();
    if (maxTime > 0 && elapsedTime >= maxTime) {
        stopAlgorithm = true;
        if (!timeLimitMessagePrinted) {
            cout << "Time limit reached: " << elapsedTime << " seconds" << endl;
            timeLimitMessagePrinted = true; }
        return false;}
    return true;}

// Read instance file and compute distance matrix as double*
ProblemData readInstance(const string& filename) {
    cout << "Starting to read instance from file: " << filename << "\n";
    ProblemData data;
    ifstream infile(filename);
    if (!infile) {
        cerr << "Cannot open file: " << filename << endl;
        exit(1);
    }
    string line;
    while (getline(infile, line)) {
        if (line.find("CUST NO.") != string::npos) {
            getline(infile, line);
            break;
        } else if (line.find("NUMBER") != string::npos) {
            getline(infile, line);
            istringstream iss(line);
            iss >> data.maxVehicles >> data.vehicleCapacity;}}
    while (getline(infile, line)) {
        istringstream issCust(line);
        Customer cust;
        if (issCust >> cust.id >> cust.x >> cust.y >> cust.demand >> cust.earliest >> cust.latest >> cust.serviceTime) {
            data.customers.push_back(cust);
        } else {
            break;}}
    data.maxRouteDuration = data.customers[0].latest;
    data.numCustomers = data.customers.size();
    data.indexToId.resize(data.numCustomers);
    for (size_t i = 0; i < data.numCustomers; ++i) {
        data.idToIndex[data.customers[i].id] = i;
        data.indexToId[i] = data.customers[i].id;}
    data.distanceMatrix = new double[data.numCustomers * data.numCustomers];
    for (size_t i = 0; i < data.numCustomers; ++i) {
        for (size_t j = 0; j < data.numCustomers; ++j) {
            if (i == j) {
                data.distanceMatrix[i * data.numCustomers + j] = 0.0;
            } else {
                double dx = data.customers[i].x - data.customers[j].x;
                double dy = data.customers[i].y - data.customers[j].y;
                double distance = sqrt(dx * dx + dy * dy);
                data.distanceMatrix[i * data.numCustomers + j] = distance;} }}
    infile.close();
    cout << "Instance read successfully. Number of customers: " << data.numCustomers << ", Vehicle capacity: " << data.vehicleCapacity << "\n";
    return data;}

// Compute a Nearest-Neighbor tour length (excluding the depot index 0)
double computeNearestNeighborLength(const ProblemData& data) {
    size_t n = data.numCustomers;
    vector<bool> visited(n, false);
    visited[0] = true;
    int current = 0;
    double length = 0.0;
    for (size_t count = 1; count < n; ++count) {
        double bestDist = numeric_limits<double>::max();
        int next = -1;
        for (size_t j = 1; j < n; ++j) {
            if (!visited[j] && data.distanceMatrix[current * n + j] < bestDist) {
                bestDist = data.distanceMatrix[current * n + j];
                next = j;}}
        if (next == -1) break;
        visited[next] = true;
        length += bestDist;
        current = next;}
    // return to depot
    length += data.distanceMatrix[current * n + 0];
    return length;}

// Inline function to check if a customer can be added to the end
inline bool canAddToEnd(int current, double currentTime, int load, int customerIndex, const ProblemData& data) {
    if (load + data.customers[customerIndex].demand > data.vehicleCapacity) return false;
    double arrivalTime = currentTime + data.distanceMatrix[current * data.numCustomers + customerIndex];
    if (arrivalTime > data.customers[customerIndex].latest) return false;
    double serviceStartTime = max(arrivalTime, data.customers[customerIndex].earliest);
    double departureTime = serviceStartTime + data.customers[customerIndex].serviceTime;
    double returnTime = departureTime + data.distanceMatrix[customerIndex * data.numCustomers + 0];
    if (returnTime > data.maxRouteDuration) return false;
    return true;}

// Function to append a customer to the end of the route incrementally
bool appendCustomerToRoute(Route& route, int customerIndex, const ProblemData& data, bool isFinal = false) {
    if (!isFinal && stopAlgorithm) return false;  // Stop if stopAlgorithm is active and not final
    if (route.customerIds.empty()) {  // Route is empty
        double arrivalTime = data.distanceMatrix[0 * data.numCustomers + customerIndex];  // Arrival time from depot
        if (arrivalTime > data.customers[customerIndex].latest) return false;  // Check time window
        double serviceStartTime = max(arrivalTime, data.customers[customerIndex].earliest);  // Start of service
        double departureTime = serviceStartTime + data.customers[customerIndex].serviceTime;  // Departure time
        double returnTime = departureTime + data.distanceMatrix[customerIndex * data.numCustomers + 0];  // Return time to depot
        if (returnTime > data.maxRouteDuration) return false;  // Check route duration limit
        route.customerIds.push_back(customerIndex);  // Add customer
        route.arrivalTimes.push_back(arrivalTime);  // Record arrival time
        route.departureTimes.push_back(departureTime);  // Record departure time
        route.load += data.customers[customerIndex].demand;  // Update load
        route.totalDistance += data.distanceMatrix[0 * data.numCustomers + customerIndex];  // Update distance
        route.min_slacks.push_back(data.customers[customerIndex].latest - arrivalTime);  // Calculate slack
        return true;
    } else {  // Route has customers
        int lastCust = route.customerIds.back();  
        double currentTime = route.departureTimes.back();  
        if (!canAddToEnd(lastCust, currentTime, route.load, customerIndex, data)) return false;  // Check if can add
        double travelTime = data.distanceMatrix[lastCust * data.numCustomers + customerIndex];  // Travel time
        double arrivalTime = currentTime + travelTime;  // Arrival time at new customer
        double serviceStartTime = max(arrivalTime, data.customers[customerIndex].earliest);  // Start of service
        double departureTime = serviceStartTime + data.customers[customerIndex].serviceTime;  // Departure time
        route.customerIds.push_back(customerIndex);  // Add customer
        route.arrivalTimes.push_back(arrivalTime);  // Record arrival time
        route.departureTimes.push_back(departureTime);  // Record departure time
        route.load += data.customers[customerIndex].demand;  // Update load
        route.totalDistance += travelTime;  // Update distance
        double slack = data.customers[customerIndex].latest - arrivalTime;  // Calculate new slack
        if (!route.min_slacks.empty()) {
            slack = min(slack, route.min_slacks.back());  // Minimum slack
        }
        route.min_slacks.push_back(slack);  // Record slack
        return true;}}

// Inline function to check route feasibility
inline bool isRouteFeasible(const Route& route, const ProblemData& data, bool isFinal = false) {
    if (!isFinal && stopAlgorithm) return false;
    int capacityUsed = 0;
    double currentTime = 0.0;
    int currentIndex = 0;
    for (int custIndex : route.customerIds) {
        capacityUsed += data.customers[custIndex].demand;
        if (capacityUsed > data.vehicleCapacity) return false;
        double travelTime = data.distanceMatrix[currentIndex * data.numCustomers + custIndex];
        double arrivalTime = currentTime + travelTime;
        double serviceStartTime = max(arrivalTime, data.customers[custIndex].earliest);
        if (serviceStartTime > data.customers[custIndex].latest) return false;
        currentTime = serviceStartTime + data.customers[custIndex].serviceTime;
        currentIndex = custIndex;
    }
    double returnTravelTime = data.distanceMatrix[currentIndex * data.numCustomers + 0];
    currentTime += returnTravelTime;
    if (currentTime > data.maxRouteDuration) return false;
    return true;
}

// Check solution feasibility
bool isSolutionFeasible(const vector<Route>& routes, const ProblemData& data, bool isFinal = false) {
    if (!isFinal && stopAlgorithm) return false;
    int usedVehicles = 0;
    for (const auto& r : routes) {
        if (!r.customerIds.empty())
            ++usedVehicles;
    }
    if (usedVehicles > data.maxVehicles) return false;
    unordered_set<int> visitedCustomers;
    for (const auto& route : routes) {
        if (!isRouteFeasible(route, data, isFinal)) return false;
        for (int custIndex : route.customerIds) {
            if (visitedCustomers.count(custIndex)) return false;
            visitedCustomers.insert(custIndex);
        }
    }
    for (size_t i = 1; i < data.customers.size(); ++i) {
        if (!visitedCustomers.count(i)) return false;
    }
    return true;
}

// Update route with reserved vectors
void updateRoute(Route& route, const ProblemData& data, bool isFinal = false) {
    if (!isFinal && stopAlgorithm) return;
    route.arrivalTimes.clear();
    route.departureTimes.clear();
    route.load = 0;
    route.totalDistance = 0;
    route.min_slacks.clear();
    if (route.customerIds.empty()) return;
    size_t numCustomers = route.customerIds.size();
    route.arrivalTimes.reserve(numCustomers + 1);
    route.departureTimes.reserve(numCustomers);
    route.min_slacks.reserve(numCustomers + 1);
    double currentTime = 0.0;
    route.arrivalTimes.push_back(currentTime);
    int currentIndex = 0;
    for (int custIndex : route.customerIds) {
        double travelTime = data.distanceMatrix[currentIndex * data.numCustomers + custIndex];
        currentTime += travelTime;
        route.totalDistance += travelTime;
        if (currentTime < data.customers[custIndex].earliest) currentTime = data.customers[custIndex].earliest;
        route.arrivalTimes.push_back(currentTime);
        currentTime += data.customers[custIndex].serviceTime;
        route.departureTimes.push_back(currentTime);
        route.load += data.customers[custIndex].demand;
        currentIndex = custIndex;
    }
    double returnDist = data.distanceMatrix[currentIndex * data.numCustomers + 0];
    route.totalDistance += returnDist;
    currentTime += returnDist;
    route.arrivalTimes.push_back(currentTime);
    route.departureTimes.push_back(currentTime);
    route.min_slacks.resize(numCustomers + 1);
    route.min_slacks.back() = numeric_limits<double>::max();
    for (int i = numCustomers - 1; i >= 0; --i) {
        double slack = data.customers[route.customerIds[i]].latest - route.arrivalTimes[i];
        route.min_slacks[i] = min(slack, route.min_slacks[i + 1]);}}

// Objective function
pair<int, double> objectiveFunction(const vector<Route>& routes, long long& evalCount, int maxEvaluations, bool isFinal = false) {
    if (!isFinal && stopAlgorithm) return {0, 0.0};
    if (evalCount >= maxEvaluations) {
        stopAlgorithm = true;
        return {0, 0.0};}
    evalCount++;
    int vehicles = 0;
    double totalDistance = 0.0;
    for (const auto& route : routes) {
        if (!route.customerIds.empty()) {
            vehicles++;
            totalDistance += route.totalDistance;}}
    return {vehicles, totalDistance};}

// Inline function to check if a customer can be inserted
inline bool canInsert(const Route& route, int pos, int customerIndex, const ProblemData& data, bool isFinal = false) {
    if (!isFinal && stopAlgorithm) return false;
    int prevIndex = (pos == 0) ? 0 : route.customerIds[pos - 1];
    int nextIndex = (pos == route.customerIds.size()) ? 0 : route.customerIds[pos];
    int newLoad = route.load + data.customers[customerIndex].demand;
    if (newLoad > data.vehicleCapacity) return false;

    double dist_prev_next = (prevIndex == nextIndex) ? 0.0 : data.distanceMatrix[prevIndex * data.numCustomers + nextIndex];
    double lowerDelta = data.distanceMatrix[prevIndex * data.numCustomers + customerIndex] +
                        data.customers[customerIndex].serviceTime +
                        data.distanceMatrix[customerIndex * data.numCustomers + nextIndex] -
                        dist_prev_next;

    if (pos < route.min_slacks.size() && lowerDelta > route.min_slacks[pos]) {
        return false;
    }
    double arrivalTime = (pos == 0) ? data.distanceMatrix[0 * data.numCustomers + customerIndex] : route.departureTimes[pos - 1] + data.distanceMatrix[prevIndex * data.numCustomers + customerIndex];
    double serviceStartTime = max(arrivalTime, data.customers[customerIndex].earliest);
    if (serviceStartTime > data.customers[customerIndex].latest) return false;
    double departureTime = serviceStartTime + data.customers[customerIndex].serviceTime;
    double currentTime = departureTime;
    int index = customerIndex;
    for (size_t i = pos; i < route.customerIds.size(); ++i) {
        int nextCustIndex = route.customerIds[i];
        double travelTime = data.distanceMatrix[index * data.numCustomers + nextCustIndex];
        arrivalTime = currentTime + travelTime;
        serviceStartTime = max(arrivalTime, data.customers[nextCustIndex].earliest);
        if (serviceStartTime > data.customers[nextCustIndex].latest) return false;
        currentTime = serviceStartTime + data.customers[nextCustIndex].serviceTime;
        index = nextCustIndex;}
    double returnTime = currentTime + data.distanceMatrix[index * data.numCustomers + 0];
    if (returnTime > data.maxRouteDuration) return false;
    return true;
}

// Inline function to calculate insertion cost
inline double calculateInsertionCost(const Route& route, int pos, int customerIndex, const ProblemData& data) {
    int prevIndex = (pos == 0) ? 0 : route.customerIds[pos - 1];
    int nextIndex = (pos == route.customerIds.size()) ? 0 : route.customerIds[pos];
    double spatialCost = data.distanceMatrix[prevIndex * data.numCustomers + customerIndex] +
                         data.distanceMatrix[customerIndex * data.numCustomers + nextIndex] -
                         data.distanceMatrix[prevIndex * data.numCustomers + nextIndex];
    double temporalCost = 0.0;
    if (pos > 0) {
        double arrivalTime = route.departureTimes[pos - 1] + data.distanceMatrix[prevIndex * data.numCustomers + customerIndex];
        temporalCost = max(0.0, data.customers[customerIndex].earliest - arrivalTime);}
    double alpha_val = 0.5;
    return alpha_val * spatialCost + (1 - alpha_val) * temporalCost;}

// Extract arcs from routes
vector<pair<int, int>> getArcs(const vector<Route>& routes, const ProblemData& data) {
    vector<pair<int, int>> arcs;
    arcs.reserve(routes.size() * 10);
    for (const auto& route : routes) {
        if (route.customerIds.empty()) continue;
        int prev = 0;
        for (int cust : route.customerIds) {
            arcs.emplace_back(prev, cust);
            prev = cust;}
        arcs.emplace_back(prev, 0);}
    return arcs;}

// 2-opt local search for a single route
bool twoOpt(Route& route, const ProblemData& data) {
    bool improved = false;
    size_t n = route.customerIds.size();
    if (n < 2) return false; // برای 2-opt حداقل 2 مشتری لازم است

    for (size_t i = 0; i < n - 1; ++i) {
        for (size_t j = i + 2; j <= n; ++j) {
            int a = (i == 0) ? 0 : route.customerIds[i - 1];
            int b = route.customerIds[i];
            int c = route.customerIds[j - 1];
            int d = (j == n) ? 0 : route.customerIds[j];

            double oldDist = data.distanceMatrix[a * data.numCustomers + b] +
                             data.distanceMatrix[c * data.numCustomers + d];
            double newDist = data.distanceMatrix[a * data.numCustomers + c] +
                             data.distanceMatrix[b * data.numCustomers + d];

            if (newDist < oldDist) {
                Route tempRoute = route;
                reverse(tempRoute.customerIds.begin() + i, tempRoute.customerIds.begin() + j);
                updateRoute(tempRoute, data);
                if (isRouteFeasible(tempRoute, data)) {
                    route = tempRoute;
                    improved = true;
                }}}}
    return improved;}

// Relocate local search between routes
bool relocate(vector<Route>& routes, const ProblemData& data) {
    bool improved = false;
    for (size_t r1 = 0; r1 < routes.size(); ++r1) {
        if (routes[r1].customerIds.empty()) continue;
        for (size_t i = 0; i < routes[r1].customerIds.size(); ++i) {
            int cust = routes[r1].customerIds[i];
            Route tempR1 = routes[r1];
            tempR1.customerIds.erase(tempR1.customerIds.begin() + i);
            updateRoute(tempR1, data);

            for (size_t r2 = 0; r2 < routes.size(); ++r2) {
                if (r1 == r2) continue;
                Route& route2 = routes[r2];
                for (size_t pos = 0; pos <= route2.customerIds.size(); ++pos) {
                    if (canInsert(route2, pos, cust, data)) {
                        Route tempR2 = route2;
                        tempR2.customerIds.insert(tempR2.customerIds.begin() + pos, cust);
                        updateRoute(tempR2, data);
                        if (isRouteFeasible(tempR2, data)) {
                            double oldDist = routes[r1].totalDistance + routes[r2].totalDistance;
                            double newDist = tempR1.totalDistance + tempR2.totalDistance;
                            if (newDist < oldDist) {
                                routes[r1] = tempR1;
                                routes[r2] = tempR2;
                                improved = true;
                                return improved; // Early return after improvement
                            }
                        }
                    }
                }
            }
        }
    }
    return improved;
}

// RemoveRoute local search operator
bool removeRoute(vector<Route>& routes, const ProblemData& data) {
    if (routes.size() <= 1) return false; // حداقل یک مسیر باید باقی بماند

    // انتخاب مسیری با کمترین تعداد مشتری برای حذف
    size_t minCustomers = numeric_limits<size_t>::max();
    size_t routeToRemove = -1;
    for (size_t r = 0; r < routes.size(); ++r) {
        if (!routes[r].customerIds.empty() && routes[r].customerIds.size() < minCustomers) {
            minCustomers = routes[r].customerIds.size();
            routeToRemove = r;
        }
    }

    if (routeToRemove == -1) return false;

    Route removedRoute = routes[routeToRemove];
    routes.erase(routes.begin() + routeToRemove);

    // تلاش برای تخصیص مجدد مشتریان به مسیرهای دیگر
    for (int customer : removedRoute.customerIds) {
        bool inserted = false;
        for (auto& route : routes) {
            for (size_t pos = 0; pos <= route.customerIds.size(); ++pos) {
                if (canInsert(route, pos, customer, data)) {
                    route.customerIds.insert(route.customerIds.begin() + pos, customer);
                    updateRoute(route, data);
                    inserted = true;
                    break;
                }
            }
            if (inserted) break;
        }
        if (!inserted) {
            // اگر تخصیص مشتری ممکن نبود، تغییرات را لغو می‌کنیم
            routes.insert(routes.begin() + routeToRemove, removedRoute);
            return false;
        }
    }
    return true; // حذف موفق
}

// Combined local search with removeRoute
void localSearch(vector<Route>& routes, const ProblemData& data) {
    bool improved = true;
    while (improved) {
        improved = false;
        for (auto& route : routes) {
            if (twoOpt(route, data)) {
                improved = true;
            }
        }
        if (relocate(routes, data)) {
            improved = true;
        }
        if (removeRoute(routes, data)) {
            improved = true;
        }
    }
}

// Evaluate solution
double evaluateSolution(const vector<Route>& routes, const ProblemData& data, long long& evalCount, int maxEvaluations, bool isFinal) {
    if (!isFinal && stopAlgorithm) return numeric_limits<double>::max();
    if (!isSolutionFeasible(routes, data, isFinal)) {
        return numeric_limits<double>::max();}
    auto [vehicles, distance] = objectiveFunction(routes, evalCount, maxEvaluations, isFinal);
    if (evalCount >= maxEvaluations) {
        stopAlgorithm = true;
        return numeric_limits<double>::max();}
    return vehicles * 100.0 + 0.001 * distance;}

// Construct solution for one ant
vector<Route> constructSolution(
    vector<vector<double>>& tau,
    const vector<vector<double>>& eta,
    const ProblemData& data,
    mt19937& rng,
    chrono::steady_clock::time_point startTime,
    int maxTime,
    double expoAlpha,
    double beta,
    double phi,
    double q0,
    double tau_0
) {
    if (stopAlgorithm) return {};
    size_t n = data.numCustomers;
    unordered_set<int> unassigned;
    for (size_t i = 1; i < n; ++i) {
        unassigned.insert(i);}
    vector<Route> solution;
    solution.reserve(data.maxVehicles);
    while (!unassigned.empty() && solution.size() < static_cast<size_t>(data.maxVehicles)) {
        Route currentRoute;
        currentRoute.customerIds.reserve(n);
        currentRoute.arrivalTimes.reserve(n + 1);
        currentRoute.departureTimes.reserve(n);
        currentRoute.min_slacks.reserve(n + 1);
        int current = 0;
        double currentTime = 0.0;
        int load = 0;
        while (true) {
            if (stopAlgorithm) return solution;
            vector<int> candidates;
            for (int j : unassigned) {
                if (canAddToEnd(current, currentTime, load, j, data)) {
                    candidates.push_back(j);}}
            if (candidates.empty()) break;
            double r = uniform_real_distribution<double>(0.0, 1.0)(rng);
            int chosen = candidates[0];
            if (r < q0) {
                double maxValue = -1.0;
                for (int j : candidates) {
                    double value = pow(tau[current][j], expoAlpha) * pow(eta[current][j], beta);
                    if (value > maxValue) {
                        maxValue = value;
                        chosen = j;}}
            } else {
                vector<double> weights;
                for (int j : candidates) {
                    weights.push_back(pow(tau[current][j], expoAlpha) * pow(eta[current][j], beta));
                }
                discrete_distribution<int> dist(weights.begin(), weights.end());
                int index = dist(rng);
                chosen = candidates[index];}
            if (appendCustomerToRoute(currentRoute, chosen, data)) {
                tau[current][chosen] = (1 - phi) * tau[current][chosen] + phi * tau_0;
                current = chosen;
                currentTime = currentRoute.departureTimes.back();
                load = currentRoute.load;
                unassigned.erase(chosen);
            } else {
                break;}}
        if (!currentRoute.customerIds.empty()) {
            updateRoute(currentRoute, data, false);
            solution.push_back(currentRoute);
        } else {
            break;}}
    return solution;}

// Ant Colony System algorithm 
vector<Route> antColonySystem(
    const ProblemData& data,
    mt19937& rng,
    int maxTime,
    int maxEvaluations,
    int maxiteration, 
    chrono::steady_clock::time_point startTime,
    bool useLocalSearch
) {
    stopAlgorithm = false;
    timeLimitMessagePrinted = false;
    long long evalCount = 0;

    int m = 80;                // Number of ants
    double expoAlpha = 1.0;    // Exponent for tau
    double beta = 4.0;         // Exponent for eta
    double phi = 0.1;          // Local pheromone update rate
    double alpha = 0.1;        // Global pheromone update rate
    double q0 = 0.9;           // ACS q0 parameter

    size_t n = data.numCustomers;
    double L_nn = computeNearestNeighborLength(data);
    double tau_0 = 1.0 / ((n - 1) * L_nn);

    vector<vector<double>> tau(n, vector<double>(n, tau_0));
    vector<vector<double>> eta(n, vector<double>(n, 0.0));
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            if (i != j) eta[i][j] = 1.0 / data.distanceMatrix[i * n + j];}}
    vector<Route> best_routes;
    double best_score = numeric_limits<double>::max();
    double best_total_distance = numeric_limits<double>::max();
    int best_vehicles = numeric_limits<int>::max();
    for (int iter = 0; iter < maxiteration && !stopAlgorithm && evalCount < maxEvaluations; ++iter) {
        if (!checkTimeLimit(startTime, maxTime)) break;
        double iteration_best_score = numeric_limits<double>::max();
        double iteration_best_dist = numeric_limits<double>::max();
        int iteration_best_ant = -1;
        vector<Route> iteration_best_routes;
        // Construct solutions for all ants
        for (int ant = 0; ant < m && evalCount < maxEvaluations; ++ant) {
            vector<Route> routes = constructSolution(
                tau, eta,
                data, rng, startTime, maxTime,
                expoAlpha, beta, phi, q0, tau_0);
            if (isSolutionFeasible(routes, data)) {
                double score = evaluateSolution(routes, data, evalCount, maxEvaluations);
                double current_dist = 0.0;
                for (const auto& r : routes) current_dist += r.totalDistance;
                if (score < iteration_best_score) {
                    iteration_best_score = score;
                    iteration_best_dist = current_dist;
                    iteration_best_ant = ant;
                    iteration_best_routes = routes;}
                if (score < best_score) {
                    best_score = score;
                    best_total_distance = current_dist;
                    best_vehicles = 0;
                    for (const auto& r : routes) if (!r.customerIds.empty()) best_vehicles++;
                    best_routes = routes;
                }
            } else {
                cout << "Iteration " << iter+1 << ", Ant " << ant+1 << ": Infeasible solution\n";
}}
        // Apply local search to the best ant of this iteration
        if (iteration_best_ant >= 0) {
            int numRoutes = 0;
            for (const auto& r : iteration_best_routes) {
                if (!r.customerIds.empty()) {
                    numRoutes++;}}
            cout << "Best Ant of Iteration " << iter+1 << " is Ant " << iteration_best_ant+1 
                 << " with Score = " << fixed << setprecision(2) << iteration_best_score 
                 << ", Distance = " << fixed << setprecision(2) << iteration_best_dist 
                 << ", and Number of Routes = " << numRoutes << "\n";
            if (useLocalSearch) {
                localSearch(iteration_best_routes, data);
                if (isSolutionFeasible(iteration_best_routes, data)) {
                    double score_after_ls = evaluateSolution(iteration_best_routes, data, evalCount, maxEvaluations);
                    double dist_after_ls = 0.0;
                    for (const auto& r : iteration_best_routes) dist_after_ls += r.totalDistance;
                    if (score_after_ls < best_score) {
                        best_score = score_after_ls;
                        best_total_distance = dist_after_ls;
                        best_vehicles = 0;
                        for (const auto& r : iteration_best_routes) if (!r.customerIds.empty()) best_vehicles++;
                        best_routes = iteration_best_routes;
                        cout << "  >> Local Search improved the best solution at Iteration " << iter+1 << "!\n";
                    }}    }
            cout << "\n";
        } else {
            cout << "*** No feasible ants in Iteration " << iter+1 << " ***\n\n";
}
        // Global pheromone update
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < n; ++j) {
                tau[i][j] *= (1 - alpha);}}
        if (!best_routes.empty()) {
            double update_value = 1.0 / best_total_distance;
            auto arcs = getArcs(best_routes, data);
            for (auto& arc : arcs) {
                tau[arc.first][arc.second] += alpha * update_value;}}}
    cout << "\n--- Summary ---\n";
    if (best_routes.empty()) {
        cout << "No feasible solution found.\n";
    } else {
        cout << "Best Solution: Vehicles = " << best_vehicles << ", Total Distance = " << fixed << setprecision(2) << best_total_distance << "\n";
    }
    cout << "Local Search was " << (useLocalSearch ? "ENABLED" : "DISABLED") << ".\n";
    cout << "---------------\n";
    return best_routes;}

// Main function
int main(int argc, char* argv[]) {
    if (argc != 4) {
        cerr << "Usage: " << argv[0] << " <instanceFile> <maxTime> <maxEvaluations>" << endl;
        return 1;
    }
    const string instanceFile = argv[1];
    const int maxTime = stoi(argv[2]);
    const int maxEvaluations = stoi(argv[3]);
    const int maxiteration = 100000;  
    ProblemData data = readInstance(instanceFile);
    auto seed = chrono::system_clock::now().time_since_epoch().count();
    mt19937 rng(seed);
    auto startTime = chrono::steady_clock::now();
    vector<Route> solution = antColonySystem(data, rng, maxTime, maxEvaluations, maxiteration, startTime, false);
    auto endTime = chrono::steady_clock::now();
    double executionTime = chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count() / 1000.0;
    printSolution(solution, data, "Final Solution");
    if (solution.empty()) {
        cout << "Final solution is empty.\n";
    } else {
        bool feasible = isSolutionFeasible(solution, data, true);
        cout << "Final solution is " << (feasible ? "FEASIBLE" : "INFEASIBLE") << ".\n";
    }
    cout << "Execution Time: " << fixed << setprecision(2) << executionTime << " seconds\n";
    delete[] data.distanceMatrix; // Clean up
    return 0;
}