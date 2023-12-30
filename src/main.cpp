#include <unistd.h>
#include <wait.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <mutex>
#include <thread>

#include "SimpleIni.h"

bool has_cycle(const int &node,
                const std::unordered_map<int, std::vector<int>> &graph,
                std::unordered_map<int, bool> &visited,
                std::unordered_map<int, bool> &path_st) {
    
    if (!visited[node]) {
        visited[node] = true;
        path_st[node] = true;
        for (const auto &neighbor: graph.at(node)) {
            if (!visited[neighbor] && has_cycle(neighbor, graph, visited, path_st)) {
                return true;
            }
            else if (path_st[neighbor]) { 
                return true;
            }
        }
    }
    path_st[node] = false;
    return false;
}

bool has_cycle(const std::unordered_map<int, std::vector<int>> &graph) {
    
    std::unordered_map<int, bool> visited;
    std::unordered_map<int, bool> path_st;
    for (const auto &node: graph) {
        if (has_cycle(node.first, graph, visited, path_st)) {
            return true;
        }
    }
    return false;
}

void DFS(const int &node, 
            const std::unordered_map<int, std::vector<int>> &graph, 
            std::unordered_map<int, bool> &visited) {
    
    visited[node] = true;
    for (const auto &neighbor: graph.at(node)) {
        if (!visited[neighbor]) {
            DFS(neighbor, graph, visited);
        }
    }
}

bool one_component(std::unordered_map<int, std::vector<int>> graph) {
    
    for (const auto &node : graph) {
        for (const auto &edge : node.second) {
            if (graph.find(edge) == graph.end()) {
                std::vector<int> nv;
                nv.push_back(node.first);
                graph[edge] = nv;
            } else {
                graph[edge].push_back(node.first);
            }
        }
    }
    std::unordered_map<int, bool> visited;
    int components = 0;
    for (const auto &node: graph) {
        if (!visited[node.first]) {
            DFS(node.first, graph, visited);
            components++;
            if (components > 1) {
                return false;
            }
        }
    }
    return true;
}

std::vector<int> starting(const std::unordered_map<int, std::vector<int>> &graph) {
    
    std::unordered_set<int> nodes_in;
    for (const auto &node: graph) {
        if (!node.second.empty()) {
            for (const auto &edge: node.second) {
                nodes_in.insert(edge);
            }
        }
    }
    std::vector<int> start_nodes;
    for (const auto &node: graph) {
        if (nodes_in.find(node.first) == nodes_in.end()) {
            start_nodes.push_back(node.first);
        }
    }
    return start_nodes;
}

std::vector<int> ending(const std::unordered_map<int, std::vector<int>> &graph) {
    
    std::unordered_set<int> nodes_out;
    for (const auto &node: graph) {
        if (!node.second.empty()) {
            nodes_out.insert(node.first);
        }
    }
    std::vector<int> end_nodes;
    for (const auto &node: graph) {
        if (nodes_out.find(node.first) == nodes_out.end()) {
            end_nodes.push_back(node.first);
        }
    }
    return end_nodes;
}

int main() {
    //std::unordered_map<int, std::string> bars;
    //std::unordered_map<std::string, std::vector<int>> bar_nodes;
    std::unordered_map<int, std::vector<int>> graph;
    CSimpleIniA ini;
    ini.LoadFile("config.ini");
    CSimpleIniA::TNamesDepend sections;
    ini.GetAllSections(sections);
    for (const auto &section: sections) {
        std::string key = section.pItem;
        std::string edge_str = ini.GetValue(key.c_str(), "edges", "");
        std::istringstream edge_input(edge_str);
        std::vector<int> edges;
        int num;
        while (edge_input >> num) {
            edges.push_back(num);
            if (edge_input.peek() == ',') {
                edge_input.ignore();
            }
        }
        int num_key = stoi(key);
        graph[num_key] = edges;
        /*std::string bar = ini.GetValue(key.c_str(), "bars", "");
        if (bar!= "") {
            bars[num_key] = bar;
            if (bar_nodes.find(bar) == bar_nodes.end()) {
                std::vector<int> nv;
                nv.push_back(num_key);
                bar_nodes[bar] = nv;
            } else {
                bar_nodes[bar].push_back(num_key);
            }
        }*/
    }
    bool stop = false;
    if (has_cycle(graph)) {
        std::cout << "Есть цикл!" << std::endl;
        stop = true;
    }
    if (!one_component(graph)) {
        std::cout << "Число компонент связности графа не равно 1!" << std::endl;
        stop = true;
    }
    std::vector<int> start_jobs = starting(graph);
    if (start_jobs.empty()) {
        std::cout << "Нет начальных точек у графа" << std::endl;
        stop = true;
    }
    std::vector<int> final_jobs = ending(graph);
    if (final_jobs.empty()) {
        std::cout << "Нет конечных точек у графа" << std::endl;
        stop = true;
    }
    if (stop) return 1;

    int max_jobs = 3;
    /*std::cout << "Введите максимальное число одновременных процессов: " << std::endl;
    std::cin >> max_jobs;
    if (max_jobs<=0) {
        std::cout << "Неверное число!" << std::endl;
        return 2;
    }*/

    std::unordered_map<int, int> enter_step;
    for (const auto &node: graph) {
        enter_step[node.first] = 0;
    }
    for (const auto &node: graph) {
        for (const auto &edge: node.second) {
            enter_step[edge]++;
        }
    }
    std::queue<int> planned_jobs;

    for (const auto &elem: start_jobs) {
        planned_jobs.push(elem);
    }
    std::queue<std::pair<int, pid_t>> running_jobs;
    std::mutex active_mutex, end_mutex;

    int active_amount = 0;
    bool finished = false;
    std::thread capture_thread([&](){
        while (!finished) {
            while (!running_jobs.empty()) {
                int id;
                pid_t pid;
                {
                    std::lock_guard<std::mutex> lock(active_mutex);
                    id = running_jobs.front().first;
                    pid = running_jobs.front().second;
                    running_jobs.pop();
                }
                int status;
                waitpid(pid, &status, WUNTRACED);
                if (!WIFEXITED(status)) {
                    std::lock_guard<std::mutex> lock(end_mutex);
                    std::cout << "Job " << id << " завершился с ошибкой" << std::endl;
                    finished = true;
                    kill(pid,SIGKILL);
                    while (!running_jobs.empty()) {
                        pid = running_jobs.front().second;
                        running_jobs.pop();
                        kill(pid,SIGKILL);
                    }
                } else {
                    if (graph[id].size() == 0) {
                        final_jobs.erase(std::remove(final_jobs.begin(), final_jobs.end(), id), final_jobs.end());
                    } else {
                        for (const auto &edge: graph.at(id)) {
                            enter_step[edge]--;
                            if (enter_step[edge] == 0) {
                                planned_jobs.push(edge);
                            }
                        }
                    }
                    std::lock_guard<std::mutex> lock(active_mutex);
                    active_amount--;
                }
            }
        }
    });
    while (!final_jobs.empty()) {
        if (finished) break;
        while (!planned_jobs.empty() && active_amount < max_jobs) {
            std::lock_guard<std::mutex> lock(end_mutex);
            if (finished) break;
            int job = planned_jobs.front();
            planned_jobs.pop();
            pid_t pid = fork();
            if (pid == -1) {
                return 3;
            } else if (pid == 0) {
                execl("job", "job", std::to_string(job).c_str(), NULL);
                exit(-1);
            }
            {
                std::lock_guard<std::mutex> lock(active_mutex);
                running_jobs.emplace(job, pid);
                active_amount++;
            }
        }
    }
    finished = true;
    capture_thread.join();
}