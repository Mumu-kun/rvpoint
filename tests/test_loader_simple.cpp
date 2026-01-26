#include <iostream>
#include <fstream>
#include <string>
#include <vector>

int main() {
    std::ifstream file("bunny.pcd");
    if (!file.is_open()) {
        std::cerr << "Fail open" << std::endl;
        return 1;
    }
    std::string line;
    int lines = 0;
    while(std::getline(file, line)) {
        lines++;
    }
    std::cout << "Lines: " << lines << std::endl;
    return 0;
}
