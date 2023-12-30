#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <random>

int main(int argc, char* argv[]){
    std::cout << "Запущена русская рулетка " << argv[1] << std::endl;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> distribution(0, 9);
    if (distribution(gen) == 0) {
        raise(SIGSTOP);
    }
}