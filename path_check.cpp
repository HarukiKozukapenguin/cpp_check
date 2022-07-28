#include <iostream>
#include <string>
#include <filesystem>
namespace fs = std::filesystem;


int main(void)
{
     std::cout << fs::current_path() << std::endl;
    return 0;
}