#include <fstream>
#include <sstream>
#include <string>
#include <vector>

// using namespace std;

std::vector<std::string> split(std::string &input, char delimiter) {
  std::istringstream stream(input);
  std::string field;
  std::vector<std::string> result;
  while (std::getline(stream, field, delimiter)) {
    result.push_back(field);
  }
  return result;
}

int main() {
  std::ifstream ifs("data.csv");

  std::string line;
  while (std::getline(ifs, line)) {

    std::vector<std::string> strvec = split(line, ',');

    for (int i = 1; i < 3; i++) {
      printf("%5f\n", stof(strvec.at(i)));
    }
  }
}