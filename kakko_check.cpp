#include <iostream>
// using namespace std;
#include <eigen3/Eigen/Eigen>
using Scalar = double; // numpy float32
using Quaternion = Eigen::Quaternion<Scalar>;
static constexpr int Dynamic = Eigen::Dynamic;
template <int rows = Dynamic, int cols = Dynamic>
using Matrix = Eigen::Matrix<Scalar, rows, cols>;
template <int rows = Dynamic>
using Vector = Matrix<rows, 1>;
int main(void)
{
    // Your code here!
    int a = 2;
    float b = (int)a;
    std::cout << b << std::endl;
    std::cout << typeid(b).name() << std::endl;
    std::cout << typeid((int)b).name() << std::endl;
}