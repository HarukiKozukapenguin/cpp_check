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
    Vector<3> a = {0, 1, 0};
    Vector<3> b;
    for (size_t i = 0; i < 3; ++i)
    {
        std::cout << i << std::endl;
        b[i] = a[i];
    }
    // b[0] = a[0];
    std::cout << b << std::endl;
}