#include <iostream>
// using namespace std;
#include <eigen3/Eigen/Eigen>
using Scalar = double;  // numpy float32
using Quaternion = Eigen::Quaternion<Scalar>;
static constexpr int Dynamic = Eigen::Dynamic;
template<int rows = Dynamic, int cols = Dynamic>
using Matrix = Eigen::Matrix<Scalar, rows, cols>;
template<int rows = Dynamic>
using Vector = Matrix<rows, 1>;
int main(void){
    // Your code here!
    Quaternion a = {0,1,0,0};
    Vector<3> b = Vector<3>::UnitZ();
    Vector<3> product = a * b;
    std::cout << product << std::endl;
}