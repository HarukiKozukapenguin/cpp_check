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
    Vector<4> a = {1,2,3,4};
    std::cout << a.segment<2>(1) << std::endl;
}