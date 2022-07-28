#include<iostream>
#include <unsupported/Eigen/CXX11/Tensor>
static constexpr int Dynamic = Eigen::Dynamic;
using Scalar = double;

template<int rows = Dynamic, int cols = Dynamic, int width = Dynamic>
using MatrixRowMajor = Eigen::Matrix<Scalar, rows, cols, width, Eigen::RowMajor>;

int main(void){
    Tensor<Scalar, 3> t_3d(2, 3, 4);
// t_3d(0, 1, 0) = 12.0f;
    std::cout << t_3d << std::endl;
}