#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <vector>
#include <numeric>
using Scalar = double;
static constexpr int Dynamic = Eigen::Dynamic;
template <int rows = Dynamic, int cols = Dynamic>
using Matrix = Eigen::Matrix<Scalar, rows, cols>;
template <int rows = Dynamic>
using Vector = Matrix<rows, 1>;

Scalar inner_product(Vector<3> a, Vector<3> b)
{
    Scalar inner_product = 0;
    for (int i = 0; i < 3; i++)
    {
        inner_product += a[i] * b[i];
    }
    return inner_product;
}

int main(void)
{
    Vector<3> a = {1, 2, 3};
    Vector<3> b = {4, 5, 6};
    Scalar c = inner_product(a, a - b);

    std::cout
        << "c" << std::endl
        << c << std::endl;
}