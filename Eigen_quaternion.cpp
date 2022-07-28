#include <iostream>
#include <eigen3/Eigen/Eigen>
using Scalar = double;
static constexpr int Dynamic = Eigen::Dynamic;
template <int rows = Dynamic, int cols = Dynamic>
using Matrix = Eigen::Matrix<Scalar, rows, cols>;
template <int rows = Dynamic>
using Vector = Matrix<rows, 1>;
int main(void)
{
    Scalar roll = -1.5707, pitch = 0, yaw = 0.707;
    Eigen::Quaternion<Scalar> q;
    q = Eigen::AngleAxis<Scalar>(roll, Vector<3>::UnitX()) *
        Eigen::AngleAxis<Scalar>(pitch, Vector<3>::UnitY()) *
        Eigen::AngleAxis<Scalar>(yaw, Vector<3>::UnitZ());
    std::cout << "Quaternion" << std::endl
              << q.coeffs() << std::endl;

    Vector<3> euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    std::cout << "Euler from quaternion in roll, pitch, yaw" << std::endl
              << euler << std::endl;
}