#include <iostream>
#include <eigen3/Eigen/Eigen>
using Scalar = double;
static constexpr int Dynamic = Eigen::Dynamic;

template <int rows = Dynamic, int cols = Dynamic>
using Matrix = Eigen::Matrix<Scalar, rows, cols>;

template <int rows = Dynamic>
using Vector = Matrix<rows, 1>;
using Quaternion = Eigen::Quaternion<Scalar>;
template <class Derived>
using Ref = Eigen::Ref<Derived>;

Scalar inner_product(Vector<3> a, Vector<3> b);
void quaternionToEuler(const Quaternion &quat, Ref<Vector<3>> euler); // check of usage this function
int main(void)
{
    // Matrix<3,3> R;
    // R << 1,2,3, 4,5,6, 7,8,9;
    // std::cout << R << std::endl;
    // std::cout << R*R << std::endl;
    // R.transposeInPlace();
    // std::cout << R << std::endl;

    // std::cout << sqrt(45) <<std::endl;
    Scalar roll = -0.1, pitch = 0, yaw = -0.01;
    Eigen::Quaternion<Scalar> q;
    q = Eigen::AngleAxis<Scalar>(roll, Vector<3>::UnitX()) *
        Eigen::AngleAxis<Scalar>(pitch, Vector<3>::UnitY()) *
        Eigen::AngleAxis<Scalar>(yaw, Vector<3>::UnitZ());
    std::cout << "Quaternion" << std::endl
              << q.coeffs() << std::endl;

    Matrix<3, 3> R = q.toRotationMatrix();
    std::cout
        << "R" << std::endl
        << R << std::endl;

    Vector<3> euler = R.eulerAngles(0, 1, 2);
    std::cout
        << "euler" << std::endl
        << euler << std::endl;

    Vector<3> sim_euler;
    quaternionToEuler(q, sim_euler);

    std::cout
        << "sim_euler" << std::endl
        << sim_euler << std::endl;

    // yaw is about pi if roll < 0, so I change how to calculate euler angle
}

Scalar inner_product(Vector<3> a, Vector<3> b)
{
    Scalar inner_product = 0;
    for (int i = 0; i < 3; i++)
    {
        inner_product += a[i] * b[i];
    }
    return inner_product;
}

void quaternionToEuler(const Quaternion &quat, Ref<Vector<3>> euler)
{
    euler.x() = std::atan2(2 * quat.w() * quat.x() + 2 * quat.y() * quat.z(),
                           quat.w() * quat.w() - quat.x() * quat.x() -
                               quat.y() * quat.y() + quat.z() * quat.z());     //[-pi,pi]
    euler.y() = -std::asin(2 * quat.x() * quat.z() - 2 * quat.w() * quat.y()); //[-pi/2,-pi/2]
    euler.z() = std::atan2(2 * quat.w() * quat.z() + 2 * quat.x() * quat.y(),
                           quat.w() * quat.w() + quat.x() * quat.x() -
                               quat.y() * quat.y() - quat.z() * quat.z()); //[-pi,pi]
}