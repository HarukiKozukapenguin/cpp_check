#include <iostream>
#include <eigen3/Eigen/Eigen>
#define PI 3.14159265359
using Scalar = double;
static constexpr int Dynamic = Eigen::Dynamic;
template<int rows = Dynamic, int cols = Dynamic>
using Matrix = Eigen::Matrix<Scalar, rows, cols>;
template<int rows = Dynamic>
using Vector = Matrix<rows, 1>;
Vector<3> getspherical(Vector<3> cartesian);
int main(void){
    Vector<3> cv = {1,0,0};
    std::cout << getspherical(cv) << std::endl;
    cv = {0,0,1};
    std::cout << getspherical(cv) << std::endl;
    cv = {0,1,0};
    std::cout << getspherical(cv) << std::endl;
    std::cout << ' ' << std::endl;
    std::cout << PI/2 << std::endl;
}

Vector<3> getspherical(Vector<3> cartesian){
  Scalar r=sqrt(pow(cartesian[0], 2) + pow(cartesian[1], 2) + pow(cartesian[2], 2));
  Vector<3> spherical
  = {r,
  acos(cartesian[2] / r),
  atan2(cartesian[1],cartesian[0])
  };
  return spherical;
};