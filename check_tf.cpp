#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
using Scalar = double;
// #define ROS_INFO_STREAM(logger, _msgStream)                                    \
//   std::stringstream stream_buffer;                                             \
//   stream_buffer << _msgStream;                                                 \
//   ROS_INFO(logger, stream_buffer.str().c_str());
// static constexpr int Dynamic = Eigen::Dynamic;
// template <int rows = Dynamic, int cols = Dynamic>
// using Matrix = Eigen::Matrix<Scalar, rows, cols>;
// template <int rows = Dynamic>
// using Vector = Matrix<rows, 1>;

// cf. http://wiki.ros.org/tf2/Tutorials/Quaternions

int main(void) {
  Scalar roll = 1.5707, pitch = 0, yaw = 0;
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(roll, pitch, yaw);

  // Eigen::Quaternion<Scalar> q;
  // q = Eigen::AngleAxis<Scalar>(roll, Vector<3>::UnitX()) *
  //     Eigen::AngleAxis<Scalar>(pitch, Vector<3>::UnitY()) *
  //     Eigen::AngleAxis<Scalar>(yaw, Vector<3>::UnitZ());

  //   ROS_INFO_STREAM("x: " << myQuaternion.getX() << " y: " <<
  //   myQuaternion.getY()
  //                         << " z: " << myQuaternion.getZ()
  //                         << " w: " << myQuaternion.getW());

  // std::cout << "Quaternion" << std::endl
  // << q.coeffs() << std::endl;
  // geometry_msgs::Quaternion quat_msg;

  return 0;
}