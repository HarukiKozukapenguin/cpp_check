#include <iostream>
#include <eigen3/Eigen/Eigen>
#define PI 3.14159265359
#define N 2 //how many cuts they will in spherical coordinate.
using Scalar = double;
static constexpr int Dynamic = Eigen::Dynamic;
template<int rows = Dynamic, int cols = Dynamic>
using Matrix = Eigen::Matrix<Scalar, rows, cols>;
template<int rows = Dynamic>
using Vector = Matrix<rows, 1>;

Vector<2*N*N> getsphericalboxel(std::vector<Vector<4>,
Eigen::aligned_allocator<Vector<4>>>& s);

Scalar getclosestpoint(std::vector<Vector<4>,
Eigen::aligned_allocator<Vector<4>>>& s, 
Scalar tmin, Scalar tmax, Scalar fmin, Scalar fmax);
Scalar max_detection_range;
Vector<3> getspherical(Vector<3> cartesian);
Vector<3> getBoundaryVec(Scalar t, Scalar f);
Scalar inner_product(Vector<3> a, Vector<3> b);
void comp(Scalar& rmin, Scalar r);
Scalar getclosestpoint2(Scalar distance, Scalar theta, Scalar size);

int main(void){
    Vector<4> s1 = {1.5, -2.0,1.5,2.5}; //observation matrix at cartian coordinate
    // Vector<4> s2 = {2.0,0,0,1}; //observation matrix at cartian coordinate
    // std::cout << getspherical(cv) << std::endl;
    std::vector<Vector<4>, Eigen::aligned_allocator<Vector<4>>> s;
    max_detection_range = 10;
    s.push_back(s1);
    // s.push_back(s2);
    for (size_t i = 0; i < s.size(); ++i) {
        // std::cout << s[i] << std::endl;
        // std::cout << ' ' << std::endl;
    }
    Vector<2*N*N> sphericalboxel = getsphericalboxel(s);
}

Vector<2*N*N> getsphericalboxel(std::vector<Vector<4>,
 Eigen::aligned_allocator<Vector<4>>>& s){
    // for (size_t t = 0; t < N; ++t) {
    //     for (int f = -1*N; f < N; ++f) {
    //         Scalar tmin = t*(PI/N);
    //         Scalar tmax = (t+1)*(PI/N);
    //         Scalar fmin = f*(PI/N);
    //         Scalar fmax = (f+1)*(PI/N);
    //         Scalar waste = getclosestpoint(s, tmin, tmax, fmin, fmax);
    //         // std::cout << waste << std::endl;
    //         // std::cout << tmin << ' '<< tmax << ' '<< fmin << ' '<< fmax <<std::endl;
    //     }
    // }
    size_t t = 0;
    int f = -1*N;
    Scalar tmin = t*(PI/N);
    Scalar tmax = (t+1)*(PI/N);
    Scalar fmin = f*(PI/N);
    Scalar fmax = (f+1)*(PI/N);
    Scalar waste = getclosestpoint(s, tmin, tmax, fmin, fmax);
    Vector<2*N*N> relative_pos;
    return relative_pos;
};

Scalar getclosestpoint(std::vector<Vector<4>,
Eigen::aligned_allocator<Vector<4>>>& s, 
Scalar tmin, Scalar tmax, Scalar fmin, Scalar fmax){
    Scalar rmin = max_detection_range;
        for (size_t i = 0; i < s.size(); ++i) {
            Vector<3> Cartesian = {s[i][0],s[i][1],s[i][2]}; //R*relative_pos[sort_idx]
            Scalar size = s[i][3]; //obstacle_radius_[sort_idx]
            Vector<3> Spherical = getspherical(Cartesian); // change s to spherical coordinate
            //getspherical(R*relative_pos[sort_idx]) (r, theta, fai)
            std::cout << Cartesian << std::endl;
            std::cout << size << std::endl;
            std::cout << Spherical << std::endl;
            if ((Spherical[1]< tmin || tmax < Spherical[1]) && (Spherical[2]< fmin || fmax < Spherical[2])){
                // std::cout << "true" << std::endl;
                Vector<3> Unit;
                if (Spherical[1]< tmin && Spherical[2]< fmin){
                    Unit = getBoundaryVec(tmin,fmin);
                }
                else if (Spherical[1]< tmin && fmax < Spherical[2]){
                    Unit = getBoundaryVec(tmin,fmax);
                }
                else if (tmax < Spherical[1]&& Spherical[2]< fmin){
                    Unit = getBoundaryVec(tmax,fmin);
                }
                else if (tmax < Spherical[1]&& fmax < Spherical[2]){
                    Unit = getBoundaryVec(tmax,fmax);
                    // std::cout << Unit << std::endl;
                }
                Vector<3> dist_vec = Cartesian - inner_product(Cartesian, Unit)*Unit;
                Scalar dist = dist_vec.norm();
                // std::cout << dist << std::endl;
                if (dist < size){
                    comp(rmin,inner_product(Cartesian,Unit)-sqrt(std::pow(size,2)-std::pow(dist,2)));
                }
                // std::cout<<rmin<<std::endl; //s1 = {1.5,-2.0,-1.5,2.5},size_t t = 0; int f = -1*N; has no problem in this code
            }
            else if (((Spherical[1]< tmin || tmax < Spherical[1]) && (fmin < Spherical[2] < fmax)) || 
            ((Spherical[2]< fmin || fmax < Spherical[2]) && (tmin < Spherical[1] < tmax))){
                std::cout << "true" << std::endl;

                Scalar alpha, beta;
                if (Spherical[1]< tmin || tmax < Spherical[1]){
                    alpha = Spherical[1];
                    if (Spherical[1]< tmin){
                        beta = tmin;
                    }
                    else{
                        beta = tmax;
                    }
                }
                if (Spherical[2]< fmin || fmax < Spherical[2]){
                    std::cout << "fai" << std::endl;
                    alpha = Spherical[2];
                    alpha = Spherical[2];
                    if (Spherical[2]< fmin){
                        beta = fmin;
                    }
                    else{
                        beta = fmax;
                    }
                }
                Scalar theta = std::fabs(alpha - beta);
                if (theta < PI/2 && Spherical[0]* std::sin(theta)< size){
                    comp(rmin, getclosestpoint2(Spherical[0], theta, size));
                }
            std::cout << rmin << std::endl;
            }
    }
    return rmin;
};

Vector<3> getspherical(Vector<3> cartesian){
  Scalar r=sqrt(pow(cartesian[0], 2) + pow(cartesian[1], 2) + pow(cartesian[2], 2));
  Vector<3> spherical
  = {r,
  acos(cartesian[2] / r),
  atan2(cartesian[1],cartesian[0])
  };
  return spherical;
  };

Vector<3> getBoundaryVec(Scalar t, Scalar f){
  Vector<3> cartesian
  = {std::sin(t)*std::cos(f),
  std::sin(t)*std::sin(f),
  std::cos(t)
  };
  return cartesian;
};

Scalar inner_product(Vector<3> a, Vector<3> b){
    Scalar inner_product = 0;
  for (int i = 0; i < N; i++) {
    inner_product += a[i] * b[i];
  }
  return inner_product;
};

void comp(Scalar& rmin, Scalar r){
    if (r<rmin){
        rmin=r;
    }
};

Scalar getclosestpoint2(Scalar distance, Scalar theta, Scalar size){
    return distance*std::cos(theta) - sqrt(std::pow(size,2)-std::pow(distance*std::sin(theta),2));
};