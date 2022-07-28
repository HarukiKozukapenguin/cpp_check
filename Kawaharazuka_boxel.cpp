#include <iostream>
#include <eigen3/Eigen/Eigen>
#define PI 3.14159265359
#define Cuts 8 //how many cuts they will in spherical coordinate.
using Scalar = double;
static constexpr int Dynamic = Eigen::Dynamic;
template<int rows = Dynamic, int cols = Dynamic>
using Matrix = Eigen::Matrix<Scalar, rows, cols>;
template<int rows = Dynamic>
using Vector = Matrix<rows, 1>;

Vector<2*Cuts*Cuts> getsphericalboxel(std::vector<Vector<3>,
 Eigen::aligned_allocator<Vector<3>>>& pos_b_list, std::vector<Scalar> pos_norm_list, std::vector<Scalar> obs_radius_list);
Scalar getClosestDistance(std::vector<Vector<3>,
 Eigen::aligned_allocator<Vector<3>>>& pos_b_list, std::vector<Scalar> pos_norm_list, std::vector<Scalar> obs_radius_list, 
Scalar tcell, Scalar fcell);
Vector<3> getCartesianFromAng(Scalar t, Scalar f);
Scalar inner_product(Vector<3> a, Vector<3> b);
void comp(Scalar& rmin, Scalar r);
Scalar getclosestpoint(Scalar distance, Scalar theta, Scalar size);

Scalar max_detection_range_;
size_t obstacle_num;

int main(void){
    std::vector<Vector<3>, Eigen::aligned_allocator<Vector<3>>> pos_b_list;
    std::vector<Scalar> pos_norm_list;
    std::vector<Scalar> obs_radius_list;
    obstacle_num = 0;
    max_detection_range_ = 10;

    Vector<3> pos_b1 = {3.0, 0, 0}; //observation matrix at cartian coordinate from body
    Scalar pos_norm1 = pos_b1.norm();
    Scalar obs_radius1 = 2.5;

    obstacle_num +=1; // obstacle_num == pos_b_list.size(), pos_norm_list.size(), obs_radius_list.size()

    pos_b_list.push_back(pos_b1);
    pos_norm_list.push_back(pos_norm1);
    obs_radius_list.push_back(obs_radius1);

    // s.push_back(s2);
    Vector<2*Cuts*Cuts> sphericalboxel = getsphericalboxel(pos_b_list, pos_norm_list, obs_radius_list);
    std::cout << sphericalboxel << std::endl;
}

Vector<2*Cuts*Cuts> getsphericalboxel(std::vector<Vector<3>,
 Eigen::aligned_allocator<Vector<3>>>& pos_b_list, std::vector<Scalar> pos_norm_list, std::vector<Scalar> obs_radius_list){
    Vector<2*Cuts*Cuts> obstacle_obs;
    for (size_t t = 0; t < Cuts; ++t) {
        for (int f = -1*Cuts; f < Cuts; ++f) {
            Scalar tcell = (t+0.5)*(PI/Cuts);
            Scalar fcell = (f+0.5)*(PI/Cuts);
            obstacle_obs[t*2*Cuts+(f+Cuts)] = getClosestDistance(pos_b_list, pos_norm_list, obs_radius_list, tcell,fcell);
        }
    }   
    return obstacle_obs;
    // size_t t = 0;
    // int f = -1*N;
    // Scalar tcell = (t+0.5)*(PI/N);
    // Scalar fcell = (f+0.5)*(PI/N);
    // std::cout << tcell << std::endl;
    // std::cout << fcell << std::endl;
    // Vector<2*N*N> obstacle_obs;
    // obstacle_obs[0] = getClosestDistance(s, tcell,fcell);
    // std::cout << obstacle_obs[0] << std::endl;
    // return obstacle_obs;
}

Scalar getClosestDistance(std::vector<Vector<3>,
 Eigen::aligned_allocator<Vector<3>>>& pos_b_list, std::vector<Scalar> pos_norm_list, std::vector<Scalar> obs_radius_list, 
Scalar tcell, Scalar fcell){
    Vector<3> Cell = getCartesianFromAng(tcell,fcell);
    // std::cout << "Cell is " << Cell << std::endl;
    Scalar rmin = max_detection_range_;
    for (size_t i = 0; i < obstacle_num; ++i) {
        Vector<3> Cartesian = pos_b_list[i]; //R*relative_pos[sort_idx]
        Scalar cdist = pos_norm_list[i];
        // std::cout << "cdist is " << cdist << std::endl;
        Scalar size = obs_radius_list[i]; //obstacle_radius_[sort_idx]
        Scalar ts = std::asin(size/cdist);
        // std::cout << ts << std::endl;
        // std::cout << "inner product is " << inner_product(Cell,Cartesian) << std::endl;
        Scalar tb = std::acos(inner_product(Cell,Cartesian)/(1*cdist));
        // std::cout << "tb is " << tb << std::endl;
        if (tb < ts){
            comp(rmin,getclosestpoint(cdist,tb,size));
        }
    }
    return rmin/max_detection_range_;
}

Vector<3> getCartesianFromAng(Scalar t, Scalar f){
  Vector<3> cartesian
  = {std::cos(t),
    std::sin(t)*std::cos(f),
  std::sin(t)*std::sin(f)
  };
  return cartesian;
}

Scalar inner_product(Vector<3> a, Vector<3> b){
    Scalar inner_product = 0;
  for (int i = 0; i < 3; i++) {
    inner_product += a[i] * b[i];
  }
  return inner_product;
}

void comp(Scalar& rmin, Scalar r){
    if (r<rmin){
        rmin=r;
    }
}

Scalar getclosestpoint(Scalar distance, Scalar theta, Scalar size){
    return distance*std::cos(theta) - sqrt(std::pow(size,2)-std::pow(distance*std::sin(theta),2));
}