#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

int main(){
    Isometry3d T_b_c1 = Isometry3d::Identity(); //一定要初始化，不然仿射矩阵叠加时会有bug
    Matrix3d R_b_c1 = Matrix3d::Identity();
    R_b_c1 << 0.0148655429818, -0.999880929698, 0.00414029679422,
         0.999557249008, 0.0149672133247, 0.025715529948,
         -0.0257744366974, 0.00375618835797, 0.999660727178;

    Vector3d t_b_c1;
    t_b_c1 << -0.0216401454975, -0.064676986768, 0.00981073058949;
        T_b_c1.rotate(R_b_c1);
    T_b_c1.pretranslate(t_b_c1);
    
    Isometry3d T_c1_c2 = Isometry3d::Identity();
    Matrix3d R_c1_c2 = Matrix3d::Identity();
    R_c1_c2 << 0.999997256477797,-0.002317135723275,-0.000343393120620,
         0.002312067192432,0.999898048507103,-0.014090668452683,
         0.000376008102320,0.014089835846691,0.999900662638081;
    Vector3d t_c1_c2;
    t_c1_c2 << 0.110074137800478, -0.000156612054392, 0.000889382785432;
    T_c1_c2.rotate(R_c1_c2);
    T_c1_c2.pretranslate(t_c1_c2);

    Isometry3d T_b_c2 = Isometry3d::Identity();
    T_b_c2 = T_b_c1 * T_c1_c2;

    Matrix3d R_b_c2;
    R_b_c2 = R_b_c1 * R_c1_c2;
    std::cout << "R_b_c2:" << std::endl;
    std::cout << R_b_c2 << std::endl;
    std::cout << "T_b_c2:" << std::endl;
    std::cout << T_b_c2.matrix() << std::endl;
    return 0;
}