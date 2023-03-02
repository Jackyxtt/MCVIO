#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>

using namespace std;
using namespace Eigen;

std::ofstream fout;

int main(){
    // imu到雷达
    Isometry3d T_l_b = Isometry3d::Identity(); //一定要初始化，不然仿射矩阵叠加时会有bug
    Matrix3d R_l_b = Matrix3d::Identity();
    R_l_b << 1, 0, 0,
             0, 1, 0, 
             0, 0, 1;

    Vector3d t_l_b;
    t_l_b << -0.27255, 0.00053, -0.17954;
    T_l_b.rotate(R_l_b);
    T_l_b.pretranslate(t_l_b);
    
    // d435i彩色相机到雷达
    Isometry3d T_l_color = Isometry3d::Identity();
    Matrix3d R_l_color = Matrix3d::Identity();
    R_l_color << 0, 0, 1,
                -1, 0, 0,
                 0, -1, 0;
    Vector3d t_l_color;
    t_l_color << 0.30456, 0.00065, 0.65376;
    T_l_color.rotate(R_l_color);
    T_l_color.pretranslate(t_l_color);

    // 左相机到雷达
    Isometry3d T_l_leftcam = Isometry3d::Identity();
    Matrix3d R_l_leftcam = Matrix3d::Identity();
    R_l_leftcam << 0, 0, 1,
                -1, 0, 0,
                0, -1, 0;
    Vector3d t_leftcam_l;
    t_leftcam_l << 0.24221, 0.16123, -0.16711;
    T_l_leftcam.rotate(R_l_leftcam);
    T_l_leftcam.pretranslate(t_leftcam_l);

    // 右相机到雷达
    Isometry3d T_l_rightcam = Isometry3d::Identity();
    Matrix3d R_l_rightcam = Matrix3d::Identity();
    R_l_rightcam << 0, 0, 1,
                    -1, 0, 0,
                    0, -1, 0;
    Vector3d t_l_rightcam;
    t_l_rightcam << 0.242013, -0.16025, -0.16724;
    T_l_rightcam.rotate(R_l_rightcam);
    T_l_rightcam.pretranslate(t_l_rightcam);

    // 红外相机到雷达
    Isometry3d T_l_thermal = Isometry3d::Identity();
    Matrix3d R_l_thermal = Matrix3d::Identity();
    R_l_thermal << 0, 0, 1,
                  -1, 0, 0,
                   0, -1, 0;
    Vector3d t_l_thermal;
    t_l_thermal << 0.30456, 0.17065, 0.65376;
    T_l_thermal.rotate(R_l_thermal);
    T_l_thermal.pretranslate(t_l_thermal);

    // 右中相机到雷达
    Isometry3d T_l_midright = Isometry3d::Identity();
    Matrix3d R_l_midright = Matrix3d::Identity();
    R_l_midright << -1, 0, 0,
                   0, 0, -1,
                   0, -1, 0;
    Vector3d t_l_midright;
    t_l_midright << 0.00021, -0.16013, -0.16674;
    T_l_midright.rotate(R_l_midright);
    T_l_midright.pretranslate(t_l_midright);

    // 惯导到d435i彩色相机
    Isometry3d T_b_color = Isometry3d::Identity();
    T_b_color = T_l_b.inverse() * T_l_color;

    // 惯导到红外相机
    Isometry3d T_b_thermal = Isometry3d::Identity();
    T_b_thermal = T_l_b.inverse() * T_l_thermal;
    
    // 惯导到左相机
    Isometry3d T_b_leftcam = Isometry3d::Identity();
    T_b_leftcam = T_l_b.inverse() * T_l_leftcam;

    // 惯导到右相机
    Isometry3d T_b_rightcam = Isometry3d::Identity();
    T_b_rightcam = T_l_b.inverse() * T_l_rightcam;

    // 惯导到右相机
    Isometry3d T_b_midright = Isometry3d::Identity();
    T_b_midright = T_l_b.inverse() * T_l_midright;

    fout.open("/home/ros/dev_workspace/MCVIO_ws/src/MCVIO/MCVIO/config/MCVIO/m2dgr/process_result.txt", ios::out | ios::trunc);
    fout << "T_b_color: " << endl;
    fout << T_b_color.matrix() << endl;
    fout << "T_b_thermal: " << endl;
    fout << T_b_thermal.matrix() << endl;
    fout << "T_b_leftcam: " << endl;
    fout << T_b_leftcam.matrix() << endl;
    fout << "T_b_rightcam: " << endl;
    fout << T_b_rightcam.matrix() << endl;
    fout << "T_b_midright: " << endl;
    fout << T_b_midright.matrix() << endl;
    fout.close();
    // cout << T_b_color << endl;
    
    return 0;
}