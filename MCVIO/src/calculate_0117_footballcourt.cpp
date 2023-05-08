#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>

using namespace std;
using namespace Eigen;

std::ofstream fout;

int main(){

    // 需要求解的量：
    // 两个相机的T_cam_imu
    // T_cn_cnm1

    // Tlr: !!opencv-matrix
    //   rows: 3
    //   cols: 4
    //   dt: f
    //   data: [1.0, 0.0, 0.0, 0.0499585,
    //          0.0, 1.0, 0.0, 0.0,
    //          0.0, 0.0, 1.0, 0.0,
    //          0.0, 0.0, 0.0, 1.0]

    // # Transformation from body-frame (imu) to left camera
    // Tbc: !!opencv-matrix
    // rows: 4
    // cols: 4
    // dt: f
    // data: [0,0,1,0.2328,
    //         1,0,0,-0.0305,
    //         0,1,0,-0.118,
    //         0.0, 0.0, 0.0, 1.0]

    // 相机到imu
    Isometry3d T_b_c0 = Isometry3d::Identity(); //一定要初始化，不然仿射矩阵叠加时会有bug
    Matrix3d R_b_c0 = Matrix3d::Identity();
    R_b_c0 << 0, 0, 1,
            1, 0, 0,
            0, 1, 0;

    Vector3d t_b_c0;
    t_b_c0 << 0.2328, -0.0305, -0.118;
    T_b_c0.rotate(R_b_c0);
    T_b_c0.pretranslate(t_b_c0);


//     cam1:
    Isometry3d T_l_r = Isometry3d::Identity(); //一定要初始化，不然仿射矩阵叠加时会有bug
    Matrix3d R_l_r = Matrix3d::Identity();

    Vector3d t_l_r;
    t_l_r << 0.0499585, 0, 0;
    T_l_r.rotate(R_l_r);
    T_l_r.pretranslate(t_l_r);

    fout.open("/home/ros/dev_workspace/MCVIO_ws/src/MCVIO/MCVIO/config/MCVIO/0117/process_result.txt", ios::out | ios::trunc);
    fout << "Newer College data calibration file " << endl;
    fout << "T_cam0_b: " << endl;
    fout << T_b_c0.inverse().matrix() << endl;
    fout << "T_cam1_b: " << endl;
    // T_r_l * T_c0_b = T_r_b
    fout << (T_l_r.inverse() * T_b_c0.inverse()).matrix() << endl;
    // cout << T_b_color << endl;
    
    return 0;
}