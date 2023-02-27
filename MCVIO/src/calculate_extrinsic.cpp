#include <iostream>
#include <cmath>

using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

int main()
{
    Isometry3d T_l_t = Isometry3d::Identity();
    Matrix3d rotation_matrix = Matrix3d::Identity();
    rotation_matrix << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    T_l_t.rotate(rotation_matrix);                                     // 按照rotation_vector进行旋转
    T_l_t.pretranslate(Vector3d(0.30456, 0.17065, 0.65376));                     // 把平移向量设成(1,3,4)
    cout << "Thermal camera to LiDAR Transform matrix = \n" << T_l_t.matrix() << endl;

    Isometry3d T_l_ximu = Isometry3d::Identity();
    rotation_matrix = Matrix3d::Identity();
    T_l_ximu.rotate(rotation_matrix);                                     // 按照rotation_vector进行旋转
    T_l_ximu.pretranslate(Vector3d(0.15905, 0.00067, -0.16824));                     // 把平移向量设成(1,3,4)
    cout << "Xsense imu to LiDAR Transform matrix = \n" << T_l_ximu.matrix() << endl;

    Isometry3d T_l_d435cam = Isometry3d::Identity();
    rotation_matrix = Matrix3d::Identity();
    rotation_matrix << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    T_l_d435cam.rotate(rotation_matrix);                                     // 按照rotation_vector进行旋转
    T_l_d435cam.pretranslate(Vector3d(0.30456, 0.00065, 0.65376));                     // 把平移向量设成(1,3,4)
    cout << "d435i camera to LiDAR Transform matrix = \n" << T_l_d435cam.matrix() << endl;

    Isometry3d T_l_d435imu = Isometry3d::Identity();
    rotation_matrix = Matrix3d::Identity();
    rotation_matrix << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    T_l_d435imu.rotate(rotation_matrix);                                     // 按照rotation_vector进行旋转
    T_l_d435imu.pretranslate(Vector3d(0.30456, 0.00065, 0.65376));                     // 把平移向量设成(1,3,4)
    cout << "d435i imu to LiDAR Transform matrix = \n" << T_l_d435imu.matrix() << endl;

    Isometry3d T_l_handsfreeimu = Isometry3d::Identity();
    rotation_matrix = Matrix3d::Identity();
    T_l_handsfreeimu.rotate(rotation_matrix);                                     // 按照rotation_vector进行旋转
    T_l_handsfreeimu.pretranslate(Vector3d(-0.27255, 0.00053, -0.17954));                     // 把平移向量设成(1,3,4)
    cout << "handsfreeimu to LiDAR Transform matrix = \n" << T_l_handsfreeimu.matrix() << endl;

    Isometry3d T_handsfreeimu_thermalcam;
    T_handsfreeimu_thermalcam = T_l_handsfreeimu.inverse() * T_l_t;
//    cout << T_l_handsfreeimu.inverse().matrix() << endl;
//    cout << T_l_t.matrix() << endl;
    cout << "thermal camera to handsfreeimu Transform matrix = \n" << T_handsfreeimu_thermalcam.matrix() << endl;

    Isometry3d T_d435imu_d435cam;
    T_d435imu_d435cam = T_l_d435imu.inverse() * T_l_d435cam;
//    cout << T_l_handsfreeimu.inverse().matrix() << endl;
//    cout << T_l_t.matrix() << endl;
    cout << "d435 camera to d435 imu Transform matrix = \n" << T_d435imu_d435cam.matrix() << endl;

    return 0;
}