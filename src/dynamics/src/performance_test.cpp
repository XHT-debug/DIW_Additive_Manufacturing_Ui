#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <chrono>
#include <random>

// 六轴机械臂DH参数 
double g = 9.81; // 重力加速度
Eigen::VectorXd alpha(6);
Eigen::VectorXd a(6);
Eigen::VectorXd d(6);
Eigen::VectorXd theta_offset(6);
Eigen::MatrixXd m_assemble(1, 6);
Eigen::MatrixXd centroid_vector_assemble(3, 3 * 6);
Eigen::MatrixXd A_Screw_assemble(6, 6);

// 反对称矩阵计算函数
Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0,    -v.z(), v.y(),
         v.z(), 0,    -v.x(),
         -v.y(), v.x(), 0;
    return m;
}

// 伴随变换矩阵计算函数
Eigen::MatrixXd Ad_T(const Eigen::MatrixXd& T)
{
    Eigen::MatrixXd Ad_T = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd R = T.block<3, 3>(0, 0);
    Eigen::VectorXd p = T.block<3, 1>(0, 3);
    Ad_T.block<3, 3>(0, 0) = R;
    Ad_T.block<3, 3>(3, 3) = R;
    Ad_T.block<3, 3>(3, 0) = skewSymmetric(p) * R;
    return Ad_T;
}

// 关节转换矩阵计算函数
Eigen::MatrixXd joint_transform_matrix(const Eigen::VectorXd& theta)
{
    Eigen::MatrixXd T_i_1_i_assemble = Eigen::MatrixXd::Zero(4, 4 * 7);
    for (int i = 1; i <= 6; i++) {
        T_i_1_i_assemble.block<4, 4>(0, 4 * (i - 1)) << cos(theta(i-1)), -sin(theta(i-1)) * cos(alpha(i-1)), sin(theta(i-1)) * sin(alpha(i-1)), a(i-1) * cos(theta(i-1)),
                                                        sin(theta(i-1)), cos(theta(i-1)) * cos(alpha(i-1)), -cos(theta(i-1)) * sin(alpha(i-1)), a(i-1) * sin(theta(i-1)),
                                                        0, sin(alpha(i-1)), cos(alpha(i-1)), d(i-1),
                                                        0, 0, 0, 1;
    }
    T_i_1_i_assemble.block<4, 4>(0, 4 * 6) << 1, 0, 0, 0,
                                            0, 1, 0, 0,
                                            0, 0, 1, 0,
                                            0, 0, 0, 1;
    return T_i_1_i_assemble;
}

// 雅可比矩阵计算函数
Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& theta, const Eigen::MatrixXd& T_i_1_i_assemble)
{
    Eigen::MatrixXd Jacobian = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd T_0_i = Eigen::MatrixXd::Zero(4, 4);
    Eigen::MatrixXd T_0_i_1 = Eigen::MatrixXd::Identity(4, 4);

    for (int i = 1; i <= 6; i++) {
        T_0_i = T_0_i_1 * T_i_1_i_assemble.block<4, 4>(0, 4 * (i - 1));
        T_0_i_1 = T_0_i;
        Jacobian.block<6, 1>(0, i - 1) = Ad_T(T_0_i) * A_Screw_assemble.block<6, 1>(0, i - 1);
    }
    return Jacobian;
}

// 牛顿-欧拉逆动力学函数
Eigen::MatrixXd NewtonEulerInverseDynamics(const Eigen::VectorXd& theta, const Eigen::VectorXd& dtheta, const Eigen::VectorXd& d2theta, const Eigen::VectorXd& F_Screw_tip, const bool whether_gravity, const Eigen::MatrixXd& T_i_1_i_assemble)
{
    Eigen::MatrixXd V_Screw_assemble = Eigen::MatrixXd::Zero(6, 1 * 7); 
    Eigen::MatrixXd dV_Screw_assemble = Eigen::MatrixXd::Zero(6, 1 * 7); 
    Eigen::MatrixXd F_Screw_assemble = Eigen::MatrixXd::Zero(6, 1 * 7);
    Eigen::MatrixXd tau(6, 1);

    // 初始化矩阵集合
    V_Screw_assemble.block<6, 1>(0, 0) = Eigen::VectorXd::Zero(6);
    if (whether_gravity) 
    {
        dV_Screw_assemble.block<6, 1>(0, 0) << 0, 0, 0, 0, 0, -g;
    }else
    {
        dV_Screw_assemble.block<6, 1>(0, 0) << 0, 0, 0, 0, 0, 0;
    }
    F_Screw_assemble.block<6, 1>(0, 6) = F_Screw_tip;

    // 正向迭代
    for (int i = 1; i <= 6; i++) {
        Eigen::MatrixXd T_i_i_1 = T_i_1_i_assemble.block<4, 4>(0, 4 * (i - 1)).inverse();
        Eigen::MatrixXd A_Screw_i = A_Screw_assemble.block<6, 1>(0, i - 1);
        Eigen::MatrixXd ad_V_Screw_i_forward = Eigen::MatrixXd::Zero(6, 6);

        Eigen::MatrixXd V_Screw_i = Ad_T(T_i_i_1) * V_Screw_assemble.block<6, 1>(0, i - 1) + A_Screw_i * dtheta(i-1);
        V_Screw_assemble.block<6, 1>(0, i) = V_Screw_i;

        ad_V_Screw_i_forward.block<3, 3>(0, 0) = skewSymmetric(V_Screw_i.block<3, 1>(0, 0));
        ad_V_Screw_i_forward.block<3, 3>(3, 0) = skewSymmetric(V_Screw_i.block<3, 1>(3, 0));
        ad_V_Screw_i_forward.block<3, 3>(3, 3) = ad_V_Screw_i_forward.block<3, 3>(0, 0);

        Eigen::MatrixXd dV_Screw_i = Ad_T(T_i_i_1) * dV_Screw_assemble.block<6, 1>(0, i - 1) + ad_V_Screw_i_forward * A_Screw_i * dtheta(i-1) + A_Screw_i * d2theta(i-1);
        dV_Screw_assemble.block<6, 1>(0, i) = dV_Screw_i;
    }
    
    // 逆向迭代
    for (int i = 6; i >= 1; i--) {
        Eigen::MatrixXd spatial_inertia_i = Eigen::MatrixXd::Zero(6, 6);
        Eigen::MatrixXd T_i__1_i = T_i_1_i_assemble.block<4, 4>(0, 4 * (i - 1)).inverse();
        
        Eigen::MatrixXd I_cm = Eigen::MatrixXd::Zero(3, 3);
        Eigen::VectorXd centroid_vector_i = centroid_vector_assemble.block<3, 1>(0, 3 * (i - 1));

        Eigen::MatrixXd ad_V_Screw_i_inverse = Eigen::MatrixXd::Zero(6, 6);
        ad_V_Screw_i_inverse.block<3, 3>(0, 0) = skewSymmetric(V_Screw_assemble.block<3, 1>(0, i-1));
        ad_V_Screw_i_inverse.block<3, 3>(3, 0) = skewSymmetric(V_Screw_assemble.block<3, 1>(3, i-1));
        ad_V_Screw_i_inverse.block<3, 3>(3, 3) = ad_V_Screw_i_inverse.block<3, 3>(0, 0);
        
        spatial_inertia_i.block<3, 3>(0, 0) = I_cm + m_assemble(0, i-1) * skewSymmetric(centroid_vector_i) * skewSymmetric(centroid_vector_i).transpose();
        spatial_inertia_i.block<3, 3>(3, 0) = m_assemble(0, i-1) * skewSymmetric(centroid_vector_i).transpose();
        spatial_inertia_i.block<3, 3>(0, 3) = m_assemble(0, i-1) * skewSymmetric(centroid_vector_i);
        spatial_inertia_i.block<3, 3>(3, 3) <<  m_assemble(0, i-1), 0, 0,
                                                0, m_assemble(0, i-1), 0,
                                                0, 0, m_assemble(0, i-1);

        // spatial_inertia_i = Ad_T(T_i__1_i) * spatial_inertia_i * T_i__1_i;
        Eigen::MatrixXd F_Screw_i__1 = F_Screw_assemble.block<6, 1>(0, i);

        Eigen::MatrixXd F_Screw_i = Ad_T(T_i__1_i).transpose() * F_Screw_i__1 + spatial_inertia_i * dV_Screw_assemble.block<6, 1>(0, i) - ad_V_Screw_i_inverse.transpose() * spatial_inertia_i * V_Screw_assemble.block<6, 1>(0, i);
        F_Screw_assemble.block<6, 1>(0, i - 1) = F_Screw_i;

        tau(i - 1, 0) = (F_Screw_i.transpose() * A_Screw_assemble.block<6, 1>(0, i - 1)).value();
    }

    return tau;
}

// 牛顿-欧拉正动力学函数
Eigen::VectorXd NewtonEulerForwardDynamics(const Eigen::VectorXd& theta, const Eigen::VectorXd& dtheta, const Eigen::VectorXd& tau)
{
    Eigen::VectorXd d2theta(6);
    Eigen::MatrixXd M_theta(6, 6);
    Eigen::MatrixXd h_theta_dtheta(6, 1);
    Eigen::MatrixXd J_theta(6, 6);
    Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);
    Eigen::MatrixXd T_i_1_i_asb = joint_transform_matrix(theta);

    // 计算惯性矩阵M(theta)
    for (int i = 1; i <= 6; i++) {
        Eigen::VectorXd d2theta_i = Eigen::VectorXd::Zero(6);
        d2theta_i(i - 1) = 1;
        M_theta.block<6, 1>(0, i - 1) = NewtonEulerInverseDynamics(theta, dtheta, d2theta_i, zero, false, T_i_1_i_asb);
    }

    // 计算科里奥利力矩,离心力矩,重力力矩h(theta, dtheta)
    h_theta_dtheta = NewtonEulerInverseDynamics(theta, dtheta, zero, zero, true, T_i_1_i_asb);

    // 计算雅可比矩阵J(theta)
    J_theta = CalculateJacobian(theta, T_i_1_i_asb);

    // 求解线性方程组 M(theta) * d2theta = tau - h(theta, dtheta)
    Eigen::VectorXd b = tau - h_theta_dtheta;
    d2theta = M_theta.colPivHouseholderQr().solve(b);

    return d2theta;
}




// 性能测试函数
void performance_test()
{
    std::cout << "开始性能测试..." << std::endl;
    
    // 初始化随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(-M_PI, M_PI);
    
    // 生成测试数据
    Eigen::VectorXd theta(6);
    Eigen::VectorXd dtheta(6);
    Eigen::VectorXd tau(6);
    
    for (int i = 0; i < 6; i++) {
        theta(i) = dis(gen);
        dtheta(i) = dis(gen);
        tau(i) = dis(gen);
    }
    
    // 预热运行
    std::cout << "预热运行..." << std::endl;
    for (int i = 0; i < 10; i++) {
        NewtonEulerForwardDynamics(theta, dtheta, tau);
    }
    
    // 性能测试
    const int num_runs = 1000;
    std::cout << "开始 " << num_runs << " 次性能测试..." << std::endl;
    
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < num_runs; i++) {
        // 稍微改变输入以避免编译器优化
        Eigen::VectorXd theta_test = theta + Eigen::VectorXd::Constant(6, i * 1e-6);
        Eigen::VectorXd dtheta_test = dtheta + Eigen::VectorXd::Constant(6, i * 1e-6);
        Eigen::VectorXd tau_test = tau + Eigen::VectorXd::Constant(6, i * 1e-6);
        
        NewtonEulerForwardDynamics(theta_test, dtheta_test, tau_test);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    double total_time_ms = duration.count() / 1000.0;
    double avg_time_ms = total_time_ms / num_runs;
    double avg_time_us = avg_time_ms * 1000.0;
    
    std::cout << "\n=== 性能测试结果 ===" << std::endl;
    std::cout << "总运行次数: " << num_runs << std::endl;
    std::cout << "总运行时间: " << total_time_ms << " ms" << std::endl;
    std::cout << "平均运行时间: " << avg_time_ms << " ms" << std::endl;
    std::cout << "平均运行时间: " << avg_time_us << " μs" << std::endl;
    std::cout << "每秒可运行次数: " << (1000.0 / avg_time_ms) << std::endl;
    std::cout << "===================" << std::endl;
}

int main()
{
    // 初始化DH参数（UR3机械臂参数）
    alpha << 0, M_PI/2, 0, 0, M_PI/2, -M_PI/2;
    a << 0, -0.24365, -0.21325, 0, 0, 0;
    d << 0.1519, 0, 0, 0.11235, 0.08535, 0.0819;
    theta_offset << 0, 0, 0, 0, 0, 0;
    
    // 初始化质量参数
    m_assemble << 3.7, 8.393, 2.33, 1.219, 1.219, 0.1879;
    
    // 初始化质心向量
    centroid_vector_assemble.setZero();
    
    // 初始化旋量轴矩阵
    A_Screw_assemble << 0, 0, 0, 0, 0, 0,
                       1, 0, 0, 1, -1, 0,
                       0, 1, 1, 0, 0, 1,
                       0, 0.24365, 0.21325, 0, 0, 0,
                       0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0;
    
    // 运行性能测试
    performance_test();
    
    return 0;
} 