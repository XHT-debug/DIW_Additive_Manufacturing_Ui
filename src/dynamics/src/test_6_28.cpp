

// 拓展卡尔曼滤波初始化
// 目标：初始化DH参数，质量参数，质心向量，旋量轴矩阵
// 输入：无
// 输出：无
void expend_kalman_filter_setup()
{
    //  初始化DH参数（UR3机械臂参数）
    alpha << M_PI/2, 0, 0, M_PI/2, -M_PI/2, 0;
    a << 0, -0.24365, -0.21325, 0, 0, 0;
    d << 0.1519, 0, 0, 0.11235, 0.08535, 0.0819;
    
    // 初始化质量参数
    m_assemble << 2.000, 3.420, 1.260, 0.800, 0.800, 0.350;
    
    // 初始化质心向量
    centroid_vector_assemble <<     0,    0.13,   0.05,    0,    0,     0,
                                 -0.02,       0,      0,    0,    0,     0,
                                     0, 0.01157, 0.0238, 0.01, 0.01, -0.02;
    
    // 初始化旋量轴矩阵
    A_Screw_assemble << 0,       0,       0, 0,  0, 0,
                        1,       0,       0, 1, -1, 0,  
                        0,       1,       1, 0,  0, 1,
                        0, 0.24365, 0.21325, 0,  0, 0,
                        0,       0,       0, 0,  0, 0,
                        0,       0,       0, 0,  0, 0;
    
}

// 反对称矩阵计算函数
// 目标：计算向量的反对称矩阵
// 输入：向量
// 输出：反对称矩阵
Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0,    -v.z(), v.y(),
         v.z(), 0,    -v.x(),
         -v.y(), v.x(), 0;
    return m;
}

// 伴随变换矩阵计算函数
// 目标：计算坐标系转换矩阵的伴随变换矩阵，与常规的伴随矩阵不同
// 输入：坐标系转换矩阵
// 输出：伴随变换矩阵
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
// 目标：当前关节角度下的关节转换矩阵
// 输入：关节角度
// 输出：关节转换矩阵
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

// 末端力旋量计算函数
// 目标：当前关节角度下的末端力旋量
// 输入：关节转换矩阵
// 输出：末端力旋量
Eigen::MatrixXd CalculateF_Screw_Tip(const Eigen::MatrixXd& T_i_1_i_asb)
{
    Eigen::MatrixXd F_Screw_tip = Eigen::MatrixXd::Zero(6, 1);

    // 计算不同姿态下末端喷嘴等附加设备在末端坐标系下的力旋量
    Eigen::MatrixXd T_0_6 = T_i_1_i_asb.block<4, 4>(0, 0) * T_i_1_i_asb.block<4, 4>(0, 4 * 1) * T_i_1_i_asb.block<4, 4>(0, 4 * 2) * T_i_1_i_asb.block<4, 4>(0, 4 * 3) * T_i_1_i_asb.block<4, 4>(0, 4 * 4) * T_i_1_i_asb.block<4, 4>(0, 4 * 5);
    Eigen::VectorXd g_tip = T_0_6.block<3, 3>(0, 0) * Eigen::Vector3d(0, 0, -g);
    F_Screw_tip.block<3, 1>(0, 0) = m_tip * g_tip;
    F_Screw_tip.block<3, 1>(3, 0) = skewSymmetric(centroid_vector_tip) * F_Screw_tip.block<3, 1>(0, 0);

    return F_Screw_tip;
}


// 雅可比矩阵计算函数
// 目标：当前关节角度下的雅可比矩阵
// 输入：关节角度、关节转换矩阵、关节转换矩阵的伴随矩阵
// 输出：雅可比矩阵
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
// 目标：实现ur六轴机械臂的逆向动力学计算
// 输入：关节角度、关节速度、关节加速度
// 输出：关节力矩
Eigen::MatrixXd NewtonEulerInverseDynamics(co// 反对称矩阵计算函数
// 目标：计算向量的反对称矩阵
// 输入：向量
// 输出：反对称矩阵nst Eigen::VectorXd& theta, const Eigen::VectorXd& dtheta, const Eigen::VectorXd& d2theta, const Eigen::VectorXd& F_Screw_tip, const bool whether_gravity, const Eigen::MatrixXd& T_i_1_i_assemble)
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
        Eigen::VectorXd centroid_vector_i = centroid_vector_assemble.block<3, 1>(0, i - 1);

        Eigen::MatrixXd ad_V_Screw_i_inverse = Eigen::MatrixXd::Zero(6, 6);
        ad_V_Screw_i_inverse.block<3, 3>(0, 0) = skewSymmetric(V_Screw_assemble.block<3, 1>(0, i - 1));
        ad_V_Screw_i_inverse.block<3, 3>(3, 0) = skewSymmetric(V_Screw_assemble.block<3, 1>(3, i - 1));
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
// 目标：实现ur六轴机械臂的正向动力学计算
// 输入：关节角度、关节速度、关节力矩
// 输出：关节加速度
Eigen::VectorXd NewtonEulerForwardDynamics(const Eigen::VectorXd& theta, const Eigen::VectorXd& dtheta, const Eigen::VectorXd& tau)
{
    Eigen::VectorXd d2theta(6);
    Eigen::MatrixXd M_theta(6, 6);
    Eigen::MatrixXd h_theta_dtheta(6, 1);
    Eigen::MatrixXd J_theta(6, 6);
    Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);
    Eigen::MatrixXd T_i_1_i_asb = joint_transform_matrix(theta);
    Eigen::MatrixXd F_Screw_tip = CalculateF_Screw_Tip(T_i_1_i_asb);

    // 计算惯性矩阵M(theta)
    for (int i = 1; i <= 6; i++) {
        Eigen::VectorXd d2theta_i = Eigen::VectorXd::Zero(6);
        d2theta_i(i - 1) = 1;
        M_theta.block<6, 1>(0, i - 1) = NewtonEulerInverseDynamics(theta, dtheta, d2theta_i, zero, false, T_i_1_i_asb);
    }

    // 计算科里奥利力矩,离心力矩,重力力矩 h(theta, dtheta)
    h_theta_dtheta = NewtonEulerInverseDynamics(theta, dtheta, zero, zero, true, T_i_1_i_asb);

    // 计算雅可比矩阵 J(theta)
    J_theta = CalculateJacobian(theta, T_i_1_i_asb);

    // 求解线性方程组 M(theta) * d2theta = tau - h(theta, dtheta) - J_theta.transpose() * F_Screw_tip
    Eigen::VectorXd b = tau - h_theta_dtheta - J_theta.transpose() * F_Screw_tip;
    d2theta = M_theta.colPivHouseholderQr().solve(b);

    return d2theta;
}

// 六轴机械臂的连续时间状态空间模型计算
// 目标：实现ur六轴机械臂的连续时间状态空间模型计算
// 输入：关节角度、关节速度、关节力矩
// 输出：状态空间模型
Eigen::MatrixXd ContinuousStateSpaceModel(const Eigen::VectorXd& theta, const Eigen::VectorXd& dtheta, const Eigen::VectorXd& tau)
{
    // 状态空间模型为：
    // x_dot = A * x + B * u
    // y = C * x + D * u
    // 其中x为状态变量，u为输入变量，y为输出变量
    // A为状态空间矩阵，B为输入矩阵，C为输出矩阵，D为直接传递矩阵
    // 定义状态变量x为关节角度和关节速度，即x = [theta, dtheta]
    // 定义输入变量u为关节力矩，即u = tau
    // 定义输出变量y为关节加速度，即y = d2theta
    // 输出：状态空间模型

    // 系统在某角度下的状态空间模型
    Eigen::MatrixXd A(12, 12);
    // Eigen::MatrixXd B(12, 6);
    // Eigen::MatrixXd C(6, 12);
    // Eigen::MatrixXd D(6, 6);

    Eigen::MatrixXd T_i_1_i_asb = joint_transform_matrix(theta);
    Eigen::MatrixXd M_theta(6, 6);
    A.block<6, 6>(0, 0) = Eigen::MatrixXd::Zero(6, 6);
    A.block<6, 6>(0, 6) = Eigen::MatrixXd::Identity(6, 6);

    // 计算惯性矩阵M(theta)
    for (int i = 1; i <= 6; i++) {
        Eigen::VectorXd d2theta_i = Eigen::VectorXd::Zero(6);
        d2theta_i(i - 1) = 1;
        M_theta.block<6, 1>(0, i - 1) = NewtonEulerInverseDynamics(theta, dtheta, d2theta_i, Eigen::VectorXd::Zero(6), false, T_i_1_i_asb);
    }

    // 计算当前关节角度，角速度的系统矩阵，通过牛顿欧拉动力学计算
    // 系统动力学方程为：M(theta) * d2theta = tau - h(theta, dtheta) - J_theta.transpose() * F_Screw_tip
    // 1.计算每个关节角度对关节角加速度的微分，微分布长设置1e-6,采用2阶中心差分法
    for (int i = 1; i <= 6; i++) {
        Eigen::VectorXd theta_perturb = theta;
        theta_perturb(i - 1) += theta_epsilon;
        Eigen::VectorXd d2theta_perturb1 = NewtonEulerForwardDynamics(theta_perturb, dtheta, tau);
        theta_perturb(i - 1) -= 2 * theta_epsilon;
        Eigen::VectorXd d2theta_perturb2 = NewtonEulerForwardDynamics(theta_perturb, dtheta, tau);
        A.block<6, 1>(6, i - 1) =  (d2theta_perturb1 - d2theta_perturb2) / (2 * theta_epsilon);
    }

    // 2.计算每个关节角度对关节角速度的微分，微分布长设置1e-6,采用2阶中心差分法
    // 动力学方程中仅有h(theta, dtheta)对dtheta的微分，因此需要计算h(theta, dtheta)对dtheta的微分,减少计算量
    for (int i = 1; i <= 6; i++) {
        Eigen::VectorXd dtheta_perturb = dtheta;
        dtheta_perturb(i - 1) += dtheta_epsilon;
        // 计算科里奥利力矩,离心力矩,重力力矩 h(theta, dtheta)
        Eigen::VectorXd h_theta_dtheta_perturb1 = NewtonEulerInverseDynamics(theta, dtheta_perturb, Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6), true, T_i_1_i_asb);
        dtheta_perturb(i - 1) -= 2 * dtheta_epsilon;
        Eigen::VectorXd h_theta_dtheta_perturb2 = NewtonEulerInverseDynamics(theta, dtheta_perturb, Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6), true, T_i_1_i_asb);
        A.block<6, 1>(6, 6 + i - 1) =  M_theta.inverse() * (h_theta_dtheta_perturb1 - h_theta_dtheta_perturb2) / (2 * dtheta_epsilon);
    }
    
    return A;
}

// 六轴机械臂的离散时间状态空间模型计算
// 目标：实现ur六轴机械臂的离散时间状态空间模型计算
// 输入：关节角度、关节速度、关节力矩
// 输出：离散时间状态空间模型
Eigen::MatrixXd DiscreteStateSpaceModel(const Eigen::VectorXd& theta, const Eigen::VectorXd& dtheta, const Eigen::VectorXd& tau)
{

    // 计算连续时间状态空间模型系统矩阵A
    Eigen::MatrixXd A = ContinuousStateSpaceModel(theta, dtheta, tau);

    // 采用一阶保持(First-Order Hold)方法将连续时间状态空间模型离散化
    // 计算离散时间状态空间模型系统矩阵Ad
    Eigen::MatrixXd Ad = Eigen::MatrixXd::Identity(12, 12) + A * dt;

    return Ad;    
}   


// // 性能测试函数
// void performance_test()
// {
//     std::cout << "开始性能测试..." << std::endl;
    
//     // 初始化随机数生成器
//     std::random_device rd;
//     std::mt19937 gen(rd());
//     std::uniform_real_distribution<double> dis(-M_PI, M_PI);
    
//     // 生成测试数据
//     Eigen::VectorXd theta(6);
//     Eigen::VectorXd dtheta(6);
//     Eigen::VectorXd tau(6);
    
//     for (int i = 0; i < 6; i++) {
//         theta(i) = dis(gen);
//         dtheta(i) = dis(gen);
//         tau(i) = dis(gen);
//     }
    
//     // 预热运行
//     std::cout << "预热运行..." << std::endl;
//     for (int i = 0; i < 10; i++) {
//         StateSpaceModel(theta, dtheta, tau);
//     }
    
//     // 性能测试
//     const int num_runs = 1000;
//     std::cout << "开始 " << num_runs << " 次性能测试..." << std::endl;
    
//     auto start = std::chrono::high_resolution_clock::now();
    
//     for (int i = 0; i < num_runs; i++) {
//         // 稍微改变输入以避免编译器优化
//         Eigen::VectorXd theta_test = theta + Eigen::VectorXd::Constant(6, i * 1e-6);
//         Eigen::VectorXd dtheta_test = dtheta + Eigen::VectorXd::Constant(6, i * 1e-6);
//         Eigen::VectorXd tau_test = tau + Eigen::VectorXd::Constant(6, i * 1e-6);
        
//         StateSpaceModel(theta_test, dtheta_test, tau_test);
//     }
    
//     auto end = std::chrono::high_resolution_clock::now();
//     auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
//     double total_time_ms = duration.count() / 1000.0;
//     double avg_time_ms = total_time_ms / num_runs;
//     double avg_time_us = avg_time_ms * 1000.0;
    
//     std::cout << "\n=== 性能测试结果 ===" << std::endl;
//     std::cout << "总运行次数: " << num_runs << std::endl;
//     std::cout << "总运行时间: " << total_time_ms << " ms" << std::endl;
//     std::cout << "平均运行时间: " << avg_time_ms << " ms" << std::endl;
//     std::cout << "平均运行时间: " << avg_time_us << " μs" << std::endl;
//     std::cout << "每秒可运行次数: " << (1000.0 / avg_time_ms) << std::endl;
//     std::cout << "===================" << std::endl;
// }

// int main()
// {    
//     // 初始化DH参数（UR3机械臂参数）
//     alpha << M_PI/2, 0, 0, M_PI/2, -M_PI/2, 0;
//     a << 0, -0.24365, -0.21325, 0, 0, 0;
//     d << 0.1519, 0, 0, 0.11235, 0.08535, 0.0819;
    
//     // 初始化质量参数
//     m_assemble << 2.000, 3.420, 1.260, 0.800, 0.800, 0.350;
    
//     // 初始化质心向量
//     centroid_vector_assemble <<     0,    0.13,   0.05,    0,    0,     0,
//                                  -0.02,       0,      0,    0,    0,     0,
//                                      0, 0.01157, 0.0238, 0.01, 0.01, -0.02;
    
//     // 初始化旋量轴矩阵
//     A_Screw_assemble << 0,       0,       0, 0,  0, 0,
//                         1,       0,       0, 1, -1, 0,  
//                         0,       1,       1, 0,  0, 1,
//                         0, 0.24365, 0.21325, 0,  0, 0,
//                         0,       0,       0, 0,  0, 0,
//                         0,       0,       0, 0,  0, 0;
    
//     // 运行性能测试
//     performance_test();
    
//     return 0;
// } 