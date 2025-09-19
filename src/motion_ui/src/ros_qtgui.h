#ifndef ROS_QTGUI_H
#define ROS_QTGUI_H

#include <QWidget>
#include <QLabel>
#include <QTimer>
#include <qlineedit.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "my_custom_msgs/msg/diw_ui_print.hpp"
#include "my_custom_msgs/msg/trajectory_output.hpp"
#include "my_custom_msgs/msg/test_forward_position_controller.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "qcustomplot.h"
#include <memory>
#include <QObject>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_default_plugins/view_controllers/orbit/orbit_view_controller.hpp>
#include <rviz_common/view_manager.hpp>
#include "rviz_rendering/render_window.hpp"
#include <QVBoxLayout>
#include <rclcpp/node.hpp>
#include <vector>
#include <cmath>
#include <chrono>
#include <Eigen/Dense>
#include <Eigen/SVD>

namespace Ui {
class ros_qtgui;
}

class ros_qtgui : public QWidget, public rclcpp::Node, public std::enable_shared_from_this<ros_qtgui>
{
    Q_OBJECT

public:
    explicit ros_qtgui(QWidget *parent = nullptr);
    ~ros_qtgui();
    void init_rviz();

    // 添加静态创建函数
    static std::shared_ptr<ros_qtgui> create(QWidget *parent = nullptr) {
        return std::make_shared<ros_qtgui>(parent);
    }

private slots:
    void on_pressure_control_open_pushButton_clicked();
    void on_pressure_control_close_pushButton_clicked();
    void on_spinBox_valueChanged(int arg1);
    void on_pressure_adjust_pushButton_clicked();
    void update_plot();
    void on_textEdit_in_printing_textChanged();
    void on_radioButton_trajectory_priority_clicked(bool checked);
    void on_radioButton_velocity_priority_clicked(bool checked);
    void on_checkBox_start_correct_distance_stateChanged(int arg1);
    void on_checkBox_whether_distance_collect_stateChanged(int arg1);
    void on_spinBox_pressure_adjust_in_printing_valueChanged(int arg1);
    void on_doubleSpinBox_velocity_in_printing_valueChanged(double arg1);
    void on_pushButton_printing_start_clicked();
    void on_checkBox_start_expend_kalman_filter_stateChanged(int arg1);
    void on_doubleSpinBox_distance_platform_valueChanged(double arg1);
    void on_spinBox_theta_data_number_valueChanged(int arg1);
    void on_pushButton_theta_data_collect_start_clicked();

    void on_pushButton_open_kalman_clicked();

    void on_pushButton_close_kalman_clicked();

    void on_pushButton_start_debug_clicked();

    void on_pushButton_torque_correction_clicked();

    void on_doubleSpinBox_planning_velocity_valueChanged(double arg1);

    void on_spinBox_contorl_period_valueChanged(int arg1);

    void on_pushButton_start_planning_clicked();

    void on_doubleSpinBox_move_tcp_x_valueChanged(double arg1);

    void on_doubleSpinBox_move_tcp_y_valueChanged(double arg1);

    void on_doubleSpinBox_move_tcp_z_valueChanged(double arg1);

    void on_pushButton_move_tcp_clicked();



    void on_doubleSpinBox_test_forward_position_controller_printing_velocity_valueChanged(double arg1);

    void on_spinBox_test_forward_position_controller_control_period_valueChanged(int arg1);

    void on_textEdit_test_forward_position_controller_file_path_textChanged();

    void on_pushButton_test_forward_position_controller_clicked();

private:
    Ui::ros_qtgui *ui;
    QLabel *label_pressure_control_state;
    QLabel *label_velocity_linear_x, *label_velocity_linear_y, *label_velocity_linear_z, *label_velocity_scalar;
    QLabel *label_tcp_x, *label_tcp_y, *label_tcp_z;
    QLabel *label_realtime_pressure;
    QLabel *label_logo;
    QLabel *label_distance_z_left, *label_distance_z_right;

    QCustomPlot *velocity_plot;
    QTimer *plot_timer;
    QCPGraph *graph_x, *graph_y, *graph_z, *graph_scalar;
    QVector<double> time_data;
    QVector<double> x_data, y_data, z_data, scalar_data;
    
    const double PI = 3.14159265358979323846; // 双精度
    double start_time;
    int y_axis_max;
    std::string text_file_path;
    bool is_trajectory_priority = false, is_velocity_priority = false, is_start_correct_distance = false, is_distance_collect = false, start_expend_kalman_filter = false;
    int pressure_adjust_value, pressure_adjust_value_in_printing, theta_data_number;
    double velocity_in_printing = 0.0, distance_platform = 0.0;

    // 规划参数
    double velocity_in_planning = 0, control_period = 0.008;

    // 移动末端点
    double move_tcp_x = 0, move_tcp_y = 0, move_tcp_z =0;

    // test_forward_position_controller参数
    double test_forward_position_controller_printing_velocity = 1.0, test_forward_position_controller_control_period = 0.008;
    std::string text_test_forward_position_controller_file_path = "/home/xht/桌面/test.txt";

    std::shared_ptr<rviz_common::RenderPanel> _render_panel;
    std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> _rvizRosNodeTmp;
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr _rvizRosNode;
    std::shared_ptr<rviz_common::VisualizationManager> _manager;
    rviz_default_plugins::view_controllers::OrbitViewController *orbit_controller_;
    rviz_common::ViewManager *view_manager_;
    rviz_common::Display * _grid;
    rviz_common::Display * robot_model_display_;
    
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pressure_control_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pressure_adjust_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr theta_data_number_publisher_;
    rclcpp::Publisher<my_custom_msgs::msg::DiwUiPrint>::SharedPtr diw_ui_print_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_kalman_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_debug_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr torque_correction_publisher_;
    rclcpp::Publisher<my_custom_msgs::msg::TrajectoryOutput>::SharedPtr whether_trajectory_output_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr move_tcp_publisher_;
    rclcpp::Publisher<my_custom_msgs::msg::TestForwardPositionController>::SharedPtr test_forward_position_controller_publisher_;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_curve_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr realtime_pressure_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr tcp_pose_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr distance_z_left_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr distance_z_right_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pressure_control_subscription_;

    void velocity_curve_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void realtime_pressure_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void tcp_pose_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void distance_z_left_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void distance_z_right_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void pressure_control_callback(const std_msgs::msg::Int32::SharedPtr msg);

    void setup_plot();
    Eigen::MatrixXd joint_transform_matrix(const Eigen::MatrixXd& theta);
};

#endif // ROS_QTGUI_H
