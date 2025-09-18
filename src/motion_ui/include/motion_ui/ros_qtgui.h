#ifndef ROS_QTGUI_H
#define ROS_QTGUI_H

#include <QWidget>
#include <QTimer>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "qcustomplot.h"

namespace Ui {
class ros_qtgui;
}

class ros_qtgui : public QWidget, public rclcpp::Node
{
    Q_OBJECT

public:
    explicit ros_qtgui(QWidget *parent = nullptr);
    ~ros_qtgui();

private slots:
    void on_pressure_control_open_pushButton_clicked();
    void on_pressure_control_close_pushButton_clicked();
    void on_spinBox_valueChanged(int arg1);
    void on_pressure_adjust_pushButton_clicked();
    void update_plot();

private:
    void setup_plot();
    void velocity_curve_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void pressure_value_callback(const std_msgs::msg::Int32::SharedPtr msg);
    
    Ui::ros_qtgui *ui;
    QTimer *plot_timer;
    QCustomPlot *velocity_plot;
    QCPGraph *graph_x, *graph_y, *graph_z;
    QVector<double> time_data, x_data, y_data, z_data;
    double start_time;
    int y_axis_max;
    
    QLabel *label_pressure_control_state;
    QLabel *label_velocity_linear_x;
    QLabel *label_velocity_linear_y;
    QLabel *label_velocity_linear_z;
    QLabel *label_velocity_scalar;
    int pressure_adjust_value;
    
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pressure_control_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pressure_adjust_publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pressure_value_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_curve_subscription_;
};

#endif // ROS_QTGUI_H 