#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include "ros_qtgui.h"
#include <QDebug>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication a(argc, argv);
    
    qDebug() << "Creating ros_qtgui instance...";
    auto w = ros_qtgui::create();
    
    qDebug() << "Initializing RViz...";
    // w->init_rviz();
    qDebug() << "RViz initialization completed";
    
    std::thread spin_thread([w]() -> void { rclcpp::spin(w); });
    spin_thread.detach();
    
    qDebug() << "Showing main window...";
    w->show();
    qDebug() << "Entering Qt event loop...";
    
    return a.exec();
}

