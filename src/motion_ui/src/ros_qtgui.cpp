#include "ros_qtgui.h"
#include "ui_ros_qtgui.h"


ros_qtgui::ros_qtgui(QWidget *parent)
    : QWidget(parent)
    , Node("motion_ui")
    , ui(new Ui::ros_qtgui)
{
    ui->setupUi(this);
    this->setWindowTitle("DIW增材制造控制系统");  // 设置窗口标题
    label_pressure_control_state = ui->label_pressure_control_state;
    label_velocity_linear_x = ui->label_velocity_linear_x;
    label_velocity_linear_y = ui->label_velocity_linear_y;
    label_velocity_linear_z = ui->label_velocity_linear_z;
    label_velocity_scalar = ui->label_velocity_scalar;
    label_tcp_x = ui->label_tcp_x;
    label_tcp_y = ui->label_tcp_y;
    label_tcp_z = ui->label_tcp_z;
    label_distance_z_left = ui->label_distance_z_left;
    label_distance_z_right = ui->label_distance_z_right;
    label_realtime_pressure = ui->label_realtime_pressure;

    QPixmap pixmap("/home/xht/qt_workspace/src/motion_ui/out.png");
    if (pixmap.isNull()) {
        qDebug() << "Failed to load image from absolute path";
    } else {
        qDebug() << "Successfully loaded image from absolute path";
    }
    ui->label_logo->setPixmap(pixmap);
    label_pressure_control_state->setStyleSheet("color: red;");
    pressure_adjust_value = 0;
    theta_data_number = 0;

    // 初始化图表
    velocity_plot = ui->velocity_plot;
    setup_plot();
    start_time = this->now().seconds();
    
    // 创建定时器用于更新图表
    plot_timer = new QTimer(this);
    connect(plot_timer, &QTimer::timeout, this, &ros_qtgui::update_plot);
    plot_timer->start(50);  // 每50ms更新一次，即20Hz
    
    // 初始化发布者
    pressure_control_publisher_ = this->create_publisher<std_msgs::msg::Int32>(
        "/pressure_control_ui", 10);

    pressure_adjust_publisher_ = this->create_publisher<std_msgs::msg::Int32>(
        "/pressure_adjust", 10);

    diw_ui_print_publisher_ = this->create_publisher<my_custom_msgs::msg::DiwUiPrint>(
        "/diw_ui_printing", 10);

    theta_data_number_publisher_ = this->create_publisher<std_msgs::msg::Int32>(
        "/theta_data_number", 10);

    start_kalman_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
        "/start_expend_kalman_filter", 10);

    start_debug_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
        "/start_debug", 10);

    torque_correction_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
        "/start_torque_correction", 10);

    whether_trajectory_output_publisher_ = this->create_publisher<my_custom_msgs::msg::TrajectoryOutput>(
        "/whether_trajectory_output", 10);

    move_tcp_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>(
        "/move_tcp", 10);

    test_forward_position_controller_publisher_ = this->create_publisher<my_custom_msgs::msg::TestForwardPositionController>(
        "/test_forward_position_controller_topic", 10);

    // 初始化订阅者
    velocity_curve_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/velocity_curve", 10,
        std::bind(&ros_qtgui::velocity_curve_callback, this, std::placeholders::_1));

    realtime_pressure_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
        "/realtime_pressure", 10,
        std::bind(&ros_qtgui::realtime_pressure_callback, this, std::placeholders::_1));

    tcp_pose_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&ros_qtgui::tcp_pose_callback, this, std::placeholders::_1));

    distance_z_left_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
        "/realtime_distance_left", 10,
        std::bind(&ros_qtgui::distance_z_left_callback, this, std::placeholders::_1));

    distance_z_right_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
        "/realtime_distance_right", 10,
        std::bind(&ros_qtgui::distance_z_right_callback, this, std::placeholders::_1));

    pressure_control_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
        "/pressure_control", 10,
        std::bind(&ros_qtgui::pressure_control_callback, this, std::placeholders::_1));

   
    init_rviz();

}

void ros_qtgui::init_rviz()
{
    _rvizRosNodeTmp = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("rviz_render_node");
    _rvizRosNode = _rvizRosNodeTmp;
    
    QApplication::processEvents();
    _render_panel = std::make_shared<rviz_common::RenderPanel>();
    QApplication::processEvents();

    _render_panel->getRenderWindow()->initialize();
    rviz_common::WindowManagerInterface * wm = nullptr;

    auto clock = _rvizRosNode.lock()->get_raw_node()->get_clock();

    _manager = std::make_shared<rviz_common::VisualizationManager>(_render_panel.get(), _rvizRosNode, wm, clock);
    _render_panel->initialize(_manager.get());

    QApplication::processEvents();

    _render_panel->setMouseTracking(true);
    _render_panel->setFocusPolicy(Qt::StrongFocus);

    _manager->setFixedFrame("base_link");
    _manager->initialize();
    _manager->removeAllDisplays();
    _manager->startUpdate();

    _manager->getViewManager()->setCurrentViewControllerType("rviz_default_plugins/Orbit");

    auto orbit_view_controller = _manager->getViewManager()->getCurrent();
    if (!orbit_view_controller) {
        qDebug() << "Orbit view controller could not be set.";
        return;
    }

    // Set default distance and focal point for the camera
    orbit_view_controller->subProp("Distance")->setValue(10.0);
    orbit_view_controller->subProp("Focal Point")->setValue(QVariant::fromValue(QVector3D(0.0, 0.0, 0.0)));

    // Set initial orientation of the camera
    orbit_view_controller->subProp("Pitch")->setValue(1.5708);  // Example angle in radians
    orbit_view_controller->subProp("Yaw")->setValue(3.14);     // Example angle in radians

    // Set Interact tool as the active tool to enable mouse interactions
    auto tool_manager = _manager->getToolManager();
    tool_manager->setCurrentTool(tool_manager->addTool("rviz_default_plugins/Interact"));

    // 设置render_panel的大小策略
    _render_panel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    
    // 创建一个新的QWidget作为容器
    QWidget* container = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(container);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setAlignment(Qt::AlignCenter);
    layout->addWidget(_render_panel.get());
    
    // 将容器添加到scrollArea
    ui->scrollArea->setWidget(container);

    // 添加网格显示
    _grid = _manager->createDisplay("rviz_default_plugins/Grid", "adjustable grid", true);
    if (_grid == NULL) {
        throw std::runtime_error("Error creating grid display");
    }

    // 配置网格样式
    _grid->subProp("Line Style")->setValue("Lines");
    _grid->subProp("Alpha")->setValue(0.5f);
    _grid->subProp("Color")->setValue(QColor(Qt::white));
    _grid->subProp("Cell Size")->setValue(1.5f);
    _grid->subProp("Plane Cell Count")->setValue(5);

    // 创建机器人模型显示
    robot_model_display_ = _manager->createDisplay("rviz_default_plugins/RobotModel", "RobotModel Display", true);
    if (robot_model_display_) {
        robot_model_display_->subProp("Description Topic")->setValue("/robot_description");  
        robot_model_display_->subProp("TF Prefix")->setValue("");  
        robot_model_display_->subProp("Mass Properties")->setValue(1);
        qDebug() << "RobotModel display configured for /robot_description topic.";
    } else {
        qDebug() << "Failed to create RobotModel display.";
    }
}

ros_qtgui::~ros_qtgui()
{
    delete ui;
}

void ros_qtgui::on_pressure_control_open_pushButton_clicked()
{
    auto message_control = std_msgs::msg::Int32();
    message_control.data = 0;  // 0 means pressure control open
    pressure_control_publisher_->publish(message_control);
    label_pressure_control_state->setText("Opened");
    label_pressure_control_state->setStyleSheet("color: red;");
    RCLCPP_INFO(this->get_logger(), "Published pressure open command!");
}

void ros_qtgui::on_pressure_control_close_pushButton_clicked()
{
    auto message_control = std_msgs::msg::Int32();
    message_control.data = 1;  // 1 means pressure control close
    pressure_control_publisher_->publish(message_control);
    RCLCPP_INFO(this->get_logger(), "Published pressure close command!");
}

void ros_qtgui::on_spinBox_valueChanged(int arg1)
{
    pressure_adjust_value = arg1;
}

void ros_qtgui::on_pressure_adjust_pushButton_clicked()
{
    auto message_adjust = std_msgs::msg::Int32();
    message_adjust.data = pressure_adjust_value;
    pressure_adjust_publisher_->publish(message_adjust);
    RCLCPP_INFO(this->get_logger(), "Published pressure adjust command: %d!", message_adjust.data);
}

void ros_qtgui::setup_plot()
{
    // 设置图表标题和轴标签
    velocity_plot->plotLayout()->insertRow(0);
    velocity_plot->plotLayout()->addElement(0, 0, new QCPTextElement(velocity_plot, "速度曲线", QFont("sans", 12, QFont::Bold)));
    velocity_plot->xAxis->setLabel("时间 (s)");
    velocity_plot->yAxis->setLabel("速度 (mm/s)");
    
    // 创建三条曲线
    graph_x = velocity_plot->addGraph();
    graph_y = velocity_plot->addGraph();
    graph_z = velocity_plot->addGraph();
    graph_scalar = velocity_plot->addGraph();

    // 设置曲线颜色
    graph_x->setPen(QPen(Qt::red));
    graph_y->setPen(QPen(Qt::green));
    graph_z->setPen(QPen(Qt::blue));
    graph_scalar->setPen(QPen(Qt::black));

    // 设置图例
    graph_x->setName("X轴速度");
    graph_y->setName("Y轴速度");
    graph_z->setName("Z轴速度");
    graph_scalar->setName("标量速度");
    velocity_plot->legend->setVisible(true);
    
    // 设置坐标轴范围
    y_axis_max = 20;
    velocity_plot->xAxis->setRange(0, 10);
    velocity_plot->yAxis->setRange(-y_axis_max, y_axis_max);
    
    // 允许用户交互
    velocity_plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
}

void ros_qtgui::velocity_curve_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    // 更新标签显示
    label_velocity_linear_x->setText(QString::number(msg->twist.linear.x * 1000, 'f', 2));
    label_velocity_linear_y->setText(QString::number(msg->twist.linear.y * 1000, 'f', 2));
    label_velocity_linear_z->setText(QString::number(msg->twist.linear.z * 1000, 'f', 2));
    label_velocity_scalar->setText(QString::number(msg->twist.angular.x * 1000, 'f', 2));
    
    // 更新图表数据
    double current_time = this->now().seconds() - start_time;
    time_data.append(current_time);
    x_data.append(msg->twist.linear.x * 1000);  // 转换为mm/s
    y_data.append(msg->twist.linear.y * 1000);
    z_data.append(msg->twist.linear.z * 1000);
    scalar_data.append(msg->twist.angular.x * 1000);
    
    // 更新图表数据
    graph_x->setData(time_data, x_data);
    graph_y->setData(time_data, y_data);
    graph_z->setData(time_data, z_data);
    graph_scalar->setData(time_data, scalar_data);

    // 自动调整X轴范围
    if (current_time > velocity_plot->xAxis->range().upper)
    {
        velocity_plot->xAxis->setRange(current_time - 10, current_time);
    }

    // 自动调整Y轴范围
    double abs_x = std::abs(msg->twist.linear.x * 1000);  // 转换为mm/s并取绝对值
    double abs_y = std::abs(msg->twist.linear.y * 1000);
    double abs_z = std::abs(msg->twist.linear.z * 1000);
    double abs_scalar = std::abs(msg->twist.angular.x * 1000);
    
    // 找出三个方向中的最大绝对值
    double max_abs = std::max({abs_x, abs_y, abs_z, abs_scalar});
    
    // 如果最大值超过当前范围，则更新Y轴范围
    if (max_abs > y_axis_max)
    {
        y_axis_max = max_abs + 1;
        velocity_plot->yAxis->setRange(-y_axis_max, y_axis_max);
    }
}

void ros_qtgui::realtime_pressure_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    label_realtime_pressure->setText(QString::number(msg->data, 'f', 2));
}

void ros_qtgui::update_plot()
{
    velocity_plot->replot();
}

void ros_qtgui::on_textEdit_in_printing_textChanged()
{
    text_file_path = ui->textEdit_in_printing->toPlainText().toStdString();
}

void ros_qtgui::on_radioButton_trajectory_priority_clicked(bool checked)
{
    is_trajectory_priority = checked;
}

void ros_qtgui::on_radioButton_velocity_priority_clicked(bool checked)
{
    is_velocity_priority = checked;
}

void ros_qtgui::on_spinBox_pressure_adjust_in_printing_valueChanged(int arg1)
{
    pressure_adjust_value_in_printing = arg1;
}

void ros_qtgui::on_doubleSpinBox_velocity_in_printing_valueChanged(double arg1)
{
    velocity_in_printing = arg1;
}

void ros_qtgui::on_pushButton_printing_start_clicked()
{
    auto msg_diw_ui_print = my_custom_msgs::msg::DiwUiPrint();
    msg_diw_ui_print.pressure_adjust_value = pressure_adjust_value_in_printing;
    msg_diw_ui_print.printing_velocity = velocity_in_printing;
    msg_diw_ui_print.file_path = text_file_path;
    if (is_trajectory_priority && is_velocity_priority)
    {
        msg_diw_ui_print.trajectory_type = 0;
    }
    else if (is_trajectory_priority)
    {
        msg_diw_ui_print.trajectory_type = 0;
    }
    else if (is_velocity_priority) 
    {
        msg_diw_ui_print.trajectory_type = 1;
    }
    else
    {
        msg_diw_ui_print.trajectory_type = 0;
    }
    msg_diw_ui_print.whether_start_correct_distance = is_start_correct_distance;
    msg_diw_ui_print.whether_distance_collect = is_distance_collect;
    msg_diw_ui_print.start_expend_kalman_filter = start_expend_kalman_filter;
    msg_diw_ui_print.distance_platform = distance_platform;
    diw_ui_print_publisher_->publish(msg_diw_ui_print);
    RCLCPP_INFO(this->get_logger(), "Published DIW printing command!");
}

void ros_qtgui::tcp_pose_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Eigen::MatrixXd position = Eigen::MatrixXd::Zero(6, 1);
    // position << msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4], msg->position[5];
    // Eigen::MatrixXd T_i_1_i_assemble = joint_transform_matrix(position);

    // Eigen::MatrixXd T_6 = Eigen::MatrixXd::Zero(4, 1);
    // T_6 << 0, 0, 0, 1;

    // for(int i=1;i<=6;i++)
    // {
    //     T_6 = T_i_1_i_assemble.block<4, 4>(0, 4*(6 - i)) * T_6;
    // }

    // label_tcp_x->setText(QString::number(T_6(0, 0) * 1000, 'f', 2));
    // label_tcp_y->setText(QString::number(T_6(1, 0) * 1000, 'f', 2));
    // label_tcp_z->setText(QString::number(T_6(2, 0) * 1000, 'f', 2));
}

void ros_qtgui::on_checkBox_start_correct_distance_stateChanged(int arg1)
{
    if (arg1 == 0)
    {
        is_start_correct_distance = false;
    }else
    {
        is_start_correct_distance = true;
    }
}

void ros_qtgui::on_checkBox_whether_distance_collect_stateChanged(int arg1)
{
    if (arg1 == 0)
    {
        is_distance_collect = false;
    }else
    {
        is_distance_collect = true;
    }
}


void ros_qtgui::on_checkBox_start_expend_kalman_filter_stateChanged(int arg1)
{
    if (arg1 == 0)
    {
        start_expend_kalman_filter = false;
    }else
    {
        start_expend_kalman_filter = true;
    }
}

void ros_qtgui::on_doubleSpinBox_distance_platform_valueChanged(double arg1)
{
    distance_platform = arg1;
}

void ros_qtgui::distance_z_left_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    label_distance_z_left->setText(QString::number(msg->data));
}

void ros_qtgui::distance_z_right_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    label_distance_z_right->setText(QString::number(msg->data));
}

void ros_qtgui::pressure_control_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    if (msg->data == 0)
    {
        label_pressure_control_state->setText("Opened");
        label_pressure_control_state->setStyleSheet("color: red;");
    }else
    {
        label_pressure_control_state->setText("Closed");
        label_pressure_control_state->setStyleSheet("color: red;");
    }
}

void ros_qtgui::on_spinBox_theta_data_number_valueChanged(int arg1)
{
    theta_data_number = arg1;
}


void ros_qtgui::on_pushButton_theta_data_collect_start_clicked()
{
    std_msgs::msg::Int32 msg_theta_data_number;
    msg_theta_data_number.data = theta_data_number;
    theta_data_number_publisher_->publish(msg_theta_data_number);
    RCLCPP_INFO(this->get_logger(), "Published theta data collecting command!");
}



void ros_qtgui::on_pushButton_open_kalman_clicked()
{
    std_msgs::msg::Bool msg_open_kalman;
    msg_open_kalman.data = true;
    start_kalman_publisher_->publish(msg_open_kalman);
}


void ros_qtgui::on_pushButton_close_kalman_clicked()
{
    std_msgs::msg::Bool msg_open_kalman;
    msg_open_kalman.data = false;
    start_kalman_publisher_->publish(msg_open_kalman);
}


void ros_qtgui::on_pushButton_start_debug_clicked()
{
    std_msgs::msg::Bool msg_start_debug;
    msg_start_debug.data = true;
    start_debug_publisher_->publish(msg_start_debug);
}

Eigen::MatrixXd ros_qtgui::joint_transform_matrix(const Eigen::MatrixXd& theta)
{
    Eigen::VectorXd alpha_dynamic(6);
    Eigen::VectorXd a_dynamic(6);
    Eigen::VectorXd d_dynamic(6);
    alpha_dynamic << PI/2, 0, 0, PI/2, -PI/2, 0;
    a_dynamic << 0, -0.24376288177085403, -0.21344066092027106, 0, 0, 0;
    d_dynamic << 0.15187159190950295, 0, 0, 0.11209388124617316, 0.085378607490435937, 0.08241227224463675;
    Eigen::MatrixXd T_i_1_i_assemble = Eigen::MatrixXd::Zero(4, 4 * 7);
    for (int i = 1; i <= 6; i++) {
        double cos_theta = std::cos(std::fmod(theta(i-1, 0), 2 * PI));
        double sin_theta = std::sin(std::fmod(theta(i-1, 0), 2 * PI));
        double cos_alpha = std::cos(std::fmod(alpha_dynamic(i-1, 0), 2 * PI));
        double sin_alpha = std::sin(std::fmod(alpha_dynamic(i-1, 0), 2 * PI));
        T_i_1_i_assemble.block<4, 4>(0, 4 * (i - 1)) << cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, a_dynamic(i-1) * cos_theta,
            sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, a_dynamic(i-1) * sin_theta,
            0, sin_alpha, cos_alpha, d_dynamic(i-1),
            0, 0, 0, 1;
    }

    T_i_1_i_assemble.block<4, 4>(0, 4 * 6) << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    return T_i_1_i_assemble;
}


void ros_qtgui::on_pushButton_torque_correction_clicked()
{
    std_msgs::msg::Bool msg_torque_correction;
    msg_torque_correction.data = true;
    torque_correction_publisher_->publish(msg_torque_correction);
}


void ros_qtgui::on_doubleSpinBox_planning_velocity_valueChanged(double arg1)
{
    velocity_in_planning = arg1;
}

void ros_qtgui::on_spinBox_contorl_period_valueChanged(int arg1)
{
    control_period = arg1 * 0.001;
}

void ros_qtgui::on_pushButton_start_planning_clicked()
{
    auto msg_trajectory_output = my_custom_msgs::msg::TrajectoryOutput();
    msg_trajectory_output.control_period = control_period;
    msg_trajectory_output.planning_velocity = velocity_in_planning;
    msg_trajectory_output.file_path = text_file_path;
    msg_trajectory_output.whether_trajectory_output = true;
    whether_trajectory_output_publisher_->publish(msg_trajectory_output);
}


void ros_qtgui::on_doubleSpinBox_move_tcp_x_valueChanged(double arg1)
{
    move_tcp_x = arg1;
}


void ros_qtgui::on_doubleSpinBox_move_tcp_y_valueChanged(double arg1)
{
    move_tcp_y = arg1;
}


void ros_qtgui::on_doubleSpinBox_move_tcp_z_valueChanged(double arg1)
{
    move_tcp_z = arg1;
}


void ros_qtgui::on_pushButton_move_tcp_clicked()
{
    auto msg_move = geometry_msgs::msg::Pose();
    msg_move.position.x = move_tcp_x;
    msg_move.position.y = move_tcp_y;
    msg_move.position.z = move_tcp_z;
    msg_move.orientation.x = 0.7071067812;
    msg_move.orientation.y = 0.7071067812;
    msg_move.orientation.z = 0;
    msg_move.orientation.w = 0;
    move_tcp_publisher_->publish(msg_move);
}

void ros_qtgui::on_doubleSpinBox_test_forward_position_controller_printing_velocity_valueChanged(double arg1)
{
    test_forward_position_controller_printing_velocity = arg1;
}


void ros_qtgui::on_spinBox_test_forward_position_controller_control_period_valueChanged(int arg1)
{
    test_forward_position_controller_control_period = arg1 * 0.001;
}


void ros_qtgui::on_textEdit_test_forward_position_controller_file_path_textChanged()
{
    text_test_forward_position_controller_file_path = ui->textEdit_test_forward_position_controller_file_path->toPlainText().toStdString();
}


void ros_qtgui::on_pushButton_test_forward_position_controller_clicked()
{
    auto msg_test_forward_position_controller = my_custom_msgs::msg::TestForwardPositionController();
    msg_test_forward_position_controller.control_period = test_forward_position_controller_control_period;
    msg_test_forward_position_controller.printing_velocity = test_forward_position_controller_printing_velocity;
    msg_test_forward_position_controller.file_path = text_test_forward_position_controller_file_path;
    test_forward_position_controller_publisher_->publish(msg_test_forward_position_controller);
}
