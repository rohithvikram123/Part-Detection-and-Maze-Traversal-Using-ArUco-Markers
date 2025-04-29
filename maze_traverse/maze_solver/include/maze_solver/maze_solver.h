#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <vector>
#include "mage_msgs/msg/advanced_logical_camera_image.hpp"
#include "mage_msgs/msg/part.hpp"

class MazeSolver : public rclcpp::Node{

public:
    MazeSolver(std::string maze_solver): Node(maze_solver){
        // aruco marker id
        this->declare_parameter<std::string>("aruco_marker_0");
        this->declare_parameter<std::string>("aruco_marker_1");
        this->declare_parameter<std::string>("aruco_marker_2");
        m_aruco_marker_0 = this->get_parameter("aruco_marker_0").as_string();
        m_aruco_marker_1 = this->get_parameter("aruco_marker_1").as_string();
        m_aruco_marker_2 = this->get_parameter("aruco_marker_2").as_string();

        // map to determine the color and part from the Part.msg file
        color_map[mage_msgs::msg::Part::RED] = "Red";
        color_map[mage_msgs::msg::Part::GREEN] = "Green";
        color_map[mage_msgs::msg::Part::BLUE] = "Blue";
        color_map[mage_msgs::msg::Part::ORANGE] = "Orange";
        color_map[mage_msgs::msg::Part::PURPLE] = "Purple";
        part_map[mage_msgs::msg::Part::BATTERY] = "battery";
        part_map[mage_msgs::msg::Part::PUMP] = "pump";
        part_map[mage_msgs::msg::Part::SENSOR] = "sensor";
        part_map[mage_msgs::msg::Part::REGULATOR] = "regulator";

        m_msg = geometry_msgs::msg::Twist();
        
        // MultiThreading
        m_callback_group_odom = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto subscription_odom = rclcpp::SubscriptionOptions();
        subscription_odom.callback_group = m_callback_group_odom;

        m_callback_group_aruco = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto subscription_aruco = rclcpp::SubscriptionOptions();
        subscription_aruco.callback_group = m_callback_group_aruco;

        // m_callback_group_logical_camera = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        // auto subscription_logical_camera = rclcpp::SubscriptionOptions();
        // subscription_logical_camera.callback_group = m_callback_group_logical_camera;

        // timer
        m_timer = this->create_wall_timer(std::chrono::milliseconds((int)(500)), std::bind(&MazeSolver::timer_callback, this));
        m_timer_listener = this->create_wall_timer(std::chrono::milliseconds((int)(500)), std::bind(&MazeSolver::listener_callback, this));
        // m_timer_listener2 = this->create_wall_timer(std::chrono::milliseconds((int)(500)), std::bind(&MazeSolver::listener2_callback, this));

        // publisher
        m_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Subscriber
        aruco_subscriber = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_markers", 10, std::bind(&MazeSolver::aruco_callback, this, std::placeholders::_1), subscription_aruco);
        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&MazeSolver::odom_callback, this, std::placeholders::_1), subscription_odom);
        logical_camera_subscriber = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/advanced_logical_camera/image", rclcpp::SensorDataQoS(), std::bind(&MazeSolver::logical_camera_callback, this, std::placeholders::_1));

        // broadcaster
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        // tf_broadcaster2 = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Listener
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());               // aruco marker
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        // tf_buffer2 = std::make_unique<tf2_ros::Buffer>(this->get_clock());               // parts
        // tf_listener2 = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    }


    private:
        std::string m_aruco_marker_0;
        std::string m_aruco_marker_1;
        std::string m_aruco_marker_2; 
        double aruco_z_position;     // which marker to choose; Setting a random high value;
        int m_id;                   // Selecting the marker id of the nearest marker
        std::string m_turn;         // direction to turn 

        // Part
        std::map<int, std::string> color_map;
        std::map<int, std::string> part_map;
        std::string part_name_type;
        std::map<std::string, int> color_part_map;
        // std::vector<geometry_msgs::msg::TransformStamped> tf_msgs;
        geometry_msgs::msg::TransformStamped t_aruco;
        // geometry_msgs::msg::TransformStamped t_part;

        int flag{1};

        double target_angle = 0;

        geometry_msgs::msg::Twist m_msg;
        std::vector<int64_t> marker_ids;
        // double odomToAruco;
        // double odomToBasefootprint;
        double arucoToBasefootprint;

        double robot_yaw_angle;
        // double angle = 0;

        // MultiThreading
        rclcpp::CallbackGroup::SharedPtr m_callback_group_odom;
        rclcpp::CallbackGroup::SharedPtr m_callback_group_aruco;
        rclcpp::CallbackGroup::SharedPtr m_callback_group_logical_camera;

        // Subscriber
        rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_subscriber;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr logical_camera_subscriber;

        // Timer
        rclcpp::TimerBase::SharedPtr m_timer;
        rclcpp::TimerBase::SharedPtr m_timer_listener;      // aruco
        // rclcpp::TimerBase::SharedPtr m_timer_listener2;     // parts

        
        // Publisher
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher;
        
        // broadcaster
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;          // broadcasting aruco position
        // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster2;         // broadcasting parts position

        // Listener
        std::shared_ptr<tf2_ros::Buffer> tf_buffer;                             // aruco marker
        std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};

        // std::shared_ptr<tf2_ros::Buffer> tf_buffer2;                            // parts
        // std::shared_ptr<tf2_ros::TransformListener> tf_listener2{nullptr};

        void aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);
        void timer_callback();
        void listener_callback();
        // void listener2_callback();
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void logical_camera_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
};