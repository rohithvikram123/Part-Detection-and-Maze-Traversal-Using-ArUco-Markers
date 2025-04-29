#include <rclcpp/rclcpp.hpp>
#include "maze_solver.h"

/**
 * @fn timer_callback
 * @brief makes the turtlebot move in the forward direction 
 * at all time until it reaches the marker 2
 */
void MazeSolver::timer_callback(){
    if(flag == 1){
        // RCLCPP_INFO(this->get_logger(), "timer_callback");
        m_msg.linear.x = 0.1;
        m_publisher->publish(m_msg);
    }
}
/**
 * @fn aruco_callback
 * @brief 
 * 1. chooses the marker which is close to the robot
 * 2. broadcasts the transform between the camera and the marker
 * 3. decides which side to turn
 */
void MazeSolver::aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg){
    if (msg!=nullptr){

        marker_ids = msg->marker_ids;

        for (std::size_t i{0}; i < marker_ids.size(); i++){
            aruco_z_position = 999;
            if (msg->poses[i].position.z < aruco_z_position){
                aruco_z_position = msg->poses[i].position.z;
                m_id = i;

            }
        }

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = msg->header.stamp;
        t.header.frame_id = "camera_rgb_optical_frame";
        t.child_frame_id = "aruco_marker";

        t.transform.translation.x = msg->poses[m_id].position.x;
        t.transform.translation.y = msg->poses[m_id].position.y;
        t.transform.translation.z = msg->poses[m_id].position.z;

        t.transform.rotation.x = msg->poses[m_id].orientation.x;
        t.transform.rotation.y = msg->poses[m_id].orientation.y;
        t.transform.rotation.z = msg->poses[m_id].orientation.z;
        t.transform.rotation.w = msg->poses[m_id].orientation.w;
        
        t_aruco = t;

        tf_broadcaster->sendTransform(t_aruco);

        // RCLCPP_INFO_STREAM(this->get_logger(), "m_id: " << m_id);

        switch(marker_ids[m_id]){
            case 0:
                m_turn = m_aruco_marker_0;
                break;
            
            case 1:
                m_turn = m_aruco_marker_1;
                break;

            case 2:
                m_turn = m_aruco_marker_2;
                break;
        }
        // RCLCPP_INFO_STREAM(this->get_logger(), "m_turn: " << m_turn);
    }
}

/**
 * @fn listener_callback
 * @brief 
 * 1. listens transform between base_footprint and aruco_marker
 * 2. turns the robot or stops it according to the marker id
 */
void MazeSolver::listener_callback(){  
    
    try {
        
        geometry_msgs::msg::TransformStamped transformStamped = tf_buffer->lookupTransform("base_footprint", "aruco_marker", tf2::TimePointZero);
        arucoToBasefootprint = transformStamped.transform.translation.x - 0.49;                 // reduced by 0.49 to consider the camera range
        // RCLCPP_INFO_STREAM(this->get_logger(), "Distance between Aruco and Robot: " << arucoToBasefootprint);
        rclcpp::Rate rate(10);

        if (arucoToBasefootprint <= 0.4 && m_turn == "right_90") {
            
            target_angle -= 1.48;               // turn 1.48 radians right
            // RCLCPP_INFO_STREAM(this->get_logger(), "target angle: " << target_angle);
            
            while(robot_yaw_angle > target_angle){
                m_msg.linear.x = 0;
                m_msg.angular.z = -0.5;
                m_publisher->publish(m_msg);
                rate.sleep();
            }
            m_msg.angular.z = 0;
            m_publisher->publish(m_msg); 
            rate.sleep();
        }

        else if(arucoToBasefootprint <= 0.4 && m_turn == "left_90"){
           
            target_angle += 1.42;                   // turn 1.42 radians left
            while(robot_yaw_angle < target_angle){
                m_msg.linear.x = 0;
                m_msg.angular.z = 0.5;
                m_publisher->publish(m_msg);
                rate.sleep();
            }

            m_msg.angular.z = 0;
            m_publisher->publish(m_msg);
            rate.sleep();
            
        }

        else if(arucoToBasefootprint <= 0.4 && m_turn == "end"){
            flag = 0;
            m_msg.linear.x = 0;
            m_publisher->publish(m_msg);

        }

    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    }

}

/**
 * @fn odom_callback
 * @brief to determine the current angle of the robot
 */
void MazeSolver::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
    // odomToBasefootprint = msg->pose.pose.position.x;

    tf2::Quaternion quat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    
    double roll, pitch;
    
    tf2::Matrix3x3(quat).getRPY(roll, pitch, robot_yaw_angle);

    // RCLCPP_INFO_STREAM(this->get_logger(), "robot angle: " << robot_yaw_angle);
}

/**
 * @fn logical_camera_callback
 * @brief 
 * 1. broadcast transform between camera and the part
 * 2. listener to get the transform between camera and part
 * 3. print the part's color, name, position and orientation  
 */
void MazeSolver::logical_camera_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){
    if (!msg->part_poses.empty()){
        
        part_name_type =  color_map[msg->part_poses[0].part.color] + " " +part_map[msg->part_poses[0].part.type];
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = t_aruco.header.stamp;          // using the aruco msgs header stamp, since no header stamp for logical camera msg
        t.header.frame_id = "logical_camera_link";
        t.child_frame_id = part_name_type;

        t.transform.translation.x = msg->part_poses[0].pose.position.x;
        t.transform.translation.y = msg->part_poses[0].pose.position.y;
        t.transform.translation.z = msg->part_poses[0].pose.position.z;

        t.transform.rotation.x = msg->part_poses[0].pose.orientation.x;
        t.transform.rotation.y = msg->part_poses[0].pose.orientation.y;
        t.transform.rotation.z = msg->part_poses[0].pose.orientation.z;
        t.transform.rotation.w = msg->part_poses[0].pose.orientation.w;
        
        // have_part = true;
        tf_broadcaster->sendTransform(t);
        rclcpp::Rate rate(10);
        rate.sleep();
        if (!color_part_map.count(part_name_type)){
            try {
                geometry_msgs::msg::TransformStamped transformStamped2 = tf_buffer->lookupTransform("odom", part_name_type, tf2::TimePointZero);

                
                    double pose_x, pose_y, pose_z;
                    pose_x = transformStamped2.transform.translation.x;
                    pose_y = transformStamped2.transform.translation.y;
                    pose_z = transformStamped2.transform.translation.z;
                    tf2::Quaternion quat(transformStamped2.transform.rotation.x, transformStamped2.transform.rotation.y,transformStamped2.transform.rotation.z, transformStamped2.transform.rotation.w);
                    double roll, pitch, yaw;
                    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
                    RCLCPP_INFO_STREAM(this->get_logger(), part_name_type << " detected at xyz=[" << pose_x << ", " << pose_y << ", " << pose_z << "] rpy=[" << roll << ", " << pitch << ", " << yaw << "]");
                    color_part_map[part_name_type] = 1;
                
            
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
            }
        }
    }
}

