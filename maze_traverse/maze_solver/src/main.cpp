#include "maze_solver.h"

int main(int argc, char * argv[]){
    //init
    rclcpp::init(argc, argv);   //initialize the communication with ROS 

    rclcpp::executors::MultiThreadedExecutor executor;
    //node
    auto node = std::make_shared<MazeSolver>("maze_solver");     // creating a node "hello" with shared pointer from Hello class
    
    executor.add_node(node);
    executor.spin();

    //shutdown
    rclcpp::shutdown();
}