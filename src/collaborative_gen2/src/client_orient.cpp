#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <iostream>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("client_b_orientation");
    
    // Publicador de Orientación
    auto pub_orient = node->create_publisher<geometry_msgs::msg::Twist>("/collaborative/orientation_cmd", 10);
    // Publicador de Gripper
    auto pub_grip = node->create_publisher<std_msgs::msg::Bool>("/collaborative/gripper_cmd", 10);
    
    RCLCPP_INFO(node->get_logger(), "=== USUARIO B (ORIENTACIÓN Y GRIPPER) ===");
    RCLCPP_INFO(node->get_logger(), "Rotación: I/K (Pitch), J/L (Roll), U/O (Yaw)");
    RCLCPP_INFO(node->get_logger(), "Gripper: 1 (Abrir), 2 (Cerrar)");

    while (rclcpp::ok()) {
        char c;
        std::cin >> c;
        geometry_msgs::msg::Twist cmd_rot;
        std_msgs::msg::Bool cmd_grip;
        bool is_rotation = true;

        // Mapeo de teclas para rotación
        if (c == 'i') cmd_rot.angular.y = 1.0;       // Pitch Arriba
        else if (c == 'k') cmd_rot.angular.y = -1.0; // Pitch Abajo
        else if (c == 'j') cmd_rot.angular.x = -1.0; // Roll Izq
        else if (c == 'l') cmd_rot.angular.x = 1.0;  // Roll Der
        else if (c == 'u') cmd_rot.angular.z = 1.0;  // Yaw Izq
        else if (c == 'o') cmd_rot.angular.z = -1.0; // Yaw Der
        
        // Mapeo para Gripper
        else if (c == '1') { is_rotation = false; cmd_grip.data = false; } // Abrir
        else if (c == '2') { is_rotation = false; cmd_grip.data = true; }  // Cerrar
        else continue;

        if (is_rotation) {
            pub_orient->publish(cmd_rot);
            RCLCPP_INFO(node->get_logger(), "Comando de ROTACIÓN enviado.");
        } else {
            pub_grip->publish(cmd_grip);
            RCLCPP_INFO(node->get_logger(), "Comando de GRIPPER enviado.");
        }
    }
    rclcpp::shutdown();
    return 0;
}