#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("client_a_position");
    auto pub = node->create_publisher<geometry_msgs::msg::Twist>("/collaborative/position_cmd", 10);
    
    RCLCPP_INFO(node->get_logger(), "=== USUARIO A (POSICIÓN) ===");
    RCLCPP_INFO(node->get_logger(), "Teclas: W/S (Adelante/Atras), A/D (Izq/Der), Q/E (Arriba/Abajo)");

    while (rclcpp::ok()) {
        char c;
        std::cin >> c;
        geometry_msgs::msg::Twist cmd; // Todo a 0 por defecto

        if (c == 'w') cmd.linear.x = 1.0;      // +X
        else if (c == 's') cmd.linear.x = -1.0;// -X
        else if (c == 'a') cmd.linear.y = 1.0; // +Y
        else if (c == 'd') cmd.linear.y = -1.0;// -Y
        else if (c == 'q') cmd.linear.z = 1.0; // +Z
        else if (c == 'e') cmd.linear.z = -1.0;// -Z
        else continue;

        pub->publish(cmd);
        RCLCPP_INFO(node->get_logger(), "Comando de POSICIÓN enviado.");
    }
    rclcpp::shutdown();
    return 0;
}