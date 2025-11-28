#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <cmath>
#include <algorithm> // Para clamp

class CentralServer : public rclcpp::Node {
public:
    CentralServer() : Node("central_server_node") {
        // Publicadores
        pub_arm_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/arm_controller/joint_trajectory", 10);
        pub_grip_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/gripper_controller/joint_trajectory", 10);

        // Suscripción al estado real
        sub_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&CentralServer::state_callback, this, std::placeholders::_1));

        // Clientes
        sub_pos_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/collaborative/position_cmd", 10, std::bind(&CentralServer::pos_callback, this, std::placeholders::_1));
        
        sub_orient_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/collaborative/orientation_cmd", 10, std::bind(&CentralServer::orient_callback, this, std::placeholders::_1));
        
        // AHORA EL GRIPPER USA TWIST (Continuo)
        sub_gripper_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/collaborative/gripper_continuous_cmd", 10, std::bind(&CentralServer::gripper_callback, this, std::placeholders::_1));

        current_joints_.resize(6, 0.0);
        current_gripper_val_ = 0.0; // Empezamos asumiendo 0 (abierto)
        
        RCLCPP_INFO(this->get_logger(), ">>> SERVIDOR CONTINUO LISTO <<<");
    }

private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_grip_;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_states_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_pos_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_orient_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_gripper_;

    std::vector<double> current_joints_;
    double current_gripper_val_;
    bool state_received_ = false;

    void state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); i++) {
            if (msg->name[i] == "j2n6s300_joint_1") current_joints_[0] = msg->position[i];
            if (msg->name[i] == "j2n6s300_joint_2") current_joints_[1] = msg->position[i];
            if (msg->name[i] == "j2n6s300_joint_3") current_joints_[2] = msg->position[i];
            if (msg->name[i] == "j2n6s300_joint_4") current_joints_[3] = msg->position[i];
            if (msg->name[i] == "j2n6s300_joint_5") current_joints_[4] = msg->position[i];
            if (msg->name[i] == "j2n6s300_joint_6") current_joints_[5] = msg->position[i];
            
            // Sincronizamos la memoria del gripper con la realidad
            if (msg->name[i] == "j2n6s300_joint_finger_1") current_gripper_val_ = msg->position[i];
        }
        state_received_ = true;
    }

    void pos_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (!state_received_) return;
        std::vector<double> target = current_joints_;
        target[0] += msg->linear.y * 0.05; // Base
        target[1] += msg->linear.x * 0.05; // Hombro
        target[2] += msg->linear.z * 0.05; // Codo
        send_arm_command(target);
    }

    void orient_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (!state_received_) return;
        std::vector<double> target = current_joints_;
        target[3] += msg->angular.y * 0.1; 
        target[4] += msg->angular.x * 0.1; 
        target[5] += msg->angular.z * 0.1; 
        send_arm_command(target);
    }

    // --- GRIPPER CONTINUO ---
    void gripper_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Usamos linear.x como señal de abrir/cerrar
        // +1.0 = Cerrar, -1.0 = Abrir
        double step = msg->linear.x * 0.05; // Velocidad de cierre
        current_gripper_val_ += step;

        // Limites de seguridad (0.0 a 1.2)
        current_gripper_val_ = std::max(0.0, std::min(1.2, current_gripper_val_));

        send_gripper_command(current_gripper_val_);
    }

    // --- ESTA ES LA VERSIÓN CORREGIDA ---
    void send_gripper_command(double val) {
        trajectory_msgs::msg::JointTrajectory traj;
        
        // CORRECCIÓN: Enviamos SOLO las 3 bases. 
        // Las puntas (tips) se moverán por física (resortes), no por comando.
        traj.joint_names = {
            "j2n6s300_joint_finger_1", 
            "j2n6s300_joint_finger_2", 
            "j2n6s300_joint_finger_3"
        };
        
        trajectory_msgs::msg::JointTrajectoryPoint point;
        
        // CORRECCIÓN: Solo enviamos 3 posiciones.
        // val es el valor calculado en gripper_callback (entre 0.0 y 1.2)
        point.positions = {val, val, val}; 
        
        point.time_from_start.sec = 0;
        point.time_from_start.nanosec = 100000000; // 0.1s Respuesta rápida

        traj.points.push_back(point);
        pub_grip_->publish(traj);
        
        // Log para depurar (opcional)
        RCLCPP_INFO(this->get_logger(), "Gripper Cmd: %f", val);
    }

    void send_arm_command(const std::vector<double>& joints) {
        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = {
            "j2n6s300_joint_1", "j2n6s300_joint_2", "j2n6s300_joint_3",
            "j2n6s300_joint_4", "j2n6s300_joint_5", "j2n6s300_joint_6"
        };
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = joints;
        point.time_from_start.sec = 0; 
        point.time_from_start.nanosec = 200000000; // 0.2s

        traj.points.push_back(point);
        pub_arm_->publish(traj);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CentralServer>());
    rclcpp::shutdown();
    return 0;
}