#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <cmath>

// Definición básica de joints para Kinova Gen2 6DOF
// (Esto es una simplificación: en un robot real usaríamos KDL/TracIK, 
//  pero para moverlo en Gazebo esto basta para "controlar" las articulaciones).

class CentralServer : public rclcpp::Node {
public:
    CentralServer() : Node("central_server_node") {
        // Publicadores a los controladores de Gazebo
        pub_arm_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/arm_controller/joint_trajectory", 10);
        pub_grip_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/gripper_controller/joint_trajectory", 10);

        // Suscripción al estado actual (para saber dónde está el robot)
        sub_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&CentralServer::state_callback, this, std::placeholders::_1));

        // Suscripciones de Clientes
        sub_pos_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/collaborative/position_cmd", 10, std::bind(&CentralServer::pos_callback, this, std::placeholders::_1));
        sub_orient_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/collaborative/orientation_cmd", 10, std::bind(&CentralServer::orient_callback, this, std::placeholders::_1));
        sub_gripper_ = this->create_subscription<std_msgs::msg::Bool>(
            "/collaborative/gripper_cmd", 10, std::bind(&CentralServer::gripper_callback, this, std::placeholders::_1));

        // Inicializar posiciones en 0
        current_joints_.resize(6, 0.0);
        
        RCLCPP_INFO(this->get_logger(), ">>> SERVIDOR DIRECTO LISTO (BYPASS MOVEIT) <<<");
    }

private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_grip_;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_states_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_pos_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_orient_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_gripper_;

    std::vector<double> current_joints_;
    bool state_received_ = false;

    // Actualiza la memoria de dónde está el robot
    void state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Mapeo simple: Buscamos los nombres que empiecen con j2n6s300_joint_1...6
        // Nota: El orden en msg->name no siempre es 1,2,3,4,5,6. Hay que buscar.
        for (size_t i = 0; i < msg->name.size(); i++) {
            if (msg->name[i] == "j2n6s300_joint_1") current_joints_[0] = msg->position[i];
            if (msg->name[i] == "j2n6s300_joint_2") current_joints_[1] = msg->position[i];
            if (msg->name[i] == "j2n6s300_joint_3") current_joints_[2] = msg->position[i];
            if (msg->name[i] == "j2n6s300_joint_4") current_joints_[3] = msg->position[i];
            if (msg->name[i] == "j2n6s300_joint_5") current_joints_[4] = msg->position[i];
            if (msg->name[i] == "j2n6s300_joint_6") current_joints_[5] = msg->position[i];
        }
        state_received_ = true;
    }

    // Cliente A: Control XYZ (Mapeado a Joints Base/Hombro/Codo)
    void pos_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (!state_received_) return;
        
        // Lógica simplificada: 
        // X (Adelante/Atras) -> Joint 2 (Hombro)
        // Y (Izquierda/Derecha) -> Joint 1 (Base)
        // Z (Arriba/Abajo) -> Joint 3 (Codo)
        
        std::vector<double> target = current_joints_;
        target[0] += msg->linear.y * 0.1; // Base gira
        target[1] += msg->linear.x * 0.1; // Hombro levanta/baja
        target[2] += msg->linear.z * 0.1; // Codo abre/cierra
        
        send_arm_command(target);
    }

    // Cliente B: Control Orientación (Mapeado a Muñeca)
    void orient_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (!state_received_) return;

        std::vector<double> target = current_joints_;
        // Mapping directo a las articulaciones de la muñeca (4, 5, 6)
        target[3] += msg->angular.y * 0.1; // Wrist 1
        target[4] += msg->angular.x * 0.1; // Wrist 2
        target[5] += msg->angular.z * 0.1; // Wrist 3

        send_arm_command(target);
    }

    void gripper_callback(const std_msgs::msg::Bool::SharedPtr msg) {
            trajectory_msgs::msg::JointTrajectory traj;
            
            // SOLO BASES
            traj.joint_names = {
                "j2n6s300_joint_finger_1", "j2n6s300_joint_finger_2", "j2n6s300_joint_finger_3"
            };
            
            trajectory_msgs::msg::JointTrajectoryPoint point;
            double val = msg->data ? 1.0 : 0.0; // 1.3 es un buen valor de cierre
            
            point.positions = {val, val, val}; 
            point.time_from_start.sec = 1;

            traj.points.push_back(point);
            pub_grip_->publish(traj);
            RCLCPP_INFO(this->get_logger(), "Comando Gripper (Bases) Enviado");
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
        point.time_from_start.nanosec = 500000000; // 0.5 segundos (respuesta rápida)

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