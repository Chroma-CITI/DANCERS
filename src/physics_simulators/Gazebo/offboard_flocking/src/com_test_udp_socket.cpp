#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#define PORT 8080

// Global number of nodes
int N = 4;

void server_thread_func(int sockfd){
    struct sockaddr_in cliaddr;
    socklen_t len;
    int n;

    memset(&cliaddr, 0, sizeof(cliaddr));

    len = sizeof(cliaddr);  //len is value/result
    turtlesim::msg::Pose rcv_pose;

    while (1)
    {
        n = recvfrom(sockfd, &rcv_pose, sizeof(rcv_pose), 
                    MSG_WAITALL, ( struct sockaddr *) &cliaddr,
                    &len);
        std::string rcv_posestr = "x: "+std::to_string(rcv_pose.x)+" y: "+std::to_string(rcv_pose.y)+" theta: "+std::to_string(rcv_pose.theta);        
        printf("Server received : %s\n", rcv_posestr.c_str());
    }
}


// ROS2 Node declaration
class ComTest : public rclcpp::Node {
    public:
        // Constructor
        ComTest() : rclcpp::Node("ComTest"){

            // Declare one parameters for this ros2 node
            auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
            param_desc.description = "ID of the robot. This is the system id of each robot (instance number + 1)";
            this->declare_parameter("robot_id", 1, param_desc);

            robot_id = this->get_parameter("robot_id").get_parameter_value().get<uint8_t>();
            node_namespace = this->get_namespace();

            this->my_ip_addr = "192.168.0." + std::to_string(robot_id);
            std::string peer_ip_addr;
            if(robot_id == 1){
                peer_ip_addr = "192.168.0.2";
            } else {
                peer_ip_addr = "192.168.0.1";
            }


            // Prepare the QoS that we will assign to the publishers / subscribers  
            auto qos_sub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
	        auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local();

            this->my_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
                                        node_namespace+"/turtle1/pose", 
                                        qos_sub,
                                        [this](const turtlesim::msg::Pose &pose){this->pose_clbk(pose);}
            );
            std::cout << "Subscribed to " << node_namespace << "/turtle1/pose" << std::endl;

            // ------------------ Sockets ------------------

            client_sock_fd = create_udp_socket(my_ip_addr, 8090);
            server_sock_fd = create_udp_socket(my_ip_addr, 8091);

            // start server thread
            server_thread = std::thread(server_thread_func, server_sock_fd);

            // ------------------ End Sockets ------------------


        }
    private:
        int client_sock_fd;
        int server_sock_fd;
        int create_udp_socket(std::string ip_address, uint16_t port);
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr my_pose_sub_;
        int robot_id;
        std::string node_namespace;
        std::string my_ip_addr;
        std::thread server_thread;
        // std::vector< std::shared_ptr< rclcpp::Subscription<turtlesim::msg::Pose> > > robots_poses_subs_;
        void pose_clbk(const turtlesim::msg::Pose & pose);
};

int ComTest::create_udp_socket(std::string ip_address, uint16_t port){
    int sockfd;
    struct sockaddr_in address;
    
    // Creating socket file descriptor
	if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
		perror("socket creation failed");
		return -1;
	}

    memset(&address, 0, sizeof(address));

    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    address.sin_addr.s_addr = inet_addr(ip_address.c_str());

        // Bind the socket with the address
    if ( bind(sockfd, (const struct sockaddr *)&address, 
            sizeof(address)) < 0 )
    {
        perror("bind failed");
        return -1;
    }

    return sockfd;
}

// Callback called at each Pose message from turtle
void ComTest::pose_clbk(const turtlesim::msg::Pose & pose) {
    // Print the pose
    struct sockaddr_in servaddr;

    memset(&servaddr, 0, sizeof(servaddr));

	// Filling server information
	servaddr.sin_family = AF_INET;
	servaddr.sin_port = htons(8091);
	servaddr.sin_addr.s_addr = inet_addr("192.168.0.2");

    std::string posestr = "x: "+std::to_string(pose.x)+" y: "+std::to_string(pose.y)+" theta: "+std::to_string(pose.theta);

    // RCLCPP_INFO(this->get_logger(), "Sending pose to server:\n%s", posestr.c_str());

    sendto(this->client_sock_fd, &pose, sizeof(pose), 0, (const struct sockaddr *)&servaddr, sizeof(servaddr));
}



// main function that spins a ROS2 node
int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ComTest>());
    rclcpp::shutdown();
    return 0;
}