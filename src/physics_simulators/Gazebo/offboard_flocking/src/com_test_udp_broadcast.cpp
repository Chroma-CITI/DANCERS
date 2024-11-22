#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#define PORT 8090

void server_thread_func(int sockfd, int id){
    struct sockaddr_in cliaddr;
    socklen_t len;
    int n;

    memset(&cliaddr, 0, sizeof(cliaddr));

    len = sizeof(cliaddr);  //len is value/result
    nav_msgs::msg::Odometry rcv_odom;

    while (1)
    {
        n = recvfrom(sockfd, &rcv_odom, sizeof(rcv_odom), 
                    MSG_WAITALL, ( struct sockaddr *) &cliaddr,
                    &len);
        if(inet_ntoa(cliaddr.sin_addr) != "10.0.0."+std::to_string(id)){
            printf("Received odom from %s : %lf; %lf; %lf\n", inet_ntoa(cliaddr.sin_addr), rcv_odom.pose.pose.position.x, rcv_odom.pose.pose.position.y, rcv_odom.pose.pose.position.z);
        }
    }
}

int create_udp_rcv_socket(uint16_t port){
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
    address.sin_addr.s_addr = INADDR_ANY;

        // Bind the socket with the address
    if ( bind(sockfd, (const struct sockaddr *)&address, 
            sizeof(address)) < 0 )
    {
        perror("bind failed");
        return -1;
    }

    return sockfd;
}

static int open_broadcast_socket(){
   int sock;                        
   int broadcastPermission;         

   if ((sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0){
        fprintf(stderr, "socket error");
        exit(1);
   }

   broadcastPermission = 1;
   if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (void *) &broadcastPermission,sizeof(broadcastPermission)) < 0){
       fprintf(stderr, "setsockopt error");
       exit(1);
   }
   return sock;

}



static void boardcast_msg(const nav_msgs::msg::Odometry &odom){
   int sock;                        
   struct sockaddr_in broadcastAddr; 
   const char *broadcastIP;                
   unsigned short broadcastPort;     
//    const char *sendString;                 
   int broadcastPermission;         
   int sendOdomLen;                

   broadcastIP = "10.0.0.255";  
   broadcastPort = PORT;

//    sendString = mess;             /*  string to broadcast */

   if ((sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0){
        fprintf(stderr, "socket error");
        exit(1);
   }

   broadcastPermission = 1;
   if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (void *) &broadcastPermission,sizeof(broadcastPermission)) < 0){
       fprintf(stderr, "setsockopt error");
       exit(1);
   }

   /* Construct local address structure */
   memset(&broadcastAddr, 0, sizeof(broadcastAddr));   
   broadcastAddr.sin_family = AF_INET;                 
   broadcastAddr.sin_addr.s_addr = inet_addr(broadcastIP);
   broadcastAddr.sin_port = htons(broadcastPort);       

   sendOdomLen = sizeof(odom);  
   printf("Sending odom : %lf; %lf; %lf\n", odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);

    /* Broadcast odom in datagram to clients */
    if (sendto(sock, &odom, sendOdomLen, 0, (struct sockaddr *)&broadcastAddr, sizeof(broadcastAddr)) != sendOdomLen){
        fprintf(stderr, "sendto error");
        exit(1);
    }

    close(sock);
}

// main function that spins a ROS2 node
int main(int argc, char * argv[]){
    // rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<ComTest>());
    // rclcpp::shutdown();

    int id = strtol(argv[1], NULL, 10);

    int server_sock_fd = create_udp_rcv_socket(PORT);
    // start server thread
    std::thread server_thread = std::thread(server_thread_func, server_sock_fd, id);

    double x = 0.0;
    while(1){
        nav_msgs::msg::Odometry odom;
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = 0.0;
        odom.pose.pose.position.z = 3.0;
        boardcast_msg(odom);
        x += 0.1;
        sleep(1);
    }
    
    // server_thread.join();
    return 0;
}