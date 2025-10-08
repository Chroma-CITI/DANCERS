#include <connector_core/connector.hpp>

class EmptyConnector : public Connector
{
public:
    EmptyConnector()
    : Connector("empty_connector_phy")
    {
        RCLCPP_INFO(this->get_logger(), "EmptyConnector (PHY) started");

        this->ReadConfigFile();

        // Start Loop() in a separate thread so constructor can finish
        loop_thread_ = std::thread(&EmptyConnector::Loop, this);
    }

private:
    int64_t sleep_time_;

    /**
     * @brief Read config file and set variables
     * 
     * It is mandatory to set the socket variables to enable the connector
     * to communicate with the simulator !
     */
    void ReadConfigFile() override
    {
        this->use_uds = getYamlValue<bool>(this->config_, "phy_use_uds");
        this->uds_server_address = getYamlValue<std::string>(this->config_, "phy_uds_server_address");
        this->ip_server_address = getYamlValue<std::string>(this->config_, "phy_ip_server_address");
        this->ip_server_port = getYamlValue<unsigned int>(this->config_, "phy_ip_server_port");

        // Read the synchronization interval and the step length as ints to easily verify that they are compatible
        unsigned int sync_window_int = config_["sync_window"].as<unsigned int>();
        unsigned int step_size_int = config_["phy_step_size"].as<unsigned int>();
        if (sync_window_int % step_size_int != 0)
        {
            RCLCPP_FATAL(this->get_logger(), "Sync window must be a multiple of the physics step size, aborting.");
            exit(EXIT_FAILURE);
        }
        this->step_size = step_size_int / 1000000.0f;
        this->it_end_sim = uint64_t(this->simulation_length / this->step_size);

        this->sleep_time_ = getYamlValue<int64_t>(this->config_, "phy_sleep_time");

    }

    /**
     * @brief Function called at each step, the connector is responsible to read the update message, step the simulator, 
     * and return the repsonse update message
     */
    dancers_update_proto::DancersUpdate StepSimulation(dancers_update_proto::DancersUpdate) override
    {
        // Sleep 1 sec
        std::this_thread::sleep_for(std::chrono::microseconds(this->sleep_time_));

        RCLCPP_INFO(this->get_logger(), "Finished iteration %ld (time = %f s)", this->it, this->it*this->step_size);

        return this->GenerateResponseProtobuf();
    }

    /**
     * @brief Generate the response protobuf message to send to the coordinator
     */
    dancers_update_proto::DancersUpdate GenerateResponseProtobuf()
    {
        dancers_update_proto::DancersUpdate response_msg;
        response_msg.set_msg_type(dancers_update_proto::DancersUpdate::END);
        return response_msg;
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EmptyConnector>());
    rclcpp::shutdown();
    return 0;
}