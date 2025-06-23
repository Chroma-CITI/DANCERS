#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <dancers_msgs/msg/agent_struct.hpp>
#include <dancers_msgs/msg/target.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <memory>

using namespace std::chrono_literals;

// A simple struct to define an event
struct ScheduledEvent
{
    rclcpp::Time target_sim_time;
    std::function<void()> callback; // The action to perform
    bool executed;

    // Custom comparison for sorting
    bool operator<(const ScheduledEvent &other) const
    {
        return target_sim_time < other.target_sim_time;
    }
};

class ScenarioManager : public rclcpp::Node
{
public:
    ScenarioManager() : Node("scenario_manager")
    {
        rclcpp::QoS reliable_qos(10); // depth of 10
        reliable_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        reliable_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        reliable_qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

        std::string events_namespace = "/events/";
        // publishers
        this->pub_spawn_uav_ = this->create_publisher<dancers_msgs::msg::AgentStruct>(events_namespace + "spawn_uav", reliable_qos);
        this->pub_disconnect_uav_ = this->create_publisher<std_msgs::msg::UInt32>(events_namespace + "disconnect_uav", reliable_qos);
        this->pub_uav_failure_ = this->create_publisher<std_msgs::msg::UInt32>(events_namespace + "uav_failure", reliable_qos);
        this->pub_update_target_ = this->create_publisher<dancers_msgs::msg::Target>(events_namespace + "update_target", reliable_qos);
        this->pub_delete_target_ = this->create_publisher<dancers_msgs::msg::Target>(events_namespace + "delete_target", reliable_qos);

        // Subscribe to the /clock topic
        clock_subscription_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock",
            rclcpp::QoS(1).best_effort(), // /clock can be best effort, but if you need every tick, consider reliable
            std::bind(&ScenarioManager::clock_callback, this, std::placeholders::_1));

        // SPAWN agent 11 at 10 seconds
        // add_scheduled_event(
        //     rclcpp::Time(10, 0, this->get_clock()->get_clock_type()), // Use the node's clock type
        //     [this]()
        //     {
        //         // Publish the new agent struct
        //         auto agent = dancers_msgs::msg::AgentStruct();
        //         agent.agent_id = 11;
        //         agent.agent_role = dancers_msgs::msg::AgentStruct::AGENT_ROLE_IDLE;
        //         agent.heartbeat_received = 0;
        //         agent.heartbeat_sent = 0;
        //         agent.state.position.x = 0.0;
        //         agent.state.position.y = 0.0;
        //         agent.state.position.z = 1.0;
        //         agent.state.velocity_heading.velocity.x = 0.0;
        //         agent.state.velocity_heading.velocity.y = 0.0;
        //         agent.state.velocity_heading.velocity.z = 0.0;
        //         agent.state.velocity_heading.heading = 0.0;

        //         RCLCPP_INFO(this->get_logger(), "Event: Spawn agent %d at (%f, %f, %f)", agent.agent_id, agent.state.position.x, agent.state.position.y, agent.state.position.z);
        //         pub_spawn_uav_->publish(agent);
        //     });

        // FAIL agent 2 at 20 seconds
        // add_scheduled_event(
        //     rclcpp::Time(20, 0, this->get_clock()->get_clock_type()), // Use the node's clock type
        //     [this]()
        //     {
        //         auto failed_uav_id = std_msgs::msg::UInt32();
        //         failed_uav_id.data = 2;
        //         RCLCPP_INFO(this->get_logger(), "Event: Agent %d failed!", failed_uav_id.data);
        //         pub_uav_failure_->publish(failed_uav_id);
        //     });

        // MOVE target 1
        add_recursive_scheduled_event(
            rclcpp::Time(1, 0, this->get_clock()->get_clock_type()),
            rclcpp::Time(100, 0, this->get_clock()->get_clock_type()),
            rclcpp::Duration(0.1, 0),
            [this](rclcpp::Time start_time, rclcpp::Time /*end_time*/, rclcpp::Duration /*interval*/)
            {
                double C_x = 100.0;
                double C_y = 200.0;
                double R = 50.0;
                double omega = 0.1;

                auto target = dancers_msgs::msg::Target();
                target.target_id = 1;
                target.is_sink = false;
                target.position.x = C_x + R * cos(omega * this->current_sim_time_.seconds());
                target.position.y = C_y + R * sin(omega * this->current_sim_time_.seconds());
                target.position.z = 10.0;
                target.concerned_agents.push_back(-1);
                RCLCPP_INFO(this->get_logger(), "[%f] Event: Move target %d to (%f, %f, %f)", this->current_sim_time_.seconds(), target.target_id, target.position.x, target.position.y, target.position.z);
                pub_update_target_->publish(target);
            });

        // Sort events by time to process efficiently
        std::sort(scheduled_events_.begin(), scheduled_events_.end());

        RCLCPP_INFO(this->get_logger(), "ScenarioManager initialized");
    }

private:
    // subscribers
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_;

    // publishers
    rclcpp::Publisher<dancers_msgs::msg::AgentStruct>::SharedPtr pub_spawn_uav_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub_disconnect_uav_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub_uav_failure_;
    rclcpp::Publisher<dancers_msgs::msg::Target>::SharedPtr pub_update_target_;
    rclcpp::Publisher<dancers_msgs::msg::Target>::SharedPtr pub_delete_target_;

    rclcpp::Time current_sim_time_;
    std::vector<std::shared_ptr<ScheduledEvent>> scheduled_events_;

    void add_scheduled_event(rclcpp::Time time, std::function<void()> callback)
    {
        scheduled_events_.push_back(std::make_shared<ScheduledEvent>(ScheduledEvent{time, callback, false}));
    }

    void add_recursive_scheduled_event(
        rclcpp::Time start_time,
        rclcpp::Time end_time,
        rclcpp::Duration interval,
        std::function<void(rclcpp::Time, rclcpp::Time, rclcpp::Duration)> callback)
    {
        auto event_ptr = std::make_shared<ScheduledEvent>(ScheduledEvent{
            start_time,
            [this, start_time, end_time, interval, callback]() mutable
            {
                callback(start_time, end_time, interval);
                rclcpp::Time next_time = start_time + interval;
                // Schedule the next event if within end_time
                if (next_time <= end_time)
                {
                    add_recursive_scheduled_event(next_time, end_time, interval, callback);
                }
            },
            false});
        scheduled_events_.push_back(event_ptr);
        std::sort(scheduled_events_.begin(), scheduled_events_.end());
    }

    void clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
    {
        current_sim_time_ = msg->clock;

        // Step 1: Collect due events
        std::vector<std::shared_ptr<ScheduledEvent>> to_execute;
        for (auto &event_ptr : scheduled_events_)
        {
            if (!event_ptr->executed && current_sim_time_ >= event_ptr->target_sim_time)
            {
                to_execute.push_back(event_ptr);
            }
        }

        // Step 2: Execute callbacks (may add new events)
        for (auto &event_ptr : to_execute)
        {
            RCLCPP_DEBUG(this->get_logger(), "Executing event at sim time %f (Target: %f)",
                        current_sim_time_.seconds(), event_ptr->target_sim_time.seconds());
            event_ptr->callback();
            event_ptr->executed = true;
        }

        // Step 3: Remove executed events
        scheduled_events_.erase(
            std::remove_if(scheduled_events_.begin(), scheduled_events_.end(),
                        [](const std::shared_ptr<ScheduledEvent> &e)
                        { return e->executed; }),
            scheduled_events_.end());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScenarioManager>());
    rclcpp::shutdown();
    return 0;
}
