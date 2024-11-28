#pragma once

#include <vector>
#include <Eigen/Core>

#include <dancers_msgs/msg/agent_struct.hpp>


namespace agent_util
{

    /**
     * @brief Role of agent. Iddle means the agent is not part of the mission's data routhing path while Mission is.
     */
    enum AgentRoleType
    {
        Iddle=0,
        Mission
    };

    /**
     * @brief Structure of all states of an agent at a given time.
     */
    struct AgentState_t 
    {
        int id;
        agent_util::AgentRoleType role_type;
        
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
    };


    /**
     * @brief Returns an AgentState_t object based on an agent_struct ROS message.
     * @param agent_struct ROS message from which to create the AgentState_t.
     * @return Return a struct containing the agent states. 
     */
    inline AgentState_t create_agent_state_from_ROS_message(const dancers_msgs::msg::AgentStruct& agent_struct)
    {
        AgentState_t agent_state;
        agent_state.id = agent_struct.agent_id;
        
        if (agent_struct.agent_role == agent_struct.AGENT_ROLE_IDDLE)
        {
            agent_state.role_type = AgentRoleType::Iddle;
        }
        else if(agent_struct.agent_role == agent_struct.AGENT_ROLE_MISSION)
        {
            agent_state.role_type = AgentRoleType::Mission;
        }

        agent_state.position[0] = agent_struct.state.position.x;
        agent_state.position[1] = agent_struct.state.position.y;
        agent_state.position[2] = agent_struct.state.position.z;

        agent_state.velocity[0] = agent_struct.state.velocity_heading.velocity.x;
        agent_state.velocity[1] = agent_struct.state.velocity_heading.velocity.y;
        agent_state.velocity[2] = agent_struct.state.velocity_heading.velocity.z;
        
        return agent_state; 
    }
}

