#pragma once

#include <vector>
#include <Eigen/Core>

#include <dancers_msgs/msg/agent_struct.hpp>


namespace agent_util
{

    /**
     * @brief Role of agent. Idle means the agent is not part of the mission's data routhing path while Mission is.
     */
    enum AgentRoleType
    {
        Undefined = 0,
        Mission,
        Potential,
        Idle
    };

    /**
     * @brief Structure containing the information about the neighbors.
     */
    struct NeighborInfo_t
    {
        int id;
        float link_quality;
        agent_util::AgentRoleType role_type;
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
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

        std::vector<NeighborInfo_t> neighbors;
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
        
        if (agent_struct.agent_role == agent_struct.AGENT_ROLE_IDLE)
        {
            agent_state.role_type = AgentRoleType::Idle;
        }
        else if(agent_struct.agent_role == agent_struct.AGENT_ROLE_MISSION)
        {
            agent_state.role_type = AgentRoleType::Mission;
        }
        else if(agent_struct.agent_role == agent_struct.AGENT_ROLE_POTENTIAL)
        {
            agent_state.role_type = AgentRoleType::Potential;
        }
        else if(agent_struct.agent_role == agent_struct.AGENT_ROLE_UNDEFINED)
        {
            agent_state.role_type = AgentRoleType::Undefined;
        }
        else
        {
            // Default to Undefined if the role is not recognized 
            agent_state.role_type = AgentRoleType::Undefined;
        }

        for(auto& neighbor_msg : agent_struct.neighbor_array.neighbors)
        {
            NeighborInfo_t neighbor_info;
            neighbor_info.id = neighbor_msg.agent_id;
            neighbor_info.link_quality = neighbor_msg.link_quality;

            if (neighbor_msg.agent_role == neighbor_msg.AGENT_ROLE_IDLE)
            {
                neighbor_info.role_type = AgentRoleType::Idle;
            }
            else if(neighbor_msg.agent_role == neighbor_msg.AGENT_ROLE_MISSION)
            {
                neighbor_info.role_type = AgentRoleType::Mission;
            }
            else if(neighbor_msg.agent_role == neighbor_msg.AGENT_ROLE_POTENTIAL)
            {
                neighbor_info.role_type = AgentRoleType::Potential;
            }
            else if(neighbor_msg.agent_role == neighbor_msg.AGENT_ROLE_UNDEFINED)
            {
                neighbor_info.role_type = AgentRoleType::Undefined;
            }
            else
            {
                // Default to Undefined if the role is not recognized 
                neighbor_info.role_type = AgentRoleType::Undefined;
            }

            neighbor_info.position[0] = neighbor_msg.position.x;
            neighbor_info.position[1] = neighbor_msg.position.y;
            neighbor_info.position[2] = neighbor_msg.position.z;
            neighbor_info.velocity[0] = neighbor_msg.velocity.x;
            neighbor_info.velocity[1] = neighbor_msg.velocity.y;
            neighbor_info.velocity[2] = neighbor_msg.velocity.z;

            agent_state.neighbors.push_back(neighbor_info);
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

