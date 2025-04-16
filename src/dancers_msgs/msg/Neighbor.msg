# This message assumes that the position and the role of the neighbor can be found purely by knowing the agent_id. 
uint8 AGENT_ROLE_UNDEFINED=0
uint8 AGENT_ROLE_MISSION=1
uint8 AGENT_ROLE_POTENTIAL=2
uint8 AGENT_ROLE_IDLE=3

uint32 agent_id
float64 link_quality
uint8 agent_role
geometry_msgs/Vector3 position
geometry_msgs/Vector3 velocity