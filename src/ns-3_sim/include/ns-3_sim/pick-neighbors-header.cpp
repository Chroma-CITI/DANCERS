#include "pick-neighbors-header.h"

namespace ns3
{

TypeId
PickNeighborsHeader::GetTypeId()
{
    static TypeId tid = TypeId("PickNeighborsHeader")
                            .SetParent<Header>()
                            .SetGroupName("Internet")
                            .AddConstructor<PickNeighborsHeader>();
    return tid;
}

TypeId
PickNeighborsHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

void
PickNeighborsHeader::Print(std::ostream& os) const
{
    os << "\nlength: " << GetSerializedSize() << "\n Best neighbors: ";
    for (uint32_t n : m_best_neighbors)
    {
        os << n << " ";
    }
    os << "\n LR: " << m_leader_rank << std::endl;
}

uint32_t
PickNeighborsHeader::GetSerializedSize() const
{
    return m_best_neighbors.size()*4 + 4*2;
}

void
PickNeighborsHeader::Serialize(Buffer::Iterator start) const
{
    Buffer::Iterator i = start;

    i.WriteHtonU32(1);
    for (uint32_t n : m_best_neighbors)
    {
        i.WriteHtonU32(n);
    }
    i.WriteHtonU32(m_leader_rank);
}

uint32_t
PickNeighborsHeader::Deserialize(Buffer::Iterator start)
{
    Buffer::Iterator i = start;
    uint32_t num_best_neighbors = i.ReadNtohU32();
    for (uint32_t n = 0 ; n < num_best_neighbors ; n++)
    {
        m_best_neighbors.push_back(i.ReadNtohU32());
    }
    m_leader_rank = i.ReadNtohU32();

    return GetSerializedSize();
}

void 
PickNeighborsHeader::SetLeaderRank(uint32_t leader_rank)
{
    m_leader_rank = leader_rank;
}
void 
PickNeighborsHeader::SetBestNeighbors(std::vector<uint32_t> best_neighbors)
{
    m_best_neighbors = best_neighbors;
}

std::vector<uint32_t> 
PickNeighborsHeader::GetBestNeighbors() const
{
    return m_best_neighbors;
}

uint32_t
PickNeighborsHeader::GetLeaderRank() const
{
    return m_leader_rank;
}


}