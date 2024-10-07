#ifndef PICK_NEIGHBORS_HEADER_H
#define PICK_NEIGHBORS_HEADER_H


#include "ns3/header.h"
#include "ns3/ipv4-address.h"

namespace ns3
{

class PickNeighborsHeader : public Header
{
    public:
        /**
         * \brief Get the type ID.
         * \return the object TypeId
         */
        static TypeId GetTypeId();
        TypeId GetInstanceTypeId() const override;
        void Print(std::ostream& os) const override;
        uint32_t GetSerializedSize() const override;
        void Serialize(Buffer::Iterator start) const override;
        uint32_t Deserialize(Buffer::Iterator start) override;

        void SetLeaderRank(uint32_t leader_rank);
        void SetBestNeighbors(std::vector<uint32_t> best_neighbors);

        std::vector<uint32_t> GetBestNeighbors() const;
        uint32_t GetLeaderRank() const;

    private:
        uint32_t m_leader_rank;
        std::vector<uint32_t> m_best_neighbors;
};

}

#endif // PICK_NEIGHBORS_HEADER_H