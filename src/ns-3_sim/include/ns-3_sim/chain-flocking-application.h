#ifndef CHAIN_FLOCKING_APPLICATION_H
#define CHAIN_FLOCKING_APPLICATION_H

#include "ns3/application.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/stats-module.h"

#include "pick-neighbors-header.h"

using namespace ns3;

class ChainFlocking : public Application
{
public:
    /**
     * \brief Get the type ID.
     * \return The object TypeId.
     */
    static TypeId GetTypeId();
    ChainFlocking();
    ~ChainFlocking() override;

    /**
     * \brief Returns the number of received packets
     * \return the number of received packets
     */
    uint64_t GetSent() const;

    /**
     * \brief Returns the number of received packets
     * \return the number of received packets
     */
    uint64_t GetReceived() const;

    /**
     * Set the link qualities.
     * \param linkQualities The link qualities with all known agents.
     */
    void SetLinkQualities(std::map<uint32_t, double> linkQualities);

    /**
     * Set the leader rank.
     * \param leaderRank The leader rank
     */
    void SetLeaderRank(uint32_t leaderRank);

    /**
     * Set the flow id.
     * \param flowId The flow id
     */
    void SetFlowId(uint32_t flowId);

    /**
     * Set the counter calculator for received packets.
     * \param calc The CounterCalculator.
     */
    void SetCounter(Ptr<CounterCalculator<>> calc);

    /**
     * Set the delay tracker for received packets.
     * \param delay The Delay calculator.
     */
    void SetDelayTracker(Ptr<TimeMinMaxAvgTotalCalculator> delay);

    /**
     * Get a list of the current flocking neighbors.
     * \return a list of the current flocking neighbors
     */
    std::map<uint32_t, double> GetCurrentNeighbors();

protected:
    void DoDispose() override;

private:
    void StartApplication() override;
    void StopApplication() override;

    /**
     * Send a packet.
     */
    void SendPacket();

    /**
     * Receive a packet.
     * \param socket The receiving socket.
     */
    void Receive(Ptr<Socket> socket);

    /**
     * Select the best neighbors.
     * \param maxNeighbors The maximum number of neighbors to select.
     * \return a list of the best neighbors
     */
    std::vector<uint32_t> SelectBestRelays();

    uint32_t m_pktSize;                     //!< The packet size.
    uint32_t m_port;                        //!< Application port.
    Ptr<ConstantRandomVariable> m_interval; //!< Rng for sending packets.
    uint32_t m_numRelays;                   //!< Maximum number of neighbors
    Time m_timeout;                         //!< Timeout after which an neighbor expires
    uint32_t m_leaderRank;
    uint32_t m_flowId;

    std::map<uint32_t, double> m_currentNeighborsUp;   //!< Map of neighbor ID and link quality (current neighbors)
    std::map<uint32_t, double> m_currentNeighborsDown; //!< Map of neighbor ID and link quality (current neighbors)
    std::map<uint32_t, Time> m_lastReceivedUpdate;     //!< Map of neighbor ID and last time an update was received (Potential neighbors)
    std::map<uint32_t, double> m_linkQualities;        //!< Map of neighbor ID and link quality (external to the application, have to be updated regularly)

    Ptr<Socket> m_SendSocket; //!< Sending socket.
    Ptr<Socket> m_RecvSocket; //!< Receiving socket.

    EventId m_sendEvent; //!< Send packet event.
    uint64_t m_sent;     //!< Number of packets sent.
    uint64_t m_received; //!< Number of received packets

    /// Tx TracedCallback.
    TracedCallback<Ptr<const Packet>> m_txTrace;

    /// Rx TracedCallback.
    TracedCallback<Ptr<const Packet>> m_rxTrace;

    Ptr<CounterCalculator<>> m_calc;           //!< Counter of the number of received packets.
    Ptr<TimeMinMaxAvgTotalCalculator> m_delay; //!< Delay calculator.
};

#endif // CHAIN_FLOCKING_APPLICATION_H