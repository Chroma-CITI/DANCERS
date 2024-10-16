#ifndef FLOCKING_APPLICATION_H
#define FLOCKING_APPLICATION_H

#include "ns3/application.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/stats-module.h"

#include "pick-neighbors-header.h"

using namespace ns3;

struct AlgoNeighbors
{
  bool is_leader;
  bool has_target;
  uint16_t leader_rank_up;
  uint16_t leader_rank_down;
  std::vector<int16_t> neighbors_up;
  std::vector<int16_t> neighbors_down;
  std::map<uint16_t, Time> neighbors;
  std::map<uint16_t, Time> potential_neighbors;
  int k=1;
  void print()
  {
    std::cout << "is_leader: " << is_leader << std::endl;
    std::cout << "has_target: " << has_target << std::endl;
    std::cout << "leader_rank_up: " << leader_rank_up << std::endl;
    std::cout << "leader_rank_down: " << leader_rank_down << std::endl;
    std::cout << "neighbors_up: ";
    for (auto neighbor : neighbors_up)
    {
      std::cout << neighbor << " ";
    }
    std::cout << std::endl;
    std::cout << "neighbors_down: ";
    for (auto neighbor : neighbors_down)
    {
      std::cout << neighbor << " ";
    }
    std::cout << std::endl;
    std::cout << "neighbors: ";
    for (auto neighbor : neighbors)
    {
      std::cout << neighbor << " ";
    }
    std::cout << std::endl;
    std::cout << "potential_neighbors: ";
    for (auto neighbor : potential_neighbors)
    {
      std::cout << neighbor << " ";
    }
    std::cout << std::endl;
    std::cout << "k: " << k << std::endl;
  }
};

/**
 * FlockingBroadcaster application.
 */
class FlockingBroadcaster : public Application
{
  public:
    /**
     * \brief Get the type ID.
     * \return The object TypeId.
     */
    static TypeId GetTypeId();
    FlockingBroadcaster();
    ~FlockingBroadcaster() override;

    /**
     * \brief Returns the number of received packets
     * \return the number of received packets
     */
    uint64_t GetSent() const;


  protected:
    void DoDispose() override;

  private:
    void StartApplication() override;
    void StopApplication() override;

    /**
     * Send a packet.
     */
    void SendPacket();

    uint32_t m_pktSize;                     //!< The packet size.
    uint32_t m_destPort;                    //!< Destination port.
    Ptr<ConstantRandomVariable> m_interval; //!< Rng for sending packets.

    Ptr<Socket> m_socket;                   //!< Sending socket.
    EventId m_sendEvent;                    //!< Send packet event.
    uint64_t m_sent;                        //!< Number of packets sent.

    /// Tx TracedCallback.
    TracedCallback<Ptr<const Packet>> m_txTrace;

    uint32_t m_count;                       //!< Number of packets sent.

    // end class Sender
};

/**
 * FlockingReceiver application.
 */
class FlockingReceiver : public Application
{
  public:
    /**
     * \brief Get the type ID.
     * \return The object TypeId.
     */
    static TypeId GetTypeId();
    FlockingReceiver();
    ~FlockingReceiver() override;

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
     * \brief Returns the number of received packets
     * \return the number of received packets
     */
    uint64_t GetReceived() const;

  protected:
    void DoDispose() override;

  private:
    void StartApplication() override;
    void StopApplication() override;

    /**
     * Receive a packet.
     * \param socket The receiving socket.
     */
    void Receive(Ptr<Socket> socket);

    uint32_t m_port;                //!< Listening port.
    uint32_t m_max_neighbors;       //!< Maximum number of neighbors
    Time m_timeout;                 //!< Timeout after which an neighbor expires
    
    Ptr<Socket> m_socket;           //!< Receiving socket.
    uint64_t m_received;            //!< Number of received packets

    Ptr<CounterCalculator<>> m_calc;           //!< Counter of the number of received packets.
    Ptr<TimeMinMaxAvgTotalCalculator> m_delay; //!< Delay calculator.

    /// Rx TracedCallback.
    TracedCallback<Ptr<const Packet>, int> m_rxTrace;

    // end class Receiver
};

#endif // FLOCKING_APPLICATION_H