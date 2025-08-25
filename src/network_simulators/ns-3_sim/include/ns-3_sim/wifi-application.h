/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Joe Kopena <tjkopena@cs.drexel.edu>
 *
 * These applications are used in the WiFi Distance Test experiment,
 * described and implemented in test02.cc.  That file should be in the
 * same place as this file.  The applications have two very simple
 * jobs, they just generate and receive packets.  We could use the
 * standard Application classes included in the NS-3 distribution.
 * These have been written just to change the behavior a little, and
 * provide more examples.
 *
 */

#include "ns3/application.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/stats-module.h"

using namespace ns3;

/**
 * Sender application.
 */
class Sender : public Application
{
  public:
    /**
     * \brief Get the type ID.
     * \return The object TypeId.
     */
    static TypeId GetTypeId();
    Sender();
    ~Sender() override;

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
    Ipv4Address m_destAddr;                 //!< Destination address.
    uint32_t m_destPort;                    //!< Destination port.
    Ptr<ConstantRandomVariable> m_interval; //!< Rng for sending packets.
    uint32_t m_numPkts;                     //!< Number of packets to send.
    uint32_t m_flowId;                      //!< Flow Id tagged on the packet

    uint32_t m_target_id;                   //!< The ID of the target assigned to this leader
    bool m_in_target_area;                  //!< Flag indicating if the agent is in its target area

    Ptr<Socket> m_socket; //!< Sending socket.
    EventId m_sendEvent;  //!< Send packet event.
    uint64_t m_sent;      //!< Number of packets sent.

    /// Tx TracedCallback.
    TracedCallback<Ptr<const Packet>> m_txTrace;

    uint32_t m_count; //!< Number of packets sent.

    // end class Sender
};

/**
 * Receiver application.
 */
class Receiver : public Application
{
  public:
    /**
     * \brief Get the type ID.
     * \return The object TypeId.
     */
    static TypeId GetTypeId();
    Receiver();
    ~Receiver() override;

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

    Ptr<Socket> m_socket; //!< Receiving socket.
    uint32_t m_port;      //!< Listening port.
    uint64_t m_received;             //!< Number of received packets

    Ptr<CounterCalculator<>> m_calc;           //!< Counter of the number of received packets.
    Ptr<TimeMinMaxAvgTotalCalculator> m_delay; //!< Delay calculator.

    /// Rx TracedCallback.
    TracedCallback<Ptr<const Packet>> m_rxTrace;

    // end class Receiver
};

/**
 * Timestamp tag - it carries when the packet has been sent.
 *
 * It would have been more realistic to include this info in
 * a header. Here we show how to avoid the extra overhead in
 * a simulation.
 */
class myTimestampTag : public Tag
{
  public:
    /**
     * \brief Get the type ID.
     * \return The object TypeId.
     */
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;

    uint32_t GetSerializedSize() const override;
    void Serialize(TagBuffer i) const override;
    void Deserialize(TagBuffer i) override;

    /**
     * Set the timestamp.
     * \param time The timestamp.
     */
    void SetTimestamp(Time time);
    /**
     * Get the timestamp.
     * \return the timestamp.
     */
    Time GetTimestamp() const;

    void Print(std::ostream& os) const override;

  private:
    Time m_timestamp; //!< Timestamp.

    // end class myTimestampTag
};

/**
 * MissionHeader
 */
class MissionHeader : public Header
{
public:
  MissionHeader();
  ~MissionHeader() override;

  /**
   * \brief Get the type ID.
   * \return The object TypeId.
   */
  static TypeId GetTypeId();
  TypeId GetInstanceTypeId() const override;
  void Print(std::ostream& os) const override;
  uint32_t GetSerializedSize() const override;
  void Serialize(Buffer::Iterator start) const override;
  uint32_t Deserialize(Buffer::Iterator start) override;

  void SetInTargetArea(bool in_target_area);
  bool GetInTargetArea() const;
  void SetTargetId(uint32_t target_id);
  uint32_t GetTargetId() const;

private:
  bool m_in_target_area;
  uint32_t m_target_id;
};
