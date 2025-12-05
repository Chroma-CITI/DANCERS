#pragma once

#include "ns3/application.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"

using namespace ns3;

/**
 * WaypointBroadcaster application.
 */
class WaypointBroadcaster : public Application
{
public:
  /**
   * \brief Get the type ID.
   * \return The object TypeId.
   */
  static TypeId GetTypeId();
  WaypointBroadcaster();
  ~WaypointBroadcaster() override;

  /**
   * \brief Returns the number of received packets
   * \return the number of received packets
   */
  uint64_t GetSent() const;

  /**
   * \brief Set the waypoint.
   * \param waypoint The waypoint.
   */
  void SetWaypoint(Vector waypoint);

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
  uint32_t m_flowId;                      //!< Flow Id tagged in the packets
  Vector m_waypoint;                      //!< Current waypoint

  Ptr<Socket> m_socket; //!< Sending socket.
  EventId m_sendEvent;  //!< Send packet event.
  uint64_t m_sent;      //!< Number of packets sent.

  /// Tx TracedCallback.
  TracedCallback<Ptr<const Packet>> m_txTrace;

  uint32_t m_count; //!< Number of packets sent.

  // end class Sender
};


/**
 * WaypointReceiver application.
 */
class WaypointReceiver : public Application
{
public:
  /**
   * \brief Get the type ID.
   * \return The object TypeId.
   */
  static TypeId GetTypeId();
  WaypointReceiver();
  ~WaypointReceiver() override;

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

  uint32_t m_port;          //!< Listening port.

  Ptr<Socket> m_socket; //!< Receiving socket.
  uint64_t m_received;  //!< Number of received packets

  /// Rx TracedCallback.
  TracedCallback<Ptr<const Packet>, int> m_rxTrace;

  // end class Receiver
};


/**
 * WaypointHeader
 */
class WaypointHeader : public Header
{
public:

  WaypointHeader();
  ~WaypointHeader() override;

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

  void SetPosition(Vector position);

  Vector GetPosition() const;

private:
  Vector m_position;

};
