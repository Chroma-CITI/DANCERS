#include "waypoint_broadcast.h"

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/stats-module.h"
#include "ns3/mobility-module.h"

using namespace ns3;

#define WAYPOINT_HEADER_SIZE 12

NS_LOG_COMPONENT_DEFINE("WiFiAppWaypointBroadcast");

//----------------------------------------------------------------------
//-- WaypointBroadcaster
//------------------------------------------------------
TypeId
WaypointBroadcaster::GetTypeId()
{
    static TypeId tid = TypeId("WaypointBroadcaster")
                            .SetParent<Application>()
                            .AddConstructor<WaypointBroadcaster>()
                            .AddAttribute("AdditionalSize",
                                          "The size in bytes of additional data appended after the waypoint.",
                                          UintegerValue(0),
                                          MakeUintegerAccessor(&WaypointBroadcaster::m_pktSize),
                                          MakeUintegerChecker<uint32_t>())
                            .AddAttribute("Port",
                                          "Destination app port.",
                                          UintegerValue(4000),
                                          MakeUintegerAccessor(&WaypointBroadcaster::m_destPort),
                                          MakeUintegerChecker<uint32_t>())
                            .AddAttribute("Interval",
                                          "Delay between transmissions in seconds.",
                                          StringValue("ns3::ConstantRandomVariable[Constant=0.5]"),
                                          MakePointerAccessor(&WaypointBroadcaster::m_interval),
                                          MakePointerChecker<RandomVariableStream>())
                            .AddAttribute("FlowId",
                                          "Tag Id of this flow",
                                          UintegerValue(2),
                                          MakeUintegerAccessor(&WaypointBroadcaster::m_flowId),
                                          MakeUintegerChecker<uint32_t>())
                            .AddTraceSource("Tx",
                                            "A new packet is created and is sent",
                                            MakeTraceSourceAccessor(&WaypointBroadcaster::m_txTrace),
                                            "ns3::Packet::TracedCallback");
    return tid;
}

WaypointBroadcaster::WaypointBroadcaster()
{
    NS_LOG_FUNCTION_NOARGS();
    m_interval = CreateObject<ConstantRandomVariable>();
    m_socket = nullptr;
    m_sent = 0;
}

WaypointBroadcaster::~WaypointBroadcaster()
{
    NS_LOG_FUNCTION_NOARGS();
}

void
WaypointBroadcaster::DoDispose()
{
    NS_LOG_FUNCTION_NOARGS();

    m_socket = nullptr;
    // chain up
    Application::DoDispose();
}

void
WaypointBroadcaster::StartApplication()
{
    NS_LOG_FUNCTION_NOARGS();

    if (!m_socket)
    {
        Ptr<SocketFactory> socketFactory =
            GetNode()->GetObject<SocketFactory>(UdpSocketFactory::GetTypeId());
        m_socket = socketFactory->CreateSocket();
        m_socket->Bind();
    }

    m_count = 0;
    
    m_socket->SetAllowBroadcast(true);

    Simulator::Cancel(m_sendEvent);
    m_sendEvent = Simulator::ScheduleNow(&WaypointBroadcaster::SendPacket, this);

    // end WaypointBroadcaster::StartApplication
}

void
WaypointBroadcaster::StopApplication()
{
    NS_LOG_FUNCTION_NOARGS();
    Simulator::Cancel(m_sendEvent);
    // end WaypointBroadcaster::StopApplication
}

void
WaypointBroadcaster::SendPacket()
{
    // NS_LOG_FUNCTION_NOARGS ();
    NS_LOG_INFO("Broadcasting packet at " << Simulator::Now());

    Ptr<Packet> packet;
    if (this->m_pktSize > 0)
    {
        packet = Create<Packet>(this->m_pktSize);
    }
    else
    {
        packet = Create<Packet>();
    }

    WaypointHeader waypoint_header;
    waypoint_header.SetPosition(m_waypoint);
    packet->AddHeader(waypoint_header);

    FlowIdTag flow_id;
    flow_id.SetFlowId(m_flowId);
    packet->AddPacketTag(flow_id);

    // Could connect the socket since the address never changes; using SendTo
    // here simply because all of the standard apps do not.
    m_socket->SendTo(packet, 0, InetSocketAddress(Ipv4Address::GetBroadcast(), m_destPort));

    // Report the event to the trace.
    m_txTrace(packet);

    m_sent++;

    std::cout << "Sent packet: " << m_sent << std::endl;

    m_sendEvent =
        Simulator::Schedule(Seconds(m_interval->GetValue()), &WaypointBroadcaster::SendPacket, this);

    // end WaypointBroadcaster::SendPacket
}

uint64_t WaypointBroadcaster::GetSent() const
{
    return m_sent;
}

void WaypointBroadcaster::SetWaypoint(Vector waypoint)
{
    m_waypoint = waypoint;
}


//----------------------------------------------------------------------
//-- WaypointReceiver
//------------------------------------------------------
TypeId
WaypointReceiver::GetTypeId()
{
    static TypeId tid = TypeId("WaypointReceiver")
                            .SetParent<Application>()
                            .AddConstructor<WaypointReceiver>()
                            .AddAttribute("Port",
                                          "Listening port.",
                                          UintegerValue(4000),
                                          MakeUintegerAccessor(&WaypointReceiver::m_port),
                                          MakeUintegerChecker<uint32_t>())
                            .AddTraceSource("Rx",
                                            "A new packet is received",
                                            MakeTraceSourceAccessor(&WaypointReceiver::m_rxTrace),
                                            "ns3::Packet::TracedCallback");
    return tid;
}

WaypointReceiver::WaypointReceiver()
    : m_received(0)
{
    NS_LOG_FUNCTION_NOARGS();
    m_socket = nullptr;
}

WaypointReceiver::~WaypointReceiver()
{
    NS_LOG_FUNCTION_NOARGS();
}

void
WaypointReceiver::DoDispose()
{
    NS_LOG_FUNCTION_NOARGS();

    m_socket = nullptr;
    // chain up
    Application::DoDispose();
}

void
WaypointReceiver::StartApplication()
{
    NS_LOG_FUNCTION_NOARGS();

    if (!m_socket)
    {
        Ptr<SocketFactory> socketFactory =
            GetNode()->GetObject<SocketFactory>(UdpSocketFactory::GetTypeId());
        m_socket = socketFactory->CreateSocket();
        InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), m_port);
        if (m_socket->Bind(local) == -1)
        {
            NS_FATAL_ERROR("Failed to bind socket");
        }
    }

    m_socket->SetRecvCallback(MakeCallback(&WaypointReceiver::Receive, this));

    // end WaypointReceiver::StartApplication
}

void
WaypointReceiver::StopApplication()
{
    NS_LOG_FUNCTION_NOARGS();

    if (m_socket)
    {
        m_socket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
    }

    // end WaypointReceiver::StopApplication
}

void
WaypointReceiver::Receive(Ptr<Socket> socket)
{
    // NS_LOG_FUNCTION (this << socket << packet << from);

    Ptr<Packet> packet;
    Address from;
    while ((packet = socket->RecvFrom(from)))
    {
        if (InetSocketAddress::IsMatchingType(from))
        {
            Ipv4Address peer_address = InetSocketAddress::ConvertFrom(from).GetIpv4();
            NS_LOG_INFO("Received " << packet->GetSize() << " bytes from "
                                    << peer_address);
            // We consider /24 addresses with the last 8 bits corresponding to the peer ID + 1 ( e.g. agent 0 has address 10.0.0.1/24 )
            uint32_t peer_id = peer_address.CombineMask("0.0.0.255").Get() - 1;

            // Report the event to the trace.
            m_rxTrace(packet, peer_id);
            m_received++;

        }
        // end receiving packets
    }

    // end WaypointReceiver::Receive
}

uint64_t WaypointReceiver::GetReceived() const
{
    return m_received;
}


//----------------------------------------------------------------------
//-- WaypointHeader
//------------------------------------------------------
WaypointHeader::WaypointHeader()
{
}

WaypointHeader::~WaypointHeader()
{
}

TypeId WaypointHeader::GetTypeId()
{
    static TypeId tid = TypeId("WaypointHeader")
                            .SetParent<Header>()
                            .AddConstructor<WaypointHeader>();
    return tid;
}

TypeId
WaypointHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint32_t WaypointHeader::GetSerializedSize() const
{
    return WAYPOINT_HEADER_SIZE;
}

void
WaypointHeader::Print(std::ostream& os) const
{
    os << "Waypoint: (" << m_position.x << ", " << m_position.y << ", " << m_position.z << ")";
}

void
WaypointHeader::Serialize(Buffer::Iterator start) const
{
    Buffer::Iterator i = start;
    i.WriteHtonU32(std::bit_cast<uint32_t>((float)m_position.x));
    i.WriteHtonU32(std::bit_cast<uint32_t>((float)m_position.y));
    i.WriteHtonU32(std::bit_cast<uint32_t>((float)m_position.z));
}

uint32_t
WaypointHeader::Deserialize(Buffer::Iterator start)
{
    Buffer::Iterator i = start;
    m_position.x = std::bit_cast<float>(i.ReadNtohU32());
    m_position.y = std::bit_cast<float>(i.ReadNtohU32());
    m_position.z = std::bit_cast<float>(i.ReadNtohU32());
    return GetSerializedSize();
}

void
WaypointHeader::SetPosition(Vector position)
{
    m_position = position;
}

Vector
WaypointHeader::GetPosition() const
{
    return m_position;
}