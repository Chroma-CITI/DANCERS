#include "flocking-application.h"

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/stats-module.h"

#include "pick-neighbors-header.h"

#include <ostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WiFiFlockingApp");

//----------------------------------------------------------------------
//-- FlockingBroadcaster
//------------------------------------------------------
TypeId
FlockingBroadcaster::GetTypeId()
{
    static TypeId tid = TypeId("FlockingBroadcaster")
                            .SetParent<Application>()
                            .AddConstructor<FlockingBroadcaster>()
                            .AddAttribute("PacketSize",
                                          "The size of packets transmitted.",
                                          UintegerValue(64),
                                          MakeUintegerAccessor(&FlockingBroadcaster::m_pktSize),
                                          MakeUintegerChecker<uint32_t>(1))
                            .AddAttribute("Port",
                                          "Destination app port.",
                                          UintegerValue(4000),
                                          MakeUintegerAccessor(&FlockingBroadcaster::m_destPort),
                                          MakeUintegerChecker<uint32_t>())
                            .AddAttribute("Interval",
                                          "Delay between transmissions.",
                                          StringValue("ns3::ConstantRandomVariable[Constant=0.5]"),
                                          MakePointerAccessor(&FlockingBroadcaster::m_interval),
                                          MakePointerChecker<RandomVariableStream>())
                            .AddAttribute("FlowId",
                                          "Tag Id of this flow",
                                          UintegerValue(2),
                                          MakeUintegerAccessor(&FlockingBroadcaster::m_flowId),
                                          MakeUintegerChecker<uint32_t>())
                            .AddTraceSource("Tx",
                                            "A new packet is created and is sent",
                                            MakeTraceSourceAccessor(&FlockingBroadcaster::m_txTrace),
                                            "ns3::Packet::TracedCallback");
    return tid;
}

FlockingBroadcaster::FlockingBroadcaster()
{
    NS_LOG_FUNCTION_NOARGS();
    m_interval = CreateObject<ConstantRandomVariable>();
    m_socket = nullptr;
    m_sent = 0;
}

FlockingBroadcaster::~FlockingBroadcaster()
{
    NS_LOG_FUNCTION_NOARGS();
}

void
FlockingBroadcaster::DoDispose()
{
    NS_LOG_FUNCTION_NOARGS();

    m_socket = nullptr;
    // chain up
    Application::DoDispose();
}

void
FlockingBroadcaster::StartApplication()
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
    m_sendEvent = Simulator::ScheduleNow(&FlockingBroadcaster::SendPacket, this);

    // end FlockingBroadcaster::StartApplication
}

void
FlockingBroadcaster::StopApplication()
{
    NS_LOG_FUNCTION_NOARGS();
    Simulator::Cancel(m_sendEvent);
    // end FlockingBroadcaster::StopApplication
}

void
FlockingBroadcaster::SendPacket()
{
    // NS_LOG_FUNCTION_NOARGS ();
    NS_LOG_INFO("Broadcasting packet at " << Simulator::Now());

    Ptr<Packet> packet = Create<Packet>(m_pktSize);

    FlowIdTag flow_id;
    flow_id.SetFlowId(m_flowId);
    packet->AddPacketTag(flow_id);


    // Could connect the socket since the address never changes; using SendTo
    // here simply because all of the standard apps do not.
    m_socket->SendTo(packet, 0, InetSocketAddress(Ipv4Address::GetBroadcast(), m_destPort));

    // Report the event to the trace.
    m_txTrace(packet);

    m_sent++;

    m_sendEvent =
        Simulator::Schedule(Seconds(m_interval->GetValue()), &FlockingBroadcaster::SendPacket, this);

    // end FlockingBroadcaster::SendPacket
}

uint64_t FlockingBroadcaster::GetSent() const
{
    return m_sent;
}

//----------------------------------------------------------------------
//-- FlockingReceiver
//------------------------------------------------------
TypeId
FlockingReceiver::GetTypeId()
{
    static TypeId tid = TypeId("FlockingReceiver")
                            .SetParent<Application>()
                            .AddConstructor<FlockingReceiver>()
                            .AddAttribute("Port",
                                          "Listening port.",
                                          UintegerValue(4000),
                                          MakeUintegerAccessor(&FlockingReceiver::m_port),
                                          MakeUintegerChecker<uint32_t>())
                            .AddAttribute("MaxNeighbors",
                                          "Maximum number of neighbors authorized.",
                                          UintegerValue(1),
                                          MakeUintegerAccessor(&FlockingReceiver::m_max_neighbors),
                                          MakeUintegerChecker<uint32_t>())
                            .AddTraceSource("Rx",
                                            "A new packet is received",
                                            MakeTraceSourceAccessor(&FlockingReceiver::m_rxTrace),
                                            "ns3::Packet::TracedCallback");
    return tid;
}

FlockingReceiver::FlockingReceiver()
    : m_calc(nullptr),
      m_delay(nullptr),
      m_received(0)
{
    NS_LOG_FUNCTION_NOARGS();
    m_socket = nullptr;
}

FlockingReceiver::~FlockingReceiver()
{
    NS_LOG_FUNCTION_NOARGS();
}

void
FlockingReceiver::DoDispose()
{
    NS_LOG_FUNCTION_NOARGS();

    m_socket = nullptr;
    // chain up
    Application::DoDispose();
}

void
FlockingReceiver::StartApplication()
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

    m_socket->SetRecvCallback(MakeCallback(&FlockingReceiver::Receive, this));

    // end FlockingReceiver::StartApplication
}

void
FlockingReceiver::StopApplication()
{
    NS_LOG_FUNCTION_NOARGS();

    if (m_socket)
    {
        m_socket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
    }

    // end FlockingReceiver::StopApplication
}

void
FlockingReceiver::SetCounter(Ptr<CounterCalculator<>> calc)
{
    m_calc = calc;
    // end FlockingReceiver::SetCounter
}

void
FlockingReceiver::SetDelayTracker(Ptr<TimeMinMaxAvgTotalCalculator> delay)
{
    m_delay = delay;
    // end FlockingReceiver::SetDelayTracker
}

void
FlockingReceiver::Receive(Ptr<Socket> socket)
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
            // Save the reception timestamp in the neighbors_last_received map
            // We consider /24 addresses with the last 8 bits corresponding to the peer ID + 1 ( e.g. agent 0 has address 10.0.0.1/24 )
            uint32_t peer_id = peer_address.CombineMask("0.0.0.255").Get() - 1;
            uint32_t node_id = this->GetNode()->GetId();

            // Report the event to the trace.
            m_rxTrace(packet, peer_id);
            m_received++;


        }

        if (m_calc)
        {
            m_calc->Update();
        }

        // end receiving packets
    }

    // end FlockingReceiver::Receive
}

uint64_t FlockingReceiver::GetReceived() const
{
    return m_received;
}
