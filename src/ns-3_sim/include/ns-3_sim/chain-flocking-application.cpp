#include "chain-flocking-application.h"

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/stats-module.h"

#include "pick-neighbors-header.h"

#include <ostream>

using namespace ns3;

//----------------------------------------------------------------------
//-- ChainFlocking
//------------------------------------------------------
TypeId
ChainFlocking::GetTypeId()
{
    static TypeId tid = TypeId("ChainFlocking")
                            .SetParent<Application>()
                            .AddConstructor<ChainFlocking>()
                            .AddAttribute("PacketSize",
                                          "The size of packets transmitted.",
                                          UintegerValue(64),
                                          MakeUintegerAccessor(&ChainFlocking::m_pktSize),
                                          MakeUintegerChecker<uint32_t>(1))
                            .AddAttribute("Port",
                                          "App port.",
                                          UintegerValue(4000),
                                          MakeUintegerAccessor(&ChainFlocking::m_port),
                                          MakeUintegerChecker<uint32_t>())
                            .AddAttribute("Interval",
                                          "Delay between transmissions.",
                                          StringValue("ns3::ConstantRandomVariable[Constant=0.5]"),
                                          MakePointerAccessor(&ChainFlocking::m_interval),
                                          MakePointerChecker<RandomVariableStream>())
                            .AddAttribute("NumRelays",
                                          "Maximum number of neighbors authorized.",
                                          UintegerValue(1),
                                          MakeUintegerAccessor(&ChainFlocking::m_numRelays),
                                          MakeUintegerChecker<uint32_t>())
                            .AddAttribute("Timeout",
                                          "Time after which a neighbor is discarded if no new update has been received.",
                                          TimeValue(Seconds(1)),
                                          MakeTimeAccessor(&ChainFlocking::m_timeout),
                                          MakeTimeChecker())
                            .AddTraceSource("Tx",
                                            "A new packet is created and is sent",
                                            MakeTraceSourceAccessor(&ChainFlocking::m_txTrace),
                                            "ns3::Packet::TracedCallback")
                            .AddTraceSource("Rx",
                                            "A new packet is received",
                                            MakeTraceSourceAccessor(&ChainFlocking::m_rxTrace),
                                            "ns3::Packet::TracedCallback");

    return tid;
}

ChainFlocking::ChainFlocking()
{
    NS_LOG_FUNCTION_NOARGS();
    m_interval = CreateObject<ConstantRandomVariable>();
    m_SendSocket = nullptr;
    m_RecvSocket = nullptr;
    m_sent = 0;
    m_received = 0;
    m_calc = nullptr;
    m_delay = nullptr;
    m_leaderRank = UINT32_MAX;
}

ChainFlocking::~ChainFlocking()
{
    NS_LOG_FUNCTION_NOARGS();
}

void
ChainFlocking::DoDispose()
{
    NS_LOG_FUNCTION_NOARGS();

    m_SendSocket = nullptr;
    m_RecvSocket = nullptr;
    // chain up
    Application::DoDispose();
}

void
ChainFlocking::StartApplication()
{
    NS_LOG_FUNCTION_NOARGS();

    // Create Receiving socket if not already created
    if (!m_RecvSocket)
    {
        Ptr<SocketFactory> socketFactory =
            GetNode()->GetObject<SocketFactory>(UdpSocketFactory::GetTypeId());
        m_RecvSocket = socketFactory->CreateSocket();
        InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), m_port);
        if (m_RecvSocket->Bind(local) == -1)
        {
            NS_FATAL_ERROR("Failed to bind receiving socket");
        }
    }

    // Create Sending socket if not already created
    if (!m_SendSocket)
    {
        Ptr<SocketFactory> socketFactory =
            GetNode()->GetObject<SocketFactory>(UdpSocketFactory::GetTypeId());
        m_SendSocket = socketFactory->CreateSocket();
        if (m_SendSocket->Bind())
        {
            NS_FATAL_ERROR("Failed to bind sending socket");
        }
    }

    m_RecvSocket->SetRecvCallback(MakeCallback(&ChainFlocking::Receive, this));

    m_SendSocket->SetAllowBroadcast(true);

    Simulator::Cancel(m_sendEvent);
    m_sendEvent = Simulator::ScheduleNow(&ChainFlocking::SendPacket, this);

    // end ChainFlocking::StartApplication
}

void
ChainFlocking::StopApplication()
{
    NS_LOG_FUNCTION_NOARGS();

    if (m_RecvSocket)
    {
        m_RecvSocket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
    }
    Simulator::Cancel(m_sendEvent);

    // end ChainFlocking::StopApplication
}

void
ChainFlocking::SendPacket()
{
    NS_LOG_FUNCTION_NOARGS();
    NS_LOG_INFO("Broadcasting packet at " << Simulator::Now());

    Ptr<Packet> packet = Create<Packet>(m_pktSize);

    if (m_leaderRank < UINT32_MAX && m_leaderRank != UINT32_MAX-1)
    {
        std::vector<uint32_t> bestNeighbors = this->SelectBestRelays();

        if (m_currentNeighborsDown.find(bestNeighbors[0]) == m_currentNeighborsDown.end())
        {
            std::cout << this->GetNode()->GetId() << " of rank " << m_leaderRank << " is looking for a relay !" << std::endl;
            m_currentNeighborsDown.clear();
            PickNeighborsHeader pickNeighborsHeader;
            pickNeighborsHeader.SetBestNeighbors(bestNeighbors);
            for (auto neighbor : bestNeighbors)
            {
                m_currentNeighborsDown[neighbor] = m_linkQualities[neighbor];
            }
            pickNeighborsHeader.SetLeaderRank(m_leaderRank);
            packet->AddHeader(pickNeighborsHeader);
        }
    }

    // Could connect the socket since the address never changes; using SendTo
    // here simply because all of the standard apps do not.
    m_SendSocket->SendTo(packet, 0, InetSocketAddress(Ipv4Address::GetBroadcast(), m_port));

    // Report the event to the trace.
    m_txTrace(packet);

    m_sent++;

    m_sendEvent =
        Simulator::Schedule(Seconds(m_interval->GetValue()), &ChainFlocking::SendPacket, this);

    // end ChainFlocking::SendPacket
}

void
ChainFlocking::Receive(Ptr<Socket> socket)
{
    Ptr<Packet> packet;
    Address from;
    while ((packet = socket->RecvFrom(from)))
    {
        if (InetSocketAddress::IsMatchingType(from))
        {
            Ipv4Address peer_address = InetSocketAddress::ConvertFrom(from).GetIpv4();
            uint32_t peer_id = peer_address.CombineMask("0.0.0.255").Get() - 1;
            NS_LOG_INFO("Received " << packet->GetSize() << " bytes from "
                                    << peer_address);

            PickNeighborsHeader pickNeighborsHeader;
            if (packet->PeekHeader(pickNeighborsHeader) > 0)
            {
                for (auto msg_best_neighbor : pickNeighborsHeader.GetBestNeighbors())
                {
                    std::cout << this->GetNode()->GetId() << " Received a relay election from " << peer_id;
                    if (msg_best_neighbor == this->GetNode()->GetId() && m_leaderRank > pickNeighborsHeader.GetLeaderRank())
                    {
                        std::cout << " --> it's me !" << std::endl;
                        // "Sink" nodes don't change their leader rank
                        if (m_leaderRank != UINT32_MAX-1)
                        {
                            m_leaderRank = pickNeighborsHeader.GetLeaderRank() + 1;
                        }
                        // Verify that we have a link quality information for this neighbor
                        if (m_linkQualities.find(peer_id) != m_linkQualities.end())
                        {
                            m_currentNeighborsUp[peer_id] = m_linkQualities[peer_id];
                        }
                        else
                        {
                            NS_LOG_WARN("Link quality not found for neighbor " << peer_id);
                        }
                    }
                    else
                    {
                        std::cout << " --> it's not me !" << std::endl;
                    }
                }
            }

            // Report the event to the trace.
            m_rxTrace(packet);

            m_received++;


            m_lastReceivedUpdate[peer_id] = Simulator::Now();

        }

        if (m_calc)
        {
            m_calc->Update();
        }
        // end receiving packets
    }

    // end ChainFlocking::Receive
}


uint64_t 
ChainFlocking::GetSent() const
{
    return m_sent;
    // end ChainFlocking::GetSent
}

uint64_t 
ChainFlocking::GetReceived() const
{
    return m_received;
    // end ChainFlocking::GetReceived

}

std::map<uint32_t, double> 
ChainFlocking::GetCurrentNeighbors()
{
    if (m_linkQualities.empty())
    {
        std::cout << "LinkQualities is empty! This application must receive link qualities from underlying layer !" << std::endl;
        exit(EXIT_FAILURE);
    }

    // // Create a vector of ordered link qualities
    // std::vector<double> ordered_link_qualities;
    // for (auto link_qual : m_linkQualities)
    // {
    //     ordered_link_qualities.push_back(link_qual.second);
    // }
    // std::sort(ordered_link_qualities.begin(), ordered_link_qualities.end(), std::greater<double>());

    // std::map<uint32_t, double> currentNeighbors;

    // for (double link_qual : ordered_link_qualities)
    // {
    //     for (auto neigh : m_lastReceivedUpdate)
    //     {
    //         if (m_linkQualities[neigh.first] == link_qual           // Search by value the neighbor corresponding to this pathloss (two neighbors should not have same pathloss, I hope) 
    //             && currentNeighbors.size() < m_numRelays       // Check if we have reached the maximum number of authorized neighbors                
    //             && neigh.second - Simulator::Now() < m_timeout)     // Check if the neighbor has not timed out
    //         {
    //             currentNeighbors[neigh.first] = link_qual;
    //             break;
    //         }
    //     }
    // }
    std::map<uint32_t, double> currentNeighbors;
    for (auto neighbor : m_currentNeighborsUp)
    {
        currentNeighbors[neighbor.first] = neighbor.second;
    }
    for (auto neighbor : m_currentNeighborsDown)
    {
        if (currentNeighbors.find(neighbor.first) == currentNeighbors.end())
        {
            currentNeighbors[neighbor.first] = neighbor.second;
        }
        else
        {
            NS_FATAL_ERROR("Neighbor " << neighbor.first << " is both up and down neighbor");
        }
    }

    return currentNeighbors;
    // end ChainFlocking::GetCurrentNeighbors
}

void 
ChainFlocking::SetLinkQualities(std::map<uint32_t, double> linkQualities)
{
    m_linkQualities = linkQualities;
    // end ChainFlocking::SetLinkQualities
}

void 
ChainFlocking::SetLeaderRank(uint32_t leaderRank)
{
    m_leaderRank = leaderRank;
    // end ChainFlocking::SetLeaderRank
}

void
ChainFlocking::SetCounter(Ptr<CounterCalculator<>> calc)
{
    m_calc = calc;
    // end ChainFlocking::SetCounter
}

void
ChainFlocking::SetDelayTracker(Ptr<TimeMinMaxAvgTotalCalculator> delay)
{
    m_delay = delay;
    // end ChainFlocking::SetDelayTracker
}

std::vector<uint32_t>
ChainFlocking::SelectBestRelays()
{
    std::vector<uint32_t> bestRelays;
    std::vector<std::pair<uint32_t, double>> sortedNeighbors;

    for (auto &n : m_lastReceivedUpdate)
    {
        if (Simulator::Now() - n.second < m_timeout                                 // not timed out
            && m_currentNeighborsUp.find(n.first) == m_currentNeighborsUp.end()        // not already a "up" neighbor
            && m_linkQualities.find(n.first) != m_linkQualities.end())              // we have link quality info
        {
            sortedNeighbors.push_back(std::make_pair(n.first, m_linkQualities[n.first]));
        }
        else
        {
            std::cout << "Discarded neighbor " << n.first << std::endl;
        }
    }

    std::sort(sortedNeighbors.begin(), sortedNeighbors.end(), [](const auto &a, const auto &b)
              { return a.second > b.second; });

    for (size_t i = 0; i < std::min((size_t)m_numRelays, sortedNeighbors.size()); ++i)
    {
        bestRelays.push_back(sortedNeighbors[i].first);
    }

    return bestRelays;
    // end ChainFlocking::SelectBestNeighbors
}