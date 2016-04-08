#include <fstream>
#include <iostream>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
#include "ns3/polsr-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"
#include "ns3/nstime.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-helper.h"
#include "WifiPhyStats.h"
#include "TimestampTag.h"
#include "ns3/god.h"

using namespace ns3;
class RoutingExperiment
{
public:
  RoutingExperiment ();
  void Setup (std::string CSVfileName);
  //static void SetMACParam (ns3::NetDeviceContainer & devices,
  //                                 int slotDistance);
  std::string CommandSetup (int argc, char **argv);
private:
  Ptr<Socket> SetupPacketReceive (Ipv4Address addr, Ptr<Node> node);
  void ReceivePacket (Ptr<Socket> socket);
  void CheckThroughput ();
  void SentPacket (Ptr<const Packet> p);
  void PingRtt (std::string context, Time rtt);
  void SetMobility(NodeContainer nodes,double m_nodeSpeed,double nodePause);
  uint32_t port;
  uint32_t bytesTotal;
  uint32_t bytesSentTotal;
  uint32_t bytesOnce;
  double rttTotal;
  double rtt_count;
  uint32_t packetsReceived;
  std::string m_CSVfileName;
  std::string m_CSVfileName_all;
  int m_nSinks;
  int m_nWifis;//节点数
  int m_nodeSpeed; //速度 m/s
  double m_xsize;//x轴长度 m
  double m_ysize;//y轴长度 m
  std::string rate;//应用层速度
  std::string m_protocolName;
  double m_txp;
  bool m_traceMobility;
  bool m_ping;
  uint32_t m_protocol;
  double TotalTime;//总运行时间
  double UdpStartTime;//udp发送时间
  Ptr<WifiPhyStats> m_wifiPhyStats;//wifi 物理状态
  Ptr<LocationService> m_locationService;//地理位置测试
};
RoutingExperiment::RoutingExperiment ()
  : port (9),
    bytesTotal (0),
    bytesSentTotal (0),
    bytesOnce (0),
    rttTotal(0),
    rtt_count (0),
    packetsReceived (0),
    m_CSVfileName ("manet-routing.output.csv"),
    m_CSVfileName_all("manet-routing_all.csv"),
    m_nSinks(5),
    m_nWifis (10),//节点数 10
    m_nodeSpeed (20),//速度 20m/s
    m_xsize (500),//x轴长度 500m
    m_ysize (500),//y轴长度 500m
    rate ("2048bps"),//应用层速度2048bps
    m_traceMobility (false),
    m_ping(true),
    m_protocol (2), // AODV    
    TotalTime(100),//总运行时间 100s
    UdpStartTime (50) //udp发送时间
{
  m_wifiPhyStats = CreateObject<WifiPhyStats> ();
  m_locationService = CreateObject<GodLocationService> ();
}
//print packet
static inline std::string PrintReceivedPacket (Ptr<Socket> socket, Ptr<Packet> packet)
{
  std::ostringstream oss;
  SocketAddressTag tag;
  bool found;
  found = packet->PeekPacketTag (tag);
  oss << Simulator::Now ().GetSeconds () << " " << socket->GetNode ()->GetId ();
  if (found)
    {
      InetSocketAddress addr = InetSocketAddress::ConvertFrom (tag.GetAddress ());
      oss << " received one packet from " << addr.GetIpv4 ();
    }
  else
    {
      oss << " received one packet!";
    }
  return oss.str ();
}
//ReceivePacket
void RoutingExperiment::ReceivePacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  while ((packet = socket->Recv ()))
  {
    bytesTotal += packet->GetSize ();
    bytesOnce += packet->GetSize ();
    packetsReceived += 1;
    std::cout<<(PrintReceivedPacket (socket, packet))<<std::endl;
    TimestampTag timestamp;
    if (packet->FindFirstMatchingByteTag (timestamp)) {
      Time tx = timestamp.GetTimestamp ();
      Time rx = Simulator::Now ();
      double d_t = (rx-tx).ToDouble(Time::MS);
      rttTotal += d_t;
      rtt_count++;
    }
  }
}
//检查throughput
void RoutingExperiment::CheckThroughput ()
{
  double kbs = (bytesOnce * 8.0) / 1000;
  double kbs_avr = (bytesTotal*8.0)/1000/(Simulator::Now ().GetSeconds ()-UdpStartTime);
  double kbs_send = (bytesSentTotal*8.0)/1000/(Simulator::Now ().GetSeconds ()-UdpStartTime);
  double kbs_con = (m_wifiPhyStats->GetTxBytes()-bytesTotal)*8.0/1000/Simulator::Now ().GetSeconds ();
  double rtt = rttTotal/rtt_count;
  bytesOnce = 0;
  std::cout<<"time = "<<Simulator::Now ().GetSeconds ()
            <<"\tcontrol rate = "<<kbs_con<<"kbps"
            <<"\tapp rate = "<<kbs<<"kbps"
            <<"\tsend avr rate = "<<kbs_send<<"kbps"
            <<"\tavr rate = "<<kbs_avr<<"kbps"
            <<"\tdelay = "<<rtt<<"ms"<<std::endl;
            
  //std::cout<<"Node0 Pos:\t"<<
  //      m_locationService->GetPosition(Ipv4Address("10.1.1.1"))<<std::endl;
        
  std::ofstream out (m_CSVfileName.c_str (), std::ios::app);

  out << (Simulator::Now ()).GetSeconds () << ","
      << kbs << ","
      << packetsReceived << ","
      << m_nSinks << ","
      << m_protocolName << ","
      << m_txp << ","
      << std::endl;

  out.close ();
  packetsReceived = 0;
  if(TotalTime-Simulator::Now ().GetSeconds ()<=1)
  {
    std::ofstream out_all(m_CSVfileName_all.c_str (), std::ios::app);
    out_all<< m_protocolName << ","
    << m_xsize <<","
    << m_nWifis <<","
    << m_nSinks <<","
    << rate <<","
    << kbs_avr <<","
    << kbs_con<<","
    << rtt<<","
    << kbs_avr/kbs_send<<""
    << std::endl;
    out_all.close ();
  }
  Simulator::Schedule (Seconds (1.0), &RoutingExperiment::CheckThroughput, this);  
}
//安装packet接收器
Ptr<Socket> RoutingExperiment::SetupPacketReceive (Ipv4Address addr, Ptr<Node> node)
{
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> sink = Socket::CreateSocket (node, tid);
  InetSocketAddress local = InetSocketAddress (addr, port);
  sink->Bind (local);
  sink->SetRecvCallback (MakeCallback (&RoutingExperiment::ReceivePacket, this));
  return sink;
}
//初始化命令
std::string RoutingExperiment::CommandSetup (int argc, char **argv)
{
  CommandLine cmd;
  int seed=12345;
  cmd.AddValue ("csv", "The name of the CSV output file name", m_CSVfileName);
  cmd.AddValue ("csvall","The name of the CSV sum output file",m_CSVfileName_all);
  cmd.AddValue ("traceMobility", "Enable mobility tracing", m_traceMobility);
  cmd.AddValue ("protocol", "0=POLSR;1=OLSR;2=AODV;3=DSDV;4=DSR", m_protocol);
  cmd.AddValue ("nodes", "the number of wifi nodes",m_nWifis);
  cmd.AddValue ("speed", "the speed of nodes",m_nodeSpeed);
  cmd.AddValue ("xlength", "the length of x",m_xsize);
  cmd.AddValue ("ylength", "the length of y",m_ysize);
  cmd.AddValue ("rate", "the rate of udp sink",rate);
  cmd.AddValue ("totaltime", "the total time of this simulation",TotalTime);
  cmd.AddValue ("udptime", "the time of start udp sending",UdpStartTime);
  cmd.AddValue ("links", "the number of udp links",m_nSinks);
  cmd.AddValue ("ping","ping to get delay of mesh",m_ping);
  cmd.AddValue ("seed","seed of the random function",seed);
  cmd.Parse (argc, argv);
  SeedManager::SetSeed (seed);
  return m_CSVfileName;
}
//设置移动性
void RoutingExperiment::SetMobility(NodeContainer nodes,double m_nodeSpeed,double nodePause){
    MobilityHelper mobilityAdhoc;
    int64_t streamIndex = 0; // used to get consistent mobility across scenarios
    ObjectFactory pos;
    pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
    std::stringstream xsize,ysize;
    xsize<<"ns3::UniformRandomVariable[Min=0.0|Max="<<m_xsize<<"]";
    pos.Set ("X", StringValue (xsize.str()));
    ysize<<"ns3::UniformRandomVariable[Min=0.0|Max="<<m_ysize<<"]";
    pos.Set ("Y", StringValue (ysize.str()));

    Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
    streamIndex += taPositionAlloc->AssignStreams (streamIndex);

    std::stringstream ssSpeed;
    ssSpeed << "ns3::UniformRandomVariable[Min="<<m_nodeSpeed/2<<"|Max=" << m_nodeSpeed << "]";
    std::stringstream ssPause;
    ssPause << "ns3::ConstantRandomVariable[Constant=" << nodePause << "]";
    mobilityAdhoc.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                  "Speed", StringValue (ssSpeed.str ()),
                                  "Pause", StringValue (ssPause.str ()),
                                  "PositionAllocator", PointerValue (taPositionAlloc));
    mobilityAdhoc.SetPositionAllocator (taPositionAlloc);
    mobilityAdhoc.Install (nodes);
    streamIndex += mobilityAdhoc.AssignStreams (nodes, streamIndex);   
}
