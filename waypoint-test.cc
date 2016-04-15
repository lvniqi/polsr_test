/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 IITP RAS
 *
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
 * This is an example script for AODV manet routing protocol. 
 *
 * Authors: Pavel Boyko <boyko@iitp.ru>
 */
#include "ns3/config-store-module.h"
#include "ns3/olsr-helper.h"
#include "ns3/flow-monitor-module.h"

#include <vector>
#include <string>

#include "ns3/olsr-module.h"
#include "ns3/polsr-module.h"
#include "ns3/aodv-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/wifi-module.h" 
#include "ns3/v4ping-helper.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-helper.h"
#include <iostream>
#include <cmath>

#include "ns3/netanim-module.h"

#include "WifiPhyStats.h"

#include "TestBench.h"

#include "TimestampTag.h"
void FlowMonitor_Print(FlowMonitorHelper &flowmon,Ptr<FlowMonitor> &monitor
  ,std::string protocolName,double speed,std::string csv_name);
using namespace ns3;
class WayPointTest 
{
public:
  WayPointTest ();
  /// Configure script parameters, \return true on successful configuration
  bool Configure (int argc, char **argv);
  /// Run simulation
  void Run ();
  /// Report results
  void Report (std::ostream & os);
  //测试吞吐量
  void CheckThroughput();
  void ReceivePacket (Ptr<Socket> socket);
private:

  // parameters
  /// Number of nodes
  //节点数
  uint32_t node_size;
  /// Distance between nodes, meters
  double step;
  /// Simulation time, seconds
  double totalTime;
  /// Write per-device PCAP traces if true
  bool pcap;
  /// Print routes if true
  bool printRoutes;
  int m_protocol;
  double m_seed;
  double m_speed;
  int m_testBench;
  AodvHelper aodv;
  POlsrHelper polsr;
  DsdvHelper dsdv;
  OlsrHelper olsr;
  // network
  std::string m_protocolName;
  NodeContainer nodes;
  NetDeviceContainer devices;
  Ipv4InterfaceContainer interfaces;
  // Install FlowMonitor on all nodes
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor;
  Ptr<WifiPhyStats> m_wifiPhyStats;//wifi 物理状态
  //跟踪文件
  std::string animFile;
  //测试时延
  uint32_t bytesTotal;
  uint32_t bytesSentTotal;
  uint32_t bytesOnce;
  uint32_t packetsReceived;
  double rttTotal;
  double rttCount;
  double UdpStartTime;
  bool m_mobility;
  std::string rate;//应用层速度
private:
  void CreateNodes ();
  void CreateDevices ();
  void InstallInternetStack ();
  void InstallApplications ();
  void InstallThoughtApplications();
  void InstallDelayApplications();
  void InstallPositon(Ptr<WaypointMobilityModel> mob,
      double mid_x,double mid_y,double range, double speed,double start_w);
  void InstallRandomMobility();
  void InstallScanMobility();
  Ptr<Socket> SetupPacketReceive (Ipv4Address addr, Ptr<Node> node,uint16_t sinkPort);
  void SentPacket (Ptr<const Packet> p);
};

int main (int argc, char **argv)
{
  WayPointTest test;
  if (!test.Configure (argc, argv))
    NS_FATAL_ERROR ("Configuration failed. Aborted.");

  test.Run ();
  
  test.Report (std::cout);
  return 0;
}

//-----------------------------------------------------------------------------
WayPointTest::WayPointTest () :
  node_size (13),
  step (100),
  totalTime (100),
  pcap (false),
  printRoutes (false),
  m_protocol (0),
  m_seed(12345),
  m_speed(20),
  m_testBench(0),
  bytesTotal(0),
  bytesSentTotal(0),
  bytesOnce(0),
  packetsReceived(0),
  rttTotal(0),
  rttCount(0),
  UdpStartTime(0),
  m_mobility(false)
{
  animFile = "waypoint/waypoint-animation.xml" ;  // Name of file for animation output
  m_wifiPhyStats = CreateObject<WifiPhyStats> ();
  rate = "100kbps";
}

//导入参数
bool
WayPointTest::Configure (int argc, char **argv)
{
  // Enable AODV logs by default. Comment this if too noisy
  // LogComponentEnable("AodvRoutingProtocol", LOG_LEVEL_ALL);

  CommandLine cmd;
  cmd.AddValue ("pcap", "Write PCAP traces.", pcap);
  cmd.AddValue ("printRoutes", "Print routing table dumps.", printRoutes);
  cmd.AddValue ("size", "Number of nodes.", node_size);
  cmd.AddValue ("time", "Simulation time, s.", totalTime);
  cmd.AddValue ("step", "Grid step, m", step);
  cmd.AddValue ("speed","speed of node, m/s",m_speed);
  cmd.AddValue ("protocol", "0=POLSR;1=OLSR;2=AODV;3=DSDV.", m_protocol);
  cmd.AddValue ("seed", "the seed of the Random module.",m_seed);
  cmd.AddValue ("test", "0=Delay;1=Throughput.",m_testBench);
  cmd.AddValue ("mobility","false=random;true=cycle",m_mobility);
  cmd.Parse (argc, argv);
  return true;
}

void
WayPointTest::Run ()
{
//  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", UintegerValue (1)); // enable rts cts all the time.
  SeedManager::SetSeed (m_seed);
  CreateNodes ();
  CreateDevices ();
  InstallInternetStack ();
  //InstallApplications ();
  switch(m_testBench){
    case 0:InstallDelayApplications();break;
    case 1:InstallThoughtApplications();break;
    default :exit(0);
  }
  //InstallThoughtApplications();
  //InstallDelayApplications();
  std::cout << "Starting simulation for " << totalTime << " s ...\n";
  //CheckThroughput();
  Simulator::Stop (Seconds (totalTime));
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();
  // Create the animation object and configure for specified output
  /*AnimationInterface anim (animFile);
  anim.SetMaxPktsPerTraceFile(5000000);
  anim.EnablePacketMetadata (); // Optional
  anim.EnableIpv4L3ProtocolCounters (Seconds (0), Seconds (totalTime)); // Optional
  anim.EnableIpv4RouteTracking ("waypoint/waypoint-routingtable.xml", Seconds (0), Seconds (totalTime), Seconds (0.25));
  */
  Simulator::Run ();
  std::cout << "Animation Trace file created:" << animFile.c_str ()<< std::endl;
  std::string csv_name;
  if(m_mobility){
    csv_name = "waypoint/waypoint_throught_random.csv";
  }else{
    csv_name = "waypoint/waypoint_throught.csv";
  }
  FlowMonitor_Print(flowmon,monitor,m_protocolName,m_speed,csv_name);
  Simulator::Destroy ();
}

void
WayPointTest::Report (std::ostream &)
{ 
}

//原点x 原点y 半径 速度 开始角度
void WayPointTest::InstallPositon(Ptr<WaypointMobilityModel> mob,
      double mid_x,double mid_y,double range, double speed,double start_w){
  // Waypoint added at time 0 will override initial position
  double pi = 3.14;
  double length = 2*pi*range;//周长
  double t_sum = length/speed;//一圈时长
  //std::cout<<"speed:"<<speed<<std::endl;
  double t_step = 2*pi/t_sum;
  //一圈100*i
  for(int i=0;i<100*totalTime;i++){
    //周长
    double d_t = i/100.0;
    double pos_x = range*sin(t_step*d_t+start_w);
    double pos_y = range*cos(t_step*d_t+start_w);
    Waypoint wpt (Seconds (d_t), Vector(mid_x+pos_x,mid_y+pos_y,0));
    mob->AddWaypoint (wpt);
  }
}
//Z字搜索 原点x 原点y 宽度 速度 方向
/*void WayPointTest::InstallPositon(Ptr<WaypointMobilityModel> mob,
    double start_x,double start_y,double range, double speed,double start_w){
  double t_step = range/speed;
  const double n = speed*totalTime/range;
  mob->AddWaypoint(Waypoint(0, Vector(start_x,start_y,0)) );
  for(int i=0;i<n;i++){
    //周长
    double pos_x = 
    double pos_y = range*cos(t_step*d_t+start_w);
    Waypoint wpt (Seconds (t_step), Vector(mid_x+mid_y+pos_x,pos_y,0));
    mob->AddWaypoint (wpt);
  }
}*/
void
WayPointTest::CreateNodes ()
{
  std::cout << "Creating " << (unsigned)node_size << " nodes " << step << " m apart.\n";
  nodes.Create (node_size);
  // Name nodes
  for (uint32_t i = 0; i < node_size; ++i)
    {
      std::ostringstream os;
      os << "node-" << i;
      Names::Add (os.str (), nodes.Get (i));
    }
  if(m_mobility){
    InstallRandomMobility();
  }else{
    InstallScanMobility();
  }
}
void
WayPointTest::InstallRandomMobility()
{
  std::cout<<"InstallRandomMobility"<<std::endl;
  MobilityHelper mobility;
  int64_t streamIndex = 0; // used to get consistent mobility across scenarios
  ObjectFactory pos;
  pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
  std::stringstream xsize,ysize;
  xsize<<"ns3::UniformRandomVariable[Min=-2000.0|Max="<<"2000"<<"]";
  pos.Set ("X", StringValue (xsize.str()));
  ysize<<"ns3::UniformRandomVariable[Min=-2000.0|Max="<<"2000"<<"]";
  pos.Set ("Y", StringValue (ysize.str()));

  Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
  streamIndex += taPositionAlloc->AssignStreams (streamIndex);

  std::stringstream ssSpeed;
  ssSpeed << "ns3::UniformRandomVariable[Min="<<m_speed/2<<"|Max=" << m_speed+0.0001 << "]";
  std::stringstream ssPause;
  ssPause << "ns3::ConstantRandomVariable[Constant=" << 0 << "]";
  mobility.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                "Speed", StringValue (ssSpeed.str ()),
                                "Pause", StringValue (ssPause.str ()),
                                "PositionAllocator", PointerValue (taPositionAlloc));
  mobility.SetPositionAllocator (taPositionAlloc);
  mobility.Install (nodes);
  streamIndex += mobility.AssignStreams (nodes, streamIndex);  
}
void
WayPointTest::InstallScanMobility()
{
  std::cout<<"InstallScanMobility"<<std::endl;
  // Create static grid
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::WaypointMobilityModel",
                             "InitialPositionIsWaypoint", BooleanValue (false),
                             "LazyNotify", BooleanValue (false));
                             
  mobility.Install (nodes);
  // Get back a pointer to this
  for(int i=0;i<3;i++){
      {
        Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
        double speed_r = var->GetValue (0,m_speed);
        double pos = var->GetValue (0,2*3.14);
        Ptr<WaypointMobilityModel> mob = nodes.Get (4*i+0)->GetObject<WaypointMobilityModel> ();
        InstallPositon(mob,-1280+-640+i*1280,0,640,speed_r,pos);
      }
      {
        Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
        double speed_r = var->GetValue (0,m_speed);
        double pos = var->GetValue (0,2*3.14);
        Ptr<WaypointMobilityModel> mob = nodes.Get (4*i+1)->GetObject<WaypointMobilityModel> ();
        InstallPositon(mob,-1280+i*1280,-640,640,speed_r,pos);
      }
      {
        Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
        double speed_r = var->GetValue (0,m_speed);
        double pos = var->GetValue (0,2*3.14);
        Ptr<WaypointMobilityModel> mob = nodes.Get (4*i+2)->GetObject<WaypointMobilityModel> ();
        InstallPositon(mob,-1280+640+i*1280,0,640,speed_r,pos);
      }
      {
        Ptr<WaypointMobilityModel> mob = nodes.Get (4*i+3)->GetObject<WaypointMobilityModel> ();
        InstallPositon(mob,-1280+i*1280,0,640,0,0);
      }
  }
  {
    Ptr<WaypointMobilityModel> mob = nodes.Get (node_size-1)->GetObject<WaypointMobilityModel> ();
    InstallPositon(mob,-1280+2*1280,1280/2,640,0,0);
  }
}
void
WayPointTest::CreateDevices ()
{
  /*NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  wifiMac.SetType ("ns3::AdhocWifiMac");
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  wifiPhy.SetChannel (wifiChannel.Create ());
  WifiHelper wifi = WifiHelper::Default ();
  wifi.SetStandard ( WIFI_PHY_STANDARD_80211n_2_4GHZ);
  //wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue ("OfdmRate6Mbps"), "RtsCtsThreshold", UintegerValue (0));
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode",StringValue("DsssRate1Mbps"),"ControlMode",StringValue ("DsssRate1Mbps"));*/
  // setting up wifi phy and channel using helpers
  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211n_5GHZ);

  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
  wifiPhy.SetChannel (wifiChannel.Create ());
  wifiPhy.Set ("RxGain", DoubleValue (+3) ); 
  wifiPhy.Set ("TxGain", DoubleValue (+3) ); 
  // Add a non-QoS upper mac, and disable rate control
  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode",
    StringValue ("OfdmRate6Mbps"));

  wifiMac.SetType ("ns3::AdhocWifiMac");
  
  devices = wifi.Install (wifiPhy, wifiMac, nodes); 

  if (pcap)
    {
      wifiPhy.EnablePcapAll (std::string ("aodv"));
    }
}

void
WayPointTest::InstallInternetStack ()
{
  // you can configure AODV attributes here using aodv.Set(name, value)
  InternetStackHelper stack;
  switch (m_protocol)
  {
    case 0:
      polsr.Set("HelloInterval",TimeValue (Seconds (1)));
      stack.SetRoutingHelper (polsr); // has effect on the next Install ()
      m_protocolName = "POLSR";
      break;
    case 1:
      olsr.Set("HelloInterval",TimeValue (Seconds (1)));
      stack.SetRoutingHelper (olsr);
      m_protocolName = "OLSR";
      break;
    case 2:
      stack.SetRoutingHelper (aodv);
      m_protocolName = "AODV";
      break;
    case 3:
      stack.SetRoutingHelper (dsdv);
      m_protocolName = "DSDV";
      break;
    default:
      NS_FATAL_ERROR ("No such protocol:" << m_protocol);
  }
  
  stack.Install (nodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.0.0.0", "255.0.0.0");
  interfaces = address.Assign (devices);

  if (printRoutes)
    {
      Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("aodv.routes", std::ios::out);
      aodv.PrintRoutingTableAllAt (Seconds (8), routingStream);
    }
}

void
WayPointTest::InstallApplications ()
{
  //ping 发送ip
  V4PingHelper ping (interfaces.GetAddress (node_size - 1));
  //显示ping信息
  ping.SetAttribute ("Verbose", BooleanValue (true));
  //ping 发送目标节点
  ApplicationContainer p = ping.Install (nodes.Get (0));
  p.Start (Seconds (0));
  p.Stop (Seconds (totalTime) - Seconds (0.001));
  
  
  // move a node away
  Ptr<Node> node = nodes.Get (node_size/2);
  Ptr<MobilityModel> mob = node->GetObject<MobilityModel> ();
  //在一半的时候 将mob 节点 移动至远处
  //Simulator::Schedule (Seconds (totalTime/3), &MobilityModel::SetPosition, mob, Vector (1e5, 1e5, 1e5));
}

void 
WayPointTest::InstallThoughtApplications(){
	// Create Apps
  uint16_t sinkPort = 6; // use the same for all apps
  // UDP connection from N0 to end  nodes
    Address sinkAddress(InetSocketAddress (interfaces.GetAddress (node_size-1), sinkPort)); // interface of n24
    PacketSinkHelper packetSinkHelper1 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), sinkPort));
    ApplicationContainer sinkApps1 = packetSinkHelper1.Install (nodes.Get (1)); //n2 as sink
    
    sinkApps1.Start (Seconds (0.));
    sinkApps1.Stop (Seconds (totalTime));

    Ptr<Socket> sink = Socket::CreateSocket (nodes.Get (1), UdpSocketFactory::GetTypeId ()); //source at n0

    // Create UDP application at n0
    Ptr<TestBench> app1 = CreateObject<TestBench> ();
    app1->Setup (sink, sinkAddress, 600, 10000000, DataRate ("1Mbps"));
    nodes.Get (node_size-1)->AddApplication (app1);
    
    Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
    UdpStartTime = var->GetValue (totalTime/5,totalTime/5+1);
    app1->SetStartTime (Seconds (UdpStartTime));
    app1->SetStopTime (Seconds (totalTime));
}

void 
WayPointTest::InstallDelayApplications(){
  Config::SetDefault  ("ns3::OnOffApplication::PacketSize",StringValue ("1024"));
  Config::SetDefault ("ns3::OnOffApplication::DataRate",  StringValue (rate));
  OnOffHelper onoff1 ("ns3::UdpSocketFactory",Address ());
  onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
  // Create Apps
  uint16_t sinkPort = 6; // use the same for all apps
  //安装packet接收器
  Ptr<Socket> sink = SetupPacketReceive (interfaces.GetAddress (1), nodes.Get (1),sinkPort);
  //设置远程地址为node[i]
  AddressValue remoteAddress (InetSocketAddress (interfaces.GetAddress(1), sinkPort));
  onoff1.SetAttribute ("Remote", remoteAddress);
  //得到随机起始时间 
  Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
  UdpStartTime = var->GetValue (totalTime/5,totalTime/5+1);
  //安装onoff socket 为另一侧i + m_nSinks
  ApplicationContainer temp = onoff1.Install (nodes.Get (node_size-1));
  temp.Start (Seconds (UdpStartTime));
  temp.Stop (Seconds (totalTime));
  Config::ConnectWithoutContext("/NodeList/*/ApplicationList/*/$ns3::OnOffApplication/Tx",
               MakeCallback (&WayPointTest::SentPacket, this));
  Simulator::Schedule (Seconds (UdpStartTime), &WayPointTest::CheckThroughput, this);  
}

void FlowMonitor_Print(FlowMonitorHelper &flowmon,Ptr<FlowMonitor> &monitor,
  std::string protocolName,double speed,std::string csv_name){
	// Print per flow statistics
  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
  double total = 0;
  double max = 0;
  double count = 0;
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator iter = stats.begin (); iter != stats.end (); ++iter)
  {
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (iter->first);
    if(t.sourceAddress ==  Ipv4Address("10.0.0.2")&& t.destinationAddress == Ipv4Address("10.0.0.13"))
    {
      NS_LOG_UNCOND("Flow ID: " << iter->first << " Src Addr " << t.sourceAddress << " Dst Addr " << t.destinationAddress);
      NS_LOG_UNCOND("Tx Packets = " << iter->second.txPackets);
      NS_LOG_UNCOND("Rx Packets = " << iter->second.rxPackets);
      double throughput = iter->second.rxBytes * 8.0 / (iter->second.timeLastRxPacket.GetSeconds()-iter->second.timeFirstTxPacket.GetSeconds()) / 1024;
      NS_LOG_UNCOND("Throughput: " << throughput << " Kbps");
      total += throughput;
      if (throughput> max)
        max = throughput;
      count++;
    }
  }
  if(count){
    std::cout<<"Throughput: " << max << " Kbps"<<std::endl;
    std::ofstream out (csv_name.c_str (),std::ios::app);
    
    /*out << "protocol," <<
    "Throughput" <<
    std::endl;*/
    
    out << protocolName <<"," <<
    max <<"," <<
    speed <<
    std::endl;
    
    out.close ();
    
  }
  /*if(count){
    std::cout<<"Avr Throughput: " << total/count << " Kbps"<<std::endl;
  }*/
}
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
void WayPointTest::ReceivePacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  while ((packet = socket->Recv ()))
  {
    bytesTotal += packet->GetSize ();
    bytesOnce += packet->GetSize ();
    packetsReceived += 1;
    //std::cout<<(PrintReceivedPacket (socket, packet))<<std::endl;
    TimestampTag timestamp;
    if (packet->FindFirstMatchingByteTag (timestamp)) {
      Time tx = timestamp.GetTimestamp ();
      Time rx = Simulator::Now ();
      double d_t = (rx-tx).ToDouble(Time::MS);
      rttTotal += d_t;
      rttCount++;
    }
  }
}
//检查throughput
void WayPointTest::CheckThroughput ()
{
  double kbs = (bytesOnce * 8.0) / 1000;
  double kbs_avr = (bytesTotal*8.0)/1000/(Simulator::Now ().GetSeconds ()-UdpStartTime);
  double kbs_send = (bytesSentTotal*8.0)/1000/(Simulator::Now ().GetSeconds ()-UdpStartTime);
  double rtt = rttTotal/rttCount;
  bytesOnce = 0;
  std::cout<<"time = "<<Simulator::Now ().GetSeconds ()
            <<"\tapp rate = "<<kbs<<"kbps"
            <<"\tsend avr rate = "<<kbs_send<<"kbps"
            <<"\tavr rate = "<<kbs_avr<<"kbps"
            <<"\tdelay = "<<rtt<<"ms"<<std::endl;
            
  //std::cout<<"Node0 Pos:\t"<<
  //      m_locationService->GetPosition(Ipv4Address("10.1.1.1"))<<std::endl;
  packetsReceived = 0;
  if(totalTime-Simulator::Now ().GetSeconds ()<=1)
  {
    std::string csv_name;
    if(m_mobility){
        csv_name = "waypoint/waypoint_delay_random.csv";
    }else{
        csv_name = "waypoint/waypoint_delay.csv";
    }
    std::ofstream out_all(csv_name.c_str(), std::ios::app);
    out_all<< m_protocolName << ","
    << m_speed <<","
    << rate <<","
    << kbs_avr <<","
    << rtt<<","
    << kbs_avr/kbs_send<<""
    << std::endl;
    out_all.close ();
  }
  Simulator::Schedule (Seconds (1.0), &WayPointTest::CheckThroughput, this);  
}
//安装packet接收器
Ptr<Socket> WayPointTest::SetupPacketReceive (Ipv4Address addr, Ptr<Node> node,uint16_t sinkPort)
{
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> sink = Socket::CreateSocket (node, tid);
  InetSocketAddress local = InetSocketAddress (addr, sinkPort);
  sink->Bind (local);
  sink->SetRecvCallback (MakeCallback (&WayPointTest::ReceivePacket, this));
  return sink;
}
void WayPointTest::SentPacket (Ptr<const Packet> p)
{
  bytesSentTotal += p->GetSize();
  TimestampTag timestamp;
  timestamp.SetTimestamp (Simulator::Now ());
  p->AddByteTag (timestamp);
  //std::cout<<"time:"<<Simulator::Now ().GetSeconds ()
  //          << "Tx "<< p->GetSize()<<"Bytes\n";
	
}
