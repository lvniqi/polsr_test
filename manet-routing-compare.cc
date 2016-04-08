#include "ns3/config-store-module.h"
#include "ns3/olsr-helper.h"
#include "ns3/flow-monitor-module.h"

#include <vector>
#include <string>

#include "TestBench.h"

#include <fstream>
#include <iostream>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/aodv-module.h"
#include "ns3/polsr-module.h"
#include "ns3/olsr-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"
#include "ns3/nstime.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/god.h"

#include "TestBench.h"
#include "WifiPhyStats.h"
#include "TimestampTag.h"
#include "routing-experiment.h"
using namespace ns3;
using namespace dsr;

NS_LOG_COMPONENT_DEFINE ("manet-routing-compare");

void FlowMonitor_Print(FlowMonitorHelper &flowmon,Ptr<FlowMonitor> &monitor){
	// Print per flow statistics
  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
  double total = 0;
  double count = 0;
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator iter = stats.begin (); iter != stats.end (); ++iter)
  {
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (iter->first);
    if(t.sourceAddress ==  Ipv4Address("10.1.1.1"))
    {
      NS_LOG_UNCOND("Flow ID: " << iter->first << " Src Addr " << t.sourceAddress << " Dst Addr " << t.destinationAddress);
      NS_LOG_UNCOND("Tx Packets = " << iter->second.txPackets);
      NS_LOG_UNCOND("Rx Packets = " << iter->second.rxPackets);
      double throughput = iter->second.rxBytes * 8.0 / (iter->second.timeLastRxPacket.GetSeconds()-iter->second.timeFirstTxPacket.GetSeconds()) / 1024;
      NS_LOG_UNCOND("Throughput: " << throughput << " Kbps");
      total += throughput;
      count++;
    }
  }
  if(count){
    std::cout<<"Avr Throughput: " << total/count << " Kbps"<<std::endl;
  }
}

int main (int argc, char *argv[])
{
  RoutingExperiment experiment;
  std::string CSVfileName = experiment.CommandSetup (argc,argv);
  //blank out the last output file and write the column headers
  std::ofstream out (CSVfileName.c_str ());
  out << "SimulationSecond," <<
  "ReceiveRate," <<
  "PacketsReceived," <<
  "NumberOfSinks," <<
  "RoutingProtocol," <<
  "TransmissionPower" <<
  std::endl;
  out.close ();

  experiment.Setup (CSVfileName);
  
  // Install FlowMonitor on all nodes
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();
  
  Simulator::Run ();
  
  FlowMonitor_Print(flowmon,monitor);
  
  Simulator::Destroy ();
}
void RoutingExperiment::SentPacket (Ptr<const Packet> p)
{
  bytesSentTotal += p->GetSize();
  TimestampTag timestamp;
  timestamp.SetTimestamp (Simulator::Now ());
  p->AddByteTag (timestamp);
  //std::cout<<"time:"<<Simulator::Now ().GetSeconds ()
  //          << "Tx "<< p->GetSize()<<"Bytes\n";
	
}
void RoutingExperiment::Setup (std::string CSVfileName)
{
  Packet::EnablePrinting ();
  m_CSVfileName = CSVfileName;
  //物理层模式
  std::string phyMode ("OfdmRate6Mbps");
  std::string tr_name ("manet-routing-compare");
  int nodePause = 0; //in s
  m_protocolName = "protocol";

  Config::SetDefault  ("ns3::OnOffApplication::PacketSize",StringValue ("1024"));
  Config::SetDefault ("ns3::OnOffApplication::DataRate",  StringValue (rate));
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue (phyMode));
  //Set Non-unicastMode rate to unicast mode
  //Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",StringValue (phyMode));
	
  NodeContainer adhocNodes;
  adhocNodes.Create (m_nWifis);

  // setting up wifi phy and channel using helpers
  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211n_5GHZ);

  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
  wifiPhy.SetChannel (wifiChannel.Create ());
  wifiPhy.Set ("RxGain", DoubleValue (-10) ); 
  wifiPhy.Set ("TxGain", DoubleValue (-5) ); 
  // Add a non-QoS upper mac, and disable rate control
  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode",
    StringValue (phyMode));

  wifiMac.SetType ("ns3::AdhocWifiMac");
  NetDeviceContainer adhocDevices = wifi.Install (wifiPhy, wifiMac, adhocNodes);
  // the MAC/PHY overhead beyond the app-data
  Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/Tx", MakeCallback (&WifiPhyStats::PhyTxTrace, m_wifiPhyStats));
  // TxDrop, RxDrop not working yet.  Not sure what I'm doing wrong.
  Config::Connect ("/NodeList/*/DeviceList/*/ns3::WifiNetDevice/Phy/PhyTxDrop", MakeCallback (&WifiPhyStats::PhyTxDrop, m_wifiPhyStats));
  Config::Connect ("/NodeList/*/DeviceList/*/ns3::WifiNetDevice/Phy/PhyRxDrop", MakeCallback (&WifiPhyStats::PhyRxDrop, m_wifiPhyStats));
  
  SetMobility(adhocNodes,m_nodeSpeed,nodePause);

  AodvHelper aodv;
  OlsrHelper olsr;
  POlsrHelper polsr;
  DsdvHelper dsdv;
  DsrHelper dsr;
  DsrMainHelper dsrMain;
  InternetStackHelper internet;

  switch (m_protocol)
    {
    case 0:
    	internet.SetRoutingHelper (polsr);
      m_protocolName = "POLSR";
      break;
    case 1:
      internet.SetRoutingHelper (olsr);
      m_protocolName = "OLSR";
      break;
    case 2:
      internet.SetRoutingHelper (aodv);
      m_protocolName = "AODV";
      break;
    case 3:
      internet.SetRoutingHelper (dsdv);
      m_protocolName = "DSDV";
      break;
    case 4:
      m_protocolName = "DSR";
      break;
    default:
      NS_FATAL_ERROR ("No such protocol:" << m_protocol);
    }
	std::cout<<m_protocolName<<std::endl;
  internet.Install (adhocNodes);
  if (m_protocol == 4)
    {
      dsrMain.Install (dsr, adhocNodes);
    }

  NS_LOG_INFO ("assigning ip address");

  Ipv4AddressHelper addressAdhoc;
  addressAdhoc.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer adhocInterfaces;
  adhocInterfaces = addressAdhoc.Assign (adhocDevices);

  OnOffHelper onoff1 ("ns3::UdpSocketFactory",Address ());
  onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
  
  double UdpStartTime_sum = 0;
  for (int i = 0; i < m_nSinks; i++)
  {
    //安装packet接收器
    Ptr<Socket> sink = SetupPacketReceive (adhocInterfaces.GetAddress (i), adhocNodes.Get (i));
    //设置远程地址为node[i]
    AddressValue remoteAddress (InetSocketAddress (adhocInterfaces.GetAddress (i), port));
    onoff1.SetAttribute ("Remote", remoteAddress);
    //得到随机起始时间 
    Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
    double UdpStartTime_t = var->GetValue (UdpStartTime,UdpStartTime+1);
    //安装onoff socket 为另一侧i + m_nSinks
    ApplicationContainer temp = onoff1.Install (adhocNodes.Get (i + m_nSinks));
    temp.Start (Seconds (UdpStartTime_t));
    temp.Stop (Seconds (TotalTime));
    UdpStartTime_sum += UdpStartTime_t;
  }
  if(m_nSinks){
    UdpStartTime = UdpStartTime_sum/m_nSinks;
  }
  
  Config::ConnectWithoutContext("/NodeList/*/ApplicationList/*/$ns3::OnOffApplication/Tx",
                 MakeCallback (&RoutingExperiment::SentPacket, this));
  /*
  std::stringstream ss;
  ss << m_nWifis;
  std::string nodes = ss.str ();

  std::stringstream ss2;
  ss2 << m_nodeSpeed;
  std::string sm_nodeSpeed = ss2.str ();

  std::stringstream ss3;
  ss3 << nodePause;
  std::string sNodePause = ss3.str ();

  std::stringstream ss4;
  ss4 << rate;
  std::string sRate = ss4.str ();
  */

  //NS_LOG_INFO ("Configure Tracing.");
  //tr_name = tr_name + "_" + m_protocolName +"_" + nodes + "nodes_" + sm_nodeSpeed + "speed_" + sNodePause + "pause_" + sRate + "rate";

  //AsciiTraceHelper ascii;
  //Ptr<OutputStreamWrapper> osw = ascii.CreateFileStream ( (tr_name + ".tr").c_str());
  //wifiPhy.EnableAsciiAll (osw);
  //AsciiTraceHelper ascii;
  //MobilityHelper::EnableAsciiAll (ascii.CreateFileStream (tr_name + ".mob"));

  //Ptr<FlowMonitor> flowmon;
  //FlowMonitorHelper flowmonHelper;
  //flowmon = flowmonHelper.InstallAll ();
  //flowmon->SerializeToXmlFile ((tr_name + ".flowmon").c_str(), false, false);

  NS_LOG_INFO ("Run Simulation.");
  
  /*// Create Apps
  uint16_t sinkPort = 6; // use the same for all apps
  // UDP connection from N0 to all other nodes
  for(int i=1;i<m_nWifis;i++){  
    Address sinkAddress(InetSocketAddress (adhocInterfaces.GetAddress (i), sinkPort)); // interface of n24
    PacketSinkHelper packetSinkHelper1 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), sinkPort));
    ApplicationContainer sinkApps1 = packetSinkHelper1.Install (adhocNodes.Get (i)); //n2 as sink
    
    sinkApps1.Start (Seconds (0.));
    sinkApps1.Stop (Seconds (100.));

    Ptr<Socket> sink = Socket::CreateSocket (adhocNodes.Get (0), UdpSocketFactory::GetTypeId ()); //source at n0

    // Create UDP application at n0
    Ptr<TestBench> app1 = CreateObject<TestBench> ();
    app1->Setup (sink, sinkAddress, 600, 10000000, DataRate ("1Mbps"));
    adhocNodes.Get (0)->AddApplication (app1);
    
    Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
    double start_time = var->GetValue (0,1);
    
    app1->SetStartTime (Seconds (start_time));
    app1->SetStopTime (Seconds (100.));
  }*/
  Simulator::Schedule (Seconds (UdpStartTime+1), &RoutingExperiment::CheckThroughput, this);
  Simulator::Stop (Seconds (TotalTime));
}

