#ifndef TEST_BENCH_H_
#define TEST_BENCH_H_
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
using namespace ns3;
class TestBench : public Application
{
public:

  TestBench ();
  virtual ~TestBench();

  void Setup (Ptr<Socket> socket, Address address, uint32_t packetSize, uint32_t nPackets, DataRate dataRate);

private:
  virtual void StartApplication (void);
  virtual void StopApplication (void);

  void ScheduleTx (void);
  void SendPacket (void);

  Ptr<Socket>     m_socket;
  Address         m_peer;
  uint32_t        m_packetSize;
  uint32_t        packet_count;
  DataRate        data_rate;
  EventId         send_event;
  bool            is_running;
  uint32_t        m_packetsSent;
};

TestBench::TestBench ()
  : m_socket (0),
    m_peer (),
    m_packetSize (0),
    packet_count (0),
    data_rate (0),
    send_event (),
    is_running (false),
    m_packetsSent (0)
{
}

TestBench::~TestBench()
{
  m_socket = 0;
}

void
TestBench::Setup (Ptr<Socket> socket, Address address, uint32_t packetSize, uint32_t nPackets, DataRate dataRate)
{
  m_socket = socket;
  m_peer = address;
  m_packetSize = packetSize;
  packet_count = nPackets;
  data_rate = dataRate;
}

void
TestBench::StartApplication (void)
{
  is_running = true;
  m_packetsSent = 0;
  m_socket->Bind ();
  m_socket->Connect (m_peer);
  SendPacket ();
}

void TestBench::StopApplication (void){
  is_running = false;
  if (send_event.IsRunning ()){
    Simulator::Cancel (send_event);
  }
  if (m_socket){
    m_socket->Close ();
  }
}

void TestBench::SendPacket (void){
  Ptr<Packet> packet = Create<Packet> (m_packetSize);
  m_socket->Send (packet);
  if (++m_packetsSent < packet_count){
    ScheduleTx ();
  }
}

void TestBench::ScheduleTx (void){
  if (is_running){
    Time tNext (Seconds (m_packetSize * 8 / static_cast<double> (data_rate.GetBitRate ())));
    send_event = Simulator::Schedule (tNext, &TestBench::SendPacket, this);
  }
}
#endif
