#ifndef WIFI_PHY_STATS_H_
#define WIFI_PHY_STATS_H_
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/object.h"
#include "ns3/application.h"
#include "ns3/stats-module.h"
#include "ns3/integer.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-mode.h"
#include "ns3/wifi-preamble.h"

using namespace ns3;
/**
 * \brief The WifiPhyStats class collects Wifi MAC/PHY statistics
 */
class WifiPhyStats : public Object
{
public:
  /**
   * \brief Gets the class TypeId
   * \return the class TypeId
   */
  static TypeId GetTypeId (void);

  /**
   * \brief Constructor
   * \return none
   */
  WifiPhyStats ();

  /**
   * \brief Destructor
   * \return none
   */
  virtual ~WifiPhyStats ();

  /**
   * \brief Returns the number of bytes that have been transmitted
   * (this includes MAC/PHY overhead)
   * \return the number of bytes transmitted
   */
  uint32_t GetTxBytes ();
  
  uint32_t GetRxBytes ();
  /**
   * \brief Callback signiture for Phy/Tx trace
   * \param context this object
   * \param packet packet transmitted
   * \param mode wifi mode
   * \param preamble wifi preamble
   * \param txPower transmission power
   * \return none
   */
  void PhyTxTrace (std::string context, Ptr<const Packet> packet, WifiMode mode, WifiPreamble preamble, uint8_t txPower);
  
  /**
   * \brief Callback signiture for Phy/Rx trace
   * \param context this object
   * \param packet packet transmitted
   * \param mode wifi mode
   * \param preamble wifi preamble
   * \param rxPower transmission power
   * \return none
   */
  void PhyRxTrace (std::string context, Ptr<const Packet> packet, WifiMode mode, WifiPreamble preamble, uint8_t txPower);
  
  /**
   * \brief Callback signiture for Phy/TxDrop
   * \param context this object
   * \param packet the tx packet being dropped
   * \return none
   */
  void PhyTxDrop (std::string context, Ptr<const Packet> packet);

  /**
   * \brief Callback signiture for Phy/RxDrop
   * \param context this object
   * \param packet the rx packet being dropped
   * \return none
   */
  void PhyRxDrop (std::string context, Ptr<const Packet> packet);

private:
  uint32_t m_phyTxPkts;
  uint32_t m_phyTxBytes;
  uint32_t m_phyRxPkts;
  uint32_t m_phyRxBytes;
};
#endif
