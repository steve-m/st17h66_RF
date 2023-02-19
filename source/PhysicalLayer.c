
#include <stdint.h>

#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "clock.h"
#include "flash.h"
#include "global_config.h"
#include "gpio.h"
#include "ll.h"
#include "log.h"
#include "ll_hw_drv.h"
#include "log.h"
#include "rf_phy_driver.h"
#include "string.h"
#include "timer.h"

#include "PhysicalLayer.h"
#include "Zigbee.h"


#define LL_HW_MODE_STX (0x00)
#define LL_HW_MODE_SRX (0x01)
#define LL_HW_MODE_TRX (0x02)


#define PHYSICAL_LAYER_HW_BB_DELAY (90)
#define PHYSICAL_LAYER_HW_AFE_DELAY (8)
#define PHYSICAL_LAYER_HW_PLL_DELAY (60)


// Is initialized in InitializePhysicalLayer
uint8_t g_physicalLayerTaskId = 0;


struct PhysicalLayerInformation g_physicalLayerInformation;


struct PacketInformation g_currentPacketInformation;

// Information regarding interrupt requests from the physical layer.
extern uint8 ll_hw_get_tr_mode(void);
extern volatile uint32 llWaitingIrq;



void PhysicalLayerInterruptRequestHandler(void);

static uint8_t checkReceivedData(void);

static void onPacketReceived(struct PacketInformation* packet);
static void onProcessTxDoneEvent(void);
static void onProcessRxDoneEvent(void);


void PhysicalLayerSendRawPacket(struct PacketInformation* packet);

void SetPhysicalHardwareSRX(uint16_t rxTimeoutUs);


void InitializePhysicalLayer(uint8_t task_id) {
  LOG("Initializing physical layer");
  g_physicalLayerTaskId = task_id;
  // set phy irq handeler

  g_physicalLayerInformation.txIntervalMs = 500; // ms

  //Zigbee specific info
  g_physicalLayerInformation.channel = 12;
  g_physicalLayerInformation.nodeAddress = (uint16_t)0x12345987;
  g_physicalLayerInformation.panId = (uint16_t)6754;

  g_currentPacketInformation.receivedFrequencyOffset = 0;
  g_currentPacketInformation.CarrSens = 0;
  g_currentPacketInformation.receivedSignalStrength = 0;

  JUMP_FUNCTION(V4_IRQ_HANDLER) = (uint32_t)&PhysicalLayerInterruptRequestHandler;

  ZigbeeSetChannel(g_physicalLayerInformation.channel);
  ZigbeeSetupHardwareTiming();

  // Start in "idle" mode
  g_physicalLayerInformation.Status = PHYSICAL_LAYER_IDLE;


  LOG("Starting initial task");
  // Add an interrupt request to the task
  VOID osal_start_timerEx(g_physicalLayerTaskId, PPP_PERIODIC_TX_EVT, 1000);
}


void PhysicalLayerInterruptRequestHandler(void)
{
  LOG("PhysicalLayerInterruptRequestHandler called");

  uint8 mode;
  uint32_t irq_status = ll_hw_get_irq_status();

  if (!(irq_status & LIRQ_MD)) // only process IRQ of MODE DONE
  {
    ll_hw_clr_irq(); // clear irq status
    return;
  }

  llWaitingIrq = FALSE;
  HAL_ENTER_CRITICAL_SECTION();

  mode = ll_hw_get_tr_mode();

  // ===================   mode TRX process 1
  if (mode == LL_HW_MODE_STX && (g_physicalLayerInformation.Status == PHYSICAL_LAYER_TX_ONLY))
  {

      LOG("Transmission done!");
    /// Set a new event that the transmission is done
    osal_set_event(g_physicalLayerTaskId, PPP_TX_DONE_EVT);

  }else if ((mode == LL_HW_MODE_SRX && (g_physicalLayerInformation.Status == PHYSICAL_LAYER_RX_ONLY)) || (mode == LL_HW_MODE_TRX && (g_physicalLayerInformation.Status == PHYSICAL_LAYER_TRX_ONLY)))
  {

    // Get information about the packet
    rf_phy_get_pktFoot(&g_currentPacketInformation.receivedSignalStrength, &g_currentPacketInformation.receivedFrequencyOffset, &g_currentPacketInformation.CarrSens);

    if (g_currentPacketInformation.crcFormat == LL_HW_CRC_NULL)
    {
      if (0 == (irq_status & LIRQ_RTO))
      {
        uint16_t pktLen;
        uint32_t pktFoot0, pktFoot1;
        ll_hw_read_rfifo_pplus(g_currentPacketInformation.bufferReceive, &pktLen, &pktFoot0, &pktFoot1);
        rf_phy_get_pktFoot_fromPkt(pktFoot0, pktFoot1, &g_currentPacketInformation, &g_currentPacketInformation.receivedFrequencyOffset, &g_currentPacketInformation.CarrSens);

        if(!checkReceivedData())
            osal_set_event(g_physicalLayerTaskId, PPP_RX_DATA_PROCESS_EVT);
      }

    } else
    {
      if (irq_status & LIRQ_COK)
      {
        uint16_t pktLen;
        uint32_t pktFoot0, pktFoot1;
        ll_hw_read_rfifo(g_currentPacketInformation.bufferReceive, &pktLen, &pktFoot0, &pktFoot1);
        rf_phy_get_pktFoot_fromPkt(pktFoot0, pktFoot1, &g_currentPacketInformation, &g_currentPacketInformation.receivedFrequencyOffset,
                                   &g_currentPacketInformation.CarrSens);
        if(!checkReceivedData())
            osal_set_event(g_physicalLayerTaskId, PPP_RX_DATA_PROCESS_EVT);
      }
    }

      // Set a new event that receiving is done.
      if(mode == LL_HW_MODE_SRX)
          osal_set_event(g_physicalLayerTaskId, PPP_RX_DONE_EVT);
      else // mode == LL_HW_MODE_TRX
        osal_set_event(g_physicalLayerTaskId, PPP_TRX_DONE_EVT);
  }else
  {
      LOG("Other interrupt");
  }


  // post ISR process
  ll_hw_clr_irq();
  HAL_EXIT_CRITICAL_SECTION();
}

uint16_t ProcessPhysicalLayerEvents( uint8_t taskId, uint16_t events )
{
  VOID taskId;

  if (events & PPP_PERIODIC_TX_EVT)
  {
    // If the physical layer is idle, make transition to state "send packet"
    if (g_physicalLayerInformation.Status == PHYSICAL_LAYER_IDLE)
    {


          g_physicalLayerInformation.Status = PHYSICAL_LAYER_TX_ONLY;

          ZigbeeSetChannel(g_physicalLayerInformation.channel);
          prepareZigbeeBroadcastPacket(&g_physicalLayerInformation, &g_currentPacketInformation);
          PhysicalLayerSendRawPacket(&g_currentPacketInformation);

         dbg_printf("For some reason I need this here");
          osal_start_timerEx(g_physicalLayerTaskId, PPP_PERIODIC_TX_EVT, g_physicalLayerInformation.txIntervalMs);

    } else
    {
      // The physical layer isn't idle, so we send the packet again after 20ms
      LOG("Send again");
      /// Send the event again.
      osal_start_timerEx(g_physicalLayerTaskId, PPP_PERIODIC_TX_EVT, 2000);
    }

    return (events ^ PPP_PERIODIC_TX_EVT);
  }

  if (events & PPP_PERIODIC_RX_EVT)
  {
    if (g_physicalLayerInformation.Status == PHYSICAL_LAYER_IDLE)
    {
      g_physicalLayerInformation.Status = PHYSICAL_LAYER_RX_ONLY;
      //TODO
      //g_physicalLayerInformation.channel = BLE_ADV_CHN37;
      //s_phy.rxScanT0 = read_current_fine_time();
      //s_pktCfg.whitenSeed = WHITEN_SEED_CH37;
      // rf_rx(phyBufTxLen);
      osal_start_timerEx(g_physicalLayerTaskId, PPP_PERIODIC_RX_EVT, g_physicalLayerInformation.rxIntervalMs);
    } else
    {
      // Try again after 20ms
      osal_start_timerEx(g_physicalLayerTaskId, PPP_PERIODIC_RX_EVT, 20);
    }

    return (events ^ PPP_PERIODIC_RX_EVT);
  }

  if (events & PPP_RX_DATA_PROCESS_EVT)
  {
    onPacketReceived(&g_currentPacketInformation);
    return (events ^ PPP_RX_DATA_PROCESS_EVT);
  }

  if (events & PPP_TX_DONE_EVT)
  {
    onProcessTxDoneEvent();
    return (events ^ PPP_TX_DONE_EVT);
  }

  if (events & PPP_RX_DONE_EVT)
  {
    onProcessRxDoneEvent();
    return (events ^ PPP_RX_DONE_EVT);
  }

  if (events & PPP_TRX_DONE_EVT)
  {
    // TODO
    return (events ^ PPP_TRX_DONE_EVT);
  }

  return 0;
}

void SetPhysicalHardwareSRX(uint16_t rxTimeoutUs)
{
    if (g_physicalLayerInformation.Status == PHYSICAL_LAYER_RX_ONLY || g_physicalLayerInformation.Status == PHYSICAL_LAYER_RX_TXACK)
    {
        return;
    }
    else if (g_physicalLayerInformation.Status == PHYSICAL_LAYER_TX_ONLY)
    {
        // not ok in irq zb_hw_stop(); // if in tx state, abort the tx first
    }
    g_physicalLayerInformation.Status = PHYSICAL_LAYER_RX_ONLY;
    ll_hw_set_rx_timeout(rxTimeoutUs);
    ll_hw_set_srx();
    ll_hw_set_trx_settle(PHYSICAL_LAYER_HW_BB_DELAY, PHYSICAL_LAYER_HW_AFE_DELAY, PHYSICAL_LAYER_HW_PLL_DELAY);

    // reset Rx/Tx FIFO
    ll_hw_rst_rfifo();
    ll_hw_rst_tfifo();
    ll_hw_go();

}

void SetPhysicalHardwareSingleTX(void )
{
    g_physicalLayerInformation.Status = PHYSICAL_LAYER_TX_ONLY;
    ll_hw_rst_rfifo();
    ll_hw_rst_tfifo();


    ll_hw_set_stx();
    ll_hw_set_trx_settle(PHYSICAL_LAYER_HW_BB_DELAY, PHYSICAL_LAYER_HW_AFE_DELAY, PHYSICAL_LAYER_HW_PLL_DELAY); // RxAFE,PLL

}



static void onPacketReceived(struct PacketInformation* packet)
{
    uint8_t bufferLength = packet->bufferReceive[0];
    uint8_t noidea = packet->bufferReceive[1]; //TODO
    uint8_t* macAddrReverse = &packet->bufferReceive[2];
    uint8_t* packetData = macAddrReverse + 6;
    uint8_t packetLength = packetData[0];
    uint8_t packetType = packetData[1];
    uint8_t* packetPayload = packetData + 2;
    if(packetType == 0x09 && packetPayload[0] == 0x47)
    {
        /*
          if (!memcmp("Galaxy Tab S4", packetPayload, 13))
              switch_mosfet();
              */
    }
}


static void onProcessTxDoneEvent(void)
{
  g_physicalLayerInformation.Status = PHYSICAL_LAYER_IDLE;
}

static void onProcessRxDoneEvent(void)
{
  g_physicalLayerInformation.Status = PHYSICAL_LAYER_IDLE;
}


static uint8_t checkReceivedData(void)
{
  if (g_physicalLayerInformation.Status == PHYSICAL_LAYER_RX_TXACK)
  {
    // process data check
    return 1;
  }

  return 0;
}

void PhysicalLayerSendRawPacket(struct PacketInformation* packet)
{

    LOG("Sending packet of length %d", packet->bufferTransmitLength);
    SetPhysicalHardwareSingleTX();

     ll_hw_write_tfifo(&packet->bufferTransmit[0], packet->bufferTransmitLength);
     ll_hw_go();

     uint32_t newmode = ll_hw_get_tr_mode();
    // info1("rf : %d md:%d",osKernelGetTickCount(), (int)newmode);
    if (LL_HW_MODE_STX != newmode)
    {
        LOG("mode: %d != %d", (int)newmode, (int)LL_HW_MODE_STX);
    }

}

int app_main(void) {
  // debug_blink(2);
  /* Initialize the operating system */
  osal_init_system();
  osal_pwrmgr_device(PWRMGR_BATTERY);
  /* Start OSAL */
  osal_start_system(); // No Return from here
  return 0;
}


