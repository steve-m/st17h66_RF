
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
void PhysicalLayerReceiveRawPacket(uint16_t rxTimeoutUs);
void RawZigbeeToPacket(struct PacketInformation *packet);

void SetPhysicalHardwareSRX(uint16_t rxTimeoutUs);


void InitializePhysicalLayer(uint8_t task_id) {
  LOG("Initializing physical layer");
  g_physicalLayerTaskId = task_id;
  // set phy irq handeler

  g_physicalLayerInformation.txIntervalMs = 500; // ms
  g_physicalLayerInformation.rxIntervalMs = 200; // ms

  //Zigbee specific info
  g_physicalLayerInformation.channel = 11;
  g_physicalLayerInformation.nodeAddress = (uint16_t)0x00;
  g_physicalLayerInformation.panId = (uint16_t)6754;
  g_physicalLayerInformation.zigbeeInterviewState = ZIGBEE_START;
  uint8_t eAddr[8] = {0x85, 0x94, 0x2d, 0x7b, 0xef, 0x38, 0xc1, 0xa4};
  memcpy(g_physicalLayerInformation.extendedAddress,eAddr, sizeof(eAddr));

  g_currentPacketInformation.receivedFrequencyOffset = 0;
  g_currentPacketInformation.CarrSens = 0;
  g_currentPacketInformation.receivedSignalStrength = 0;
  g_currentPacketInformation.currentSequenceId = 0;

  JUMP_FUNCTION(V4_IRQ_HANDLER) = (uint32_t)&PhysicalLayerInterruptRequestHandler;

  ZigbeeSetChannel(g_physicalLayerInformation.channel);
  ZigbeeSetupHardwareTiming();

  // Start in "idle" mode
  g_physicalLayerInformation.Status = PHYSICAL_LAYER_IDLE;

  /// Start Zigbee interview
  VOID osal_start_timerEx(g_physicalLayerTaskId, ZIGBEE_INTERVIEW_EVT, 1000);
}


void PhysicalLayerInterruptRequestHandler(void)
{

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

    // We want to immediate move on to listening in case we're expecting an ACK packet.
    onProcessTxDoneEvent();
    // Immediately switch to Receive mode
    osal_set_event(g_physicalLayerTaskId, RX_EVT);
    //osal_start_timerEx(g_physicalLayerTaskId, RX_EVT, 20);

  }else if ((mode == LL_HW_MODE_SRX && (g_physicalLayerInformation.Status == PHYSICAL_LAYER_RX_ONLY)) || (mode == LL_HW_MODE_TRX && (g_physicalLayerInformation.Status == PHYSICAL_LAYER_TRX_ONLY)))
  {

    // Get information about the packet
    rf_phy_get_pktFoot(&g_currentPacketInformation.receivedSignalStrength, &g_currentPacketInformation.receivedFrequencyOffset, &g_currentPacketInformation.CarrSens);

    if(irq_status & LIRQ_RTO)
    {
        // Restart
        LOG("t");
        osal_start_timerEx(g_physicalLayerTaskId, RX_EVT, 20);
    }else if(irq_status & LIRQ_COK)
    {
        // Read the zigbee packet
        //LOG("Read packet of length %d", g_currentPacketInformation.bufferReceivedByteLength);

        RawZigbeeToPacket(&g_currentPacketInformation);

        osal_set_event(g_physicalLayerTaskId, RX_DATA_PROCESS_EVT);
    }
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

  if(events & ZIGBEE_ACK_EVT)
  {
          g_physicalLayerInformation.Status = PHYSICAL_LAYER_TX_ONLY;
          ZigbeeSetChannel(g_physicalLayerInformation.channel);
          prepareZigbeeAckPacket(&g_physicalLayerInformation, &g_currentPacketInformation);
          g_physicalLayerInformation.zigbeeInterviewState++;
          PhysicalLayerSendRawPacket(&g_currentPacketInformation);

      return events ^ ZIGBEE_ACK_EVT;
  }

  if (events & TX_EVT)
  {
    // If the physical layer is idle, make transition to state "send packet"
    if (g_physicalLayerInformation.Status == PHYSICAL_LAYER_IDLE)
    {
          // Send association request
          g_physicalLayerInformation.Status = PHYSICAL_LAYER_TX_ONLY;

          ZigbeeSetChannel(g_physicalLayerInformation.channel);

          if(g_physicalLayerInformation.zigbeeInterviewState == ZIGBEE_START)
          {
              prepareZigbeeAssociationRequestPacket(&g_physicalLayerInformation, &g_currentPacketInformation);
          }else if(g_physicalLayerInformation.zigbeeInterviewState == ZIGBEE_ACK_ASSOCIATION_REQUEST)
          {
            prepareZigbeeDataRequestAfterAssociation(&g_physicalLayerInformation, &g_currentPacketInformation);
          }
          else if(g_physicalLayerInformation.zigbeeInterviewState == ZIGBEE_ASSOCIATION_RESPONSE_RECEIVED)
          {
              prepareZigbeeAckPacket(&g_physicalLayerInformation, &g_currentPacketInformation);
          }else
          {
              LOG("INVALID");
          }
          g_physicalLayerInformation.zigbeeInterviewState++;
          PhysicalLayerSendRawPacket(&g_currentPacketInformation);

          //g_physicalLayerInformation.Status = PHYSICAL_LAYER_TX_ONLY;
          //WaitRTCCount(20);


    } else
    {
        LOG("ERROR");
      // The physical layer isn't idle, so we send the packet again after 20ms
      ///LOG("Send again");
      /// Send the event again.
      //osal_start_timerEx(g_physicalLayerTaskId, TX_EVT, 2000);
    }

    return (events ^ TX_EVT);
  }

  if (events & RX_EVT)
  {
    if (g_physicalLayerInformation.Status == PHYSICAL_LAYER_IDLE)
    {
      //HAL_ENTER_CRITICAL_SECTION();
      //TODO
      //g_physicalLayerInformation.channel = BLE_ADV_CHN37;
      //s_phy.rxScanT0 = read_current_fine_time();
      //s_pktCfg.whitenSeed = WHITEN_SEED_CH37;
      //SetPhysicalHardwareSRX(0);
      PhysicalLayerReceiveRawPacket(0);
      //osal_start_timerEx(g_physicalLayerTaskId, RX_EVT, g_physicalLayerInformation.rxIntervalMs);
      //HAL_EXIT_CRITICAL_SECTION();

    } else
    {
      //LOG("HERE");
      // Try again after 20ms
      //LOG("Retrying receive");
      //osal_start_timerEx(g_physicalLayerTaskId, RX_EVT, 20);
    }

    return (events ^ RX_EVT);
  }

  if (events & RX_DATA_PROCESS_EVT)
  {
    onPacketReceived(&g_currentPacketInformation);
    return (events ^ RX_DATA_PROCESS_EVT);
  }

  if (events & TX_DONE_EVT)
  {
    onProcessTxDoneEvent();
    return (events ^ TX_DONE_EVT);
  }

  if (events & RX_DONE_EVT)
  {
    onProcessRxDoneEvent();
    return (events ^ RX_DONE_EVT);
  }

  if (events & TRX_DONE_EVT)
  {
    // TODO
    return (events ^ TRX_DONE_EVT);
  }

  if (events & ZIGBEE_INTERVIEW_EVT)
  {
      if(g_physicalLayerInformation.zigbeeInterviewState == ZIGBEE_START)
      {
          osal_start_timerEx(g_physicalLayerTaskId, TX_EVT, 20);
      }else if(g_physicalLayerInformation.zigbeeInterviewState == ZIGBEE_ACK_ASSOCIATION_REQUEST)
      {
          osal_start_timerEx(g_physicalLayerTaskId, TX_EVT, 20);
      }else if(g_physicalLayerInformation.zigbeeInterviewState == ZIGBEE_ACK_DATA_REQUEST_AFTER_ASSOCIATION)
      {
          osal_set_event(g_physicalLayerTaskId, RX_EVT);
      }else if(g_physicalLayerInformation.zigbeeInterviewState == ZIGBEE_ASSOCIATION_RESPONSE_RECEIVED)
      {
          osal_start_timerEx(g_physicalLayerTaskId, TX_EVT, 20);
      }else
      {
          LOG("UNKNOWN STATE %d.", g_physicalLayerInformation.zigbeeInterviewState);
      }
      return (events ^ ZIGBEE_INTERVIEW_EVT);
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
    bool accept = false;
    bool sendAck = false;
    /*
    LOG("Received packet!!!\n");
    for(int i = 0; i < packet->bufferReceivedByteLength; i++)
    {
        LOG("%x ", packet->bufferReceive[i]);
    }*/
    LOG("P");
    uint16_t fcf = *(uint16_t*) &packet->bufferReceive[1];
    uint8_t frameType = fcf & 0x07;
    if(frameType == 0x0) // Beacon
    {
        uint8_t sequenceNumber = packet->bufferReceive[3];
        uint16_t sourcePAN = *(uint16_t*) &packet->bufferReceive[4];
        uint16_t source = *(uint16_t*) &packet->bufferReceive[6];
        uint16_t superFrameSpecification = *(uint16_t*) &packet->bufferReceive[8];
        if(superFrameSpecification & (1 << 15))
        {
        LOG("COORDINATOR");
        }else
        {
        //LOG("NOT");
        }
        LOG("Source PAN is %x for source %x" , sourcePAN,source);
    }else if(frameType == 0x1)
    {
        uint8_t* buffer = &packet->bufferReceive[0];
        if(g_physicalLayerInformation.nodeAddress)
        {
            if( ((g_physicalLayerInformation.nodeAddress & 0xFF) == packet->bufferReceive[6]) &&
                ((g_physicalLayerInformation.nodeAddress >> 8) & 0xFF) == packet->bufferReceive[7])
            {
                uint8_t* zigbeeNetworkData = &packet->bufferReceive[10];
                uint8_t zigBeeFrameType = *zigbeeNetworkData & 0x07;

                if(zigBeeFrameType == 0x0) //DATA
                {
                }
            }
        }

        if(fcf & (1 << 5))
        {
            sendAck = true;
            //ACK is expected.
        }
        // Data
    }else if(frameType == 0x2)
    {
        // ACK
        LOG("ACK");
        if(g_physicalLayerInformation.zigbeeInterviewState == ZIGBEE_SENT_ASSOCIATION_REQUEST)
        {
            g_physicalLayerInformation.zigbeeInterviewState = ZIGBEE_ACK_ASSOCIATION_REQUEST;
            accept = true;
        }else  if(g_physicalLayerInformation.zigbeeInterviewState == ZIGBEE_SENT_DATA_REQUEST_AFTER_ASSOCIATION)
        {
            g_physicalLayerInformation.zigbeeInterviewState = ZIGBEE_ACK_DATA_REQUEST_AFTER_ASSOCIATION;
            accept = true;
        }
    }else if(frameType == 0x3) //Command
    {
        uint16_t fcf = *(uint16_t*) &packet->bufferReceive[1];
        if(fcf & (1 << 5))
        {
            if(g_physicalLayerInformation.zigbeeInterviewState==ZIGBEE_ACK_DATA_REQUEST_AFTER_ASSOCIATION)
            {
                accept = true;
                sendAck = true;
                g_physicalLayerInformation.zigbeeInterviewState++;
                g_physicalLayerInformation.nodeAddress = (packet->bufferReceive[24] << 8) + packet->bufferReceive[23];
            }
        }

    }

    g_physicalLayerInformation.Status = PHYSICAL_LAYER_IDLE;

    //Continue the interview
    if(sendAck)
    {
        osal_set_event(g_physicalLayerTaskId, ZIGBEE_ACK_EVT);
    }else if(accept)
    {
        osal_set_event(g_physicalLayerTaskId, ZIGBEE_INTERVIEW_EVT);
    }
    else
        osal_set_event(g_physicalLayerTaskId, RX_EVT);
}


static void onProcessTxDoneEvent(void)
{
  g_physicalLayerInformation.Status = PHYSICAL_LAYER_IDLE;
}

static void onProcessRxDoneEvent(void)
{
    LOG("Processing done event");
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

    //LOG("Sending packet of length %d", packet->bufferTransmitLength);
    SetPhysicalHardwareSingleTX();

     ll_hw_write_tfifo(&packet->bufferTransmit[0], packet->bufferTransmitLength);
     ll_hw_go();
     llWaitingIrq=TRUE;

     uint32_t newmode = ll_hw_get_tr_mode();
    // info1("rf : %d md:%d",osKernelGetTickCount(), (int)newmode);
    if (LL_HW_MODE_STX != newmode)
    {
        LOG("mode: %d != %d", (int)newmode, (int)LL_HW_MODE_STX);
    }

}

void RawZigbeeToPacket(struct PacketInformation *packet)
{
            uint8_t* buffer = packet->bufferReceive;
            ll_hw_read_rfifo_zb(buffer, &packet->bufferReceivedByteLength, &packet->receivedFooter[0], &packet->receivedFooter[1]);
            packet->bufferReceivedIntLength = 1 + (packet->bufferReceivedByteLength >> 2);
}

void PhysicalLayerReceiveRawPacket(uint16_t rxTimeoutUs)
{
    //phy_hw_stop();
    ZigbeeSetChannel(g_physicalLayerInformation.channel);
    ZigbeeSetupHardwareTiming();
    SetPhysicalHardwareSRX(rxTimeoutUs); //us
    ll_hw_rst_tfifo();
    ll_hw_rst_rfifo();
    //set_max_length(0xff);

    ll_hw_go();

    llWaitingIrq=TRUE;
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


