

#include <stdint.h>
#include "rf_phy_driver.h"
#include "PhysicalLayer.h"
#include "log.h"

static uint16_t crc16(uint8_t * ptr, uint8_t size) {
        uint16_t crc = 0;

        while(--size >= 0) {
            crc ^= (unsigned short)(*ptr++) << 8;
            int i = 8;
            do{
            if(crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc = crc << 1;
            } while(--i);
        }

        return crc;
    }


void ZigbeeSetChannel(uint8_t chn) {
  uint32_t rfChnIdx = (chn - 10) * 5;

  if (g_rfPhyFreqOffSet >= 0) {
    PHY_REG_WT(0x400300B4,
               (g_rfPhyFreqOffSet << 16) + (g_rfPhyFreqOffSet << 8) + rfChnIdx);
  } else {
    PHY_REG_WT(0x400300B4, ((255 + g_rfPhyFreqOffSet) << 16) +
                               ((255 + g_rfPhyFreqOffSet) << 8) +
                               (rfChnIdx - 1));
  }
  ll_hw_ign_rfifo(LL_HW_IGN_CRC);

  uint8_t channelIndexNow = PHY_REG_RD(0x400300B4) & 0xFF;
  LOG("Channel index (before) %d, now %d" , chn, channelIndexNow);
  rf_tpCal_cfg(channelIndexNow);

}

void ZigbeeSetupHardwareTiming (void)
{
    #define LL_HW_BB_DELAY_VAL         32
    #define LL_HW_AFE_DELAY_VAL        8
    #define LL_HW_PLL_DELAY_VAL        52
    #define SCAN_RSP_DELAY_VAL         32
    #define MAX_RX_TIMEOUT             0

    //restore the ll register
    ll_hw_set_tx_rx_release     (10, 1);
    ll_hw_set_rx_tx_interval(98);               // T_IFS = 192+2us for ZB 98
    ll_hw_set_tx_rx_interval(108);              // T_IFS = 192-6us for ZB 108
    ll_hw_set_trx_settle(LL_HW_BB_DELAY_VAL, LL_HW_AFE_DELAY_VAL, LL_HW_PLL_DELAY_VAL);    // TxBB, RxAFE, PLL

}

void prepareZigbeeBroadcastPacket(struct PhysicalLayerInformation* physicalLayerInformation,struct PacketInformation* packet)
{
    LOG("Sending broadcast packet");
    /// Prepare the data
    {
        static uint8_t radioTxNum = 0;//TODO
    uint16_t panId = physicalLayerInformation->panId;
    /// TODO
    panId = 0xffff;
    #define ZIGBEE_JOIN_REQUEST 0x01
    #define BEACON_REQUEST 0x07
    #define ZIGBEE_BROADCAST_ADDRESS 0xFFFF


        uint16_t src = physicalLayerInformation->nodeAddress;

        uint8_t needAck = 0;
        if (needAck)
        {
            //    radio_tx_wait_ack = true;
            packet->bufferTransmit[1] = 0x61; //TODO
        } else
        {
            //   radio_tx_wait_ack = false;
            packet->bufferTransmit[1] = 0x41; //TODO
        }


        uint8_t* buffer = packet->bufferTransmit;

        buffer[2] = radioTxNum++;
        buffer[3] = ((panId >> 0) & (0xFF));
        buffer[4] = ((panId >> 8) & (0xFF));
        buffer[5] = ((ZIGBEE_BROADCAST_ADDRESS >> 0) & 0xFF);
        buffer[6] = ((ZIGBEE_BROADCAST_ADDRESS >> 8) & 0xFF);
        buffer[7] = 0x07;
        uint8_t bufferSize = 8;

        /*
        packet->bufferTransmit[3] = radioTxNum++;
        packet->bufferTransmit[4] = ((panId >> 0) & (0xFF));
        packet->bufferTransmit[5] = ((panId >> 8) & (0xFF));
        packet->bufferTransmit[6] = ((ZIGBEE_BROADCAST_ADDRESS >> 0) & 0xFF);
        packet->bufferTransmit[7] = ((ZIGBEE_BROADCAST_ADDRESS >> 8) & 0xFF);
        packet->bufferTransmit[8] = ((src >> 0) & 0xFF);
        packet->bufferTransmit[9] = ((src >> 8) & 0xFF);
        packet->bufferTransmit[10] = 0x3F;
        */


        uint8_t payloadSize = 0;                   // Payload size
        packet->bufferTransmit[0] = bufferSize + payloadSize + 2;        // hdr, data, crc

        /*
        uint16_t crc = crc16(&packet->bufferTransmit[0], bufferSize);
        packet->bufferTransmit[bufferSize] = (crc >> 0) & (0xFF);
        packet->bufferTransmit[bufferSize+1] = (crc >> 8) & (0xFF);
        */
        uint16_t total = 1 + bufferSize + payloadSize + 2; // lenb, hdr, data, crc
        packet->bufferTransmitLength = total;
    }


}
