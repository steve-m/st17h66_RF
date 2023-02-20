

#include <stdint.h>
#include "rf_phy_driver.h"
#include "PhysicalLayer.h"
#include "log.h"
#include "string.h"

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
  //LOG("Channel index (before) %d, now %d" , chn, channelIndexNow);
  //rf_tpCal_cfg(channelIndexNow);

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

void prepareIEE810154BroadcastPacket(struct PhysicalLayerInformation* phy, struct PacketInformation* packet)
{
        uint8_t radioTxNum = packet->currentSequenceId++;

        uint8_t* buffer = packet->bufferTransmit;

        buffer[0] = 10;
        buffer[1] = 0x03;
        buffer[2] = 0x08;
        buffer[3] = radioTxNum;
        buffer[4] = ((0xFFFF >> 0) & (0xFF));
        buffer[5] = ((0xFFFF >> 8) & (0xFF));
        buffer[6] = ((0xFFFF >> 0) & 0xFF);
        buffer[7] = ((0xFFFF >> 8) & 0xFF);
        buffer[8] = 0x07;

        packet->bufferTransmitLength = 10;
}



void prepareZigbeeAssociationRequestPacket(struct PhysicalLayerInformation* phy, struct PacketInformation* packet)
{
        uint8_t radioTxNum = packet->currentSequenceId++;
        uint8_t* buffer = packet->bufferTransmit;
        uint16_t panId = phy->panId;
        uint16_t destination = 0x00;
        uint16_t sourcePan = 0xFFFF;

        buffer[0] = 21;
        buffer[1] = 0x23;
        buffer[2] = 0xc8;

        buffer[3] = radioTxNum;
        buffer[4] = ((panId >> 0) & (0xFF));
        buffer[5] = ((panId >> 8) & (0xFF));
        buffer[6] = ((destination >> 0) & 0xFF);
        buffer[7] = ((destination >> 8) & 0xFF);
        buffer[8] = ((sourcePan >> 0) & (0xFF));
        buffer[9] = ((sourcePan >> 8) & (0xFF));

        memcpy(&buffer[10], phy->extendedAddress, 8);
        buffer[18] =0x01;
        buffer[19] = 0x8c; // Reduced function device

        packet->bufferTransmitLength = 21;

}

void prepareZigbeeAckPacket(struct PhysicalLayerInformation* phy, struct PacketInformation* packet)
{
        uint8_t sequenceNumber = packet->bufferReceive[3];
        uint8_t* buffer = packet->bufferTransmit;
        uint16_t panId = phy->panId;

        buffer[0] = 5;
        buffer[1] = 0x02;
        buffer[2] = 0x0;
        buffer[3] = sequenceNumber;
        packet->bufferTransmitLength = 5;

}

void prepareZigbeeDataRequestAfterAssociation(struct PhysicalLayerInformation* phy, struct PacketInformation* packet)
{
        uint8_t radioTxNum = packet->currentSequenceId++;
        uint8_t* buffer = packet->bufferTransmit;
        uint16_t panId = phy->panId;
        uint16_t destination = 0x00;
        uint16_t sourcePan = 0xFFFF;

        buffer[0] = 18;
        buffer[1] = 0x63;
        buffer[2] = 0xc8;

        buffer[3] = radioTxNum;
        buffer[4] = ((panId >> 0) & (0xFF));
        buffer[5] = ((panId >> 8) & (0xFF));
        buffer[6] = ((destination >> 0) & 0xFF);
        buffer[7] = ((destination >> 8) & 0xFF);

        memcpy(&buffer[8], phy->extendedAddress, 8);
        buffer[16] =0x04;

        packet->bufferTransmitLength = 18;

}


void prepareZigbeeDeviceAnnouncement(struct PhysicalLayerInformation* phy, struct PacketInformation* packet)
{
        uint8_t radioTxNum = packet->currentSequenceId++;
        uint8_t* buffer = packet->bufferTransmit;
        uint16_t panId = phy->panId;

        // Broadcast this at first
        uint16_t destination = 0xffff;
        uint16_t source = phy->nodeAddress;

        buffer[0] = 21;
        buffer[1] = 0x41;
        buffer[2] = 0x88;

        buffer[3] = radioTxNum;
        buffer[4] = ((panId >> 0) & (0xFF));
        buffer[5] = ((panId >> 8) & (0xFF));
        buffer[6] = ((destination >> 0) & 0xFF);
        buffer[7] = ((destination >> 8) & 0xFF);
        buffer[8] = ((source >> 0) & (0xFF));
        buffer[9] = ((source >> 8) & (0xFF));

        uint8_t* zigbeeBuffer = &buffer[10];
        zigbeeBuffer[0] = 0x08;
        zigbeeBuffer[1] = 0x00; //Security = false
        zigbeeBuffer[4] = ((0xfffd >> 0) & (0xFF)); //Destination
        zigbeeBuffer[5] = ((0xfffd >> 8) & (0xFF));
        zigbeeBuffer[6] = ((source >> 0) & 0xFF);
        zigbeeBuffer[7] = ((source >> 8) & 0xFF);
        zigbeeBuffer[8] = 0x1e; //Radius
        zigbeeBuffer[9] = packet->currentZigbeeSeq++; //Radius

        uint8_t* zigbeeSecurityHeader = &zigbeeBuffer[10];
        zigbeeSecurityHeader[0] = 0x28;
        *((uint32_t*)zigbeeSecurityHeader[1]) = 2055; //WTF, TODO
        memcpy(&zigbeeSecurityHeader[5] , phy->extendedAddress,8);



        packet->bufferTransmitLength = 21;

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

        buffer[1] = 0x03;
        buffer[2] = 0x08;
        buffer[3] = radioTxNum++;
        buffer[4] = ((panId >> 0) & (0xFF));
        buffer[5] = ((panId >> 8) & (0xFF));
        buffer[6] = ((ZIGBEE_BROADCAST_ADDRESS >> 0) & 0xFF);
        buffer[7] = ((ZIGBEE_BROADCAST_ADDRESS >> 8) & 0xFF);
        buffer[8] = 0x07;
        uint8_t bufferSize = 9;

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
        packet->bufferTransmit[0] = bufferSize + payloadSize +1;

        /*
        uint16_t crc = crc16(&packet->bufferTransmit[0], bufferSize);
        packet->bufferTransmit[bufferSize] = (crc >> 0) & (0xFF);
        packet->bufferTransmit[bufferSize+1] = (crc >> 8) & (0xFF);
        */
        uint16_t total =  bufferSize + payloadSize; // lenb, hdr, data, crc
        packet->bufferTransmitLength = total;
    }


}
