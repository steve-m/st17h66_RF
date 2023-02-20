#pragma once

#include <stdint.h>
#include "types.h"

/// Defines for events
#define TX_EVT         0x0001
#define RX_EVT         0x0002
#define TX_DONE_EVT             0x0004
#define RX_DONE_EVT             0x0008
#define TRX_DONE_EVT            0x0010
#define RX_DATA_PROCESS_EVT     0x0020
#define ZIGBEE_INTERVIEW_EVT    0x0040
#define ZIGBEE_ACK_EVT    0x0080

enum ZigbeeInterviewState
{
    ZIGBEE_START = 0,
    ZIGBEE_SENT_ASSOCIATION_REQUEST =1,
    ZIGBEE_ACK_ASSOCIATION_REQUEST = 2,
    ZIGBEE_SENT_DATA_REQUEST_AFTER_ASSOCIATION = 3,
    ZIGBEE_ACK_DATA_REQUEST_AFTER_ASSOCIATION = 4,
    ZIGBEE_ASSOCIATION_RESPONSE_RECEIVED=5,
    ZIGBEE_ASSOCIATION_RESPONSE_ACK=5,
};

struct PhysicalLayerInformation
{
    #define PHYSICAL_LAYER_TX_ONLY (0x00)
    #define PHYSICAL_LAYER_RX_ONLY (0x01)
    #define PHYSICAL_LAYER_TRX_ONLY (0x02)
    #define PHYSICAL_LAYER_RX_TXACK (0x03)
    #define PHYSICAL_LAYER_IDLE (0xFF)

    uint8_t Status;

    uint32_t txIntervalMs;
    uint32_t rxIntervalMs;

    uint8_t channel;
    uint16_t nodeAddress;
    uint8_t extendedAddress[8];
    uint16_t panId;

    enum ZigbeeInterviewState zigbeeInterviewState;
};


struct PacketInformation
{
    uint8_t packetFormat;
    uint8_t pduLength;
    uint8_t whiteningSeed;
    uint8_t crcFormat;
    uint32_t crcSeed;
    uint32_t syncWord;
    uint8_t currentSequenceId;
    uint8_t currentZigbeeSeq;
    ALIGN4_U8 bufferReceive[256];
    uint8_t bufferTransmitLength;
    ALIGN4_U8 bufferTransmit[256];
    uint16_t bufferReceivedByteLength;
    uint8_t bufferReceivedIntLength;
    uint32_t receivedFooter[2];

    uint16_t receivedFrequencyOffset;
    uint8_t CarrSens;
    uint8_t receivedSignalStrength;
};


extern void InitializePhysicalLayer( uint8_t task_id );

extern uint16_t ProcessPhysicalLayerEvents( uint8_t task_id, uint16_t events );
