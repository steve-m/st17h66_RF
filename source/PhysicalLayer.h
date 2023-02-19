#pragma once

#include <stdint.h>
#include "types.h"

/// Defines for events
#define PPP_PERIODIC_TX_EVT         0x0001
#define PPP_PERIODIC_RX_EVT         0x0002
#define PPP_TX_DONE_EVT             0x0004
#define PPP_RX_DONE_EVT             0x0008
#define PPP_TRX_DONE_EVT            0x0010
#define PPP_RX_DATA_PROCESS_EVT     0x0020

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
    uint16_t panId;
};


struct PacketInformation
{
    uint8_t packetFormat;
    uint8_t pduLength;
    uint8_t whiteningSeed;
    uint8_t crcFormat;
    uint32_t crcSeed;
    uint32_t syncWord;
    ALIGN4_U8 bufferReceive[256];
    uint8_t bufferTransmitLength;
    ALIGN4_U8 bufferTransmit[256];

    uint16_t receivedFrequencyOffset;
    uint8_t CarrSens;
    uint8_t receivedSignalStrength;
};


extern void InitializePhysicalLayer( uint8_t task_id );

extern uint16_t ProcessPhysicalLayerEvents( uint8_t task_id, uint16_t events );
