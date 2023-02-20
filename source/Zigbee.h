#pragma once

#include "stdint.h"

struct PacketInformation;

void prepareIEE810154BroadcastPacket(struct PhysicalLayerInformation* phy, struct PacketInformation* packet);
void prepareZigbeeBroadcastPacket(struct PhysicalLayerInformation* phy, struct PacketInformation* packet);
void ZigbeeSetChannel(uint8_t chn);
void ZigbeeSetupHardwareTiming (void);
