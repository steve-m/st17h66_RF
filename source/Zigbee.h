#pragma once

#include "stdint.h"

struct PacketInformation;

void prepareZigbeeBroadcastPacket(struct PhysicalLayerInformation* phy, struct PacketInformation* packet);
void ZigbeeSetChannel(uint8_t chn);
void ZigbeeSetupHardwareTiming (void);
