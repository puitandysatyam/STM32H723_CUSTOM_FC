#ifndef BF_DSHOT_H
#define BF_DSHOT_H

#include <stdint.h>
#include "bf_mixer.h" // For motorMixer_t

// A DShot packet is 16 bits of data. We send 18 pulses total to leave a 2-pulse gap (zero voltage) 
// to serve as the packet delimiter before the next frame is blasted over the wire.
#define DSHOT_DMA_BUFFER_SIZE 18

void dshot_Init(void);
void dshot_Write(motorMixer_t *mixerOutputs);

#endif // BF_DSHOT_H
