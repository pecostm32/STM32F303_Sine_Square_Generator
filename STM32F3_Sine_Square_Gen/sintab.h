#ifndef SINTAB_H
#define SINTAB_H

#ifdef __cplusplus
extern "C" {
#endif

#define STEPS_360_DEGREES          720     //For a full period there are 720 steps
#define MAX_COUNT_90_DEGREES       179     //Timer 3 max count
#define HALF_COUNT_90_DEGREES       90     //Timer 3 half way count
#define QUARTER_COUNT_90_DEGREES    45     //Timer 3 quarter way count
#define MAX_PHASE                  719     //Max settable phase

//Reference to the actual table
extern const uint16_t dmasinetable_1950[];
extern const uint16_t dmasinetable_1900[];

#ifdef __cplusplus
}
#endif

#endif /* SINTAB_H */

