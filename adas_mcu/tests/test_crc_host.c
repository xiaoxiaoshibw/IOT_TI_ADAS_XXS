#include <assert.h>
#include <stdio.h>
#include "crc8.h"

int main(void)
{
    const uint16_t check[] = {'1','2','3','4','5','6','7','8','9'};
    const uint16_t payload[] = {2U, 1U, 0U, 0x21U, 1U, 0U, 1U};
    assert(crc8_compute(check, 9U) == 0xA2U);
    assert(crc8_compute_frame(0x100U, payload, 7U) !=
           crc8_compute_frame(0x110U, payload, 7U));
    puts("crc host tests: PASS");
    return 0;
}
