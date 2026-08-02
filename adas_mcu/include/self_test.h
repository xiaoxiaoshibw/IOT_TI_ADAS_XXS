#ifndef SELF_TEST_H_
#define SELF_TEST_H_

#include <stdbool.h>
#include <stdint.h>

#define SELF_TEST_OK            0U
#define SELF_TEST_CONFIG        (1U << 0)
#define SELF_TEST_CRC           (1U << 1)
#define SELF_TEST_PROTOCOL      (1U << 2)

bool SelfTest_run(uint16_t *failure_mask);

#endif
