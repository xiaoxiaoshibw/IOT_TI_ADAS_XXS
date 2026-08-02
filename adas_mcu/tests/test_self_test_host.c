#include <assert.h>
#include <stdio.h>
#include "self_test.h"

int main(void)
{
    uint16_t failures = 0xFFFFU;
    assert(SelfTest_run(&failures));
    assert(failures == SELF_TEST_OK);
    puts("startup self-test host tests: PASS");
    return 0;
}
