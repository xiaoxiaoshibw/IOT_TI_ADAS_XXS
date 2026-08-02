#ifndef TEST_DEVICE_H
#define TEST_DEVICE_H
#define CANA_BASE 0U
#define I2CA_BASE 0U
#define SCIA_BASE 0U
#define DEVICE_SYSCLK_HZ 100000000UL
#define DEVICE_LSPCLK_FREQ (DEVICE_SYSCLK_HZ / 4U)
#define DEVICE_DELAY_US(x) do { (void)(x); } while (0)

static inline void Device_init(void) {}
static inline void Device_initGPIO(void) {}
static inline void Interrupt_initModule(void) {}
static inline void Interrupt_initVectorTable(void) {}
#endif
