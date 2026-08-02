#ifndef TEST_DRIVERLIB_H
#define TEST_DRIVERLIB_H
#include <stdint.h>
#include <stdbool.h>

#define __interrupt
#define DINT  do {} while (0)
#define EINT  do {} while (0)
#define ERTM  do {} while (0)

#define CPUTIMER0_BASE        0U
#define INT_TIMER0            0U
#define INTERRUPT_ACK_GROUP1  1U

typedef void (*InterruptHandler_t)(void);

#define CAN_MSG_FRAME_STD 0U
#define CAN_MSG_OBJ_TYPE_RX 0U
#define CAN_MSG_OBJ_TYPE_TX 1U
#define CAN_MSG_OBJ_NO_FLAGS 0U
#define GPIO_43_GPIO43 43U
#define GPIO_26_GPIO26 26U
#define GPIO_29_SCIA_TX 29U
#define GPIO_28_SCIA_RX 28U
#define GPIO_22_LINA_TX 22U
#define GPIO_23_LINA_RX 23U
#define GPIO_PIN_TYPE_PULLUP 0U
#define GPIO_PIN_TYPE_OD 0U
#define GPIO_PIN_TYPE_STD 0U
#define GPIO_DIR_MODE_OUT 0U
#define GPIO_QUAL_ASYNC 0U
#define SYSCTL_PERIPH_CLK_SCIA 0U
#define I2CA_BASE 0U
#define SCI_CONFIG_WLEN_8 0x0007U
#define SCI_CONFIG_STOP_ONE 0x0000U
#define SCI_CONFIG_PAR_NONE 0x0000U
#define SCI_FIFO_TX16 0x0010U
#define SCI_FIFO_RX0  0x0000U
#define EALLOW do {} while (0)
#define EDIS do {} while (0)
static inline uint32_t *test_hwreg_ptr(void)
{ static uint32_t value; return &value; }
#define HWREG(x) (*test_hwreg_ptr())
#ifndef ADAS_TEST_OMIT_BUS_OFF
#define CAN_STATUS_BUS_OFF (1UL << 7)
#endif

static inline void CAN_setupMessageObject(uint32_t a, uint16_t b, uint32_t c,
    uint16_t d, uint16_t e, uint32_t f, uint32_t g, uint16_t h)
{ (void)a;(void)b;(void)c;(void)d;(void)e;(void)f;(void)g;(void)h; }
static inline void CAN_initModule(uint32_t a) { (void)a; }
static inline void CAN_setBitRate(uint32_t a,uint32_t b,uint32_t c,uint16_t d)
{ (void)a;(void)b;(void)c;(void)d; }
static inline void CAN_enableRetry(uint32_t a) { (void)a; }
static inline void CAN_startModule(uint32_t a) { (void)a; }
static inline uint32_t CAN_getStatus(uint32_t a) { (void)a; return 0U; }
static inline bool CAN_readMessage(uint32_t a,uint16_t b,uint16_t *c)
{ (void)a;(void)b;(void)c; return false; }
static inline void CAN_sendMessage(uint32_t a,uint16_t b,uint16_t c,const uint16_t *d)
{ (void)a;(void)b;(void)c;(void)d; }
static inline void GPIO_setPinConfig(uint32_t a) { (void)a; }
static inline void GPIO_setPadConfig(uint32_t a, uint32_t b) { (void)a;(void)b; }
static inline void GPIO_setDirectionMode(uint32_t a, uint32_t b)
{ (void)a;(void)b; }
static inline uint32_t GPIO_readPin(uint32_t a) { (void)a; return 1U; }
static inline void GPIO_writePin(uint32_t a, uint32_t b) { (void)a;(void)b; }
static inline void GPIO_setQualificationMode(uint32_t a, uint32_t b)
{ (void)a;(void)b; }
static inline void I2C_disableModule(uint32_t a) { (void)a; }
static inline void SysCtl_enablePeripheral(uint32_t a) { (void)a; }
static inline void SCI_performSoftwareReset(uint32_t a) { (void)a; }
static inline void SCI_setConfig(uint32_t a,uint32_t b,uint32_t c,uint32_t d)
{ (void)a;(void)b;(void)c;(void)d; }
static inline void SCI_resetChannels(uint32_t a) { (void)a; }
static inline void SCI_enableFIFO(uint32_t a) { (void)a; }
static inline void SCI_enableModule(uint32_t a) { (void)a; }
static inline uint16_t SCI_getTxFIFOStatus(uint32_t a) { (void)a; return 0U; }
static inline uint16_t SCI_getRxFIFOStatus(uint32_t a) { (void)a; return 0U; }
static inline void SCI_writeCharNonBlocking(uint32_t a, uint16_t b)
{ (void)a;(void)b; }
static inline uint16_t SCI_readCharNonBlocking(uint32_t a, uint16_t b)
{ (void)a;(void)b; return 0U; }

static inline void Interrupt_clearACKGroup(uint16_t a) { (void)a; }
static inline void Interrupt_register(uint32_t a, InterruptHandler_t b)
{ (void)a;(void)b; }
static inline void Interrupt_enable(uint32_t a) { (void)a; }
static inline uint32_t CPUTimer_getTimerCount(uint32_t a) { (void)a; return 0U; }

#define LINA_BASE 0U
#define LINB_BASE 0U
#define LIN_SCI_STOP_ONE 0U
static inline void LIN_enableModule(uint32_t a) { (void)a; }
static inline void LIN_enterSoftwareReset(uint32_t a) { (void)a; }
static inline void LIN_exitSoftwareReset(uint32_t a) { (void)a; }
static inline void LIN_enableSCIMode(uint32_t a) { (void)a; }
static inline void LIN_setSCICharLength(uint32_t a, uint16_t b)
{ (void)a;(void)b; }
static inline void LIN_setSCIStopBits(uint32_t a, uint32_t b)
{ (void)a;(void)b; }
static inline void LIN_disableSCIParity(uint32_t a) { (void)a; }
static inline void LIN_setBaudRatePrescaler(uint32_t a, uint32_t b, uint32_t c)
{ (void)a;(void)b;(void)c; }
static inline void LIN_enableDataTransmitter(uint32_t a) { (void)a; }
static inline void LIN_enableDataReceiver(uint32_t a) { (void)a; }
static inline bool LIN_isSCIDataAvailable(uint32_t a)
{ (void)a; return false; }
static inline uint16_t LIN_readSCICharNonBlocking(uint32_t a, bool b)
{ (void)a;(void)b; return 0U; }
static inline void LIN_writeSCICharBlocking(uint32_t a, uint16_t b)
{ (void)a;(void)b; }
#endif
