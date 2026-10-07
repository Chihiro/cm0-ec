/* Host-side model of the LL operations used by ec_host_i2c.c.
 * Flag clearing and RXDR/TXDR side effects are modeled; this does not simulate
 * electrical timing, clock stretching duration, or the STM32 bus hardware. */
#ifndef TEST_STM32L0XX_CONF_H
#define TEST_STM32L0XX_CONF_H
#define __STM32L0xx_CONF_H /* Suppress the real header included by bq27220.h. */
#include <stdbool.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    uint32_t ISR, CR1, OAR1, TIMINGR;
    uint8_t RXDR, TXDR;
    bool enabled, own_enabled, stretching, stuck_scl;
} I2C_TypeDef;
typedef struct {
    uint32_t mode[16], af[16], pull[16], output_type, IDR;
} GPIO_TypeDef;
extern I2C_TypeDef mock_i2c2;
extern GPIO_TypeDef mock_gpiob;
extern uint32_t mock_primask;
extern bool mock_irq_enabled, mock_start_on_address_disable;
extern uint32_t mock_events_on_address_disable;
#define I2C2 (&mock_i2c2)
#define GPIOB (&mock_gpiob)
#define I2C2_IRQn 1
#define I2C_ISR_RXNE  (1U << 0)
#define I2C_ISR_ADDR  (1U << 1)
#define I2C_ISR_DIR   (1U << 2)
#define I2C_ISR_NACKF (1U << 3)
#define I2C_ISR_STOPF (1U << 4)
#define I2C_ISR_BERR  (1U << 5)
#define I2C_ISR_ARLO  (1U << 6)
#define I2C_ISR_OVR   (1U << 7)
#define I2C_ISR_TXIS  (1U << 8)
#define I2C_ISR_BUSY  (1U << 9)
#define I2C_CR1_PE    (1U << 0)
#define I2C_CR1_TXIE  (1U << 1)
#define I2C_CR1_RXIE  (1U << 2)
#define I2C_CR1_ADDRIE (1U << 3)
#define I2C_CR1_NACKIE (1U << 4)
#define I2C_CR1_STOPIE (1U << 5)
#define I2C_CR1_ERRIE (1U << 7)
#define LL_GPIO_PIN_13 (1U << 13)
#define LL_GPIO_PIN_14 (1U << 14)
#define LL_GPIO_AF_5 5U
#define LL_GPIO_MODE_ALTERNATE 2U
#define LL_GPIO_MODE_ANALOG 3U
#define LL_GPIO_MODE_INPUT 0U
#define LL_GPIO_PULL_NO 0U
#define LL_GPIO_OUTPUT_OPENDRAIN 1U
#define LL_GPIO_SPEED_FREQ_HIGH 3U
#define LL_IOP_GRP1_PERIPH_GPIOB 1U
#define LL_APB1_GRP1_PERIPH_I2C2 1U
#define LL_I2C_MODE_I2C 0U
#define LL_I2C_OWNADDRESS1_7BIT 0U
#define __LL_I2C_CONVERT_TIMINGS(p, c, d, h, l) (((p)<<28U)|((c)<<20U)|((d)<<16U)|((h)<<8U)|(l))
#define __DMB() ((void)0)
#define __DSB() ((void)0)
static inline uint32_t __get_PRIMASK(void) { return mock_primask; }
static inline void __disable_irq(void) { mock_primask = 1U; }
static inline void __set_PRIMASK(uint32_t value) { mock_primask = value; }
static inline void NVIC_DisableIRQ(int irq) { (void)irq; mock_irq_enabled = false; }
static inline void NVIC_EnableIRQ(int irq) { (void)irq; mock_irq_enabled = true; }
static inline void NVIC_ClearPendingIRQ(int irq) {
    (void)irq;
    assert(!I2C2->enabled); /* Never clear IRQ pending after accepting addresses. */
}
static inline void NVIC_SetPriority(int irq, uint32_t priority) { (void)irq; (void)priority; }
static inline uint32_t NVIC_GetEnableIRQ(int irq) { (void)irq; return mock_irq_enabled; }
static inline uint32_t NVIC_GetPendingIRQ(int irq) { (void)irq; return 0U; }
#define LL_IOP_GRP1_EnableClock(x) ((void)(x))
#define LL_APB1_GRP1_EnableClock(x) ((void)(x))
#define LL_APB1_GRP1_ReleaseReset(x) ((void)(x))
#define LL_APB1_GRP1_ForceReset(x) ((void)(x), memset(I2C2, 0, sizeof(*I2C2)))
static inline unsigned pin_index(uint32_t pin) { return pin == LL_GPIO_PIN_13 ? 13U : 14U; }
static inline void LL_GPIO_SetPinMode(GPIO_TypeDef *port, uint32_t pin, uint32_t value) { port->mode[pin_index(pin)] = value; }
static inline uint32_t LL_GPIO_IsInputPinSet(GPIO_TypeDef *port, uint32_t pin) {
    uint32_t levels = port->IDR;
    if (I2C2->enabled && I2C2->stuck_scl) levels &= ~LL_GPIO_PIN_13;
    return (levels & pin) == pin;
}
static inline void LL_GPIO_SetAFPin_8_15(GPIO_TypeDef *port, uint32_t pin, uint32_t value) { port->af[pin_index(pin)] = value; }
static inline void LL_GPIO_SetPinPull(GPIO_TypeDef *port, uint32_t pin, uint32_t value) { port->pull[pin_index(pin)] = value; }
static inline void LL_GPIO_SetPinOutputType(GPIO_TypeDef *port, uint32_t pins, uint32_t value) { (void)pins; port->output_type = value; }
static inline void LL_GPIO_SetPinSpeed(GPIO_TypeDef *port, uint32_t pin, uint32_t value) { (void)port; (void)pin; (void)value; }
static inline void LL_I2C_SetTiming(I2C_TypeDef *p, uint32_t value) { p->TIMINGR = value; }
static inline void LL_I2C_SetMode(I2C_TypeDef *p, uint32_t value) { (void)p; (void)value; }
static inline void LL_I2C_EnableAnalogFilter(I2C_TypeDef *p) { (void)p; }
static inline void LL_I2C_SetDigitalFilter(I2C_TypeDef *p, uint32_t value) { (void)p; (void)value; }
static inline void LL_I2C_EnableClockStretching(I2C_TypeDef *p) { p->stretching = true; }
static inline void LL_I2C_SetOwnAddress1(I2C_TypeDef *p, uint32_t address, uint32_t size) { (void)size; p->OAR1 = address; }
static inline void LL_I2C_EnableOwnAddress1(I2C_TypeDef *p) { p->own_enabled = true; p->OAR1 |= 1U << 15; }
static inline void LL_I2C_DisableOwnAddress1(I2C_TypeDef *p) {
    p->own_enabled = false;
    p->OAR1 &= ~(1U << 15);
    if (mock_start_on_address_disable) p->ISR |= I2C_ISR_BUSY;
    p->ISR |= mock_events_on_address_disable;
}
static inline void LL_I2C_Enable(I2C_TypeDef *p) {
    assert(mock_irq_enabled); /* Address matching must have an IRQ handler ready. */
    p->enabled = true;
    p->CR1 |= I2C_CR1_PE;
}
static inline void LL_I2C_Disable(I2C_TypeDef *p) { p->enabled = false; p->ISR = 0U; p->CR1 &= ~I2C_CR1_PE; }
static inline uint32_t LL_I2C_IsActiveFlag_BUSY(I2C_TypeDef *p) { return p->ISR & I2C_ISR_BUSY; }
static inline uint32_t LL_I2C_IsActiveFlag_TXIS(I2C_TypeDef *p) { return p->ISR & I2C_ISR_TXIS; }
static inline void LL_I2C_ClearFlag_TXE(I2C_TypeDef *p) { p->TXDR = 0U; }
static inline uint8_t LL_I2C_ReceiveData8(I2C_TypeDef *p) { p->ISR &= ~I2C_ISR_RXNE; return p->RXDR; }
static inline void LL_I2C_TransmitData8(I2C_TypeDef *p, uint8_t value) { p->TXDR = value; p->ISR &= ~I2C_ISR_TXIS; }
#define DEFINE_CLEAR(name, bit) static inline void LL_I2C_ClearFlag_##name(I2C_TypeDef *p) { p->ISR &= ~(bit); }
DEFINE_CLEAR(ADDR, I2C_ISR_ADDR)
DEFINE_CLEAR(NACK, I2C_ISR_NACKF)
DEFINE_CLEAR(STOP, I2C_ISR_STOPF)
DEFINE_CLEAR(BERR, I2C_ISR_BERR)
DEFINE_CLEAR(ARLO, I2C_ISR_ARLO)
DEFINE_CLEAR(OVR, I2C_ISR_OVR)
#define DEFINE_IT(name, bit) static inline void LL_I2C_EnableIT_##name(I2C_TypeDef *p) { p->CR1 |= (bit); }
DEFINE_IT(TX, I2C_CR1_TXIE)
DEFINE_IT(RX, I2C_CR1_RXIE)
DEFINE_IT(ADDR, I2C_CR1_ADDRIE)
DEFINE_IT(NACK, I2C_CR1_NACKIE)
DEFINE_IT(STOP, I2C_CR1_STOPIE)
DEFINE_IT(ERR, I2C_CR1_ERRIE)
static inline void LL_I2C_DisableIT_TX(I2C_TypeDef *p) { p->CR1 &= ~I2C_CR1_TXIE; }
#endif
