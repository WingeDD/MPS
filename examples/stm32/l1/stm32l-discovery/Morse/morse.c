#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>

#define LED_DISCO_GREEN_PORT GPIOB
#define LED_DISCO_GREEN_PIN GPIO7
#define LED_DISCO_BLUE_PORT GPIOB
#define LED_DISCO_BLUE_PIN GPIO6

#define BUTTON_DISCO_USER_PORT GPIOA
#define BUTTON_DISCO_USER_PIN GPIO0
#define BUTTON_DISCO_USER_EXTI EXTI0
#define BUTTON_DISCO_USER_isr exti0_isr
#define BUTTON_DISCO_USER_NVIC NVIC_EXTI0_IRQ
#define TIMER_BUTTON_PRESS TIM7
#define TIMER_BUTTON_PRESS_RST RST_TIM7

#define tick 500 // Single Morse tick in ms

int counter = 0;
bool send = false;
char msg = 1; 

static void delay(int t)
{
    int i;
    for (i = 0; i < t; i++)    /* Wait. */
        __asm__("nop");
}

struct state_t {
    bool falling;
    bool pressed;
    int tickcount;
    int hold_time;
};

static struct state_t state;

static void clock_setup(void)
{
    rcc_clock_setup_pll(&rcc_clock_config[RCC_CLOCK_VRANGE1_HSI_PLL_24MHZ]);
    /* Lots of things on all ports... */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);

    /* Enable clocks for USART2. */
    rcc_periph_clock_enable(RCC_USART2);

    /* And timers. */
    rcc_periph_clock_enable(RCC_TIM6);
    rcc_periph_clock_enable(RCC_TIM7);
}

static void usart_setup(void)
{
    usart_set_baudrate(USART2, 115200);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

    /* Finally enable the USART. */
    usart_enable(USART2);
}

static void gpio_setup(void)
{
    /* Setup GPIO pin GPIO7 on GPIO port B for Green LED. */
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO7);

    /* Setup GPIO pin GPIO6 on GPIO port B for Blue LED. */
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);

    /* Setup GPIO pins for USART2 transmit. */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);

    /* Setup USART2 TX pin as alternate function. */
    gpio_set_af(GPIOA, GPIO_AF7, GPIO2);
}

static void setup_tim6(void)
{
    rcc_periph_reset_pulse(RST_TIM6);
    /* 24Mhz / 10khz -1. */
    timer_set_prescaler(TIM6, 2399); /* 24Mhz/10000hz - 1 */
    /* 10khz for 10 ticks = 1 khz overflow = 1ms overflow interrupts */
    timer_set_period(TIM6, 10);

    nvic_enable_irq(NVIC_TIM6_IRQ);
    timer_enable_update_event(TIM6); /* default at reset! */
    timer_enable_irq(TIM6, TIM_DIER_UIE);
    timer_enable_counter(TIM6);
}

/*
 * Free running ms timer.
 */
static void setup_tim7(void)
{
    rcc_periph_reset_pulse(RST_TIM7);
    timer_set_prescaler(TIM7, 23999); /* 24Mhz/1000hz - 1 */
    timer_set_period(TIM7, 0xffff);
    timer_enable_counter(TIM7);
}

void BUTTON_DISCO_USER_isr(void)
{
    exti_reset_request(BUTTON_DISCO_USER_EXTI);
    if (state.falling) {
        gpio_clear(LED_DISCO_BLUE_PORT, LED_DISCO_BLUE_PIN);
        state.falling = false;
        exti_set_trigger(BUTTON_DISCO_USER_EXTI, EXTI_TRIGGER_RISING);
        unsigned int x = TIM_CNT(TIM7);
        counter++;
        if ((counter > 5) && (!send)){
            printf("Too much symbols\n");
            msg = 1;
            counter = 0;
            send = false;
        }else{
            if (x>tick){
                msg = (msg<<1) | 1; //| (1 << counter);// printf("-");// bar
            }
            else{
                msg = (msg<<1) | 0;//& (0 << counter);// printf("*");//dot
            }
        }
        state.pressed = false;
        // printf("held: %u ms\n", x);
    } else {
        gpio_set(LED_DISCO_BLUE_PORT, LED_DISCO_BLUE_PIN);
        // printf("Pushed down!\n");
        TIM_CNT(TIM7) = 0;
        state.falling = true;
        state.pressed = true;
        exti_set_trigger(BUTTON_DISCO_USER_EXTI, EXTI_TRIGGER_FALLING);
    }
}

static void setup_buttons(void)
{
    /* Enable EXTI0 interrupt. */
    nvic_enable_irq(BUTTON_DISCO_USER_NVIC);

    gpio_mode_setup(BUTTON_DISCO_USER_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE,
            BUTTON_DISCO_USER_PIN);

    /* Configure the EXTI subsystem. */
    exti_select_source(BUTTON_DISCO_USER_EXTI, BUTTON_DISCO_USER_PORT);
    state.falling = false;
    exti_set_trigger(BUTTON_DISCO_USER_EXTI, EXTI_TRIGGER_RISING);
    exti_enable_request(BUTTON_DISCO_USER_EXTI);
}


static volatile int t6ovf;

void tim6_isr(void)
{
    TIM_SR(TIM6) &= ~TIM_SR_UIF;
    if (t6ovf++ > 1000) {
        // printf("TICK %d\n", state.tickcount++);
        t6ovf = 0;
        gpio_toggle(LED_DISCO_GREEN_PORT, LED_DISCO_GREEN_PIN);
    }
}

int _write(int file, char *ptr, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        if (ptr[i] == '\n') {
            usart_send_blocking(USART2, '\r');
        }
        usart_send_blocking(USART2, ptr[i]);
    }
    return i;
}

static void greetings(void) {
    for (int i = 0; i < 3; ++i)
    {
        gpio_toggle(GPIOB, GPIO6);
        delay(300000);
        gpio_toggle(GPIOB, GPIO6);
        delay(300000);
    }
    printf("Setup complete!\n");
}

int main(void)
{
    clock_setup();
    gpio_setup();
    usart_setup();
    setup_buttons();
    setup_tim6();
    setup_tim7();

    greetings();

    while (1) {
        unsigned int x = TIM_CNT(TIM7);
        if( (!state.pressed) && (x>5000) ){
            printf("Message sent\n");
            if (msg == 1){
                printf("Empty message\n");
            }
            else{
                // printf("%d\n", msg);
                for(int i = 7; i >= 0; --i)
                    printf("%d", ((msg >> i) & 1));
                printf("\n");
            }
            TIM_CNT(TIM7)=0;
            counter = 0;
            msg = 1;
            send = false;
        }
    }
    return 0;
}