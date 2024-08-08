#include <stdlib.h>

#define TEXT_ALIVE "$it's alive\r\n"
#define TEXT_OK "ok.\r\n"
#define TEXT_ERROR "$ERROR.\r\n"
#define TEXT_ERROR_GC_DONTRECOGNIZE "$ERROR command don't recognize\r\n"
#define TEXT_ERROR_GC_TOOMANYPARAM "$ERROR too many parameters\r\n"
#define TEXT_ERROR_GC_PARSINGFAIL "$ERROR parsing fail\r\n"
#define TEXT_EOL "\r\n"

#define P0 0
#define P1 1
#define P2 2
#define P3 3
#define P4 4
#define P5 5
#define P6 6
#define P7 7
#define P8 8
#define P9 9
#define P10 10
#define P11 11
#define P12 12
#define P13 13
#define P14 14
#define P15 15

//x = (AFR<<4) | MODER
#define GPIO_MODER_MASK 0x03UL
#define GPIO_AFR_MASK 0x0FUL
#define GPIO_MODE_MODER_ROL 0
#define GPIO_MODE_AFR_ROL 4

#define GPIO_MODE_INPUT 0x00UL
#define GPIO_MODE_OUTPUT 0x01UL
#define GPIO_MODE_ALTER 0x02UL
#define GPIO_MODE_ANALOG 0x03UL
#define GPIO_MODE_AF0 0x02UL
#define GPIO_MODE_AF1 0x12UL
#define GPIO_MODE_AF2 0x22UL
#define GPIO_MODE_AF3 0x32UL
#define GPIO_MODE_AF4 0x42UL
#define GPIO_MODE_AF5 0x52UL
#define GPIO_MODE_AF6 0x62UL
#define GPIO_MODE_AF7 0x72UL
#define GPIO_MODE_AF8 0x82UL
#define GPIO_MODE_AF9 0x92UL
#define GPIO_MODE_AF10 0xA2UL
#define GPIO_MODE_AF11 0xB2UL
#define GPIO_MODE_AF12 0xC2UL
#define GPIO_MODE_AF13 0xD2UL
#define GPIO_MODE_AF14 0xE2UL
#define GPIO_MODE_AF15 0xF5UL

//X = (OSPEEDER<<4) | (PUPDR<<2) | OTYPER
#define GPIO_OTYPER_MASK 0x01UL
#define GPIO_PUPDR_MASK 0x03UL
#define GPIO_OSPEEDR_MASK 0x03UL
#define GPIO_TYPE_OTYPER_ROL 0
#define GPIO_TYPE_PUPDR_ROL 2
#define GPIO_TYPE_OSPEEDR_ROL 4

#define GPIO_TYPE_PP_NP_LS 0x00UL
#define GPIO_TYPE_OD_NP_LS 0x01UL
#define GPIO_TYPE_PP_PU_LS 0x04UL
#define GPIO_TYPE_OD_PU_LS 0x05UL
#define GPIO_TYPE_PP_PD_LS 0x08UL
#define GPIO_TYPE_OD_PD_LS 0x09UL
#define GPIO_TYPE_PP_NP_MS 0x10UL
#define GPIO_TYPE_OD_NP_MS 0x11UL
#define GPIO_TYPE_PP_PU_MS 0x14UL
#define GPIO_TYPE_OD_PU_MS 0x15UL
#define GPIO_TYPE_PP_PD_MS 0x18UL
#define GPIO_TYPE_OD_PD_MS 0x19UL
#define GPIO_TYPE_PP_NP_HS 0x20UL
#define GPIO_TYPE_OD_NP_HS 0x21UL
#define GPIO_TYPE_PP_PU_HS 0x24UL
#define GPIO_TYPE_OD_PU_HS 0x25UL
#define GPIO_TYPE_PP_PD_HS 0x28UL
#define GPIO_TYPE_OD_PD_HS 0x29UL
#define GPIO_TYPE_PP_NP_vS 0x30UL
#define GPIO_TYPE_OD_NP_vS 0x31UL
#define GPIO_TYPE_PP_PU_vS 0x34UL
#define GPIO_TYPE_OD_PU_vS 0x35UL
#define GPIO_TYPE_PP_PD_vS 0x38UL
#define GPIO_TYPE_OD_PD_vS 0x39UL

#ifdef STM32F4
    #include "stm32f4xx.h"

    // NUCLEOF446
    #define PORT_LD2 GPIOA
    #define PIO_LD2 P5

    #define PORT_DRV1EN GPIOA //PA9
    #define PIO_DRV1EN P9
    #define PORT_DRV1DIR GPIOB //PB4
    #define PIO_DRV1DIR P4
    #define PORT_DRV1STEP GPIOA //PA10
    #define PIO_DRV1STEP P10

    #define PORT_DRV2EN GPIOA //PA9
    #define PIO_DRV2EN P9
    #define PORT_DRV2DIR GPIOB //PB10
    #define PIO_DRV2DIR P10
    #define PORT_DRV2STEP GPIOB //PB3
    #define PIO_DRV2STEP P3

    #define PORT_DRV3EN GPIOA //PA9
    #define PIO_DRV3EN P9
    #define PORT_DRV3DIR GPIOA //PA8
    #define PIO_DRV3DIR P8
    #define PORT_DRV3STEP GPIOB //PB5
    #define PIO_DRV3STEP P5

    #define PORT_DRV4EN GPIOA //PA9
    #define PIO_DRV4EN P9
    #define PORT_DRV4DIR GPIOA //PA5
    #define PIO_DRV4DIR P5
    #define PORT_DRV4STEP GPIOA //PA6
    #define PIO_DRV4STEP P6

    #define UART2PORT GPIOA
/*
    //BTT OctopusPro 1.0
    #define LD2PORT (GPIOA)
    #define LD2IO (13)
    #define GPIO_MODER_LD2 (GPIO_MODER_MODER13_0)

    #define PORT_DRVEN (GPIOF) //PF15
    #define PORT_DRV1STEP (GPIOG) //PG0
    #define PORT_DRV1DIR (GPIOG) //PG1
    #define PIO_DRVEN (PIO15)
    #define PIO_DRV1STEP (PIO0)
    #define PIO_DRV1DIR (PIO1)
    #define GPIO_ODR_DRVEN (GPIO_ODR_OD15)
    #define GPIO_ODR_DRV1STEP (GPIO_ODR_ODR_0)
    #define GPIO_ODR_DRV1DIR (GPIO_ODR_ODR_1)

    #define UART2PORT (GPIOA)
*/
#endif

static volatile uint32_t time_ms = 0, tmp = 0;;

#define UART_TXBUF_LEN 100
static volatile char uart_txbuf[UART_TXBUF_LEN];
static volatile uint8_t uart_txbuf_i = 0;
static volatile uint8_t uart_txbuf_o = 0;
#define UART_RXBUF_LEN 100
static volatile char uart_rxbuf[UART_RXBUF_LEN];
#define UART_RXBUF_SLOTS_CNT 5
static volatile uint8_t uart_rxbuf_slot_ready[UART_RXBUF_SLOTS_CNT]; //slot contains some string
static volatile uint8_t uart_rxbuf_slot_start[UART_RXBUF_SLOTS_CNT]; //where strings starts (position of first char in rx_buf)
static volatile uint8_t uart_rxbuf_pos = 0;
//static volatile uint8_t uart_rxbuf_o = 0;
static volatile uint8_t uart_rxbuf_skip = 0;

#define GCMD_PAR_QTY 8
static char gcode_cmd_buf[32];
static char gcode_cmd[GCMD_PAR_QTY];
static int32_t gcode_par[GCMD_PAR_QTY];
static uint8_t gcode_par_dec[GCMD_PAR_QTY];

#define UNIT_DIST_PREC 1000 // distance unit precision: milimeters * x
#define DRIVERS_QTY 4
static int32_t unit_d_prec = UNIT_DIST_PREC;
static char axis_letters[DRIVERS_QTY];
static int32_t position_absolute[DRIVERS_QTY] = {0,0,0,0}; // absolute position in steps
static int32_t motor_resolution[DRIVERS_QTY] = {40,40,40,1}; // motors resolution steps/mm
static uint8_t motor_dir_reverse[DRIVERS_QTY] = {0,0,0,0}; // motor rotation direction: 0 - normal, 1 - reversed
static int32_t motor_max_speed[DRIVERS_QTY] = {400,50,50,50}; // motor max speed [mm/sec]
static int32_t motor_speed[DRIVERS_QTY] = {250,50,50,50}; //motor speed for current move [mm/sec]
static int32_t motor_max_acceleration[DRIVERS_QTY] = {20,20,20,20}; // motor max acceleration [mm/sec2]
static int32_t motor_acceleration[DRIVERS_QTY] = {20,20,20,10}; // motor acceleration [mm/sec2]
static int32_t move_to_pos[DRIVERS_QTY];

void delay_us(int us){
   while (us-- > 0) {
         __asm("nop");
         __asm("nop");
   }
}

void delay_ms(int ms){
   while (ms-- > 0) {
      volatile int x=1000;
      while (x-- > 0)
         __asm("nop");
   }
}

void gpio_pin_mode_set(GPIO_TypeDef * const __restrict__ gpio_port, uint32_t gpio_pin, uint32_t gpio_mode, uint32_t gpio_type){
    //GPIO_MODER - input / output / alternate / analog
    gpio_port->MODER = (gpio_port->MODER & ~(GPIO_MODER_MASK << (2*gpio_pin))) | (((gpio_mode  >> GPIO_MODE_MODER_ROL) & GPIO_MODER_MASK) << (2*gpio_pin));
    //GPIO_OTYPER - output type: push-pull / open dren
    gpio_port->OTYPER = (gpio_port->OTYPER & ~(GPIO_OTYPER_MASK << gpio_pin)) | (((gpio_type >> GPIO_TYPE_OTYPER_ROL) & GPIO_OTYPER_MASK) << gpio_pin);
    //GPIO_OSPEEDR
    gpio_port->OSPEEDR = (gpio_port->OSPEEDR & ~(GPIO_OSPEEDR_MASK << (2*gpio_pin))) | (((gpio_type >> GPIO_TYPE_OSPEEDR_ROL) & GPIO_OSPEEDR_MASK) << (2*gpio_pin));
    //GPIO_PUPDR - no pull / pull up / pull down / (reserved)
    gpio_port->PUPDR = (gpio_port->PUPDR & ~(GPIO_PUPDR_MASK << (2*gpio_pin))) | (((gpio_mode >> GPIO_TYPE_PUPDR_ROL) & GPIO_PUPDR_MASK) << (2*gpio_pin));
    //GPIO_AFRL/H - Alternate function
    if (gpio_pin <= 7){
        gpio_port->AFR[0] = (gpio_port->AFR[0] & ~(GPIO_AFR_MASK << (4*gpio_pin))) | (((gpio_mode >> GPIO_MODE_AFR_ROL) & GPIO_AFR_MASK) << (4*gpio_pin));
    }else{
        gpio_port->AFR[1] = (gpio_port->AFR[1] & ~(GPIO_AFR_MASK << (4*(gpio_pin-8)))) | (((gpio_mode >> GPIO_MODE_AFR_ROL) & GPIO_AFR_MASK) << (4*(gpio_pin-8)));
    }
}

void gpio_pin_set(GPIO_TypeDef * const __restrict__ gpio_port, uint32_t gpio_pin, uint8_t val){
    gpio_port->ODR = ((gpio_port->ODR & ~(1UL << gpio_pin)) | (val << gpio_pin));
}

void gpio_pin_tog(GPIO_TypeDef * const __restrict__ gpio_port, uint32_t gpio_pin){
    gpio_port->ODR ^= (0x01UL << gpio_pin);
}

void config_hw(void){

    //uart_tx_buf = "ok\0";
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;              // enable the clock to GPIO
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;              // enable the clock to GPIO
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;              // enable the clock to GPIO
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;              // enable the clock to GPIO
    __DSB();

    //LD2 - On board LED
    gpio_pin_mode_set(PORT_LD2, PIO_LD2, GPIO_MODE_OUTPUT, GPIO_TYPE_PP_NP_LS);

    //UART2
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;             // enable uart2 clock
    __DSB();
     
    gpio_pin_mode_set(UART2PORT, P2, GPIO_MODE_AF7, GPIO_TYPE_PP_NP_LS); // PA2 TX
    gpio_pin_mode_set(UART2PORT, P3, GPIO_MODE_AF7, GPIO_TYPE_PP_PU_LS); // PA3 RX

    USART2->BRR = 139;              //115200
    USART2->SR &= ~USART_SR_RXNE;
    USART2->SR &= ~USART_SR_TC;
    USART2->CR1 = USART_CR1_UE | USART_CR1_TCIE | USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE;
    NVIC_EnableIRQ(USART2_IRQn); //USART2 interrupt enable

    //systick    
    SysTick_Config(16000); // ms ticks

    //Stepping motor driver ports
    //gpio_pin_set(PORT_DRV1EN, PIO_DRV1EN, 0);
    gpio_pin_mode_set(PORT_DRV1EN, PIO_DRV1EN, GPIO_MODE_OUTPUT, GPIO_TYPE_PP_NP_HS);
    gpio_pin_mode_set(PORT_DRV1DIR, PIO_DRV1DIR, GPIO_MODE_OUTPUT, GPIO_TYPE_PP_NP_HS);
    gpio_pin_mode_set(PORT_DRV1STEP, PIO_DRV1STEP, GPIO_MODE_OUTPUT, GPIO_TYPE_PP_NP_HS);

    //gpio_pin_set(PORT_DRV2EN, PIO_DRV2EN, 0);
    gpio_pin_mode_set(PORT_DRV2EN, PIO_DRV2EN, GPIO_MODE_OUTPUT, GPIO_TYPE_PP_NP_HS);
    gpio_pin_mode_set(PORT_DRV2DIR, PIO_DRV2DIR, GPIO_MODE_OUTPUT, GPIO_TYPE_PP_NP_HS);
    gpio_pin_mode_set(PORT_DRV2STEP, PIO_DRV2STEP, GPIO_MODE_OUTPUT, GPIO_TYPE_PP_NP_HS);

    //gpio_pin_set(PORT_DRV3EN, PIO_DRV3EN, 0);
    gpio_pin_mode_set(PORT_DRV3EN, PIO_DRV3EN, GPIO_MODE_OUTPUT, GPIO_TYPE_PP_NP_HS);
    gpio_pin_mode_set(PORT_DRV3DIR, PIO_DRV3DIR, GPIO_MODE_OUTPUT, GPIO_TYPE_PP_NP_HS);
    gpio_pin_mode_set(PORT_DRV3STEP, PIO_DRV3STEP, GPIO_MODE_OUTPUT, GPIO_TYPE_PP_NP_HS);

}

void uart_transmit_next(void){
    if (uart_txbuf[uart_txbuf_o] != '\0'){
        USART2->DR = uart_txbuf[uart_txbuf_o];
        uart_txbuf[uart_txbuf_o] = '\0';
        uart_txbuf_o++;
    }
    else{
        uart_txbuf_o = 0;
    }
}

void uart_write_str(volatile char text[]){
    uint8_t i = 0;
    while(uart_txbuf_o != 0){}; // wait for empty tx buffer;
    uart_txbuf_i = 0;
    while(text[i] != '\0'){
        uart_txbuf[uart_txbuf_i] = text[i];
        i++;
        uart_txbuf_i++;
        if (uart_txbuf_i >= UART_TXBUF_LEN){
            uart_txbuf_i = 0;
        }
    }
    uart_transmit_next();
}

void uart_write_int32(int32_t num, uint8_t fract_pos){
    uint8_t i = 0;
    char sign = '\0';
    char tmp[] = {"\0\0\0\0\0\0\0\0\0\0\0\0"};

    if (num < 0){
        sign = '-';
        num = abs(num);
    }
    while (num > 0 || i == 0){
        tmp[i] = (char)((num % 10) | 0x30);
        num = num / 10;
        i++;
    }
    if (sign == '-'){
        tmp[i] = sign;
        i++;
    }
    while(uart_txbuf_o != 0){}; // wait for empty tx buffer;
    uart_txbuf_i = 0;
    while (i>0){
        i--;
        uart_txbuf[uart_txbuf_i] = tmp[i];
        uart_txbuf_i++;
        if (uart_txbuf_i >= UART_TXBUF_LEN){
            uart_txbuf_i = 0;
        }
    }
    uart_transmit_next();
}

void uart_write_arr_uint8(char str[], uint8_t arr[], uint8_t n){
    uart_write_str(str);
    uart_write_str("\t");
    for (uint8_t i=0; i<n; i++){
        if (i != 0) uart_write_str("\t");
        uart_write_int32(i, 0);
        uart_write_str(":");
        uart_write_int32((int32_t)arr[i], 0);
    }
    uart_write_str(TEXT_EOL);
}

void uart_write_arr_int32(char str[], int32_t arr[], uint8_t n){
    uart_write_str(str);
    uart_write_str("\t");
    for (uint8_t i=0; i<n; i++){
        if (i != 0) uart_write_str("\t");
        uart_write_int32(i, 0);
        uart_write_str(":");
        uart_write_int32(arr[i], 0);
    }
    uart_write_str(TEXT_EOL);
}

void uart_write_arr_char(char str[], char arr[], uint8_t n){
    char tmp_str_buf[] = {"\0"};
    uart_write_str(str);
    uart_write_str("\t");
    for (uint8_t i=0; i<n; i++){
        if (i != 0) uart_write_str("\t");
        uart_write_int32(i, 0);
        uart_write_str(":");
        tmp_str_buf[0] = arr[i];
        uart_write_str(tmp_str_buf);
    }
    uart_write_str(TEXT_EOL);
}

void move_step(int8_t step[], uint32_t delay){
    //set DIR pins
    gpio_pin_set(PORT_DRV1DIR, PIO_DRV1DIR, ((step[0] >> 1) & 0x01U) ^ motor_dir_reverse[0]);
    gpio_pin_set(PORT_DRV2DIR, PIO_DRV2DIR, ((step[1] >> 1) & 0x01U) ^ motor_dir_reverse[1]);
    gpio_pin_set(PORT_DRV3DIR, PIO_DRV3DIR, ((step[2] >> 1) & 0x01U) ^ motor_dir_reverse[2]);
    delay_us(2);
    //set "1" at STEP pins
    gpio_pin_set(PORT_DRV1STEP, PIO_DRV1STEP, (step[0] & 1U));
    gpio_pin_set(PORT_DRV2STEP, PIO_DRV2STEP, (step[1] & 1U));
    gpio_pin_set(PORT_DRV3STEP, PIO_DRV3STEP, (step[2] & 1U));
    delay_us(delay/2);
    //set "0" at STEP pins
    gpio_pin_set(PORT_DRV1STEP, PIO_DRV1STEP, 0);
    gpio_pin_set(PORT_DRV2STEP, PIO_DRV2STEP, 0);
    gpio_pin_set(PORT_DRV3STEP, PIO_DRV3STEP, 0);
    delay_us(delay/2);
    for(uint8_t i=0; i<DRIVERS_QTY; i++){
        position_absolute[i] += step[i];
    }
}

void move_to(int32_t position_to[]){ //move to absolute position [um]
    int32_t steps_left[DRIVERS_QTY];
    int32_t steps_done[DRIVERS_QTY];
    int32_t step_dir[DRIVERS_QTY]; //1 = "+" dir, -1 = "-" dir
    int32_t step_accum[DRIVERS_QTY];
    int32_t step_ac[DRIVERS_QTY];
    int8_t step_n[DRIVERS_QTY];
    int32_t step_i = 0, step_i_acc = 0; // step_i - count steps to destination position, step_i_acc - count steps with acceleration+
    int32_t tmp_int32 = 0;
    int64_t tmp_int64 = 0;
    int32_t step_speed = 0, step_max_speed;
    int32_t step_delay, step_delay_denom; // delay time applied after each step [us], denominator for calculatin delay time
    int32_t step_acceleration;

    //uart_write_arr_int32("moveto:", position_to, DRIVERS_QTY);
    //uart_write_arr_int32("speed:", motor_speed, DRIVERS_QTY);
    //uart_write_arr_int32("accel:", motor_acceleration, DRIVERS_QTY);

    for (uint8_t i=0; i<DRIVERS_QTY; i++){
        step_dir[i] = 0;
        step_accum[i] = 0;
        step_ac[i] = 0;
        step_n[i] = 0;
        steps_done[i] = 0;
    
        steps_left[i] = (position_to[i] * motor_resolution[i]) - (position_absolute[i] * unit_d_prec);
        if (steps_left[i] >= 0){ //check for moving direction
            step_dir[i] = 1; //set "+" direction
        }else{
            step_dir[i] = -1; //set "-" direction
        }
        steps_left[i] = abs(steps_left[i]);
        tmp_int32 = steps_left[i] % unit_d_prec;
        if (tmp_int32 > (unit_d_prec / 2)){
            steps_left[i] -= tmp_int32;
            steps_left[i] += unit_d_prec;
        }
        if (steps_left[i] > step_i) step_i = steps_left[i]; //find biggest steps qty for calculate divider
    }

    step_i /= unit_d_prec;  //convert to normal
    if (step_i > 0){
        for (uint8_t i=0; i<DRIVERS_QTY; i++){
            tmp_int64 = (int64_t)steps_left[i] * 1000; // *1000 to increase precision
            step_ac[i] =  (int32_t)(tmp_int64 / step_i);
            steps_left[i] /= unit_d_prec;
        }
    }

    if (motor_speed[0] > 0 && motor_acceleration[0] > 0){
        step_max_speed = (motor_speed[0] * 1000) ; //maximum speed *1000 to increase precision
        step_delay_denom = 1000000 * motor_resolution[0]; // microsecond * motor resolution
        step_delay =  step_delay_denom / step_max_speed; // step delay for maximum speed; microsecond * motor resolution / max speed
        step_acceleration = (motor_acceleration[0] * 1000) / motor_resolution[0]; // *1000 to increace precision

        tmp_int32 = unit_d_prec * (1000 / 2); // "1/2" accum, * 1000 to increae precision same as step_ac

        
        while (step_i > 0){ // "move" loop
            if (step_i > step_i_acc && step_speed < step_max_speed){ //speedup, acceleration plus
                step_speed += step_acceleration; //inrease speed with acceleration
                if (step_speed > step_max_speed) step_speed = step_max_speed; // prevent speed over max speed
                step_i_acc++; // increment the speedup steps counter
                step_delay = step_delay_denom / step_speed; // calculate delay in us
            }
            if (step_i < step_i_acc){ // slow down, acceleration minus
                step_speed -= step_acceleration; //decrease speed with acceleration
                if (step_speed < step_acceleration) step_speed = step_acceleration; // prevent too low speed
                step_delay = step_delay_denom / step_speed; // calculate delay in us
            }
            /*uart_transmit_int32(step_i, 0);
            uart_transmit_str(" ");
            uart_transmit_int32(step_i_acc, 0);
            uart_transmit_str(" ");
            uart_transmit_int32(step_speed, 0);
            uart_transmit_str(" ");
            uart_transmit_int32(step_delay, 0);
            uart_transmit_str(TEXT_EOL);
            */
            for (uint8_t i = 0; i<DRIVERS_QTY; i++){
                step_accum[i] += step_ac[i];
                if (steps_left[i] > 0 && step_accum[i] >= tmp_int32){ // make step when accum value is bigger then "1/2" of accumulator
                    step_accum[i] -= tmp_int32 << 1; // minus "1" from accumulator
                    steps_left[i] --;
                    steps_done[i] ++;
                    step_n[i] = step_dir[i]; //"step_dir" equals one step
                }else{
                    step_n[i] = 0;
                }
            }
            move_step(step_n, step_delay);
            step_i--;
        } //end "move" loop
    }

    for (uint8_t i=0; i<DRIVERS_QTY; i++){
        if (steps_left[i] != 0) step_i = 1;
    }
    if (step_i != 0){
        uart_write_str("DEBUG (move to):\r\n");
        uart_write_arr_int32("position: ", position_to, DRIVERS_QTY);
        uart_write_arr_int32("accum:", step_accum, 4);
        uart_write_arr_int32("steps done:", steps_done, 4);
        uart_write_arr_int32("steps left:", steps_left, 4);
        uart_write_arr_int32("position after:", position_absolute, 4);
    }
}

/*void move_to_xyz(int32_t ax, int32_t ay, int32_t az){
    int32_t new_pos[DRIVERS_QTY];
    
    for (uint8_t i=0; i<DRIVERS_QTY; i++){
        new_pos[i] = 0;
    }

    new_pos[0] = ax;
    new_pos[1] = ay;
    new_pos[2] = az;

    move_to(new_pos);

    uart_write_str(TEXT_OK);
}*/

void gcode_parse(char gcode_parse_buf[]){
    uint8_t i = 0;
    uint8_t par_i = 0;
    int32_t parse_sign;
    uint8_t parse_dec;

    for (i=0; i<GCMD_PAR_QTY; i++){
        gcode_cmd[i] = '\0';
        gcode_par[i] = 0;
        gcode_par_dec[i] = 0;
    }

    //uart_write_str(gcode_parse_buf);
    //uart_write_str(TEXT_EOL);

    i = 0;
    parse_sign = 1;
    parse_dec = 0;
    while (gcode_parse_buf[i] != '\0'){
        switch (gcode_parse_buf[i])
        {
        case 'A' ... 'Z':
            gcode_cmd[par_i] = gcode_parse_buf[i];
            parse_sign = 1;
            parse_dec = 0;
            break;
        case '0' ... '9':
            gcode_par[par_i] *= 10;
            gcode_par[par_i] += (((int8_t)gcode_parse_buf[i]) & 0x0F)*parse_sign;
            gcode_par_dec[par_i] += parse_dec; 
            break;
        case '.':
            parse_dec = 1;
            break;
        case ' ':
            if (par_i <= GCMD_PAR_QTY){
                if (gcode_cmd[par_i] != '\0') par_i++;
            }else{
                uart_write_str(TEXT_ERROR_GC_TOOMANYPARAM);
            }
            break;
        case '-':
            parse_sign = -1;
            break;
        default:
            break;
        }
        i++;
    }
    //uart_write_arr_char("par L:", gcode_cmd, GCMD_PAR_QTY);
    //uart_write_arr_int32("par v:", gcode_par, GCMD_PAR_QTY);
    //uart_write_arr_uint8("par d:", gcode_par_dec, GCMD_PAR_QTY);
}

int main(void){

    config_hw();

    //uart_rxbuf buffers clear
    for (uint8_t i=0; i<UART_RXBUF_LEN-1; i++){
        uart_rxbuf[i] = '\0';
    }
    for (uint8_t i=0; i<UART_RXBUF_SLOTS_CNT-1; i++){
        uart_rxbuf_slot_start[i] = 0;
        uart_rxbuf_slot_ready[i] = 0;
    }

    axis_letters[0] = 'X';
    axis_letters[1] = 'Y';
    axis_letters[2] = 'Z';
    axis_letters[3] = 'A';
    /*for (int i = 0; i<(motor_resolution[0] * 10); i++){
        move_step(1,0,0);
    }*/
    uart_write_str(TEXT_ALIVE);

    //char tmp_str[] = {"\0\0"};
    for(;;){
        if (uart_rxbuf_slot_ready[0] == 1){
            //move string from uart rx buffer to gcode parse buffer
            uint8_t i = uart_rxbuf_slot_start[0];
            uint8_t j = 0;
            while(uart_rxbuf[i] != '\0'){
                gcode_cmd_buf[j] = uart_rxbuf[i];
                //uart_write_int32(i, 0);
                //tmp_str[0] = uart_rxbuf[i];
                //uart_write_str(tmp_str);
                i++;
                j++;
                gcode_cmd_buf[j] = '\0'; //clear next pos because '\0' is end of string
                if (i >= UART_RXBUF_LEN) i = 0;
            }
            uart_rxbuf_slot_ready[0] = 0;
            if (j>0){
                //uart_write_str("$parsing\r\n");
                gcode_parse(gcode_cmd_buf);

                switch (gcode_cmd[0])
                {
                case 'G':
                    switch (gcode_par[0])
                    {
                    case 1: //G1
                        for(i=0; i<DRIVERS_QTY; i++){
                            move_to_pos[i] = (position_absolute[i] * 1000) / motor_resolution[i];
                        }
                        for(j=1; j<GCMD_PAR_QTY; j++){
                            for(i=0; i<DRIVERS_QTY; i++){
                                if(gcode_cmd[j] == axis_letters[i]) move_to_pos[i] = gcode_par[j];
                            }
                            if(gcode_cmd[j] == 'F'){
                                if (gcode_par[j] <= motor_max_speed[0]){
                                    motor_speed[0] = gcode_par[j];
                                }else{
                                    motor_speed[0] = motor_max_speed[0];
                                }
                            }
                        }
                        move_to(move_to_pos);
                        uart_write_str(TEXT_OK);
                        break;
                    case 21: //G21
                        uart_write_str(TEXT_OK);
                        break;
                    case 28: //G28
                        uart_write_str(TEXT_OK);
                        break;
                    case 90: //G90
                        uart_write_str(TEXT_OK);
                        break;
                    default:
                        uart_write_str(TEXT_ERROR_GC_DONTRECOGNIZE);
                        break;
                    } 
                    break; //end case "G"
                case 'M':
                    switch (gcode_par[0])
                    {
                    case 106:
                        uart_write_str(TEXT_OK);
                        break;
                    case 114:
                        for (i=0; i<DRIVERS_QTY; i++){
                            
                            uart_write_str("X:0.000 Y:0.000 Z:0.000 A:0.000\r\n");
                        }
                        uart_write_str(TEXT_OK);
                        break;
                    case 115:
                        uart_write_str("FIRMWARE_NAME: SimpleGcodeProcessor FIMWARE_VERSION: 1.0\r\n");
                        uart_write_str(TEXT_OK);
                        break;
                    case 204:
                        for(i=0; i<DRIVERS_QTY; i++){
                            for(j=1; j<GCMD_PAR_QTY; j++){
                                if(gcode_cmd[j] == 'S') {
                                    if (gcode_par[j] <= motor_max_acceleration[0]){
                                        motor_acceleration[0] = gcode_par[j];
                                    }else{
                                        motor_acceleration[0] = motor_max_acceleration[0];
                                    }
                                }
                            }
                        }
                        uart_write_str(TEXT_OK);
                        break;
                    case 400:
                        uart_write_str(TEXT_OK);
                        break;
                    default:
                        uart_write_str(TEXT_ERROR_GC_DONTRECOGNIZE);
                        break;
                    } 
                    break; //end case "M"

                default:
                    uart_write_str(TEXT_ERROR_GC_PARSINGFAIL);
                    break;
                }

            }
        }
    }
    return 0;
}

void SysTick_Handler(void){
    time_ms++;
    //gpio_pin_tog (PORT_LD2, PIO_LD2);  // toggle diode
}

void USART2_IRQHandler(void){
    if (USART2->SR & USART_SR_TC){
        USART2->SR &= ~USART_SR_TC;
        uart_transmit_next();
    }
    if (USART2->SR & USART_SR_RXNE){
        USART2->SR &= ~USART_SR_RXNE;
        char tmp;
        tmp = USART2->DR;
        switch (tmp)
        {
        case ';': //skip comment string from ";" char to the "\n" (end of line)
            uart_rxbuf_skip = 1;
        case '\0':
        case '\r':
            break;
        case '\n':
            uart_rxbuf[uart_rxbuf_pos] = '\0';
            uart_rxbuf_pos++;
            uart_rxbuf_pos = 0; // remove when ring buffer
            uart_rxbuf_slot_ready[0] = 1;
            uart_rxbuf_slot_start[0] = uart_rxbuf_pos; // for ring buffer replace "0" with "i+1" to write begin of string (uart_rxbuf_pos) to next slot
            uart_rxbuf_skip = 0;
            break;
        default:
            if(uart_rxbuf_skip == 0){ // move char to RX buffer
                uart_rxbuf[uart_rxbuf_pos] = tmp;
                uart_rxbuf_pos++;
                if (uart_rxbuf_pos >= UART_RXBUF_LEN) uart_rxbuf_pos = 0;
            }
            break;
        }
    }
}