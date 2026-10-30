// 260722 : 신규 PCB PICO 첫 코드(MOTOR + ADC + WATER(추후 추가) + FAN)
// enable Low : off / High : on
// dir 반전 / pwm만 반전 X

/*
p05L
*** 버전 관리 ****

ver 1: 260902 저격수 대회 예비 후 제원 맞게 수정함.
ver 2: P05L -> tx enc -> m/s , reset 500ms
*/

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include <stdio.h>
#define _USE_MATH_DEFINES
#include <math.h>

#define LED             PICO_DEFAULT_LED_PIN
// ++++++++++++++ UART ++++++++++++++++++
#define UART_TX_PIN_NUMBER              (0)
#define UART_RX_PIN_NUMBER              (1)

// ++++++++++++ ADC +++++++++++++
#define BAT_PIN1        20 // 75 under
#define BAT_PIN2        26 // 50 under
#define BAT_PIN3        27 // 25 under
#define BAT_PIN4        28 // 20 under

// ++++++++++ FAN +++++++++++++
#define FAN             7

// ++++++++++ ALRAM RESET ++++++++++
#define ALARM_RESET     6  

// +++++++++ WATER ++++++++++++
#define WATER           19  

// ++++++++++++ MOTOR & ENCODER +++++++++++++
#define R_ENABLE        18
#define R_DIR           17
#define R_PWM           16

#define L_ENABLE        13
#define L_DIR           14
#define L_PWM           15

#define R_ENCODER_A     8
#define R_ENCODER_B     9

#define L_ENCODER_A     2
#define L_ENCODER_B     3

#define PICO_SYS_CLK_kHz                (125000)            // 125000 kHz
#define PICO_SYS_CLK                    (125000000)         // 125 MHz
#define PWM_DUTY_CLK                    (2000)              // 2 KHz
#define Unit_K                          (1000)
#define Unit_M                          (1000000)

#define UART_ID                         (uart0)
#define BAUD_RATE                       (115200)
#define DATA_BITS                       (8)
#define STOP_BITS                       (1)
#define PARITY                          (UART_PARITY_NONE)

#define UART_DATA_LENGTH                (6)
#define UART_START_FLAG                 (0xff)

#define ENC_HZ                          20.0f
#define DT                              (1.0f/ENC_HZ)

//P05L & P05 ROBOT SETTING
#define TRACK_WIDTH                     0.782
#define WHEEL_RADIOUS                   0.19
#define PPR                             100
#define GEAR_RATIO                      20

// ++++++++++ 침수 센서 환산용 전압 기준 추가 ++++++++++
#define PICO_VREF_mV   3242 // pico 보통 전압 (실제 멀티미터 측정값 대입 시 정확도 증가) 
#define UNO_VREF_mV    5000 // uno 보통 전압

// ++++++++++ Delcare Fuction ++++++++++++++++
void StartSignal(void);
void UART_Rx_Handler(void);
void ConfigureUart(void);
void ConfigureGpio(void);
void tx_uart_data(float velocity, float angle, float l_enc_sec, float r_enc_sec, int bat);
void encoder_irq_handler(uint gpio, uint32_t events);
bool repeating_timer_callback(struct repeating_timer* t);
void reset_motor_if_uart_timeout(void);
void alarm_reset(int alarm_reset);
void configure_bat(void);
static inline uint8_t invert_dir(uint8_t v);

// ++++++++++++ UART Static Val ++++++++++++++++

static uint8_t Receive_Data[UART_DATA_LENGTH]  = {0, 0, 0, 0, 0, 0};    // enable, l_dir, l_pwm, r_dir, r_pwm, rst    
volatile static uint        Receive_Done                    = 0;
volatile static uint        Receive_Data_Count              = 0;
volatile static uint8_t     Receive_Char                    = 0;
volatile static uint        Frame_Count                     = 0;
static char buffer[128];

// ++++++++++++ Encoder Static Val ++++++++++++++
volatile int L_encoderPos = 0;
volatile int R_encoderPos = 0;

static int L_encoderOld = 0;
static int R_encoderOld = 0;

static float L_encoderSec = 0.;
static float R_encoderSec = 0.;

static float left_velocity = 0.;
static float right_velocity = 0.;
static float angular_velocity = 0.;
static float velocity = 0.;

// ++++++++++++ Shut down Sbc Check ++++++++++++++
static absolute_time_t last_uart_time;
static const uint32_t check_uart_time_out = 2000000;

// ++++++++++++ etc.(alarm, water, adc, fan...) +++++++++++++++++++
static int alarm_cmd = 0;
static int bat = 0;

int main()
{
    stdio_init_all();
    sleep_ms(10);
    
    set_sys_clock_khz(PICO_SYS_CLK_kHz, true);
    busy_wait_ms(100);

    ConfigureUart();
    sleep_ms(10);

    ConfigureGpio();
    sleep_ms(10);

    sleep_ms(10);

    StartSignal();
    last_uart_time = get_absolute_time();
    sleep_ms(10);

    struct repeating_timer timer;
    add_repeating_timer_ms((int)(1000.0/ENC_HZ), repeating_timer_callback, NULL, &timer);

    while(1) {
        tight_loop_contents();
        reset_motor_if_uart_timeout();   

        configure_bat();

        if (Receive_Done > 0) {
            if (Receive_Data[0] == 0)
            {
                gpio_put(L_ENABLE, 0);
                gpio_put(R_ENABLE, 0);
                gpio_put(L_DIR, 0);
                gpio_put(R_DIR, 0);
                pwm_set_gpio_level(L_PWM, 0);
                pwm_set_gpio_level(R_PWM, 0);
            } else {
                gpio_put(L_ENABLE, 1);
                gpio_put(L_DIR, invert_dir(Receive_Data[1]));
                pwm_set_gpio_level(L_PWM, Receive_Data[2]);

                gpio_put(R_ENABLE, 1);
                gpio_put(R_DIR, invert_dir(Receive_Data[3]));
                pwm_set_gpio_level(R_PWM, Receive_Data[4]);
            }

            alarm_cmd = Receive_Data[5];
            if(alarm_cmd == 1 ) alarm_reset(alarm_cmd);

            left_velocity = ((L_encoderSec / PPR) * (2*M_PI)) * WHEEL_RADIOUS / GEAR_RATIO;   //((초당 엔코더 펄스수 / 엔코더 레졸루션) * (2*pi) * (바퀴반지름 / 감속비)) 
            right_velocity = ((R_encoderSec / PPR) * (2*M_PI)) * WHEEL_RADIOUS / GEAR_RATIO;
            
            velocity = (left_velocity + right_velocity) / 2;
            angular_velocity = (right_velocity - left_velocity) / TRACK_WIDTH;
            
            tx_uart_data(velocity, angular_velocity, left_velocity, right_velocity, bat);
            last_uart_time = get_absolute_time();
            Receive_Done = 0;            
        }  
        Frame_Count++;
        busy_wait_ms(1);
    }
    return 0;
}

void StartSignal(void){
    gpio_put(LED, 1);
    sleep_ms(2000);
    gpio_put(LED, 0);
    sleep_ms(100);
}
void UART_Rx_Handler(void){
    irq_set_enabled(UART0_IRQ, false);

    if (uart_is_readable_within_us(UART_ID, 100))
    {
        Receive_Char = uart_getc(UART_ID);
    }

    if (Receive_Char == UART_START_FLAG)
    {
        Receive_Data_Count = 0x0;
    }
    else
    {
        if (Receive_Data_Count < UART_DATA_LENGTH)
        {
            Receive_Data[Receive_Data_Count] = Receive_Char;

            Receive_Data_Count++;

            if (Receive_Data_Count == UART_DATA_LENGTH)
            {
                Receive_Done = 1;
            }
        }
    }

    irq_set_enabled(UART0_IRQ, true);
}
void ConfigureUart(void){
    gpio_set_function(UART_TX_PIN_NUMBER, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN_NUMBER, GPIO_FUNC_UART);

    uart_init(UART_ID, BAUD_RATE);

    uart_set_hw_flow(UART_ID, false, false);

    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    uart_set_fifo_enabled(UART_ID, false);

    irq_set_exclusive_handler(UART0_IRQ, UART_Rx_Handler);
    irq_set_enabled(UART0_IRQ, true);

    uart_set_irq_enables(UART_ID, true, false);
}
void ConfigureGpio(void){
    const float pwm_freq_hz = 5000.0f;

    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);

    gpio_init(R_ENABLE);
    gpio_set_dir(R_ENABLE, GPIO_OUT);
    gpio_put(R_ENABLE, 0);

    gpio_init(R_DIR);
    gpio_set_dir(R_DIR, GPIO_OUT);

    gpio_init(L_ENABLE);
    gpio_set_dir(L_ENABLE, GPIO_OUT);
    gpio_put(L_ENABLE, 0);

    gpio_init(L_DIR);
    gpio_set_dir(L_DIR, GPIO_OUT);   

    // PWM SETTING
    gpio_set_function(R_PWM, GPIO_FUNC_PWM);
    uint R_slice_num = pwm_gpio_to_slice_num(R_PWM);
    uint R_chan      = pwm_gpio_to_channel(R_PWM);
    
    gpio_set_function(L_PWM, GPIO_FUNC_PWM);
    uint L_slice_num = pwm_gpio_to_slice_num(L_PWM);
    uint L_chan      = pwm_gpio_to_channel(L_PWM);

    pwm_set_wrap(R_slice_num, 255);
    pwm_set_wrap(L_slice_num, 255);

    float div = (float)PICO_SYS_CLK / ((255 + 1.0f) * pwm_freq_hz);
    if (div < 1.0f) div = 1.0f;
    if (div > 255.0f) div = 255.0f;

    pwm_set_clkdiv(R_slice_num, div);
    pwm_set_clkdiv(L_slice_num, div);

    pwm_set_gpio_level(R_PWM, 0);
    pwm_set_gpio_level(L_PWM, 0);

    // pwm invert
    // if (R_chan == PWM_CHAN_A) {
    //     pwm_set_output_polarity(R_slice_num, true, false);
    // } else {
    //     pwm_set_output_polarity(R_slice_num, false, true);
    // }

    // if (L_chan == PWM_CHAN_A) {
    //     pwm_set_output_polarity(L_slice_num, true, false);
    // } else {
    //     pwm_set_output_polarity(L_slice_num, false, true);
    // }

    pwm_set_enabled(R_slice_num, true);
    pwm_set_enabled(L_slice_num, true);     

    gpio_init(L_ENCODER_A);
    gpio_set_dir(L_ENCODER_A, GPIO_IN);
    gpio_pull_down(L_ENCODER_A);
    gpio_set_irq_enabled_with_callback(L_ENCODER_A, GPIO_IRQ_EDGE_RISE, true, &encoder_irq_handler);

    gpio_init(L_ENCODER_B);
    gpio_set_dir(L_ENCODER_B, GPIO_IN);
    gpio_pull_down(L_ENCODER_B);

    gpio_init(R_ENCODER_A);
    gpio_set_dir(R_ENCODER_A, GPIO_IN);
    gpio_pull_down(R_ENCODER_A);
    gpio_set_irq_enabled(R_ENCODER_A, GPIO_IRQ_EDGE_RISE, true); 

    gpio_init(R_ENCODER_B);
    gpio_set_dir(R_ENCODER_B, GPIO_IN);
    gpio_pull_down(R_ENCODER_B);

    // ++++++++++ 알람 리셋 추가 ++++++++++
    gpio_init(ALARM_RESET);
    gpio_set_dir(ALARM_RESET, GPIO_OUT);
    gpio_pull_up(ALARM_RESET);
    gpio_put(ALARM_RESET, 1); // 기본 high

    // +++++++++bat 츠기+++++++
    gpio_init(BAT_PIN1);
    gpio_set_dir(BAT_PIN1, GPIO_IN);
    gpio_init(BAT_PIN2);
    gpio_set_dir(BAT_PIN2, GPIO_IN);
    gpio_init(BAT_PIN3);
    gpio_set_dir(BAT_PIN3, GPIO_IN);
    gpio_init(BAT_PIN4);
    gpio_set_dir(BAT_PIN4, GPIO_IN);
}

void tx_uart_data(float vel, float ang, float l_es, float r_es, int bat){
    sprintf(buffer, "%.7f,%.7f,%.2f,%.2f,%d\n", vel, ang, l_es, r_es, bat);
    uart_puts(UART_ID, buffer);
}
void encoder_irq_handler(uint gpio, uint32_t events){
    if (gpio == L_ENCODER_A)    L_encoderPos += (gpio_get(L_ENCODER_A)==gpio_get(L_ENCODER_B)?1:-1);
    else if (gpio == R_ENCODER_A)   R_encoderPos += (gpio_get(R_ENCODER_A)==gpio_get(R_ENCODER_B)?-1:1);
}
bool repeating_timer_callback(struct repeating_timer* t){
    float dL = (float)(L_encoderPos - L_encoderOld);
    float dR = (float)(R_encoderPos - R_encoderOld);

    L_encoderOld = L_encoderPos;
    R_encoderOld = R_encoderPos;

    L_encoderSec = dL / DT;
    R_encoderSec = dR / DT;

    return true;
}
void reset_motor_if_uart_timeout(void){
    if (absolute_time_diff_us(last_uart_time, get_absolute_time()) >= check_uart_time_out) {
        gpio_put(L_ENABLE, 0);
        gpio_put(R_ENABLE, 0);
        gpio_put(L_DIR, 0);
        gpio_put(R_DIR, 0);
        pwm_set_gpio_level(L_PWM, 0);
        pwm_set_gpio_level(R_PWM, 0);
    }
}
void alarm_reset(int alarm_reset){
    gpio_put(ALARM_RESET, 0);
    sleep_ms(500);       
    gpio_put(ALARM_RESET, 1);
    sleep_ms(500);
}
void configure_bat(void)
{
    int b1 = gpio_get(BAT_PIN1);
    int b2 = gpio_get(BAT_PIN2);
    int b3 = gpio_get(BAT_PIN3);
    int b4 = gpio_get(BAT_PIN4);

    if (b1 == 1 && b2 == 1 && b3 == 1 && b4 == 1) {
        bat = 4;
    } else if (b1 == 0 && b2 == 1 && b3 == 1 && b4 == 1) {
        bat = 3;
    } else if (b1 == 0 && b2 == 0 && b3 == 1 && b4 == 1) {
        bat = 2;
    } else if (b1 == 0 && b2 == 0 && b3 == 0 && b4 == 1) {
        bat = 1;
    } else if (b1 == 0 && b2 == 0 && b3 == 0 && b4 == 0) {
        bat = 0;
    }
}
static inline uint8_t invert_dir(uint8_t v) {
    return (v == 0) ? 1 : 0;
}
