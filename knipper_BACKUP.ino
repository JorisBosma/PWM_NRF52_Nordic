#include <NRF.h>

#define TRIG_PIN 2
#define PWM_PIN 3
#define PWM_PIN2 4
#define PWM_PIN3 28
#define PWM_PIN4 29
#define PWM_PIN5 30
#define PWM_NUM_CH 2
#define PWM_COUNTER_TOP 20200
#define PWM_MAX 2525
#define PWM_MIN 1010
#define PWM_RANGE (PWM_MAX-PWM_MIN)
#define STAP_GROOTE (PWM_RANGE/100)

typedef enum
{
    /*lint -save -e30 -esym(628,__INTADDR__)*/
    // NRF_PWM_TASK_START    = offsetof(NRF_PWM_Type, TASKS_START),
    NRF_PWM_TASK_STOP     = offsetof(NRF_PWM_Type, TASKS_STOP),
    NRF_PWM_TASK_SEQSTART0 = offsetof(NRF_PWM_Type, TASKS_SEQSTART[0]),
    NRF_PWM_TASK_SEQSTART1 = offsetof(NRF_PWM_Type, TASKS_SEQSTART[1]),
    NRF_PWM_TASK_NEXTSTEP = offsetof(NRF_PWM_Type, TASKS_NEXTSTEP),
    /*lint -restore*/
} nrf_pwm_task_t;


/**
 * @enum nrf_pwm_event_t
 * @brief Pwm events.
 */
typedef enum
{
    /*lint -save -e30*/
    NRF_PWM_EVENT_STOPPED = offsetof(NRF_PWM_Type, EVENTS_STOPPED),
    NRF_PWM_EVENT_SEQSTARTED0 = offsetof(NRF_PWM_Type, EVENTS_SEQSTARTED[0]),
    NRF_PWM_EVENT_SEQSTARTED1 = offsetof(NRF_PWM_Type, EVENTS_SEQSTARTED[1]),
    NRF_PWM_EVENT_SEQEND0 = offsetof(NRF_PWM_Type, EVENTS_SEQEND[0]),
    NRF_PWM_EVENT_SEQEND1 = offsetof(NRF_PWM_Type, EVENTS_SEQEND[1]),
    NRF_PWM_EVENT_PWNPERIODEND = offsetof(NRF_PWM_Type, EVENTS_PWMPERIODEND),
    NRF_PWM_EVENT_LOOPSDONE = offsetof(NRF_PWM_Type, EVENTS_LOOPSDONE),
    /*lint -restore*/
} nrf_pwm_event_t;

/**
 * @enum nrf_pwm_channel_t
 * @brief Pwm channels.
 */
typedef enum
{
    NRF_PWM_CHANNEL0 = 0, /**< Pwm channel 0. */
    NRF_PWM_CHANNEL1,     /**< Pwm channel 1. */
    NRF_PWM_CHANNEL2,     /**< Pwm channel 2. */
    NRF_PWM_CHANNEL3      /**< Pwm channel 3. */
} nrf_pwm_channel_t;


/**
 * @enum nrf_pwm_channel_t
 * @brief Pwm channels.
 */
typedef enum
{
    NRF_PWM_PRESCALER_DIV_1=0,  // 0 Divide by 1 ( 16MHz)
    NRF_PWM_PRESCALER_DIV_2=1,  // 1 Divide by 2 ( 8MHz)
    NRF_PWM_PRESCALER_DIV_4=2,    // 2 Divide by 4 ( 4MHz)
    NRF_PWM_PRESCALER_DIV_8=3,    // 3 Divide by 8 ( 2MHz)
    NRF_PWM_PRESCALER_DIV_16=4,   // 4 Divide by 16 ( 1MHz)
    NRF_PWM_PRESCALER_DIV_32,   // 5 Divide by 32 ( 500kHz)
    NRF_PWM_PRESCALER_DIV_64,   // 6 Divide by 64 ( 250kHz)
    NRF_PWM_PRESCALER_DIV_128,  // 7 Divide by 128 ( 125kHz)
} nrf_pwm_prescaler_t;




/**
 * @enum nrf_pwm_mode_t
 * @brief Pwm modes.
 */
typedef enum
{
    NRF_PWM_MODE_UP = 0, /**< Pwm mode 0. */
    NRF_PWM_MODE_UPANDDOWN,     /**< Pwm mode 1. */
} nrf_pwm_mode_t;


/**
 * @enum nrf_pwm_decoder_load_t
 * @brief Pwm DECODER load field value.
 */
typedef enum
{
    NRF_PWM_DECODER_LOAD_COMMON = 0, // 0
    NRF_PWM_DECODER_LOAD_GROUPED,    // 1
    NRF_PWM_DECODER_LOAD_INDIVIDUAL, // 2
    NRF_PWM_DECODER_LOAD_WAVEFORM ,  // 3

} nrf_pwm_decoder_load_t;


/**
 * @enum nrf_pwm_decoder_mode_t
 * @brief Pwm DECODER mode field value.
 */
typedef enum
{
    NRF_PWM_DECODER_MODE_REFRESHCOUNT = 0, // 0
    NRF_PWM_DECODER_MODE_NEXTSTEP,        // 1

} nrf_pwm_decoder_mode_t;

/**
 * @enum nrf_pwm_sequence_t
 * @brief Pwm DECODER sequence number.
 */
typedef enum
{
    NRF_PWM_SEQUENCE0 = 0, // 0
    NRF_PWM_SEQUENCE1,     // 1

} nrf_pwm_sequence_t;





// Helper functions


/**
 * @brief Function for activating a specific pwm task.
 *
 * @param NRF_PWMx Pwm instance.
 *
 * @param pwm_task Pwm task.
 */
void nrf_pwm_task_trigger(NRF_PWM_Type * NRF_PWMx, nrf_pwm_task_t pwm_task)
{
    *((volatile uint32_t *)((uint8_t *)NRF_PWMx + (uint32_t)pwm_task)) = 0x1UL;
}


/**
 * @brief Function for clearing a specific pwm event.
 *
 * @param NRF_PWMx Pwm instance.
 *
 * @param pwm_event Pwm event to clear.
 */
void nrf_pwm_event_clear(NRF_PWM_Type * NRF_PWMx,
                                           nrf_pwm_event_t pwm_event)
{
    *((volatile uint32_t *)((uint8_t *)NRF_PWMx + (uint32_t)pwm_event)) = 0x0UL;
}

/**
 * @brief Function for returning the state of a specific event.
 *
 * @param NRF_PWMx Pwm instance.
 *
 * @param pwm_event Pwm event to check.
 */
bool nrf_pwm_event_check(NRF_PWM_Type * NRF_PWMx,
                                           nrf_pwm_event_t pwm_event)
{
    return (bool)*(volatile uint32_t *)((uint8_t *)NRF_PWMx + (uint32_t)pwm_event);
}


/**
 * @brief Function for setting pin for a specific channel
 *
 * @param NRF_PWMx Pwm instance.
 *
 * @param channel Channel number
 *
 * @param pin_number Pin number.
 */
void nrf_pwm_pselout_set(NRF_PWM_Type * NRF_PWMx, nrf_pwm_channel_t channel,uint32_t pin_number)
{
    NRF_PWMx->PSEL.OUT[channel] = (pin_number << PWM_PSEL_OUT_PIN_Pos) |
        (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
}

/**
 * @brief Function to enable a PWM
 *
 * @param NRF_PWMx Pwm instance.
 *
 */
void nrf_pwm_enable(NRF_PWM_Type * NRF_PWMx)
{
    NRF_PWMx->ENABLE = 1;
}

/**
 * @brief Function to disable a PWM
 *
 * @param NRF_PWMx Pwm instance.
 *
 */
void nrf_pwm_disable(NRF_PWM_Type * NRF_PWMx)
{
    NRF_PWMx->ENABLE = 0;
}

/**
 * @brief Function for setting the counter mode
 *
 * @param NRF_PWMx Pwm instance.
 *
 *
 * @param mode Counter mode.
 */

void nrf_pwm_mode_set(NRF_PWM_Type * NRF_PWMx, nrf_pwm_mode_t mode)
{
    NRF_PWMx->MODE = mode;
}

/**
 * @brief Function for setting the prescaler
 *
 * @param NRF_PWMx Pwm instance.
 *
 *
 * @param mode Prescaler mode.
 */
 void nrf_pwm_prescaler_set(NRF_PWM_Type * NRF_PWMx, nrf_pwm_prescaler_t prescaler)
{
    NRF_PWMx->PRESCALER = prescaler;
}


/**
 * @brief Function for setting the COUNTERTOP register
 *
 * @param NRF_PWMx Pwm instance.
 *
 *
 * @param countertop COUNTERTOP value.
 */
 void nrf_pwm_countertop_set(NRF_PWM_Type * NRF_PWMx, uint32_t countertop)
{
    NRF_PWMx->COUNTERTOP = countertop;
}


/**
 * @brief Function for setting the LOOP register
 *
 * @param NRF_PWMx Pwm instance.
 *
 *
 * @param loop LOOP value.
 */
 void nrf_pwm_loop_set(NRF_PWM_Type * NRF_PWMx, uint32_t loop)
{
    NRF_PWMx->LOOP = loop;
}


/**
 * @brief Function for setting the DECODER register
 *
 * @param NRF_PWMx Pwm instance.
 *
 *
 * @param load LOAD value.
 * @param mode MODE value.
 */
 void nrf_pwm_decoder_set(NRF_PWM_Type * NRF_PWMx,
                                      nrf_pwm_decoder_load_t load,
                                      nrf_pwm_decoder_mode_t mode)
{
    NRF_PWMx->DECODER = (load << PWM_DECODER_LOAD_Pos) |
        (mode << PWM_DECODER_MODE_Pos);

}



/**
 * @brief Function for setting the SEQ register
 *
 * @param NRF_PWMx Pwm instance.
 *
 *
 * @param seq Sequence number.
 *
 * @param ptr Beginning address in Data RAM
 *
 * @param cnt Amount of values (duty cycles)
 *
 * @param refresh Amount of additional PWM periods between samples loaded to compare register (load every CNT+1 PWM periods)
 *
 * @param enddelay Time added after the sequence in PWM periods
 *
 */
 void nrf_pwm_seq_set(NRF_PWM_Type * NRF_PWMx,
                                     nrf_pwm_sequence_t seq,
                                     uint32_t ptr,
                                     uint32_t cnt,
                                     uint32_t refresh,
                                     uint32_t enddelay)
{
    NRF_PWMx->SEQ[seq].PTR = ptr;
    NRF_PWMx->SEQ[seq].CNT = cnt;
    NRF_PWMx->SEQ[seq].REFRESH = refresh;
    NRF_PWMx->SEQ[seq].ENDDELAY = enddelay;
}

  uint32_t duty_cycle = 0;
  int32_t  duty_cycle_inc = 1;
  uint16_t pwm_seq[4];
  uint16_t pwm_seq1;
  //int i = 1;
 // int inc = 1;
 

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(PWM_PIN2, OUTPUT);

  //PWM 0 SETUP-----------------------------------------------------------------------------------------------
    nrf_pwm_pselout_set(NRF_PWM0, NRF_PWM_CHANNEL0, PWM_PIN);
    nrf_pwm_pselout_set(NRF_PWM0, NRF_PWM_CHANNEL1, PWM_PIN2);
    nrf_pwm_pselout_set(NRF_PWM0, NRF_PWM_CHANNEL2, PWM_PIN3);
    nrf_pwm_pselout_set(NRF_PWM0, NRF_PWM_CHANNEL3, PWM_PIN4);
    nrf_pwm_enable(NRF_PWM0);
    nrf_pwm_mode_set(NRF_PWM0, NRF_PWM_MODE_UP);
    nrf_pwm_prescaler_set(NRF_PWM0, NRF_PWM_PRESCALER_DIV_16);
    nrf_pwm_countertop_set(NRF_PWM0, PWM_COUNTER_TOP);
    nrf_pwm_loop_set(NRF_PWM0, 0); // disabled
    nrf_pwm_decoder_set(NRF_PWM0,
                        NRF_PWM_DECODER_LOAD_INDIVIDUAL, /* 1st for ch1, 2nd for ch2, etc.*/
                        NRF_PWM_DECODER_MODE_REFRESHCOUNT); /* YOU NEED TO FILL ALL REGISTERS!!! */

    nrf_pwm_seq_set(NRF_PWM0, NRF_PWM_SEQUENCE0, (uint32_t)&pwm_seq,
                    4, /* Count, amount of channels in use */
                    1, /* Refresh */
                    0);

  //PWM 1 SETUP-----------------------------------------------------------------------------------------------
    nrf_pwm_pselout_set(NRF_PWM1, NRF_PWM_CHANNEL0, PWM_PIN5);
    nrf_pwm_enable(NRF_PWM1);
    nrf_pwm_mode_set(NRF_PWM1, NRF_PWM_MODE_UP);
    nrf_pwm_prescaler_set(NRF_PWM1, NRF_PWM_PRESCALER_DIV_16);
    nrf_pwm_countertop_set(NRF_PWM1, PWM_COUNTER_TOP);
    nrf_pwm_loop_set(NRF_PWM1, 0); // disabled
    nrf_pwm_decoder_set(NRF_PWM1,
                        NRF_PWM_DECODER_LOAD_COMMON, /* 1st for ch1, 2nd for ch2, etc.*/
                        NRF_PWM_DECODER_MODE_REFRESHCOUNT); /* YOU NEED TO FILL ALL REGISTERS!!! */

    nrf_pwm_seq_set(NRF_PWM1, NRF_PWM_SEQUENCE0, (uint32_t)&pwm_seq1,
                    1, /* Count, amount of channels in use */
                    0, /* Refresh */
                    0);
}
void loop() {

  /*delay(1);
  digitalWrite(PWM_PIN, LOW);
  delay(20);
  digitalWrite(PWM_PIN, HIGH);*/

 //duty_cycle += duty_cycle_inc;

          //  pwm_seq = ((uint16_t) ((PWM_COUNTER_TOP*duty_cycle)/100)); // pwm_seq is read by the PWM
            // when task SEQSTARTx is triggered
            //i = i+inc;
            /*setPWM1(i);
             if(i == 100){
              inc = -1;
              setPWM1(i);
              //setPWM2(i);
              digitalWrite(TRIG_PIN, HIGH);
            }
            if(i == 1){
              inc = 1;
              setPWM1(i);
              //setPWM2(i);
              digitalWrite(TRIG_PIN, LOW);
            }*/
            setPWM1(90);
            setPWM2(100);
            setPWM3(50);
            setPWM4(100);
            
            nrf_pwm_task_trigger(NRF_PWM0, NRF_PWM_TASK_SEQSTART0);
            setPWM5(50);
            nrf_pwm_task_trigger(NRF_PWM1, NRF_PWM_TASK_SEQSTART0);
}

void setPWM1(unsigned int percentage){
  uint16_t result =  PWM_COUNTER_TOP - (PWM_MIN+(percentage * STAP_GROOTE));
  pwm_seq[0] = result;
  
  return;
}

void setPWM2(unsigned int percentage){
 /* if(percentage < 10){percentage = 10;}
  if(percentage > 30){percentage = 30;}*/
  uint16_t result = PWM_COUNTER_TOP - (PWM_MIN+(percentage * STAP_GROOTE));
  pwm_seq[1] = result;
  
  return;
}

void setPWM3(unsigned int percentage){
  uint16_t result =  PWM_COUNTER_TOP - (PWM_MIN+(percentage * STAP_GROOTE));
  pwm_seq[2] = result;
  
  return;
}

void setPWM4(unsigned int percentage){
  uint16_t result =  PWM_COUNTER_TOP - (PWM_MIN+(percentage * STAP_GROOTE));
  pwm_seq[3] = result;
  
  return;
}

void setPWM5(unsigned int percentage){
  uint16_t result =  PWM_COUNTER_TOP - (PWM_MIN+(percentage * STAP_GROOTE));
  pwm_seq1 = result;
  return;
}
