#include <NRF.h>

#define TRIG_PIN 2
#define PWM_PIN 3
#define PWM_PIN2 4
#define PWM_PIN3 28
#define PWM_PIN4 29
#define PWM_PIN5 30
#define PWM_NUM_CH 2
#define SERVOS 4
#define PWM_COUNTER_TOP 20200
#define PWM_MAX 2525
#define PWM_MIN 1010
#define PWM_RANGE (PWM_MAX-PWM_MIN)
#define STAP_GROOTE (PWM_RANGE/100)
#define NUM_STAPPEN (PWM_RANGE/STAP_GROOTE)
#define DELAY 75

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
  int i = 1;
  int inc = 1;
  unsigned int stepcnt = 0;

  uint16_t table_1[][(SERVOS+1)]{ {50, 40, 100, 100, DELAY}, {52, 40, 100, 100, DELAY}, {52, 40, 100, 100, DELAY}, {52, 40, 100, 100, DELAY}, {54, 40, 100, 100, DELAY}, {56, 40, 100, 100, DELAY}, {58, 40, 100, 100, DELAY}, {60, 40, 100, 100, DELAY}, {62, 40, 100, 100, DELAY}, {64, 40, 100, 100, DELAY}, {66, 40, 100, 100, DELAY}, {68, 40, 100, 100, DELAY}, {70, 40, 100, 100, DELAY}, {72, 40, 100, 100, DELAY}, {74, 40, 100, 100, DELAY}, {76, 40, 100, 100, DELAY}, {78, 40, 100, 100, DELAY}, {80, 40, 100, 100, DELAY},
                                  {82, 40, 100, 100, DELAY}, {84, 40, 100, 100, DELAY}, {86, 40, 100, 100, DELAY}, {88, 40, 100, 100, DELAY}, {90, 40, 100, 100, DELAY}/*23 beak is closed*/,     {90, 40, 98, 98, DELAY}, {90, 40, 96, 96, DELAY}, {90, 40, 94, 94, DELAY}, {90, 40, 92, 92, DELAY}, {90, 40, 90, 90, DELAY}, {90, 40, 88, 88, DELAY}, {90, 40, 86, 86, DELAY}, {90, 40, 84, 84, DELAY}, {90, 40, 82, 82, DELAY}, {90, 40, 80, 80, DELAY}, {90, 40, 78, 78, DELAY}, {90, 40, 76, 76, DELAY},
                                  {90, 40, 74, 74, DELAY}, {90, 40, 72, 72, DELAY}, {90, 40, 70, 70, DELAY}, {90, 40, 68, 68, DELAY}, {90, 40, 66, 66, DELAY}, {90, 40, 64, 64, DELAY}, {90, 40, 62, 62, DELAY}, {90, 40, 60, 60, DELAY}, {90, 40, 58, 58, DELAY}, {90, 40, 56, 56, DELAY}, {90, 40, 54, 54, DELAY}, {90, 40, 52, 52, DELAY}, {90, 40, 50, 50, DELAY}, {90, 40, 48, 48, DELAY}, {90, 40, 46, 46, DELAY}, {90, 40, 44, 44, DELAY}, {90, 40, 42, 42, DELAY}, {90, 40, 40, 40, DELAY}, {90, 40, 38, 38, DELAY}/*arm should be at box right now*/,
                                  {88, 40, 38, 38, DELAY}, {86, 40, 38, 38, DELAY}, {84, 40, 38, 38, DELAY}, {82, 40, 38, 38, DELAY}, {80, 40, 38, 38, DELAY}, {78, 40, 38, 38, DELAY}, {76, 40, 38, 38, DELAY}, {74, 40, 38, 38, DELAY}, {72, 40, 38, 38, DELAY}, {70, 40, 38, 38, DELAY}, {68, 40, 38, 38, DELAY}, {66, 40, 38, 38, DELAY}, {64, 40, 38, 38, DELAY}, {62, 40, 38, 38, DELAY}, {60, 40, 38, 38, DELAY}, {58, 40, 38, 38, DELAY}, {56, 40, 38, 38, DELAY}, {54, 40, 38, 38, DELAY}, {52, 40, 38, 38, DELAY}, {50, 40, 38, 38, DELAY}/*Beak is open again*/};


  
void setup() {
  Serial.begin(9600);
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


}
void loop() {

  /*delay(1);
  digitalWrite(PWM_PIN, LOW);
  delay(20);
  digitalWrite(PWM_PIN, HIGH);*/

          //  pwm_seq = ((uint16_t) ((PWM_COUNTER_TOP*duty_cycle)/100)); // pwm_seq is read by the PWM
            // when task SEQSTARTx is triggered
          /*  i = i+inc;
            setPWM3(i);
             if(i == 100){
              inc = -1;
              setPWM3(i);
              delay(1000);
            }
            if(i == 1){
              inc = 1;
              setPWM3(i);
              delay(1000);
            }
            delay(50);
            setPWM5
            nrf_pwm_task_trigger(NRF_PWM0, NRF_PWM_TASK_SEQSTART0);*/
            if(stepcnt == 0){
            inc = 1;
            delay(1000);
            }
            if(stepcnt == 74){          
            inc = -1;
            delay(1000);
            } 
            unsigned int Pos1 = table_1[stepcnt][0];
            unsigned int Pos2 = table_1[stepcnt][1];
            unsigned int Pos3 = table_1[stepcnt][2];
            unsigned int Pos4 = table_1[stepcnt][3];

            PWM_SetAll(Pos1, Pos2, Pos3, Pos4);
            stepcnt += inc;
            delay(table_1[stepcnt][4]);
           
          
}


void PWM_SetAll(unsigned int PWM1, unsigned int PWM2, unsigned int PWM3, unsigned int PWM4){
            setPWM1(PWM1);
            setPWM2(PWM2);
            setPWM3(PWM3);
            setPWM4(PWM4);
            nrf_pwm_task_trigger(NRF_PWM0, NRF_PWM_TASK_SEQSTART0);
}



//This function controls if the beak is open or closed
void setPWM1(unsigned int percentage){ 
  if(percentage < 50){percentage = 50;} //50 is (mostly) open, you can probably change this to 30 or 40
  if(percentage > 90){percentage = 90;} //90 is fully closed, there is no need to go further
  uint16_t result =  PWM_COUNTER_TOP - (PWM_MIN+(percentage * STAP_GROOTE));
  pwm_seq[0] = result;
  
  return;
}

//This function controls the rotation of the beak
void setPWM2(unsigned int percentage){
  if(percentage < 1){percentage = 1;} //180 degrees clockwise rotation
  if(percentage > 100){percentage = 100;} //max counter clockwise rotation
  uint16_t result = PWM_COUNTER_TOP - (PWM_MIN+(percentage * STAP_GROOTE));
  pwm_seq[1] = result;
  
  return;
}

//This function controls the highest Hinge
void setPWM3(unsigned int percentage){
  if(percentage < 10){percentage = 10;}//With values lower than 30 the servo started acting up
  if(percentage > 100){percentage = 100;}//100 is the max value anyway...
  uint16_t result =  PWM_COUNTER_TOP - (PWM_MIN+(percentage * STAP_GROOTE));
  pwm_seq[2] = result;
  
  return;
}

//This function controls the lowest Hinge
void setPWM4(unsigned int percentage){
  if(percentage < 1){percentage = 1;}//unknown
  if(percentage > 100){percentage = 100;}//unknown
  uint16_t result =  PWM_COUNTER_TOP - (PWM_MIN+(percentage * STAP_GROOTE));
  pwm_seq[3] = result;
  
  return;
}
