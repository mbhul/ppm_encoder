/*
 * PPM generator originally written by David Hasko
 * on https://code.google.com/p/generate-ppm-signal/ 
 */

//////////////////////CONFIGURATION///////////////////////////////
#define NUM_CHANNELS 8              //set the number of channels
#define CHANNEL_DEFAULT_VALUE 1500  //set the default servo value
#define FRAME_LENGTH 22000          //set the PPM frame length in microseconds (1ms = 1000µs)
#define PULSE_LENGTH 300            //set the pulse length (us)
#define ON_STATE 1                  //set polarity of the pulses: 1 is HIGH, 0 is LOW
#define SIG_PIN 10                  //set PPM signal output pin on the arduino

#define SWITCH_STEP 25              //Default PPM step size (us)
#define CMD_LEN 64                  //Max length of Serial command (# characters)
#define THROTTLE_CHAN 0             //PPM channel used for throttle output
#define DEFAULT_THROTTLE_VALUE 1000 //Default throttle value (usually different from servos)

#define BGND_TASK_PERIOD 100
#define ARRAY_FRAME_SIZE (FRAME_LENGTH / BGND_TASK_PERIOD)
//////////////////////////////////////////////////////////////////

/* GLOBAL VARIABLES */
byte inStr[CMD_LEN];
byte outStr[CMD_LEN];
boolean record_next_frame = false;

/*Channel and state variables */
boolean output_state = true;
byte cur_chan_numb = 0;

/* This array holds the servo values for the ppm signal (in us)*/
int ppm[NUM_CHANNELS];

/* Array to output data on serial for test */
byte ppm_test_array[ARRAY_FRAME_SIZE+1];

/****************************
 * ARDUINO INITIALIZATION
 ***************************/
void setup()
{  
  //initiallize default ppm values
  for(int i=0; i<NUM_CHANNELS; i++)
  {
     if (i == THROTTLE_CHAN)
     {
        ppm[i]= DEFAULT_THROTTLE_VALUE;
     }
     else
     {
        ppm[i]= CHANNEL_DEFAULT_VALUE;
     }
  }

  //Configure the PPM signal output pin and set its default state
  pinMode(SIG_PIN, OUTPUT);
  digitalWrite(SIG_PIN, !ON_STATE);  
  
  //Disable interrupts
  cli();
  
  //Clear Timer 1 channel control registers
  TCCR1A = 0;
  TCCR1B = 0; 
  
  //Clear Timer 2 channel control registers
  TCCR2A = 0;
  TCCR2B = 0;

  /*
   * Interrupt period = OCR1A / (16Mhz / prescaler)
   *  ex. for OCR1A = 100
   *  T = 100 / (16,000,000 / 8) = 50us
   */
  OCR1A = (PULSE_LENGTH * 2); // compare match register (factor of 2 comes from the prescale ratio (16 / 8))
  TCCR1B |= (1 << WGM12);     // use CTC mode
  TCCR1B |= (1 << CS11);      // 8 prescaler: 0.5 microseconds per count at 16mhz
  TIMSK1 |= (1 << OCIE1A);    // enable timer compare interrupt
  
  //Use Timer 2 as a fixed-period task for background computations or output for test
  OCR2A = (BGND_TASK_PERIOD * 2);
  TCCR2A |= (1 << WGM21);
  TCCR2B |= (1 << CS21);
  TIMSK2 |= (1 << OCIE2A);
  
  //Enable interrupts
  sei();

  Serial.begin(9600);
}

/*************************************************************
 * ARDUINO MAIN LOOP
 * 
 * This function performs the following tasks:
 *    - Check for serial input
 *    - Parse the input PPM command string
 *    - Output debug information if required
 *************************************************************/
void loop()
{
   static boolean record_prev = false;
   static int index = 0;
   byte temp = 0;

   if (Serial.available()) 
   {
      //Buffer serial data
      temp = Serial.read();
      inStr[index++] = temp;

      //Check for a complete command (buffer is full or terminating character detected)
      if(temp == ';' || index == CMD_LEN)
      {
         //Parse the input string
         ppmParseCommand(inStr, index);

         //Debug: output the parsed PPM values for validation
         for(int i = 0; i < NUM_CHANNELS; i++)
         {
            Serial.write(String(ppm[i]).c_str(), 4);
            Serial.write(",",1);
         }
         Serial.write("\r\n",2);

         //Debug: assert the recording request used in T0 ISR
         //record_next_frame = true;
  
         memset (inStr,0,CMD_LEN);
         index = 0;
      }
   }
   
   //Spit out the PPM test frame on serial as soon as it's done recording
   // if ((record_prev == true) && (record_next_frame == false))
   // {
     // Serial.write(ppm_test_array, ARRAY_FRAME_SIZE);
   // }

   record_prev = record_next_frame;
}

/*******************************************************************************
 * FUNCTION: ppmParseCommand
 * 
 * This function parses a string (ASCII encoded byte-array) into the PPM commands
 * and puts the resulting values in the global ppm[] array.
 *
 * Assumes the following form:
 *   "CH1,CH2,CH3,CH4,...,CHN;"
 * 
 * Where: 'CHx' is a value between 1000 and 2000
 ********************************************************************************/
void ppmParseCommand(byte* cmdString, int cmdLength)
{
   char temp[4] = {0,0,0,0};
   int index = 0;
   int channel_num = 0;
   int temp_ppm;

   for(int i = 0; i < cmdLength; i++)
   {
      //Comma or semi-colon always succeeds a channel ppm value
      if((cmdString[i] == ',' || cmdString[i] == ';') && channel_num < NUM_CHANNELS)
      {
         //set to previous value in case the toInt() conversion fails
         temp_ppm = ppm[channel_num];
         
         //Parse the PPM value
         temp_ppm = String(temp).toInt();
         if(temp_ppm < 1000)
         {
           temp_ppm = 1000;
         }
         else if (temp_ppm > 2000)
         {
           temp_ppm = 2000;
         }
         ppm[channel_num++] = temp_ppm;
         index = 0;
         memset(temp,0,4);
      }
      //Buffer the input PPM value characters
      else
      {  
         temp[0] = temp[1];
         temp[1] = temp[2];
         temp[2] = temp[3];
         temp[3] = cmdString[i];
      }
   }
}

/*************************************************************
 * TIMER 2 ISR
 * Recall, Timer 2 is used as a background task
 * Currently, this task is used to record a single PPM frame for test purposes
 *************************************************************/
ISR(TIMER2_COMPA_vect)
{
    static int out_index = 0;
    static int chan_num_prev = 0;
    static boolean start_frame = false;

    TCNT2 = 0;
 
    //If recording request is asserted
    if (record_next_frame)
    {
        //wait for the start of the next frame (record full frame from beginning)
        if ((cur_chan_numb == 0) && (chan_num_prev != 0) && (start_frame == false))
        {
            out_index = 0;
            start_frame = true;
        }
        else
        {
            out_index++;
        }
        
        //start recording
        if(start_frame)
        {    
            if (out_index < ARRAY_FRAME_SIZE)
            {
                ppm_test_array[out_index] = 0x30 + (byte)output_state;
            }
            else
            {
                ppm_test_array[ARRAY_FRAME_SIZE] = (byte)'\n';
                record_next_frame = false;
                start_frame = false;
            }
        }
    }
    chan_num_prev = cur_chan_numb;
}

/* *********************************************************************************************************************
 *  The Timer 1 ISR below generates the PPM frames
 *  
 *  Simplified example with 4 channels:
 * 
 *    PULSE_WIDTH
 *     |<--->|
 *      _____           _____           _____           _____           _____                             _____
 *     |     |         |     |         |     |         |     |         |     |                           |     |
 *     |     |         |     |         |     |         |     |         |     |                           |     |
 *     | CH1 |         | CH2 |         | CH3 |         | CH4 |         | EOF |                           | CH1 |
 *     |     |         |     |         |     |         |     |         |     |                           |     |
 *     |     |         |     |         |     |         |     |         |     |                           |     |
 * ____|     |_________|     |_________|     |_________|     |_________|     |_______________....________|     |__...
 * 
 *     |<------------->|<------------->|<------------->|<------------->||<--------   SYNC TIME  -------->|
 *       CH1 PPM value   CH2 PPM value   CH3 PPM value   CH4 PPM value
 *     
 *     |<----------------------------------------------------------------------------------------------->|
 *                                            FRAME_LENGTH (22ms)
 *
 * *********************************************************************************************************************/
 
/*******************************************************
 *  Timer 1 ISR
 *   - This ISR is executed when the Timer 1 interrupt occurs
 *   - The period is re-calculated based on the desired output PPM signal
 *     and the interrupt is reset with the new period
 ******************************************************/
ISR(TIMER1_COMPA_vect)
{ 
   //static boolean output_state = true;
   //static byte cur_chan_numb = 0;
   static unsigned int calc_rest = 0;

   //Reset the timer 1 counter
   TCNT1 = 0;

   //Start Pulse (rising edge)
   if (output_state) 
   {  
      digitalWrite(SIG_PIN, ON_STATE);
      OCR1A = PULSE_LENGTH * 2;
      output_state = false;
   } 
   //End pulse and calculate when to start the next pulse
   else
   {  
      digitalWrite(SIG_PIN, !ON_STATE);
      output_state = true;

      if(cur_chan_numb >= NUM_CHANNELS)
      {
         cur_chan_numb = 0;
         calc_rest += PULSE_LENGTH;
         OCR1A = (FRAME_LENGTH - calc_rest) * 2;
         calc_rest = 0;
      }
      else
      {
         OCR1A = (ppm[cur_chan_numb] - PULSE_LENGTH) * 2;
         calc_rest += ppm[cur_chan_numb];
         cur_chan_numb++;
      }     
   }
}

/************************************************* 
 * My Spektrum channel order:
 *
 *  1- Throttle
 *  2- Rudder
 *  3- Elevevators
 *  4- N/A
 *  5- N/A
 *  6- N/A
 *  7- N/A
 *  8- N/A
 ***************************************************/
