/* RoShamGlo Event Example Code
 * by: Alex Wende
 * SparkFun Electronics
 * date: 1/10/17
 * 
 * license: Creative Commons Attribution-ShareAlike 4.0 (CC BY-SA 4.0)
 * Do whatever you'd like with this code, use it for any purpose.
 * Please attribute and keep this license.
 * 
 * This examples uses the Roshamglo board to play rock, paper, scissors
 * with an opponent using IR communication. To play, turn the board on, and 
 * wait 5 seconds for the bootloader to exit. To play, move the joystick to 
 * the left/up/right position and aim the USB plug at your opponent. The LED 
 * will blink green when transmitting and red when your opponent's move is
 * made. Solid green light means win, red means lose, and alternating is a tie.
 * joystick is moved or a space character when right trigger button is pressed.
 */
 
#include <IntarIR.h>
#include <EEPROM.h>

IntarIR intar_ir;

// Memory Addresses
#define ID_HIGH_ADDRESS     0x00
#define ID_MIDDLE_ADDRESS   0x01
#define ID_LOW_ADDRESS      0x02

#define LAST_ID_H           0x03
#define LAST_ID_M           0x04
#define LAST_ID_L           0x05

#define PLAY_COUNT_ADDRESS  0x06
#define SCORE_ADDRESS       0x07

// Action Values
#define ROCK                0x10
#define PAPER               0x20
#define SCISSORS            0x30
#define UPLOAD_SCORE        0xA0
#define DISPLAY_SCORE       0xB0

// 5-way Switch Defines
#define DOWN_PIN            PA0
#define RIGHT_PIN           PA1
#define UP_PIN              PA2
#define LEFT_PIN            PA4
#define CENTER_PIN          PA6

// LED Defines
#define GRN_LED       PA7
#define RED_LED       PB2
#define IR_LED        PA5
#define IR_REC        PA3


volatile bool interrupted = 0;  // Interrupt flag used to initialize functions
volatile uint8_t selection;     // Used to convert switch presses to action values
volatile uint8_t count = 0;     // Used for timer0 ISR
volatile bool endTask = 0;

uint8_t attempt = 0;
uint8_t mode=0;                 // 0-Standby, 1-Gameplay, 2-Dump Scores
uint32_t myID;                   // ID of this badge
uint32_t opponentID;

// Message we'll send to opponent
byte shot_packet[4];
const byte shot_packet_size = 4;

// Message received from opponent
byte packet[MAX_PACKET_SIZE];

volatile uint8_t sample = 0;
volatile bool sample_waiting = 0;

void setup() {
  uint32_t myID_H, myID_M, myID_L;  // Badge ID High, Middle, and Low bytes
  
  // Set pins as outputs
  DDRA = (1<<IR_LED)|(1<<GRN_LED);
  DDRB = (1<<RED_LED);

  // Enable pullup resistors for 5-way switch
  PORTA = (1<<LEFT_PIN)|(1<<RIGHT_PIN)|(1<<UP_PIN)|(1<<DOWN_PIN)|(1<<CENTER_PIN);

  // Enable pin change interrupts on joystick pins
  GIMSK = (1<<PCIE0);
  PCMSK0 = (1<<LEFT_PIN)|(1<<RIGHT_PIN)|(1<<UP_PIN)|(1<<DOWN_PIN)|(1<<CENTER_PIN);

  // Initialize IR receiver/transmitter
  intar_ir.begin(3);
  intar_ir.enableTransmitter();
  intar_ir.enableReceiver();

  // Read the address of the badge
  myID_H = EEPROM.read(ID_HIGH_ADDRESS);
  myID_M = EEPROM.read(ID_MIDDLE_ADDRESS);
  myID_L = EEPROM.read(ID_LOW_ADDRESS);
  myID = (myID_H<<16) + (myID_M<<8) + myID_L; // Combine the 8-bit addresses

  wdtSetup();
  
  sei(); // Enable global interrupts
}

void loop() 
{
  switch (mode)
  {
    case (0): // Standby
      Standby();
      break;
      
    case (1): // Gameplay
      Gameplay();
      break;

    case (2): // Dump Score
      Dumpscore();
      break;

    case (3): // Display Score
      Displayscore();
      break;
      
    default:
      break;
  }
}

////Standby///////////////////////////////////////////////////////////////////////////////
void Standby(void)
{  
  // Wait for button to be pressed
  if(interrupted)
  {
    switch (selection)
    {
      case(UPLOAD_SCORE):
        mode = 2;
        shot_packet[0] = (myID>>16) & 0xFF;
        shot_packet[1] = (myID>>8) & 0xFF;
        shot_packet[2] = myID & 0xFF;
        shot_packet[3] = EEPROM.read(SCORE_ADDRESS)<<2;
        break;

      case (DISPLAY_SCORE):
        mode = 3;
        break;
      
      default:
        mode = 1; // User wants to play
        shot_packet[0] = (myID>>16) & 0xFF;
        shot_packet[1] = (myID>>8) & 0xFF;
        shot_packet[2] = myID & 0xFF;
        shot_packet[3] = selection; // Load button pressed to transmission packet
        break;
    }
  }
}

////Gameplay//////////////////////////////////////////////////////////////////////////////
void Gameplay(void)
{
  uint8_t gameResult = 0; // 0-TIE, 1-WIN, 2-LOSE
  uint8_t theirMove=0;    // Place we'll store our opponent's move
  uint8_t result = 0;
  
  //Initialization
  if(interrupted)
  {
    intar_ir.enableReceiver();
    interrupted = 0;
    attempt = 0;
  }

  // Generate Random number
  while(!sample_waiting); // Wait for new random number to be generated in ISR
  sample_waiting = 0;
  result = rotl(result, 1); // Spread randomness around
  result = (result^sample)&7; // XOR preserves randomness, & 7 keeps the result below 7
  
  timerDelay(result); // Delay random number
  
  // Read opponent's data
  if(intar_ir.available())
  {
    theirMove = readData();
  }
  
  sendData(shot_packet,shot_packet_size);
  
  // Wait for 100ms 
  timerDelay(3);
  
  // Both sides have all the info they need. Display results
  if((packet[3]&0x03)==2 && (shot_packet[3]&0x03) == 2)
  {
    intar_ir.disableReceiver();

    sendData(shot_packet,shot_packet_size);
    timerDelay(1);
    sendData(shot_packet,shot_packet_size);

    uint8_t ourMove = (shot_packet[3]&0xF0);

    // Figure out if we won/lost/tied
    switch(theirMove)
    {
      case (ROCK):
        if(ourMove == PAPER)
        {
          gameResult = 1; // Win
        }
        else if(ourMove == SCISSORS)
        {
          gameResult = 2; // Lose
        }
        else
        {
          gameResult = 0; // Tie
        }
        break;
      case (PAPER):
        if(ourMove == ROCK)
        {
          gameResult = 2; // Lose
        }
        else if(ourMove == SCISSORS)
        {
          gameResult = 1; // Win          
        }
        else
        {
          gameResult = 0; // Tie
        }
        break;
      case (SCISSORS):
        if(ourMove == PAPER)
        {
          gameResult = 2; // Lose
        }
        else if(ourMove == ROCK)
        {
          gameResult = 1; // Win
        }
        else
        {
          gameResult = 0; // Tie
        }
        break;
      default:
        gameResult = 0;   // Error/Tie
        break;
    }

    uint32_t lastOpponentID = (EEPROM.read(LAST_ID_H)<<16) + (EEPROM.read(LAST_ID_M)<<8) + EEPROM.read(LAST_ID_L);
    uint32_t thisOpponentID = (packet[0]<<16) + (packet[1]<<8) + packet[2];
    
    if(thisOpponentID == lastOpponentID)
    {
      if((EEPROM.read(PLAY_COUNT_ADDRESS)+1) > 14)
      {
        gameResult = 3;
      }
      else
      {
        EEPROM.write(PLAY_COUNT_ADDRESS,EEPROM.read(PLAY_COUNT_ADDRESS)+1);
      }
    }
    else
    {
      EEPROM.write(LAST_ID_H,packet[0]);
      EEPROM.write(LAST_ID_M,packet[1]);
      EEPROM.write(LAST_ID_L,packet[2]);
      EEPROM.write(PLAY_COUNT_ADDRESS,0x00);
    }

    uint8_t currentScore = EEPROM.read(SCORE_ADDRESS);
    
    // Display the results
    switch(gameResult)
    {
      case(0):  // Tie
        PORTB &= ~(1<<RED_LED);   // Turn off led
        PORTA |= (1<<GRN_LED);    // Turn on led

        // Alternate RED/GRN LEDs for ~2000ms
        for(uint8_t i=0;i<20;i++)
        {
          // Wait 100ms
          timerDelay(3);
          PORTB ^= (1<<RED_LED);  // Switch LED state
          PORTA ^= (1<<GRN_LED);  // Switch LED state
        }
        PORTB &= ~(1<<GRN_LED);   // Turn LED off
        break;
        
      case(1):  // Win
        if(currentScore > 62)
        {
          for(uint8_t i=0;i<10;i++)
          {
            // Wait 100ms
            timerDelay(3);
            PORTB ^= (1<<RED_LED);  // Toggle LED state
          }
          PORTB &= ~(1<<RED_LED); // Make sure LED is off
        }
        else
        {
          EEPROM.write(SCORE_ADDRESS, EEPROM.read(SCORE_ADDRESS)+1);  // Update our score
          
          PORTB &= ~(1<<RED_LED);   // Turn off RED LED
          PORTA |= (1<<GRN_LED);    // Turn on GRN LED
          
          // Wait for 2000ms
          timerDelay(61);
        }
        break;
        
      case(2):  // Lose        
        PORTA &= ~(1<<GRN_LED);   // Turn on LED
        PORTB |= (1<<RED_LED);    // Turn off LED

        // Wait for 2000ms
        timerDelay(61);
        break;
        
      case(3):  // Invalid (played opponent too many times)
        for(uint8_t i=0;i<10;i++)
        {
          // Wait 100ms
          timerDelay(3);
          PORTB ^= (1<<RED_LED);  // Toggle LED state
        }
        PORTB &= ~(1<<RED_LED); // Make sure LED is off
        break;
              
      default:
        break;
    }
    
    PORTA &= ~(1<<GRN_LED); // Turn off LED
    PORTB &= ~(1<<RED_LED); // Turn off LED
    
    mode = 0;               // Return to standby

    packet[0] = 0x00;
    packet[1] = 0x00;
    packet[2] = 0x00;
    packet[3] = 0x00;
    opponentID = 0;
  }

  attempt++;
  if(attempt > 30)
  {     
    mode = 0; // Return to standby
    attempt = 0;
    packet[0] = 0x00;
    packet[1] = 0x00;
    packet[2] = 0x00;
    packet[3] = 0x00;
    opponentID = 0;
  }

  if(endTask && attempt > 3)
  {
    mode = 0; // Return to standby
    attempt = 0;
    packet[0] = 0x00;
    packet[1] = 0x00;
    packet[2] = 0x00;
    packet[3] = 0x00;
    opponentID = 0;
    endTask = 0;
  }
  else
  {
    endTask = 0;
  }
}

////Dump Score////////////////////////////////////////////////////////////////////////////
void Dumpscore(void)
{
  uint8_t theirMove = 0;
  
  //Initialization
  if(interrupted)
  {
    intar_ir.enableReceiver();
    interrupted = 0;
    attempt = 0;
  }

  // Read opponent's data
  if(intar_ir.available())
  {
    theirMove = readData();
  }
  
  sendData(shot_packet,shot_packet_size);

  // Wait for 100ms 
  timerDelay(9);
  
  // Both sides have all the info they need. Display results
  if((packet[3]&0x03)==2 && (shot_packet[3]&0x03) == 2)
  {
    intar_ir.disableReceiver();
    
    EEPROM.write(SCORE_ADDRESS,0x00);
    EEPROM.write(LAST_ID_H,0x00);
    EEPROM.write(LAST_ID_M,0x00);
    EEPROM.write(LAST_ID_L,0x00);
    EEPROM.write(PLAY_COUNT_ADDRESS,0x00);
    
    mode = 0; // Return to standby

    packet[0] = 0x00;
    packet[1] = 0x00;
    packet[2] = 0x00;
    packet[3] = 0x00;

    opponentID = 0;

    PORTA |= (1<<GRN_LED);  // Turn on GRN LED
    timerDelay(61);         // Wait for 2000ms
    PORTA &= ~(1<<GRN_LED); // Turn off LED
  }

  attempt++;
  if(attempt > 20)
  {
    mode = 0; // Return to standby
    attempt = 0;
    packet[0] = 0x00;
    packet[1] = 0x00;
    packet[2] = 0x00;
    packet[3] = 0x00;
    opponentID = 0;
    
    PORTB |= (1<<RED_LED);  // Turn on GRN LED
    timerDelay(61);         // Wait for 2000ms
    PORTB &= ~(1<<RED_LED); // Turn off LED
  }

  if(endTask && attempt > 2)
  {
    mode = 0; // Return to standby
    attempt = 0;
    packet[0] = 0x00;
    packet[1] = 0x00;
    packet[2] = 0x00;
    packet[3] = 0x00;
    opponentID = 0;
    endTask = 0;
  }
  else
  {
    endTask = 0;
  }
}

////Display Score/////////////////////////////////////////////////////////////////////////
void Displayscore(void)
{
  uint8_t score = EEPROM.read(SCORE_ADDRESS);

  if(score != 0)
  {
    for(uint8_t i=0; i<score; i++)
    {
      PORTA |= (1<<GRN_LED);
      timerDelay(15);
      PORTA &= ~(1<<GRN_LED);
      timerDelay(15);

      if(endTask && i>0)
      {
        break;
      }
      else
      {
        endTask = 0;
      }
    }
  }
  else
  {
    PORTB |= (1<<RED_LED);
    timerDelay(23);
    PORTB &= ~(1<<RED_LED);
  }

  interrupted = 0;
  mode = 0;
  endTask = 0;
}



void timerDelay(uint8_t amount)
{
  // Start timer0 (used to generate delays)
  TCCR0A = 0x00;  //Clear TCCR0A
  TCCR0B = (1<<CS02)|(1<<CS00); // Set clock prescale to clk/1024
  TCNT0 = 0x00;  // Reset counter
  TIMSK0 |= (1 << OCIE0A);  // Enable timer0 interrupt
  count = 0;  // count is updated when the timer overflows (happens appox. every 32ms)
  
  while(count < amount); // delay = amount * 32ms
}

void sendData(uint8_t data[],uint8_t len)
{
  intar_ir.disableReceiver();   // Disable receiver so we don't read our own data
  PORTA |= (1<<GRN_LED);        // Turn on LED
  intar_ir.send(data, len);     // Transmit our data
  intar_ir.enableReceiver();    // Start listening for opponent's data again
  
  intar_ir.disableReceiver();   // Disable receiver so we don't read our own data
  intar_ir.send(data, len);     // Transmit our data again for good measure
  PORTA &= ~(1<<GRN_LED);       // Turn off LED
  intar_ir.flushTransmitter();
  intar_ir.enableReceiver();    // Start listening for opponent's data again
}

uint8_t readData(void)
{
  uint8_t num_bytes, theirMove; 
  uint32_t playerID, playerID_high;
  uint8_t  playerID_low, playerID_middle;  
  
  memset(packet, 0, MAX_PACKET_SIZE);   // Load data received to packet[]
  num_bytes = intar_ir.read(packet);  // Number of bytes received
  
  if(num_bytes!=0 && num_bytes!=RECV_ERROR)   // Valid packet
  {
    playerID_high = packet[0];
    playerID_middle = packet[1];
    playerID_low = packet[2];
    playerID = (playerID_high<<16) + (playerID_middle<<8) + playerID_low;
      
    if(playerID != myID) // Must be opponent's data, so read what they sent
    {
      if(opponentID == 0)
      {
        opponentID = playerID;
      }
      if(playerID == opponentID)
      {
        PORTB |= (1<<RED_LED);  // Turn on LED

        shot_packet[3] = (shot_packet[3]&0xFC) + 0x01;     // Set acknowledge
  
        if((packet[3]&0x03)==1 || (packet[3]&0x03)==2)  // Opponent has recieved our move
        {
          shot_packet[3] = (shot_packet[3]&0xFC) + 0x02;   // Update acknowledge
          theirMove = packet[3]&0xF0;  // Store our opponent's move
        }
        PORTB &= ~(1<<RED_LED); // Turn off LED
      }
    }
  }
  return theirMove;
}

byte rotl(const byte value, int shift) {
  if ((shift &= sizeof(value)*8 - 1) == 0)
    return value;
  return (value << shift) | (value >> (sizeof(value)*8 - shift));
}

// Setup of the watchdog timer.
void wdtSetup() {
  cli();
  MCUSR = 0;
  
  /* Start timed sequence */
  WDTCSR |= _BV(WDCE) | _BV(WDE);

  /* Put WDT into interrupt mode */
  /* Set shortest prescaler(time-out) value = 2048 cycles (~16 ms) */
  WDTCSR = _BV(WDIE);

  sei();
}

// Watchdog Timer Interrupt Service Routine
ISR(WDT_vect)
{
  sample = TCNT1L; // Ignore higher bits
  sample_waiting = 1;
}

// Pin Change Interrupt Service / only runs when UP, LEFT, or RIGHT is pressed
ISR(PCINT0_vect)
{
  if(mode == 0)
  {
    //Change interrupted only if button is LOW
    if(!(PINA & (1<<RIGHT_PIN)))      //Right - Scissors
    {
      selection = SCISSORS;
      interrupted = 1;
    }
    else if(!(PINA & (1<<UP_PIN)))    //Up - Paper
    {
      selection = PAPER;
      interrupted = 1;
    }
    else if(!(PINA & (1<<LEFT_PIN)))  //Left - Rock
    {
      selection = ROCK;
      interrupted = 1;
    }
    else if(!(PINA & (1<<DOWN_PIN)))  //Down - Upload score to scoreboard
    {
      selection = UPLOAD_SCORE;
      interrupted = 1;
    }
    else if(!(PINA & (1<<CENTER_PIN)))  //Center - Display Score
    {
      selection = DISPLAY_SCORE;
      interrupted = 1;
    }
    else                              //Error - shouldn't ever happen
    {
      selection = 0;
    }
  }
  else
  {
    //Change endTask only if button is LOW
    if(!(PINA & (1<<RIGHT_PIN)))      //Right - Scissors
    {
      endTask = 1;
    }
    else if(!(PINA & (1<<UP_PIN)))    //Up - Paper
    {
      endTask = 1;
    }
    else if(!(PINA & (1<<LEFT_PIN)))  //Left - Rock
    {
      endTask = 1;
    }
    else if(!(PINA & (1<<DOWN_PIN)))  //Down - Upload score to scoreboard
    {
      endTask = 1;
    }
    else if(!(PINA & (1<<CENTER_PIN)))  //Center - Display Score
    {
      endTask = 1;
    }
  }
}

// Timer0 overflow interrupt / used to generate delays
ISR(TIM0_COMPA_vect)
{
  count++;
}
