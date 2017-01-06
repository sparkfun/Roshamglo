#include <IntarIR.h>
#include <EEPROM.h>

IntarIR intar_ir;


#define ROCK        0x0F
#define PAPER       0x88
#define SCISSORS    0xFF


#define DOWN_PIN    PA0
#define RIGHT_PIN   PA1
#define UP_PIN      PA2
#define CENTER_PIN  PA6
#define LEFT_PIN    PA4


#define GRN_LED     PA7
#define RED_LED     PB2
#define IR_LED      PA5
#define IR_REC      PA3


volatile bool interrupted = 0;
volatile uint8_t selection;
volatile uint8_t count = 0;

uint8_t mode=0;

// Shot packet message
byte shot_packet[4];
const byte shot_packet_size = 4;

// Packet buffer
byte packet[MAX_PACKET_SIZE];
byte num_bytes;

uint8_t myID;


void setup() {
  // Configure IR LED, RED LED, and GRN LED as outputs
  DDRA = (1<<IR_LED)|(1<<GRN_LED);
  DDRB = (1<<RED_LED);

  // Enable pullup resistors for 5-way switch
  PORTA = (1<<LEFT_PIN)|(1<<RIGHT_PIN)|(UP_PIN)|(DOWN_PIN)|(CENTER_PIN);

  // Enable external interrupts on joystick pins UP, LEFT, and RIGHT
  GIMSK = (1<<PCIE0);   // Enable Pin Change Interrupts on Port A
  PCMSK0 = (1<<UP_PIN)|(1<<LEFT_PIN)|(1<<RIGHT_PIN);

  // Initialize IR receiver/transmitter
  intar_ir.begin(3);
  intar_ir.enableTransmitter();
  intar_ir.enableReceiver();

  myID = EEPROM.read(0);
  
  sei(); //Enable interrupts
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

    case (2): // Scoreboard
      Scoreboard();
      break;
    
    default:
      break;
  }
}





////Standby//////////////////////////////////////////////////////////////////////////////
void Standby(void)
{
  // Wait for button to be pressed
  if(interrupted)
  {
    mode = 1; // User wants to play

    shot_packet[0] = myID;
    shot_packet[1] = selection; // Load button pressed to transmission packet
    shot_packet[2] = 0;
    shot_packet[3] = 0xED;
  }
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



////Gameplay//////////////////////////////////////////////////////////////////////////////
void Gameplay(void)
{
  uint8_t gameResult = 0; // 0-TIE, 1-WIN, 2-LOSE
  uint8_t theirMove=0;    // Place we'll store our opponent's move
  uint8_t myDelay = random(3);
  //Initialization
  if(interrupted)
  {
    intar_ir.enableReceiver();
    interrupted = 0;
  }

  timerDelay(myDelay);
  
  // Read opponent's data
  if(intar_ir.available())
  {
    memset(packet, 0, MAX_PACKET_SIZE);   // Load data received to packet[]
    num_bytes = intar_ir.read(packet);  // Number of bytes received
    
    if(num_bytes!=0 && num_bytes!=RECV_ERROR)   // Valid packet
    {
      if(packet[0]!=myID && packet[3]==0xED) // Must be opponent's data, so read what they sent
      {
        PORTB |= (1<<RED_LED);  // Turn on LED
        shot_packet[2] = 1;     // Set acknowledge

        if(packet[2]==1 || packet[2]==2) // Opponent has recieved our move
        {
          shot_packet[2] = 2;     // Update acknowledge
          theirMove = packet[1];  // Store our opponent's move
        }
        timerDelay(1);
        PORTB &= ~(1<<RED_LED); // Turn off LED
      }
    }    
  }

  // Transmit our data
  intar_ir.disableReceiver(); // Disable receiver so we don't read our own data
  PORTA |= (1<<GRN_LED);        // Turn on LED
  intar_ir.send(shot_packet, shot_packet_size); // Transmit our data
  intar_ir.enableReceiver();  // Start listening for opponent's data again
  
  // Wait for 64ms (so we can see the LED)
  timerDelay(1);
  
  intar_ir.disableReceiver(); // Disable receiver so we don't read our own data
  intar_ir.send(shot_packet, shot_packet_size); // Transmit again for good measure
  PORTA &= ~(1<<GRN_LED);       // Turn off LED
  intar_ir.flushTransmitter();
  intar_ir.enableReceiver();  // Start listening for opponent's data again
  
  // Wait for 100ms 
  timerDelay(9);
  
  // Both sides have all the info they need. Display results
  if(packet[2]==2 && shot_packet[2] == 2)
  {
    intar_ir.disableReceiver();
    
    // Wait for 1000ms before displaying results
    timerDelay(32);

    // Figure out if we won/lost/tied
    switch(theirMove)
    {
      case (ROCK):
        if(shot_packet[1] == PAPER)
        {
          gameResult = 1; // Win
        }
        else if(shot_packet[1] == SCISSORS)
        {
          gameResult = 2; // Lose
        }
        else
        {
          gameResult = 0; // Tie
        }
        break;
      case (PAPER):
        if(shot_packet[1] == ROCK)
        {
          gameResult = 2; // Lose
        }
        else if(shot_packet[1] == SCISSORS)
        {
          gameResult = 1; // Win          
        }
        else
        {
          gameResult = 0; // Tie
        }
        break;
      case (SCISSORS):
        if(shot_packet[1] == PAPER)
        {
          gameResult = 2; // Lose
        }
        else if(shot_packet[1] == ROCK)
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

    if(packet[0] == EEPROM.read(0x01))
    {
      if((EEPROM.read(0x02)+1) > 4)
      {
        gameResult = 3;
      }
      else
      {
        EEPROM.write(0x02,EEPROM.read(0x02)+1);
      }
    }
    else
    {
      EEPROM.write(0x01,packet[0]);
      EEPROM.write(0x02,0x00);
    }

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
        PORTB &= ~(1<<RED_LED);   // Turn off RED LED
        PORTA |= (1<<GRN_LED);    // Turn on GRN LED

        // Wait for 2000ms
        timerDelay(61);
        break;
        
      case(2):  // Lose
        PORTA &= ~(1<<GRN_LED);   // Turn on LED
        PORTB |= (1<<RED_LED);    // Turn off LED

        // Wait for 2000ms
        timerDelay(61);
        break;
      case(3):
        for(uint8_t i=0;i<10;i++)
        {
          // Wait 100ms
          timerDelay(3);
          PORTB ^= (1<<RED_LED);  // Switch LED state
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
  }
}

////Scoreboard//////////////////////////////////////////////////////////////////////////////
void Scoreboard()
{
  
}

//void blinkPin(amount)
//{
//  for(int i=0;i<amount;i++)
//  {
//    PORTA |= (1<<PA0);
//    PORTA &= ~(1<<PA0);
//  }
//
//  count = 0;        // Reset count
//  TCNT0 = 0;        // Reset timer
//  while(count < 1); // Wait ~64ms
//}

// Pin Change Interrupt Service / only runs when UP, LEFT, or RIGHT is pressed
ISR(PCINT0_vect)
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
  else                              //Error - shouldn't ever happen
  {
    selection = 0;
  }
}

// Timer0 overflow interrupt / used to generate delays
ISR(TIM0_COMPA_vect)
{
  count++;
}
