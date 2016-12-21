#include <IntarPhys.h>


#define USER_ID 0x5A


#define ROCK        0x0F
#define PAPER       0x88
#define SCISSORS    0xFF


#define DOWN_PIN    PA0
#define RIGHT_PIN   PA1
#define UP_PIN      PA2
#define CENTER_PIN  PA6
#define LEFT_PIN    PA4


#define RED_LED     PA7
#define GRN_LED     PB2
#define IR_LED      PA5
#define IR_REC      PA3


volatile bool interrupted = 0;
volatile uint8_t selection = 0;
volatile uint8_t count = 0;

uint8_t mode=0;

// Shot packet message
uint8_t shot_packet[] = {USER_ID, 0x00, 0x00, 0xED};
uint8_t shot_packet_size = 4;

// Packet buffer
uint8_t packet[MAX_PACKET_SIZE];
uint8_t num_bytes;


void setup() {
  // Configure IR LED, RED LED, and GRN LED as outputs
  DDRA = (1<<IR_LED)|(1<<RED_LED);
  DDRB = (1<<GRN_LED);

  // Enable pullup resistors for 5-way switch
  PORTA = 0x57;
    
  // Enable external interrupts on joystick pins UP, LEFT, and RIGHT
  GIMSK = (1<<PCIE0);   // Enable Pin Change Interrupts on Port A
  PCMSK0 = (1<<UP_PIN)|(1<<LEFT_PIN)|(1<<RIGHT_PIN);

  // Initialize IR receiver/transmitter
  Intar_Phys.begin(3);
  Intar_Phys.enableTransmitter();
  Intar_Phys.enableReceiver();

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
    shot_packet[1] = selection; // Load button pressed to transmission packet
    mode = 1; // User wants to play
  }
}



////Gameplay//////////////////////////////////////////////////////////////////////////////
void Gameplay(void)
{
  uint8_t gameResult = 0; // 0-TIE, 1-WIN, 2-LOSE
  bool transmit = 1;      // Start talking to opponent
  uint8_t theirMove=0;    // Place we'll store our opponent's move
  
  if(interrupted)
  {
    // Reset values of previous game
    packet[0] = 0;
    packet[1] = 0;
    packet[2] = 0;
    packet[3] = 0;
    
    // Start timer0 (used to generate delays)
    TCCR0A = 0x00;  //Clear TCCR0A
    TCCR0B = (1<<CS02)|(1<<CS00); // Set clock prescale to clk/1024
    TCNT0 = 0x00;  // Reset counter
    TIMSK0 |= (1 << OCIE0A);  // Enable timer0 interrupt
    count = 0;  // count is updated when the timer overflows (happens appox. every 32ms)
  }
  
  // Read opponent's data
  if(Intar_Phys.available())
  {
    memset(packet, 0, MAX_PACKET_SIZE);   // Load data received to packet[]
    num_bytes = Intar_Phys.read(packet);  // Number of bytes received
    
    if(num_bytes!=0 && num_bytes!=RECV_ERROR)   // Valid packet
    {
      if(packet[0]!=USER_ID && packet[3]==0xED) // Must be opponent's data, so read what they sent
      {
        PORTA |= (1<<RED_LED);  // Turn on LED
        shot_packet[2] = 1;     // Set acknowledge

        if(packet[2]==1 || packet[2]==2) // Opponent has recieved our move
        {
          shot_packet[2] = 2;     // Update acknowledge
          theirMove = packet[1];  // Store our opponent's move
        }

        // Wait for 64ms (so we can see the LED)
        count = 0;        // Reset count
        TCNT0 = 0;        // Reset timer
        while(count < 2); // Wait ~64ms
        
        PORTA &= ~(1<<RED_LED); // Turn off LED
      }
    }
  }

  // Transmit our data
  if(transmit)
  {
    Intar_Phys.disableReceiver(); // Disable receiver so we don't read our own data
    PORTB |= (1<<GRN_LED);        // Turn on LED
    Intar_Phys.xmit(shot_packet, shot_packet_size); // Transmit our data
    Intar_Phys.xmit(shot_packet, shot_packet_size); // Transmit again for good measure
    PORTB &= ~(1<<GRN_LED);       // Turn off LED
    Intar_Phys.flushXmit();
    Intar_Phys.enableReceiver();  // Start listening for opponent's data again
    
    // Wait for 100ms 
    count = 0;        // Reset count
    TCNT0 = 0;        // Reset timer
    while(count < 3); // Wait ~100ms
  }
  
  // Both sides have all the info they need. Display results
  if(packet[2]==2 && shot_packet[2] == 2)
  {
    transmit = 0; // Stop transmitting

    // Wait for 1000ms before displaying results
    count = 0;          // Reset count
    TCNT0 = 0;          // Reset timer
    while(count < 32);  // Wait ~1000ms

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

    // Display the results
    switch(gameResult)
    {
      case(0):  // Tie
        PORTA &= ~(1<<RED_LED);   // Turn off led
        PORTB |= (1<<GRN_LED);    // Turn on led

        // Alternate RED/GRN LEDs for ~2000ms
        for(uint8_t i=0;i<20;i++)
        {
          // Wait 100ms
          count = 0;              // Reset count
          TCNT0 = 0x00;           // Reset timer
          while(count < 3);       // ~100ms delay
          PORTA ^= (1<<RED_LED);  // Switch LED state
          PORTB ^= (1<<GRN_LED);  // Switch LED state
        }
        PORTB &= ~(1<<GRN_LED);   // Turn LED off
        break;
        
      case(1):  // Win
        PORTA &= ~(1<<RED_LED);   // Turn off RED LED
        PORTB |= (1<<GRN_LED);    // Turn on GRN LED

        // Wait for 2000ms
        count = 0;                // Reset count
        TCNT0 = 0x00;             // Reset timer
        while(count < 61);        // ~2000ms delay
        break;
        
      case(2):  // Lose
        PORTB &= ~(1<<GRN_LED);   // Turn on LED
        PORTA |= (1<<RED_LED);    // Turn off LED

        // Wait for 2000ms
        count = 0;                // Reset count
        TCNT0 = 0x00;             // Reset timer
        while(count < 61);        // ~2000ms delay
        break;
        
      default:
        break;
    }
    PORTB &= ~(1<<GRN_LED); // Turn off LED
    PORTA &= ~(1<<RED_LED); // Turn off LED
    mode = 0;               // Return to standby
    interrupted = 0;        // Reset variable
  }
}

////Scoreboard//////////////////////////////////////////////////////////////////////////////
void Scoreboard()
{
  
}



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
