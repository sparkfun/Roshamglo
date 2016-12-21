#include <avr/sleep.h>
#include <avr/io.h>
#include <util/delay.h>;
#include <avr/interrupt.h>
#include <avr/wdt.h>

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
volatile bool wait = 1;
volatile uint8_t selection = 0;
volatile uint8_t opponentMove=0;
volatile uint8_t count = 0;

volatile uint8_t mode=0;



// Shot packet message
uint8_t shot_packet[] = {USER_ID, 0x00, 0x00, 0xED};
uint8_t shot_packet_size = 4;

// Packet buffer
uint8_t packet[MAX_PACKET_SIZE];
uint8_t num_bytes;


void setup() {
  DDRA = 0xA0;  // Set PA5-7 as outputs
  DDRB = (1<<GRN_LED);  // Configure GRN_LED pin as output
  PORTA = 0x57; // Enable PA0-4 pullup resistors

  PORTB |= (1<<GRN_LED);
  _delay_ms(250);
  PORTB &= ~(1<<GRN_LED);
    
  //Enable external interrupts on joystick pins
  GIMSK = (1<<PCIE0);   // Enable Pin Change Interrupts on Port A
  PCMSK0 = (1<<UP_PIN)|(1<<LEFT_PIN)|(1<<RIGHT_PIN);

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
      wdt_disable();
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
  if(interrupted)
  {
    mode = 1; // User wants to play
    shot_packet[1] = selection;
  }
}



////Gameplay//////////////////////////////////////////////////////////////////////////////
void Gameplay(void)
{
  uint8_t gameMode = 0;
  bool transmit = 1;

  if(interrupted)
  {
    packet[0] = 0;
    packet[1] = 0;
    packet[2] = 0;
    packet[3] = 0;

    TCCR0A = 0x00;  //Clear TCCR0A
    TCCR0B = (1<<CS02)|(1<<CS00); // Set clock prescale to clk/1024
    TCNT0 = 0x00;  // Reset counter
    TIMSK0 |= (1 << OCIE0A);  // Enable timer0 interrupt
    count = 0;
  }
  shot_packet[1] = ROCK;
  // Transmit our data
  if(transmit)
  {
    //PORTB |= (1<<GRN_LED);
    for(int i=0;i<1;i++)
    {
      Intar_Phys.xmit(shot_packet, shot_packet_size);
      count = 0;
      TCNT0 = 0;
      while(count < 4);
    }
    //PORTB &= ~(1<<GRN_LED);
  }
  
  // Read opponent data
  if(Intar_Phys.available())
  {
    memset(packet, 0, MAX_PACKET_SIZE);
    num_bytes = Intar_Phys.read(packet);
    
    if(num_bytes!=0 && num_bytes!=RECV_ERROR) // Valid packet
    {
      if(packet[0]!=0x5B && (packet[1]==ROCK|packet[1]==PAPER|packet[1]==SCISSORS) && packet[3]==0xED) // Not our data
      {
        shot_packet[2] = 1; // Set acknoledge

        if(packet[2]==1 || packet[2]==2) //Recieved our move
        {
          shot_packet[2] = 2; // Update acknoledge
        }
      }
    }
  }

//  packet[2]=2; shot_packet[2]=2;
//  packet[1] = PAPER;
  
  // Both sides have all the info they need. Display results
  if(packet[2]==2 && shot_packet[2] == 2)
  {
    transmit = 0; // Stop transmitting
    
    byte data = 0;
    if(packet[1] == ROCK) data = 1;
    else if(packet[1] == PAPER) data = 2;
    else if(packet[1] == SCISSORS) data = 3;
    else data = 4;
    for(int i=0; i<data; i++)
    {
      PORTA |= (1<<RED_LED);
      count = 0;
      TCNT0 = 0;
      while(count < 4);
      PORTA &= ~(1<<RED_LED);
      count = 0;
      TCNT0 = 0;
      while(count < 4);
    }
//    count = 0;
//    TCNT0 = 0;
//    while(count < 16)
//    
//    switch(packet[1])
//    {
//      case (ROCK):
//        if(shot_packet[1] == PAPER)
//        {
//          gameMode = 1; //win
//        }
//        else if(shot_packet[1] == SCISSORS)
//        {
//          gameMode = 2; //lose
//        }
//        else
//        {
//          gameMode = 0; //tie
//        }
//        break;
//      case (PAPER):
//        if(shot_packet[1] == ROCK)
//        {
//          gameMode = 2; //lose
//        }
//        else if(shot_packet[1] == SCISSORS)
//        {
//          gameMode = 1; //win          
//        }
//        else
//        {
//          gameMode = 0; //tie
//        }
//        break;
//      case (SCISSORS):
//        if(shot_packet[1] == PAPER)
//        {
//          gameMode = 2; //lose
//        }
//        else if(shot_packet[1] == ROCK)
//        {
//          gameMode = 1; //win
//        }
//        else
//        {
//          gameMode = 0; //tie
//        }
//        break;
//      default:
//        gameMode = 3;
//        break;
//    }
//    
//    switch(gameMode)
//    {
//      case(0):
//        PORTA &= ~(1<<RED_LED);
//        PORTB |= (1<<GRN_LED);
//        for(uint8_t i=0;i<20;i++)
//        {
//          count = 0;
//          TCNT0 = 0x00;  // Reset counter
//          while(count < 3)
//          {
//            // ~100ms delay
//          }
//          PORTA ^= (1<<RED_LED);
//          PORTB ^= (1<<GRN_LED);
//        }
//        PORTB &= ~(1<<GRN_LED);
//        break;
//      case(1):
//        PORTA &= ~(1<<RED_LED); // Turn off RED LED
//        PORTB |= (1<<GRN_LED);  // Turn on GRN LED
//        count = 0;
//        TCNT0 = 0x00;  // Reset counter
//        while(count < 61)
//        {
//          //~2 seconds delay
//        }
//        break;
//      case(2):
//        PORTB &= ~(1<<GRN_LED); // Turn on GRN LED
//        PORTA |= (1<<RED_LED);  // Turn off RED LED
//        count = 0;
//        TCNT0 = 0x00;  // Reset counter
//        while(count < 61)
//        {
//          // ~2 seconds delay
//        }
//        break;
//      case(3):
//        PORTA &= ~(1<<RED_LED);
//        PORTB |= (1<<GRN_LED);
//        for(uint8_t i=0;i<20;i++)
//        {
//          count = 0;
//          TCNT0 = 0x00;  // Reset counter
//          while(count < 10)
//          {
//            // ~100ms delay
//          }
//          PORTA ^= (1<<RED_LED);
//          PORTB ^= (1<<GRN_LED);
//        }
//        PORTB &= ~(1<<GRN_LED);
//        break;
//      default:
//        break;
//    }
    PORTB &= ~(1<<GRN_LED); // Turn off GRN LED
    PORTA &= ~(1<<RED_LED); // Turn off RED LED
    mode = 0;
    interrupted = 0;
    
  }
}

////Scoreboard//////////////////////////////////////////////////////////////////////////////
void Scoreboard()
{
  
}



// Pin Change Interrupt Service / is executed when PA4:0 changes states (0->1 or 1->0)
ISR(PCINT0_vect)
{ 
  //Change interrupted only if button is LOW
  if(!(PINA & (1<<RIGHT_PIN)))  //Right - Scissors
  {
    //_delay_ms(5);
    if(!(PINA & (1<<RIGHT_PIN)))
    {
      selection = SCISSORS;
      interrupted = 1;
    }
  }
  else if(!(PINA & (1<<UP_PIN)))  //Up - Paper
  {
    //_delay_ms(5);
    if(!(PINA & (1<<UP_PIN)))
    {
      selection = PAPER;
      interrupted = 1;
    }
  }
  else if(!(PINA & (1<<LEFT_PIN)))  //Left - Rock
  {
    //_delay_ms(5);
    if(!(PINA & (1<<LEFT_PIN)))
    {
      selection = ROCK;
      interrupted = 1;
    }
  }
  else  //Error - shouldn't ever happen
  {
    selection = 0;
  }
}

ISR(TIM0_COMPA_vect)
{
  count++;
}
