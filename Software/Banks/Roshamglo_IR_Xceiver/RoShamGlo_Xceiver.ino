#include <IntarIR.h>
IntarIR intar_ir;

#define USER_ID           0x000000
#define IR_RECEIVER_PIN   4
#define IR_LED_PIN        3

// Shot packet message
uint8_t shot_packet[] = {0x00, 0x00, 0x00, 0x00};
uint8_t shot_packet_size = 4;

// Packet buffer
uint8_t packet[MAX_PACKET_SIZE];

int transmit = 0;

void setup() {
  Serial.begin(9600);

  pinMode(IR_LED_PIN,OUTPUT);
  digitalWrite(IR_LED_PIN,LOW);
  
  intar_ir.begin(IR_RECEIVER_PIN);
  intar_ir.enableTransmitter();
  intar_ir.enableReceiver();
}

void loop() {
  uint8_t num_bytes;
  uint32_t playerID, playerID_low, playerID_middle, playerID_high;
  // Read opponent data
  if(intar_ir.available())
  {
    memset(packet, 0, MAX_PACKET_SIZE);
    num_bytes = intar_ir.read(packet);
    
    if(num_bytes!=0 && num_bytes!=RECV_ERROR) // Valid packet
    {      
      playerID_high = packet[0];
      playerID_middle = packet[1];
      playerID_low = packet[2];
      playerID = (playerID_high<<16) + (playerID_middle<<8) + playerID_low;
      
      if(playerID != USER_ID) // Not our data
      {
        transmit = 1;
        shot_packet[3] = 1; // Set acknoledge

        if(packet[4]==1 || packet[4]==2) //Recieved our move
        {
          shot_packet[3] = 2; // Update acknoledge
        }
      }
    }
  }

  // Transmit our data
  if(transmit)
  {
    intar_ir.disableReceiver();
    
    intar_ir.send(shot_packet, shot_packet_size);
    delay(32);
    intar_ir.send(shot_packet, shot_packet_size);
    
    intar_ir.flushTransmitter();
    intar_ir.enableReceiver();
    delay(200);
  }

  if(packet[4]==2 && shot_packet[3] == 2)
  {
    intar_ir.disableReceiver();
    
    Serial.print(playerID,HEX);
    Serial.print(',');
    Serial.println(packet[3],HEX);
    
    packet[0] = 0;
    packet[1] = 0;
    packet[2] = 0;
    packet[3] = 0; 
    packet[4] = 0;
    
    transmit = 0;
    delay(200);
    intar_ir.enableReceiver();
  }
}
