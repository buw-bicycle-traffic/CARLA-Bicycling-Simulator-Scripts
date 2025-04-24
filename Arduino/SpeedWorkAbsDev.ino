// Bicycle Simulator Arduino Interface Code
// Bergische Universität Wuppertal 
// Lehrstuhl Radverkher

#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SPI.h>

// Ethernet configuration
byte mac[]={ 0x2C, 0xF7, 0xF1, 0x08, 0x38, 0x81};
IPAddress ip(192, 168, 2, 108);
unsigned int localPort = 5000;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];
int packetSize;
EthernetUDP Udp;

unsigned int wcount = 0;

// Defining Pins
// Define Speed Sensor Pins DT=3 CLK=2 (Or Channel A)
#define speedPinCHA 2
#define speedPinDT 3
  
// Define Steering Sensor Pins
#define encoderAbsPinIn 7         //DO | absolute encoder PinIn for Steering Angle //check Bourns EMS22A50 datasheet for explanation
#define encoderAbsPinCS 6         //CS | absolute encoder PinCS for Steering Angle
#define encoderAbsClockPin 8      //CLOCK | absolute encoder Pin CLK for Steering Angle

// Define General Constants
#define UDP_FREQ 50               // hz == equivalent to 20ms
#define BAUD_RATE 9600            // Baudrate for Serial Monitor

// Encoder Resolution
#define ENC_RES 1024               
#define ENCODER_ABS_HALF_STEPS 512

volatile unsigned long counter = 0;
float Speed = 0;
volatile float Steering = 170;
long callTime = 0;
long milSt = 0;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;

int encoderAbsCalibration = 0;
int encoderAbsPosPRINT = 0;                   //copied values for serial transmit
int encoderAbsOffset = 957;
volatile int encoderAbsPos = 0;               //position of absolute encoder

void isr_counter(){
  counter ++;
}

// This function calculates the rpm of the bicycle
void calculateRpm(){ 
  counter = 0;
  milSt = millis();
  
  attachInterrupt(digitalPinToInterrupt(speedPinCHA), isr_counter, FALLING);
  while (millis() - milSt < 2){
    	// Count the number of interupts in 2ms
      // Counting happens in isr_counter()
      // after the loop, the interrupt is disabled until the next steering time
      // this is to prevent the interrupts disrupt the flow of the program
  }
  detachInterrupt(digitalPinToInterrupt(speedPinCHA));
  // Calculate the speed
  Speed = (float) (88 * 1000 * counter) / (1024 * 2);
  Speed = (float) ((.000010849) * counter * 1000 * 3.6) / (2); // 3.6 to change to kph
  callTime = millis();
}

// This function sends the UDP packet to the simulator PC
void sendUDP2(){
  Steering = 180;
  packetSize = Udp.parsePacket();
    if (true) {
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());  //Initialize Packet send
      Udp.print(String(Speed*14));
      Udp.print(", ");
      Udp.print(String(encoderAbsPosPRINT));
      Udp.endPacket(); //Packet has been sent
    }
  // Clear the packet buffer
  memset(packetBuffer, 0, UDP_TX_PACKET_MAX_SIZE);
}

// This function reads the absolute encoder value
// Function inspired by code originally developed by Patrick Malcolm
void readAbsEncoder() {
      //read values from absolute encoder
      //alternative with STANDARD functions and no delay: digitalWrite(encoderAbsPinCS, HIGH);, BUT: one operation takes ~9ms compared to 0.8ms + 1ms delay in FastMode
      digitalWrite(encoderAbsPinCS, HIGH);
      digitalWrite(encoderAbsPinCS, LOW);
      encoderAbsPos = 0;
      
      for (int i=0; i<10; i++) {
        digitalWrite(encoderAbsClockPin, LOW);
        digitalWrite(encoderAbsClockPin, HIGH);  
        encoderAbsPos <<=1; 
        encoderAbsPos += digitalRead(encoderAbsPinIn);    
      }
      
      for (int i=0; i<6; i++) {
        digitalWrite(encoderAbsClockPin, LOW);
        digitalWrite(encoderAbsClockPin, HIGH);
      }
      digitalWrite(encoderAbsClockPin, LOW);
      digitalWrite(encoderAbsClockPin, HIGH); 

      //Calculate Encoder Abs Value with offset (0 => centered)
      encoderAbsPosPRINT = encoderAbsPos;
      if (encoderAbsPosPRINT>encoderAbsOffset-ENCODER_ABS_HALF_STEPS){ 
        encoderAbsPosPRINT-=encoderAbsOffset; 
      }else{
        encoderAbsPosPRINT+=2*ENCODER_ABS_HALF_STEPS-encoderAbsOffset+1;
      }
      //calculate abs position in degree
      encoderAbsPosPRINT = (int)(((long) encoderAbsPosPRINT*360)/1024);
      encoderAbsPosPRINT -= encoderAbsCalibration; //calibrate steering angle to 0 in init position
}

void setup() {
  // Run once at startup
  
  // Setup Speed encoder pins
  pinMode(speedPinCHA, INPUT_PULLUP);
  pinMode(speedPinDT, INPUT_PULLUP);

  // Setup steering sensor pins
  pinMode (encoderAbsPinIn, INPUT);
  pinMode (encoderAbsPinCS, OUTPUT);
  digitalWrite(encoderAbsPinCS, LOW);
  pinMode (encoderAbsClockPin, OUTPUT);
  digitalWrite(encoderAbsClockPin, HIGH);

  // Initialize Ethernet and UDP
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);
  delay(500);

  readAbsEncoder();
  encoderAbsCalibration = encoderAbsPosPRINT;
  attachInterrupt(digitalPinToInterrupt(speedPinCHA), isr_counter, FALLING);

  // Initialize serial communication (for debugging): Uncomment if needed
  // Serial.begin(BAUD_RATE);
}

void loop() {
  // Run continuously
  wcount = wcount + 1;
  currentMillis = millis();

  if (currentMillis - previousMillis >= 1000/UDP_FREQ) {
    // This block runs every 20ms
    // Serial.print(String(currentMillis - previousMillis));
    previousMillis = currentMillis;

    // Measure RPM
    calculateRpm();

    // Read the absolute encoder for steering
    readAbsEncoder();

    sendUDP2();
  }
}
