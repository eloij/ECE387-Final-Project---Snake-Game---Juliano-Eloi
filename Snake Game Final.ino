#include "I2Cdev.h"
 
#include "MPU6050_6Axis_MotionApps20.h"
 
#include<Servo.h>

MPU6050 mpu;

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


#define OUTPUT_READABLE_YAWPITCHROLL

// Unccomment if you are using an Arduino-Style Board
#define ARDUINO_BOARD
 
#define LED_PIN 13      // (Galileo/Arduino is 13)
bool blinkState = false;
 
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
 
// orientation/motion vars
VectorFloat gravity;    // [x, y, z]            gravity vector
Quaternion q;           // [w, x, y, z]         quaternion container
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// Pin connected to Pin 12 of 74HC595 (Latch)
int latchPin = 8;

// Pin connected to Pin 11 of 74HC595 (Clock)
int clockPin = 12;

// Pin connected to Pin 14 of 74HC595 (Data)
int dataPin = 11;

// Screen
byte led[8];

// button pin
int btn_up = 3;
int btn_down = 4;
int btn_left = 5;
int btn_right = 6;

// game variables
typedef struct Link {
  int x;
  int y;
  struct Link * next;
} Link;

Link * pHead = NULL;
Link * pTail = NULL;

int curDirection = 4;
int newDirection = 4;
int appleX = 5;
int appleY = 5;

unsigned long oldTimer, curTimer;

boolean dead = 0;


volatile bool mpuInterrupt = true;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


void setup() {
  // Seed Random Generator with noise from analog pin 0  
  randomSeed(analogRead(0));

  // set pins to output
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  
  // set button pins to input
  pinMode(btn_up, INPUT_PULLUP);
  pinMode(btn_down, INPUT_PULLUP);
  pinMode(btn_left, INPUT_PULLUP);
  pinMode(btn_right, INPUT_PULLUP);
  
  Serial.begin(9600);
  // clear screen
  clrscr();
  
      #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        //int TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
 
    Serial.begin(115200);
    while (!Serial);
 
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
 
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(F("MPU6050 connection "));
    Serial.print(mpu.testConnection() ? F("successful") : F("failed"));
 
    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
 
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
 
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(48);
    mpu.setYGyroOffset(-12);
    mpu.setZGyroOffset(-3);
    mpu.setZAccelOffset(1334); // 1688 factory default for my test chip
 
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
 
        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
 
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
 
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void loop() {

  snakeInit();
  screenUpdate();
  oldTimer = millis();
  curTimer = millis();

  while(!dead) {
    curTimer = millis();
    
    setDirection();
    
    if(curTimer-oldTimer >= 320) {
      curDirection = newDirection;
      moveSnake(curDirection);
      screenUpdate();
      oldTimer = millis();
    }
    
    // update screen
    screenDisplay();
  }
  
  int count = 0;
  while(count<8) {
    curTimer = millis();
    if(curTimer-oldTimer >= 100) {
      led[count]=B11111111;
      oldTimer = millis();
      count++;
    }
    screenDisplay();
  }
  
  clrscr();
  
  while(1) {
    curTimer = millis();
    if(curTimer-oldTimer >= 700) {
      for(int i=0; i<8; i++) {
        led[i]=~led[i];
      }
      oldTimer = millis();
    }
    screenDisplay();
  }
  
  if (!dmpReady) return;
 
    // wait for MPU interrupt or extra packet(s) available
 
    #ifdef ARDUINO_BOARD
        while (!mpuInterrupt && fifoCount < packetSize) {
        }
    #endif
 
    #ifdef GALILEO_BOARD
        delay(10);
    #endif
 
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
 
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
 
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
 
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
 
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
 
 
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
            
        #endif
 
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

void
addHead(int x, int y)
{
  Link *temp;
  temp = (Link*) malloc (sizeof(Link));
  	
  // create new head
  temp->x = x;
  temp->y = y;
  temp->next = NULL;
  	
  if(pHead!=NULL)
    pHead->next = temp;
  	
  // point to new head
  pHead = temp;
}


void snakeInit() {
  int x = 3;
  int y = 3;
  for (int i=0; i<2; i++, x++) {
    addHead(x,y);
    if (i == 0)
      pTail = pHead;
  }
}

void setDirection() {
    if((ypr[1] * 180/M_PI)<-30 && (ypr[2]*180/M_PI)>-5 && (ypr[2]*180/M_PI)<5){//up
          if(curDirection!=2)
            newDirection = 1;
   }
   else  if((ypr[1] * 180/M_PI)>-5&&(ypr[1] * 180/M_PI)<5&&(ypr[2] * 180/M_PI)<-40){//left
           if(curDirection!=1)
             newDirection = 2;
   }
   else if((ypr[1] * 180/M_PI)>-5&&(ypr[1] * 180/M_PI)<5&&(ypr[2] * 180/M_PI)>20){//right
           if(curDirection!=4)
             newDirection = 3;
   }
    else if((ypr[1] * 180/M_PI)>30 && (ypr[2]*180/M_PI)>-5 && (ypr[2]*180/M_PI)<5){//down
           if(curDirection!=3)
             newDirection = 4;
   }

}

void moveSnake(int direction)
{
  int newX = pHead->x;
  int newY = pHead->y;
  if(direction==1)
    newY--;
  if(direction==2)
    newY++;
  if(direction==3)
    newX--;
  if(direction==4)
    newX++;

  if(newX > 8)
    newX=1;
  if(newX < 1)
    newX=8;
  if(newY > 8)
    newY=1;
  if(newY < 1)
    newY=8;

  dead |= check(newX, newY);

  if(!dead) {
    if(newX==appleX && newY==appleY) {
      addHead(newX, newY);
      newApple();
    }
    else {
      Link *temp = pTail;
      			
      // point to new tail
      pTail = pTail->next;
      			
      // new head
      pHead->next = temp;
      pHead = temp;
      			
      pHead->x = newX;
      pHead->y = newY;			
      pHead->next = NULL;
    } 
  }
}

void newApple() {
  boolean check = 0;
  Link * ptr = pTail;
  do {
    check = 0;
    appleX = random(7) + 1;
    appleY = random(7) + 1;
    Serial.println(appleX);
    while(ptr!=NULL) {
      if(appleX==(ptr->x) && appleY==(ptr->y)) {
        check = 1;
        break;
      }
      ptr = ptr->next;
    }
  } while (check == 1);
}
    
boolean check(int x, int y) {
  Link *ptr;
  ptr = pTail;
  while(ptr!=NULL)
  {
    if(x==ptr->x && y==ptr->y)
      return 1;
    ptr=ptr->next;
  }
  
  return 0;
}
  
//
// display driver
//
void screenUpdate() {
  Link * ptr;
  ptr = pTail;
  
  clrscr();
  while(ptr!=NULL) {
    led[ptr->y-1] = led[ptr->y-1] | (1<<(8-ptr->x));
    ptr = ptr->next;
  }
  
  led[appleY-1] = led[appleY-1] | (1<<(8-appleX));
}  
  
void screenDisplay() {
  byte row = B10000000;

  for (byte k = 0; k < 8; k++) {
    // Open up the latch ready to receive data
    digitalWrite(latchPin, LOW);
    
    //shiftData(~row); // if use PNP transitors
    shiftData(row);
    //shiftData(~led[k]);
    shiftData(led[k]);
    // Close the latch, sending the data in the registers out to the matrix
    digitalWrite(latchPin, HIGH);
    row = row >> 1;
  }
}

void clrscr()
{
  for(int i=0; i<8; i++) {
    led[i] = B00000000;
  }
}

void shiftData(byte data) {
  // Shift out 8 bits LSB first,
  // on rising edge of clock
  boolean pinState;

  //clear shift register read for sending data
  digitalWrite(dataPin, LOW);

  // for each bit in dataOut send out a bit
  for (int i=0; i<8; i++) 
  {
    //set clockPin to LOW prior to sending bit
    digitalWrite(clockPin, LOW);

    // if the value of data and (logical AND) a bitmask
    // are true, set pinState to 1 (HIGH)
    if (data & (1<<i)) 
    {
      pinState = HIGH;
    }
    else 
    {
      pinState = LOW;
    }

    //sets dataPin to HIGH or LOW depending on pinState
    digitalWrite(dataPin, pinState);

    //send bit out on rising edge of clock
    digitalWrite(clockPin, HIGH);
    digitalWrite(dataPin, LOW);
  }

  //stop shifting
  digitalWrite(clockPin, LOW);
}

