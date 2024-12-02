#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncUDP.h>

#include <array>

#include "RotationStabilizer.h"
#include "mpu.h"
#include "TorqueToRotors.h"

#define INTERUP

const unsigned int PWM_RANGE {1024};
const unsigned long PWM_FREQ {8000};

const unsigned short PIN {13};

#define WIFI_SSID "SSID"
#define WIFI_PASSWORD "PASSWORD"

#define SDA 4 // D2
#define SCL 0 // D3
//#define INTERRUPT_PIN 14  // D5
#define INTERRUPT_PIN 5 // D1

const unsigned short ROTOR_LB {15};  // D8
const unsigned short ROTOR_LF {13};  // D7

const unsigned short ROTOR_RB {14};  // D5
const unsigned short ROTOR_RF {12};  // D6

// applied trim values
unsigned int trim_throttle = 0;
unsigned int max_trim_deflection = 10;
Quaternion trimQuaternion {1.0f,0.0f,0.0f,0.0f};

// user input values (from remote control)
unsigned int user_throttle = 0;
unsigned int max_user_deflection = 30;
Quaternion userQuaternion {1.0f, 0.0f, 0.0f, 0.0f};

unsigned long last_registered_packet_time = 0;

unsigned int global_power = 0;
Quaternion globalRotationQuaternion {1.0f, 0.0f, 0.0f, 0.0f};


// calculated power values for each rotor
unsigned int power_ROTOR_LB = 0;
unsigned int power_ROTOR_LF = 0;
unsigned int power_ROTOR_RB = 0;
unsigned int power_ROTOR_RF = 0;

AsyncUDP udp;
AsyncWebServer server(80);
ESP8266WiFiClass wifi;

uint16_t host_port = 5000;
IPAddress host_ip(192, 168, 1, 4);

int value=127;

MPU6050 mpu;

constexpr float TrConstant = 0.0000611f;
constexpr float MrConstant = 0.0000015f;

constexpr float l = 0.02f;

constexpr std::array<const std::array<float, 4>,4> B_matrix
{ {
    {
        TrConstant,
        TrConstant,
        TrConstant,
        TrConstant
    },
    {
        TrConstant * l / sqrtConstExpr(2),
        -TrConstant * l / sqrtConstExpr(2),
        -TrConstant * l / sqrtConstExpr(2),
        TrConstant * l / sqrtConstExpr(2)
    },
    {
        TrConstant * l / sqrtConstExpr(2),
        TrConstant * l / sqrtConstExpr(2),
        -TrConstant * l / sqrtConstExpr(2),
        -TrConstant * l / sqrtConstExpr(2)
    },
    {
        MrConstant, -MrConstant, MrConstant, -MrConstant
    }
} };

constexpr std::array inverted_B_Matrix{ invertMatrix4x4(B_matrix) };

  Quaternion last_registered_quaternion;
  Quaternion registered_now;
  VectorFloat needed_thrust;

void setup() {
  Serial.begin(74880);

  analogWriteFreq(PWM_FREQ);
  analogWriteRange(PWM_RANGE);

  // rotor control pins as output (PWM)
  pinMode(ROTOR_LB, OUTPUT);
  pinMode(ROTOR_LF, OUTPUT);
  pinMode(ROTOR_RB, OUTPUT);
  pinMode(ROTOR_RF, OUTPUT);

  analogWrite(ROTOR_LB, PWM_RANGE);
  analogWrite(ROTOR_LF, PWM_RANGE);
  analogWrite(ROTOR_RB, PWM_RANGE);
  analogWrite(ROTOR_RF, PWM_RANGE);

  pinMode(PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  analogWrite(LED_BUILTIN, PWM_RANGE);

  //analogWrite(PIN, PWM_RANGE - 1);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("Wifi started");

  Serial.println("Connecting to WiFi..");
  while(WiFi.status() != WL_CONNECTED){
      delay(1000);
      Serial.println("Connecting...");
  }

  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP());

  //if(udp.connect(IPAddress(192,168,1,4),58099))
  if(udp.listen(50000))
  {
      Serial.println("UDP Connected on IP: ");
      Serial.println(WiFi.localIP());
      udp.onPacket([](AsyncUDPPacket packet){
        Serial.print("UDP Packet Type: ");
        Serial.print(packet.isBroadcast()?"Broadcast":packet.isMulticast()?"Multicast":"Unicast");
        Serial.print(", From: ");
        Serial.print(packet.remoteIP());
        Serial.print(":");
        Serial.print(packet.remotePort());
        Serial.print(", To: ");
        Serial.print(packet.localIP());
        Serial.print(":");
        Serial.print(packet.localPort());
        Serial.print(", Length: ");
        Serial.print(packet.length());
        Serial.print(", Data: ");
        Serial.write(packet.data(), packet.length());
        Serial.println();
        //reply to the client
        packet.printf("%.2f,%.2f,%.2f\n%d,%d,%d,%d", pitch,roll,yaw,power_ROTOR_LB,power_ROTOR_LF,power_ROTOR_RB,power_ROTOR_RF);
        host_ip = packet.remoteIP();
        host_port =  packet.remotePort();

        last_registered_packet_time = millis();


        unsigned int user_throttle = 0;
        unsigned int user_pitch = 0;
        unsigned int user_roll = 0;
        unsigned int user_yaw = 0;

        
        unsigned int pitch_degrees = user_pitch * max_user_deflection / 255; 
        unsigned int roll_degrees = user_roll * max_user_deflection / 255;
        unsigned int yaw_degrees = user_yaw * max_user_deflection / 255;

        userQuaternion = applyPitchRolltoQuaternion(Quaternion{1.0f, 0.0f, 0.0f, 0.0f}, pitch_degrees, roll_degrees);
        
        unsigned int trim_pitch = 0;
        unsigned int trim_roll = 0;
        unsigned int trim_yaw = 0;

        trimQuaternion = applyPitchRolltoQuaternion(Quaternion{1.0f, 0.0f, 0.0f, 0.0f}, trim_pitch, trim_roll);

        globalRotationQuaternion = Quaternion{-1.0f,0.0f,0.0f,0.0f} * userQuaternion * trimQuaternion;
      });
  }

  dmpDataReady();
  dmp_setup();
}

int value_before = 127;

unsigned long previous_time = millis();

void loop() {
  static uint32_t tTime[4];

  if( (millis() - tTime[2]) >= 1000 )
  {
    tTime[2] = millis();

    char buffer[40]{0};

    mpu.dmpGetYawPitchRoll(ypr, &registered_now, &gravity);
    mpu.dmpGetGravity(&gravity, &registered_now);
    yaw = ypr[0] * 180/M_PI;
    roll = ypr[1] * 180/M_PI;
    pitch = ypr[2] * 180/M_PI;   

    snprintf(buffer, sizeof(buffer), "%.2f,%.2f,%.2f\n%d,%d,%d,%d", pitch,roll,yaw,power_ROTOR_LB,power_ROTOR_LF,power_ROTOR_RB,power_ROTOR_RF);

    Serial.println(udp.availableForWrite());
    Serial.println(udp.connected());
    
    //Serial.println(buffer);
    AsyncUDPMessage udp_message(20);
    //Serial.println(udp_message.print(buffer));
    //Serial.println(udp.sendTo(udp_message, host_ip,host_port));
    Serial.println(udp.broadcastTo((uint8_t*)buffer, 20, host_port));
    Serial.println(host_port);
    Serial.println(host_ip);

    Serial.print("power_ROTOR_LB : ");
    Serial.println(power_ROTOR_LB);
    Serial.print("  power_ROTOR_LF : ");
    Serial.println(power_ROTOR_LF);
    Serial.print("  power_ROTOR_RB : ");
    Serial.println(power_ROTOR_RB);
    Serial.print("  power_ROTOR_RF : ");
    Serial.println(power_ROTOR_RF);
  }  

  yield();

  unsigned long current_time = millis();
    // if programming failed, don't try to do anything
  if (!dmpReady) return;

    //Serial.println("Dmp Ready");

    // wait for MPU interrupt or extra packet(s) available
    if (!mpuInterrupt && fifoCount < packetSize) return;

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    //Serial.println("Before checking for interrupt");

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      // Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      //Serial.println("Before getting quaternion");
      mpu.dmpGetQuaternion(&registered_now, fifoBuffer);

      unsigned long delta_time = current_time - previous_time;
      previous_time = current_time;
      needed_thrust = computeStabilizingVector(
          registered_now,
          globalRotationQuaternion,
          last_registered_quaternion,
          delta_time,
          0.2f,
          0.81f, 
          2.15f, 
          3.55f
          );
      
      std::array power_vector = multiplyMatrix4x4WithVector(inverted_B_Matrix,{80,needed_thrust.x,needed_thrust.y,0});
      power_ROTOR_LB = sqrtConstExpr(power_vector[0]);
      power_ROTOR_RB = sqrtConstExpr(power_vector[1]);
      power_ROTOR_RF = sqrtConstExpr(power_vector[2]);
      power_ROTOR_LF = sqrtConstExpr(power_vector[3]);

      analogWrite(PIN, PWM_RANGE - power_ROTOR_LF);
      analogWrite(ROTOR_LB, power_ROTOR_LB);
      analogWrite(ROTOR_LF, power_ROTOR_LF);
      analogWrite(ROTOR_RB, power_ROTOR_RB);
      analogWrite(ROTOR_RF, power_ROTOR_RF);

    }
}

IRAM_ATTR void dmpDataReady() {
    mpuInterrupt = true;
}

void dmp_loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    if (!mpuInterrupt && fifoCount < packetSize) return;

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      //  Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

    
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // store roll, pitch, yaw
        yaw = ypr[0] * 180/M_PI;
        roll = ypr[1] * 180/M_PI;
        pitch = ypr[2] * 180/M_PI;                       
    }
}

void dmp_setup() {
    //I2Cdev::begin(400);
    Wire.begin(SDA, SCL);
 
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //  while (Serial.available() && Serial.read()); // empty buffer
  //  while (!Serial.available());                 // wait for data
  //  while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(-2108);
    mpu.setYAccelOffset(-37);
    mpu.setZAccelOffset(998); // 1688 factory default for my test chip

    mpu.setXGyroOffset(102);
    mpu.setYGyroOffset(-80);
    mpu.setZGyroOffset(0);


    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
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