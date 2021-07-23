# Gesture Controlled Robot
I am working on a Gesture Controlled Robot at Bluestamp. The robot is controlled by a glove on my hand, so when I move it, it goes a certain direction.

| **Engineer** | **School** | **Area of Interest** | **Grade** |
|:--:|:--:|:--:|:--:|
| Shashwath D. | Miller Middle School | Electrical and Mechanical Engineering | Rising 7th Grader

![Headstone Image](https://bluestampengineering.com/wp-content/uploads/2016/05/improve.jpg)

# Presentation

[![Shashwath D Demo Night](https://res.cloudinary.com/marcomontalbano/image/upload/v1627058272/video_to_markdown/images/youtube--YnS6cLp3XrA-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://youtu.be/YnS6cLp3XrA "Shashwath D Demo Night")
  
  
# Final Milestone
My final milestone is the increased reliability and accuracy of my robot. I ameliorated the sagging and fixed the reliability of the finger. As discussed in my second milestone, the arm sags because of weight. I put in a block of wood at the base to hold up the upper arm; this has reverberating positive effects throughout the arm. I also realized that the forearm was getting disconnected from the elbow servo’s horn because of the weight stress on the joint. Now, I make sure to constantly tighten the screws at that joint. 

[![Final Milestone](https://res.cloudinary.com/marcomontalbano/image/upload/v1612573869/video_to_markdown/images/youtube--F7M7imOVGug-c05b58ac6eb4c4700831b2b3070cd403.jpg )](https://www.youtube.com/watch?v=F7M7imOVGug&feature=emb_logo "Final Milestone"){:target="_blank" rel="noopener"}


# Second Milestone

My second milestone was completing the robot. I synced up the ESPs and added the accelerometer. I also made a code to send accelerometer values between two ESPs. I did have trouble with this and I had to constantly rewrite the code for about three days, but I finally got it by modifying a code for sending a humdity sensor's valus between ESPs to send accelerometer values. Finally, I used a portable charger to make a glove for the ESP and accelerometer. The glove now sends accelerometer values to the ESP on the robot and based on the value it moves forwar, backward, right, or left. 

[![Third Milestone](https://res.cloudinary.com/marcomontalbano/image/upload/v1612574014/video_to_markdown/images/youtube--y3VAmNlER5Y-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://www.youtube.com/watch?v=y3VAmNlER5Y&feature=emb_logo "Second Milestone"){:target="_blank" rel="noopener"}


# First Milestone
  
My first milestone was making the robot. I made the car chassis and wired up the motors. Next, I coded the Arduino and made the blocks of code to move forward, backward, left, right, and to stop. I then replaced the arduino with an ESP because that is what will be receiving the input values from the other ESP in the finished product.  I had some trouble with the motor driver as it was not letting two wires into one output(I am doing a tank drive, so two motors per each side of the motor driver). For one output the screw was not going down. I was able to fix theses problems by using dupont wires which have more grip, so it wouldn't fall out while I was screwing them in and also didn't even need the screw to stay secure. I am hoping to make the glove and sync up the ESPs in the next milestone and hope to finish my modifications by my last milestone.

[![First Milestone](https://res.cloudinary.com/marcomontalbano/image/upload/v1626453148/video_to_markdown/images/youtube--z22beFnXq0o-c05b58ac6eb4c4700831b2b3070cd403.jpg)]( https://youtu.be/z22beFnXq0o "First Milestone")


# Circuit Diagrams

<p>Robot Chassis Circuit</p>

![Screen Shot 2021-07-16 at 10 53 16 AM](https://user-images.githubusercontent.com/68804388/125990282-18efdeb1-e7f6-4591-aaca-a118906f85e1.png)

<p>Glove Circuit</p>

![Screen Shot 2021-07-16 at 10 57 48 AM](https://user-images.githubusercontent.com/68804388/125990633-b0d11461-2126-4c4b-b513-b3339d3fa8ea.png)


# Code

<p>Sender Code</p>

```c++

// Modified from code over here: https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/
#include <esp_now.h>
#include <WiFi.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
int lcd_loop_counter;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;


/////////////////////////////////////////////////////////////////////////



// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x08, 0x3A, 0xF2, 0x6D, 0x09, 0x6C};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  float b;
  float c;
  bool e;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);





    // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  

//////////////////////////////////////////////////////////////







 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
  // Set values to send
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  acc_total_vector = sqrt((a.acceleration.x*a.acceleration.x)+(a.acceleration.y*a.acceleration.y)+(a.acceleration.z*a.acceleration.z));
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)a.acceleration.y/acc_total_vector)* 57.296;      
  angle_roll_acc = asin((float)a.acceleration.x/acc_total_vector)* -57.296;       

  angle_pitch_acc -= -1.89;                                             
  angle_roll_acc -= -6.1;
  
  myData.b = angle_pitch_acc;
  myData.c = angle_roll_acc;
  myData.e = true;

  Serial.print("acc_pitch: ");
  Serial.print(myData.b);
  Serial.print("    acc_roll: ");
  Serial.println(myData.c);

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
    
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(100);
}
```

<p>Receiver Code</p>

```c++

// Modified from code over here: https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/

#include <esp_now.h>
#include <WiFi.h>

int motorPin1=16; //left motor output 1
int motorPin2=17; //left motor output 2
int motorPin3=18;  //right motor output 1
int motorPin4=19;  //right motor output 2

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {

    float b;
    float c;
    bool e;
} struct_message;

int len;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  pinMode(motorPin1,OUTPUT);
  pinMode(motorPin2,OUTPUT);
  pinMode(motorPin3,OUTPUT);
  pinMode(motorPin4,OUTPUT);
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  //  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  esp_now_register_recv_cb(OnDataRecv);
  
  if(myData.e){
    Serial.print("Bytes received: ");
    Serial.println(len);
    Serial.print("Pitch: ");
    Serial.println(myData.b);
    Serial.print("Roll: ");
    Serial.println(myData.c);
    Serial.print("Bool: ");
    Serial.println(myData.e);
    Serial.println();
    myData.e = false;

    delay(500);
  }
  if(myData.c > 30){
    digitalWrite(motorPin1,HIGH);
    digitalWrite(motorPin2,LOW);
    digitalWrite(motorPin3,HIGH);
    digitalWrite(motorPin4,LOW);
  }

  else if(myData.c < -27){
    digitalWrite(motorPin1,LOW);
    digitalWrite(motorPin2,HIGH);
    digitalWrite(motorPin3,LOW);
    digitalWrite(motorPin4,HIGH);
  }
  
  else if(myData.b > 26){
    digitalWrite(motorPin1,HIGH);
    digitalWrite(motorPin2,LOW);  
    digitalWrite(motorPin3,LOW);
    digitalWrite(motorPin4,HIGH);
  }
  
  else if(myData.b < -26){
    digitalWrite(motorPin1,LOW);
    digitalWrite(motorPin2,HIGH);
    digitalWrite(motorPin3,HIGH);
    digitalWrite(motorPin4,LOW);
  }

  else{
    digitalWrite(motorPin1,LOW);
    digitalWrite(motorPin2,LOW);
    digitalWrite(motorPin3,LOW);
    digitalWrite(motorPin4,LOW);
  }
     
}
```











