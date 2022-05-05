/* PURPOSE: to detect rpm by counting pulses from encoder */

bool dir = 1;    // 1 for RIGHT
int pwm = 0;
bool motion = 0;

// DEFINING ARDUINO PINS
int pendulum_encoder_pin_A = 20;
int pendulum_encoder_pin_B = 18;

int carriage_encoder_pin_A = 21;
int carriage_encoder_pin_B = 19;

int motorA = 5;   // IF HIGH => LEFT MOTION
int motorB = 6;   // IF HIGH => RIGHT MOTION


// DEFINING REQUIRED CONSTANTS
int currentTime = 0; 
int previousTime = 0;
int duration = 40; //in ms

double dist = 0.0;
double angle = 0.0;
double dist_prev = 0.0;
double angle_prev = 0.0;
double dist_dot = 0.0;
double angle_dot = 0.0;

float pi = 3.14;
float wheel_radius = 13.4;  //mm
volatile long pulse_count_carriage = 0 ;
volatile long pulse_count_pendulum = 0 ;
int total_pulses = 0;
int pulses = 0;

// DEFINING CONTROLLER CONSTANTS 
float k[4] = { -18.257, -85.051, 1865.405, 1878.538 }; 
float nbar = -22.0;
float desired_para[4][1] = {{0.0},{0.0},{0.0},{0.0}};
float current_para[4][1] = {{0.0}, {0.0}, {0.0}, {0.0}};
float error[4][1] = {{0.0}, {0.0}, {0.0}, {0.0}};
float output = 0.0;

void setup() {
  pinMode(motorA, OUTPUT);  
  pinMode(motorB, OUTPUT);  
  pinMode(carriage_encoder_pin_B, INPUT_PULLUP);
  pinMode(pendulum_encoder_pin_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(pendulum_encoder_pin_A), StartInterrupt_pendulum, RISING); // when it gets a signal at rising edge it will call StartInterruptA() function
  attachInterrupt(digitalPinToInterrupt(carriage_encoder_pin_A), StartInterrupt_carriage, RISING); // when it gets a signal at rising edge it will call StartInterruptA() function
  Serial.begin(9600);
}

void loop() {
  if (dist<320.0 and dist>-320.0){
    if (dir==1){
    analogWrite(motorB, pwm);
    digitalWrite(motorA,LOW);
    }
    else{
      analogWrite(motorA, pwm);
      digitalWrite(motorB,LOW);
      }
  }
  else{ digitalWrite(motorA,LOW); digitalWrite(motorB,LOW); }
  currentTime = millis(); //initializing timer
  // if difference in times is greater than given duration, calculate the rpm using the pulse count and then set it to zero
  if (currentTime - previousTime > duration){

    detachInterrupt(carriage_encoder_pin_A);
    detachInterrupt(pendulum_encoder_pin_A);
    
    
    previousTime = currentTime; 

    dist += double(pulse_count_carriage)*pi*wheel_radius/600.0;
    angle += (double(pulse_count_pendulum)*2.0*pi)/600.0;
    
//    Serial.println("pulse = " + String(pulse_count_carriage) + " dist = " + String(dist));
//      Serial.println(String(dist) + " angle:  " + String(angle*180.0/pi));
//    Serial.println("pulse_count_carriage "+String(total_pulses));

    pulse_count_pendulum = 0;
    pulse_count_carriage = 0;    

    attachInterrupt(digitalPinToInterrupt(carriage_encoder_pin_A), StartInterrupt_carriage, RISING);
    attachInterrupt(digitalPinToInterrupt(pendulum_encoder_pin_A), StartInterrupt_pendulum, RISING);
    
  }
  

  
  dist_dot = (dist-dist_prev)/float(duration);
  angle_dot = (angle-angle_prev)*1000.0/float(duration);
  dist_prev = dist; 
  angle_prev = angle;

  current_para[0][0] = float(dist/1000.0);
  current_para[1][0] = float(dist_dot);
  current_para[2][0] = float(angle);
  current_para[3][0] = float(angle_dot);

  if (angle*180.0/pi>0.2){motion = 1;}
  if (motion == 1){
    for(int p = 0; p < 4; p ++) // Calculating error
      {
      error [p][0]= current_para[p][0] - desired_para[p][0]; 
  //    Serial.println("current "+ String(current_para[p][0])+ " desired "+ String(desired_para[p][0]) + " error " + String(error[p][0]));
      }
  
    output = nbar;
    for(int q = 0; q < 4; q ++){
     output += -1*k[q]*error[q][0];
  //    Serial.print(k[q]);
  //    Serial.println("k[q] "+ String(k[q])+ " error "+ String(error[q][0]) + " output " + String(output)+ " angle "+ String(angle));
     }
  
    
    if (output>250.0){output = 245;}
    else if (output<-250.0) {output = -245;}
    
    if (float(angle*180.0/3.14)>-20.0 and float(angle*180.0/3.14)<20.0){
      pwm = abs(int(output));
      if (output<0.0){
        dir = 0;
      }
      else{
        dir = 1;
      }
    }
    else {pwm = 0;}
//    Serial.println(String(pwm) + "  "  + String(angle*180.0/pi));
//    Serial.println(output); 
  }
}

void StartInterrupt_pendulum(){
    //Serial.println(digitalRead(pendulum_encoder_pin_B));
    if (!digitalRead(pendulum_encoder_pin_B)){
      pulse_count_pendulum -= 1;
    }
    else{
      pulse_count_pendulum += 1;      
    }
}

void StartInterrupt_carriage(){
//    Serial.println(digitalRead(carriage_encoder_pin_B));
    if (!digitalRead(carriage_encoder_pin_B)){
      pulse_count_carriage -= 1;
    }
    else{
            pulse_count_carriage += 1;      
    }
}
