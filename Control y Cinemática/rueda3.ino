#include <util/atomic.h> // For the ATOMIC_BLOCK macro 

#define ENCA 2 // YELLOW 
#define ENCB 3 // WHITE 
 
#define dir_1 4 
#define pwm_1 6 
 
volatile float posi = 0; 
long prevT = 0; 
float radianesprev = 0; 
 
float xe[2][1]={0,0}; 
float x[2][1]={0,0}; 
float u=0; 
float e=0; 
 
float K[1][2] = {0.9889, 0.8626}; 
float KI = 22.3607; 
float L[2][1] = {1.0e-4*0.4880,-1.0e-4*0.0021}; 
float A[2][2] = {-4,23509,-206,-29231}; 
float B[2][1] = {0,15385}; 
int C[1][2] = {1,0}; 
 
void readEncoder(){ 
  int b = digitalRead(ENCB); 
  if(b > 0){ 
    posi++; 
  } 
  else{ 
    posi--; 
  } 
} 
 
void setMotor(int dir, int pwmVal){ 
  float duty=pwmVal*100/255; 
  analogWrite(pwm_1,pwmVal); 
  
  if(dir == 1){ 
    digitalWrite(dir_1,LOW); 
  } 
  else if(dir == -1){ 
    digitalWrite(dir_1,HIGH); 
  } 
} 
 
void setup(){ 
  TCCR0B=TCCR0B & B11111000 | B00000010;
  //declare pins as INPUT/OUTPUT 
  pinMode(pwm_1,OUTPUT); 
  pinMode(dir_1,OUTPUT); 
  Serial.begin(9600); //I am using Serial Monitor instead of LCD display 
  pinMode(ENCA,INPUT); 
  pinMode(ENCB,INPUT); 
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING); 
} 
void loop() 
{ 
  if (Serial.available() > 0){
    float w3;
    String data = Serial.readStringUntil('\n');
    w3=data.toFloat();
    //w3 = w3+0.5;
    //Serial.print("You sent me: ");
    //Serial.print(w3);
  
  float target = w3                                                                                                                                                                                                                                                                                                                                                    ; 
   
  float pos = 0; 
  float velocidad = 0; 
  float radianes = 0; 
   
  // time difference 
  long currT = micros(); 
  float deltaT = ((float) (currT - prevT))/( 1.0e6 ); 
  prevT = currT; 
      
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
  { 
      pos = posi; 
      radianes = 6.28318530718*posi/3070; 
      velocidad = (radianes-radianesprev)/(deltaT); 
      radianesprev = radianes; 
  } 
  // integrador 
  e=e+deltaT*(target-velocidad); 
 
  // observador 
  float A_xe[2][1]={A[1][1]*xe[1][1]+A[1][2]*xe[2][1],A[2][1]*xe[1][1]+A[2][2]*xe[2][1]}; 
  float B_u[2][1]={B[1][1]*u,B[2][1]*u}; 
  float C_xe = C[1][1]*xe[1][1]+C[1][2]*xe[2][1]; 
  float L_dy[2][1] = {L[1][1]*(velocidad-C_xe),L[2][1]*(velocidad-C_xe)}; 
 
  float todo[2][1] = {A_xe[1][1]+B_u[1][1]+L_dy[1][1],A_xe[2][1]+B_u[2][1]+L_dy[2][1]}; 
  float quiero=todo[2][1]; 
  float xe[2][1] = {velocidad,xe[2][1]+deltaT*todo[2][1]}; 
 
   
  // control signal 
  u = -K[1][1]*velocidad-K[1][2]*xe[2][1]+KI*e; 
 
  // motor power 
  float pwr = fabs(u);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
  if( pwr > 254 ){ 
    pwr = 254; 
  } 
 
  // motor direction 
  int dir = 0; 
  if(target<0){ 
    dir = -1; 
  } 
  else if (target>0){
    dir = 1;
  }
  else if (target==0){
    pwr=0;
  }
  // signal the motor 
  setMotor(dir,pwr);
  //Serial.print(u), 
  //Serial.print(" "); 
  //Serial.print(e); 
  //Serial.print(" "); 
  Serial.print(velocidad); 
  //Serial.print(" "); 
  //Serial.print(posi,5); 
  //Serial.println(); 
  delay(deltaT); 
  }
  //while(1) continue; //terminate the program 
}
