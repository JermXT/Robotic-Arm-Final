#include <string.h>

#define EN        8  
#define X_DIR     5 
#define Y_DIR     6
#define Z_DIR     7
#define X_STP     2
#define Y_STP     3 
#define Z_STP     4 


//DRV8825
int delayTime23 = 1000; //NEMA 23
int delayTime17 = 375; //NEMA 17
int delayTime;
//450 32
//400 42


//int stps=1;// Steps to move

//Initial step state
int posX = 0;
int posY = 0;
int posZ = 0;

boolean xtf;
boolean ytf;
boolean ztf;

void step(boolean dir, byte dirPin, byte stepperPin, int steps, int type)

{
  if(type == 17)delayTime = delayTime17;
  if(type == 23)delayTime = delayTime23;
  //Serial.println(delayTime);
  digitalWrite(dirPin, dir);

  //delay(20);

  for (int i = 0; i < steps; i++) {

    digitalWrite(stepperPin, HIGH);

    delayMicroseconds(delayTime); 

    digitalWrite(stepperPin, LOW);

    delayMicroseconds(delayTime); 
    //delay(450/steps);
  }

}

void setup(){

  pinMode(X_DIR, OUTPUT); pinMode(X_STP, OUTPUT);

  pinMode(Y_DIR, OUTPUT); pinMode(Y_STP, OUTPUT);

  pinMode(Z_DIR, OUTPUT); pinMode(Z_STP, OUTPUT);

  pinMode(EN, OUTPUT);

  digitalWrite(EN, LOW);
  Serial.begin(115200);
  Serial.setTimeout(1);
  pinMode(13, OUTPUT); 
  digitalWrite(13,HIGH);
}
String input;
char a[22];
void loop(){
  
  if(Serial.available() > 21){
    
//      String inputSubstr = input.substring(startF + 1);
//      int a[10];
//      for (int i = 0; i < inputSubstr.length() - 1; i++){
//          if (isdigit(inputSubstr.substring(i, i + 1)) 
//              a[i] = inputSubstr.substring(i, i + 1);
      
 
      //input = Serial.readString();
      //digitalWrite(13,HIGH);
      //delay(500);
      //Serial.readBytes(a,22);
      
      //digitalWrite(13,LOW);
      digitalWrite(13,HIGH);
      delay(500);
      Serial.readBytesUntil('z',a,22);
      digitalWrite(13,LOW);
      
      
      
      String input = "";
      for(int i =0;i<22;i++){
        input += a[i];
      }
      int startF = input.indexOf("F");
      String inputSubstr = input.substring(startF + 1);
      int endF = inputSubstr.indexOf("F");
      inputSubstr = input.substring(startF + 1, endF);
      input = "";
      for (int i = 0; i < inputSubstr.length() - 1; i++){
          char c = inputSubstr.charAt(i);
          if (isDigit(c))
              input += c;
      }
       //step(ztf, Y_DIR, Y_STP, 10,17);
      //Serial.println(input);
  
      int xState = ((input[2]-48) + (input[1]-48)*10 + (input[0]-48)*100);
      if(xState >=200){
          xState = -(xState-200);}
      //Serial.println(posX);
      //Serial.println(xState);
      int xStep = xState - posX;

      int yState = ((input[6]-48) + (input[5]-48)*10 + (input[4]-48)*100);
      if(yState >=200){
          yState = -(yState-200);}
      //Serial.println(posY);
      //Serial.println(yState);
      int yStep = yState - posY;

      int zState = ((input[8]-48) + (input[7]-48)*10 + (input[6]-48)*100);
      if(zState >=200){
          zState = -(zState-200);}
      //Serial.println(posZ);
      //Serial.println(zState);
      int zStep = zState-posZ;
      
      if(xStep<0){
        xtf = false;  
      } else{
        xtf=true;  
      }
      posX += xStep;

      if(yStep<0){
        ytf = false;  
      } else{
        ytf=true;  
      }
      posY += yStep;
      
      if(zStep<0){
        ztf = false;  
      } else{
        ztf=true;  
      }
      posZ += zStep;
     //Serial.println(degZ);
      //Serial.println(zStep);
      step(xtf, X_DIR, X_STP, abs(xStep)*32,17); //X, Clockwise
      step(!ytf, Y_DIR, Y_STP, abs(yStep),17); //Y, Clockwise
      step(!ztf, Z_DIR, Z_STP, abs(zStep),23); //Z, Clockwise

  }else{
   digitalWrite(13,HIGH); 
   }
  //step(ztf, X_DIR, X_STP, 1,17);
  //delay(1);
  //step(!ztf, Y_DIR, Y_STP, 1,17);
  //delay(500);
  
}
