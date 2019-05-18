#include <Servo.h>

//3 sevos used
Servo cam_up;
Servo cam_side;

//Stored possitions
int cam_up_pos=47; //47 centered
int cam_side_pos=0;

int up_min= 10;
int up_max= 90;
int side_min= 0;
int side_max= 180;

char msg=0x00;
int dir= 0;
  
int follow_vel=20;
int sweep_vel=20;

 void setup() 
{
   Serial.begin(115200);
   //Pin 5 for Servo CamUp
   //Pin 6 for Servo CamSide
   cam_side.attach(5);
   cam_up.attach(6);
}
 
void loop() {

  
  //If nothing found, search
  if ( (msg&0x0F)==0x00){
    if (dir == 0 && cam_side_pos == side_max) {
      dir=1;}
    else if (dir == 1 && cam_side_pos == side_min) {
      dir=0;}
      
    cam_side_pos += (dir==0 && cam_side_pos<side_max);
    cam_side_pos -= (dir==1 && cam_side_pos>side_min);
    
    cam_side.write(cam_side_pos); 
    cam_up_pos=47;
    cam_up.write(cam_up_pos); 
    delay(sweep_vel); 
  }
  
  //Follow the ball if is found
  else {
  
    cam_side_pos += (bitRead(msg, 5) && bitRead(msg, 1) && cam_side_pos<side_max);
    cam_side_pos -= (bitRead(msg, 5) && !bitRead(msg, 1) && cam_side_pos>side_min);
    
    cam_up_pos += (bitRead(msg, 4) && !bitRead(msg, 0) && cam_up_pos<up_max);
    cam_up_pos -= (bitRead(msg, 4) && bitRead(msg, 0) && cam_up_pos>up_min);
  
    cam_side.write(cam_side_pos); 
    delay(follow_vel); 
    cam_up.write(cam_up_pos); 
    delay(follow_vel*2); 
  }
  
}

void serialEvent () {
    msg=Serial.read();
    
    //Adjust search range with received thresh
  if ( (msg >> 4)==0 ) { //0 - 180
    side_min= 0;
    side_max= 180;
    sweep_vel=20;
  } 
  else if ( (msg >> 4)==1 ) { //135
    side_min= cam_side_pos-50;
    side_max= cam_side_pos+50;
    sweep_vel=50;
  } 
  else if ( (msg >> 4)==2 ) { //90
    side_min= cam_side_pos-35;
    side_max= cam_side_pos+35;
    sweep_vel=75;
  }
  else if ( (msg >> 4)==3 ) { //45
    side_min= cam_side_pos-15;
    side_max= cam_side_pos+15;
    sweep_vel=100;
  }
  
  //Adjust range between 0 and 180
  if (side_min < 0) {
    side_min-= side_min;
    side_max-= side_min;
  }
  else if (side_max>180) {
    side_min-= side_max-180;
    side_max-= side_max-180;
  }
    
}

