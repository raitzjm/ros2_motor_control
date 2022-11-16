//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library

#include "RoboClaw.h"
#include <unistd.h>
#include <iostream>
#include <chrono>


//See limitations of Arduino SoftwareSerial
SERIAL_OBJ m_serial;	
//Use ms instead of micros
RoboClaw roboclaw(&m_serial,30);

#define address 0x80

//Velocity PID coefficients.
#define Kp 1.0
#define Ki 0.5
#define Kd 0.25
#define qpps 44000

void setup() {
  //Open Serial and roboclaw serial ports
  
  roboclaw.begin(115200,"/dev/roboclaw_motor_back_right");
  
  //Set PID Coefficients
  //roboclaw.SetM1VelocityPID(address,Kd,Kp,Ki,qpps);
  //roboclaw.SetM2VelocityPID(address,Kd,Kp,Ki,qpps);  
}

void displayspeed(void)
{
  using namespace std;
  uint8_t status1,status3;
  //uint8_t status2,status4;
  bool valid1,valid3;
  bool valid2,valid4;
  
  auto start_micros= std::chrono::high_resolution_clock::now();
  int32_t enc1= roboclaw.ReadEncM1(address, &status1, &valid1);
  //int32_t enc2 = roboclaw.ReadEncM2(address, &status2, &valid2);
  int32_t speed1 = roboclaw.ReadSpeedM1(address, &status3, &valid3);
  //int32_t speed2 = roboclaw.ReadSpeedM2(address, &status4, &valid4);
  int16_t current1 = 0;
  valid4 = 0;
  valid4 = roboclaw.ReadCurrents(address,current1);

  auto micros_now = std::chrono::high_resolution_clock::now();
  auto int_duration = std::chrono::duration_cast<std::chrono::microseconds>(micros_now - start_micros);
  std::cout<<"Commands took ms: "<< int_duration.count()/(float)1000<<std::endl;

  cout<<("Encoder1:");
  if(valid1){
    cout<<enc1<<" ";
    //cout<<status1<<" ";
  }
  else{
    cout<<("invalid ");
  }
  
  
  cout<<("Speed1:");
  if(valid3){
    cout<<speed1<<" ";

  }
  else{
    cout<<("invalid ");
    
  }
  
  
  cout<<"Current 1:";
  if(valid4)
  {
    cout<<current1<<'\n';
  }
  else
  {
    cout<<"invalid";
  }

  cout<<std::endl;
}
int main() 
{
  setup();
  
  
  uint32_t speed =12000; 
  //roboclaw.SpeedM1(address,speed * 1);
  
  while(1)
  {
        //roboclaw.SpeedM1(address,speed * 1);
        for(uint8_t i = 0;i<100;i++){
            
            
		        

            
            roboclaw.SpeedM1(address,speed * i);
            displayspeed();
            
            usleep(10* 1000);  
        }

        //roboclaw.SpeedM1(address,speed * -1);
        for(uint8_t i = 100;i>=0;i--){
            roboclaw.SpeedM1(address,speed*i); 
            displayspeed();
            usleep(10* 1000);  
        }

        

        
       
        
    }
}