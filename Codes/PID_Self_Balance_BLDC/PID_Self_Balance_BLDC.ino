[////////////////////IMU////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Wire.h>
//Gyro Variables
float elapsedTime, time, timePrev;                    //Variables for time control
int gyro_error = 0;                                   //We use this variable to only calculate once the gyro data error
float Gyr_rawX, Gyro_angle_x;                         //Here we store the raw data read and the angle value obtained with Gyro data
float Gyro_raw_error_x;                               //Here we store the initial gyro data error
//Acc Variables
int acc_error = 0;                                    //We use this variable to only calculate once the Acc data error
float rad_to_deg = 180 / 3.141592654;                 //This value is for pasing from radians to degrees values
float Acc_rawX, Acc_rawY, Acc_rawZ, Acc_angle_x;      //Here we store the raw data read and the angle value obtained with Acc data
float Acc_angle_error_x;                              //Here we store the initial Acc data error
//Total Angle
float Total_angle_x;

////////////////////BLDC////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Servo.h>
Servo right_prop;
Servo left_prop;
double throttle=1200;                                 //initial value of throttle to the motors only 20% of throttle

////////////////////PID///////////////////////////////////////////////////////////////////////////////////////////////////////////// 
float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;
double kp=1.55;//1.55
double ki=0.0003;//0.0003
double kd=0.5;//0.5
float desired_angle = 0;                              //This is the angle in which we want the balance to stay steady

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Wire.begin();                                       //begin the wire comunication
  Wire.beginTransmission(0x68);                       //begin, Send the slave adress (in this case 68)
  Wire.write(0x6B);                                   //make the reset (place a 0 into the 6B register)
  Wire.write(0x00);
  Wire.endTransmission(true);                         //end the transmission
  //Gyro config
  Wire.beginTransmission(0x68);                       //begin, Send the slave adress (in this case 68)
  Wire.write(0x1B);                                   //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                                   //Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(true);                         //End the transmission with the gyro
  //Acc config
  Wire.beginTransmission(0x68);                       //Start communication with the address found during search.
  Wire.write(0x1C);                                   //We want to write to the ACCEL_CONFIG register
  Wire.write(0x10);                                   //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);

  Serial.begin(250000);                               //Remember to set this same baud rate to the serial monitor
  time = millis();                                    //Start counting time in milliseconds
  
  Serial.println("Program Setup ...");          
  right_prop.attach(3); //attatch the right motor to p  in 3
  left_prop.attach(5);  //attatch the left motor to pin 5
  /*In order to start up the ESCs we have to send a min value
   * of PWM to them before connecting the battery. Otherwise
   * the ESCs won't start up or enter in the configure mode.
   * The min value is 1000us and max is 2000us, REMEMBER!*/
  left_prop.writeMicroseconds(1000); 
  right_prop.writeMicroseconds(1000);
  delay(7000); /*Give some delay, 7s, to have time to connect
                *the propellers and let everything start up*/
  Serial.println("Angle Calibration ...");                  
  /*Here we calculate the gyro data error before we start the loop
    I make the mean of 200 values, that should be enough*/
  if (gyro_error == 0)
  {
    for (int i = 0; i < 200; i++)
    {
      Wire.beginTransmission(0x68);                   //begin, Send the slave adress (in this case 68)
      Wire.write(0x43);                               //First adress of the Gyro data
      Wire.endTransmission(false);
      Wire.requestFrom(0x68, 2, true);                //We ask for just 4 registers

      Gyr_rawX = Wire.read() << 8 | Wire.read();      //Once again we shif and sum
      Gyro_raw_error_x = Gyro_raw_error_x + (Gyr_rawX / 32.8);
      if (i == 199)
      {
        Gyro_raw_error_x = Gyro_raw_error_x / 200;
        gyro_error = 1;
      }
    }
  }                                                   //end of gyro error calculation
  
  /*Here we calculate the acc data error before we start the loop
    I make the mean of 200 values, that should be enough*/
  if (acc_error == 0)
  {
    for (int a = 0; a < 200; a++)
    {
      Wire.beginTransmission(0x68);
      Wire.write(0x3B);                               //Ask for the 0x3B register- correspond to AcX
      Wire.endTransmission(false);
      Wire.requestFrom(0x68, 6, true);

      Acc_rawX = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
      Acc_rawY = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
      Acc_rawZ = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
      Acc_angle_error_x = Acc_angle_error_x + ((atan((Acc_rawY) / sqrt(pow((Acc_rawX), 2) + pow((Acc_rawZ), 2))) * rad_to_deg));
      if (a == 199)
      {
        Acc_angle_error_x = Acc_angle_error_x / 200;
        acc_error = 1;
      }
    } 
  }                                                   //end of acc error calculation               
  Serial.println("Program Begin ...");              
}                                                     //end of setup void


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
 
    timePrev = time;                                  // the previous time is stored before the actual time read
    time = millis();                                  // actual time read
    elapsedTime = (time - timePrev) / 1000;           //divide by 1000 in order to obtain seconds
  
    //////////////////////////////////////Gyro read/////////////////////////////////////
    Wire.beginTransmission(0x68);                     //begin, Send the slave adress (in this case 68)
    Wire.write(0x43);                                 //First adress of the Gyro data
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 2, true);                  //We ask for just 2 registers
  
    Gyr_rawX = Wire.read() << 8 | Wire.read();        //Once again we shif and sum
    /*Now in order to obtain the gyro data in degrees/seconds we have to divide first
    *the raw value by 32.8 because that's the value that the datasheet gives us for a 1000dps range
    *Then remove the initial error*/
    Gyr_rawX = (Gyr_rawX / 32.8) - Gyro_raw_error_x;
    /*Now we integrate the raw value in degrees per seconds in order to obtain the angle
    *If you multiply degrees/seconds by seconds you obtain degrees*/
    Gyro_angle_x = Gyr_rawX * elapsedTime;
    
    //////////////////////////////////////Acc read/////////////////////////////////////
    Wire.beginTransmission(0x68);                     //begin, Send the slave adress (in this case 68)
    Wire.write(0x3B);                                 //Ask for the 0x3B register- correspond to AcX
    Wire.endTransmission(false);                      //keep the transmission and next
    Wire.requestFrom(0x68, 6, true);                  //We ask for next 6 registers starting withj the 3B
    /*We have asked for the 0x3B register. The IMU will send a brust of register.
    The amount of register to read is specify in the requestFrom function.
    In this case we request 6 registers. Each value of acceleration is made out of
    two 8bits registers, low values and high values. For that we request the 6 of them
    and just make then sum of each pair. For that we shift to the left the high values
    register (<<) and make an or (|) operation to add the low values.
    If we read the datasheet, for a range of+-8g, we have to divide the raw values by 4096*/
    Acc_rawX = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
    Acc_rawY = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
    Acc_rawZ = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
    /*Now in order to obtain the Acc angles we use euler formula with acceleration values
      after that we substract the error value found before*/
    Acc_angle_x = (atan((Acc_rawY) / sqrt(pow((Acc_rawX), 2) + pow((Acc_rawZ), 2))) * rad_to_deg) - Acc_angle_error_x;
    
    //////////////////////////////////////Total angle and filter/////////////////////////////////////
    Total_angle_x = 0.98 * (Total_angle_x + Gyro_angle_x) + 0.02 * Acc_angle_x;
    Serial.print("Xº: ");
    Serial.println(Total_angle_x);
    
    /*///////////////////////////P I D///////////////////////////////////*/
    /*Remember that for the balance we will use just one axis. I've choose the x angle
    to implement the PID with. That means that the x axis of the IMU has to be paralel to
    the balance*/
    /*First calculate the error between the desired angle and 
    *the real measured angle*/
    error = Total_angle_x - desired_angle;
    Serial.print("error: ");
    Serial.println(error);
    /*Next the proportional value of the PID is just a proportional constant
    *multiplied by the error*/
    pid_p = kp*error;
    Serial.print("pid_p: ");
    Serial.println(pid_p);
    /*The integral part should only act if we are close to the
    desired position but we want to fine tune the error. That's
    why I've made a if operation for an error between -2 and 2 degree.
    To integrate we just sum the previous integral value with the
    error multiplied by  the integral constant. This will integrate (increase)
    the value each loop till we reach the 0 point*/
    if(-3 <error <3)
    {
      pid_i = pid_i+(ki*error);
      Serial.print("pid_i: ");
      Serial.println(pid_i);  
    }
    /*The last part is the derivate. The derivate acts upon the speed of the error.
    As we know the speed is the amount of error that produced in a certain amount of
    time divided by that time. For taht we will use a variable called previous_error.
    We substract that value from the actual error and divide all by the elapsed time. 
    Finnaly we multiply the result by the derivate constant*/
    pid_d = kd*((error - previous_error)/elapsedTime);
    Serial.print("pid_d: ");
    Serial.println(pid_d);
    /*The final PID values is the sum of each of this 3 parts*/
    PID = pid_p + pid_i + pid_d;
    /*We know that the min value of PWM signal is 1000us and the max is 2000. So that
    tells us that the PID value can oscilate more than -1000 and 1000 because when we
    have a value of 2000us the maximum value that we could substract is 1000 and when
    we have a value of 1000us for the PWM signal, the maximum value that we could add is 1000
    to reach the maximum 2000us*/
    if(PID < -1000)
    {
      PID=-1000;
    }
    if(PID > 1000)
    {
      PID=1000;
    }
    Serial.print("PID:  ");
    Serial.println(PID);
    /*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
    pwmRight = throttle - PID;
    pwmLeft = throttle + PID;
    /*Once again we map the PWM values to be sure that we won't pass the min
    and max values. Yes, we've already maped the PID values. But for example, for 
    throttle value of 1300, if we sum the max PID value we would have 2300us and
    that will mess up the ESC.*/
    //Right
    if(pwmRight < 1000)
    {
      pwmRight= 1000;
    }
    if(pwmRight > 2000)
    {
      pwmRight=2000;
    }
    //Lefts
    if(pwmLeft < 1000)
    {
      pwmLeft= 1000;
    }
    if(pwmLeft > 2000)
    {
      pwmLeft=2000;
    }
    Serial.print("pwmRight:  ");
    Serial.println(pwmRight);
    Serial.print("pwmLeft:  ");
    Serial.println(pwmLeft);
    /*Finnaly using the servo function we create the PWM pulses with the calculated
    width for each pulse*/
    left_prop.writeMicroseconds(pwmLeft);
    right_prop.writeMicroseconds(pwmRight);
    previous_error = error;                           //Remember to store the previous error.
}
