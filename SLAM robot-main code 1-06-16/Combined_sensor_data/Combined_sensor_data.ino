//#include <MovingAverageFilter.h>
#include <Average.h>
#include <NewPing.h>
#include <Servo.h>
#include <RunningMedian.h>
#include <PID_v1.h>
#include "MPU6050.h"
#include "Wire.h"
#include "math.h"
#include "main.h"
#include "I2Cdev.h"


int RPM = 60;
bool has_rotated_90 = false;
bool wall_hit = false;
bool initial_reading_read = false;
enum  STATE return_state = PATHING;
int initial_y = 0;

//Create sonar class with corresponding pins
NewPing sonar = { NewPing(12,13, MAX_DISTANCE)};
RunningMedian Sonar_read = RunningMedian(9); //Create running median filter for sonar
//Parameters for PID
control Vx;
control Vy;
control Turning;
control Orient;
control Align;


//Creating timer class to count elapsed time
timer Timer;

//Creating arena class to describe enclosure
arena Arena;
//Create PID class to be used for turning and going straight
PID TurningPID(&Turning.Input,&Turning.Output,&Turning.Setpoint,Turning.consKp,Turning.consKi,Turning.consKd,DIRECT);
PID VxPID(&Vx.Input,&Vx.Output,&Vx.Setpoint,Vx.consKp,Vx.consKi,Vx.consKd,DIRECT);
PID VyPID(&Vy.Input,&Vy.Output,&Vy.Setpoint,Vy.consKp,Vy.consKi,Vy.consKd,DIRECT);
PID OrientPID(&Orient.Input,&Orient.Output,&Orient.Setpoint,Orient.consKp,Orient.consKi,Orient.consKd,DIRECT);
PID AlignPID(&Align.Input,&Align.Output,&Align.Setpoint,Align.consKp,Align.consKi,Align.consKd,DIRECT);

//obstacle variables:
  uint8_t Obstacle_case = 0;
  uint8_t Obstacle_Flag = 0;
  double Previous_Sonar = 0;
  uint8_t Global_Distance = 28;
  
void setup() {
  Serial1.begin(115200);
  pingTimer = millis() + 75;
//PID setup
  TurningPID.SetOutputLimits(-1.50,1.50); //Set angular velocity limit to be within 1.75 rad/s
  Turning.aggKp=0.9, Turning.aggKi=0.0, Turning.aggKd=0.01; //aggresive gains
  Turning.consKp=0.1, Turning.consKi=0.0, Turning.consKd=0.01; //Conservative gains

  VxPID.SetOutputLimits(-1.75,1.75); //Set Vx velocity limit to be within 1.75 rad/s
  Vx.aggKp=0.9, Vx.aggKi=0, Vx.aggKd=0; //aggresive gains
  Vx.consKp=1, Vx.consKi=0.05, Vx.consKd=0.25; //Conservative gains

  VyPID.SetOutputLimits(-1.75,1.75); //Set angular Vy limit to be within 1.75 rad/s
  Vy.aggKp=0.9, Vy.aggKi=0, Vy.aggKd=0.01; //aggresive gains
  Vy.consKp=0.9, Vy.consKi=0, Vy.consKd=0; //Conservative gains

  OrientPID.SetOutputLimits(-0.75,0.75); //Set angular Vy limit to be within 1.75 rad/s
  Orient.aggKp=0.9, Orient.aggKi=0, Orient.aggKd=0; //aggresive gains
  Orient.consKp=0.5, Orient.consKi=0, Orient.consKd=0; //Conservative gains

  AlignPID.SetOutputLimits(-1.5,1.5); //Set angular Vy limit to be within 1.75 rad/s
  Align.aggKp=0.5, Align.aggKi=0.0, Align.aggKd=0.05; //aggresive gains
  Align.consKp=0.3, Align.consKi=0.0, Align.consKd=0.05; //Conservative gains

//Arena setup
  Arena.Width = 0, Arena.Length = 0, Arena.index = 0, Arena.loop_count = 0, Arena.corner_count = 0;
//MPU SETUP
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  // initialize device
  Serial1.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial1.println("Testing device connections...");
  Serial1.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // configure Arduino LED for
  pinMode(LED_PIN, Turning.Setpoint);

  MPU9150_writeSensor(MPU9150_CONFIG,0x03); //set DLPF bit3 as true(actually, i dont know how to enable this filter...)
  GB=0;
  MPU_timer = millis();
}

void loop() {
  //Sonar sensor reading
 if(millis() >= pingTimer) { //Sonar encountered problem when running in ISR so they will run in the main program loop
  pingTimer += PING_INTERVAL;

 sonar.timer_stop();
 cm = 0;
 sonar.ping_timer(echoCheck);

 }

 //MPU sensor reading
  // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    double dt = (double)(millis() - MPU_timer) / 1000; // Calculate delta time
    MPU_timer = millis();

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);//currently, we only need gyroscope to get the angle

  
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    
    double gyroZrate = ((gz +8)/ 131.0);//from the register map of MPU9150, scale range is +/- 250degree/sec and the Turning.Setpoint unit is 131/LSB
    //the offset 8 should be measured at each time before we use the MPU 
    if ( abs(gyroZrate)> 0.1){ GB=GB+gyroZrate*dt;  } //set the threshold as 0.1 and then integrate the angle velocity as angle degrees.

    //Update IR Reading
 L_M_IR_D = GetSensorData(1,1);
 R_M_IR_D = GetSensorData(2,2);
 B_L_IR_D = GetSensorData(3,3);
 F_L_IR_D = GetSensorData(4,4);    
 L_M_IR.push(L_M_IR_D);
 R_M_IR.push(R_M_IR_D);
 B_L_IR.push(B_L_IR_D);
 F_L_IR.push(F_L_IR_D);  
 Sonar_read.add(cm);
  PrintData();
  Serial1.println(wall_hit);
    Serial1.println(Obstacle_case);
  Serial1.println(Obstacle_Flag);

//Check if any sensor in the proximity has hit a wall
if ((L_M_IR.mean() < 25) && (R_M_IR.mean() < 25)) wall_hit = true;

 static STATE machine_state = INITIALISING;
 switch (machine_state) {
  case INITIALISING:
   machine_state = initialising();
      break;
  case FIND_WALL:
    machine_state = find_wall();
      break;
  case TURN_90_CW:
    machine_state = turn_90_cw();
      break;
   case ALIGN_WALL_MOV:
    machine_state = align_wall_mov();
      break;
   case PRE_MOV_ALIGN:
    machine_state = pre_mov_align();
      break;
   case GO_TO_CORNER:
    machine_state = go_to_corner();
      break;
   case PATHING:
    machine_state = pathing();
      break;
   case OBSTACLE_AVOIDANCE:
     machine_state = obstacle_avoidance();
     break;
  case RUNNING:
    machine_state = running();
      break;
 }


}

STATE initialising() {
  //Do some initialising
  Serial1.println("INITIALISING....");
  delay(1000); //One second delay to see the Serial1 string "INITIALISING...."
  Serial1.println("Enabling Motors...");
  enable_motors();
  wall_hit = false;
  return FIND_WALL;
}

STATE find_wall() {
  Serial1.println("FINDING WALL....");
  //Reset gyroscope reading for more accurate turning
   gz = 0;
   GB = 0;
  if(wall_hit == false){
    forward();
    return FIND_WALL;
  }
  else{
    if( abs(L_M_IR.mean() - R_M_IR.mean()) > 0.5) {
      if (L_M_IR.mean()>R_M_IR.mean()) {
        cw();
        return FIND_WALL;        
      }
      else {
        ccw();
        return FIND_WALL;
      }
    }
    else {
      stop();
      return_state = GO_TO_CORNER;
      return TURN_90_CW;
    }
  }
}

STATE turn_90_cw() {
  Serial1.print("Turning...");
    Turning.Setpoint = 90;
    Turning.Input = GB;
  if(GB > Turning.Setpoint-5 && GB < Turning.Setpoint+5) {  //If within desired range,start timer
    if(Timer.started == false) {
      Timer.start_time = millis();
      Timer.started = true;
      return TURN_90_CW;
    }
    else if(Timer.started == true) {
      Timer.elapsed = millis() - Timer.start_time;
    }
    if (Timer.elapsed > 2000) { //If it has remained within expected position for more than 2sec
      stop();
      gz = 0;
      GB = 0;
      Timer.started = false;
      Timer.elapsed = 0;
      return ALIGN_WALL_MOV;
    }
  }
  if(GB > Turning.Setpoint + 5 || GB < Turning.Setpoint -5) Timer.elapsed = 0; //If the Robot falls out of range, reset timer
  
  TurningPID.SetMode(AUTOMATIC);  //Turn on PID control
  double gap = abs(GB - Turning.Setpoint); //Calculate the amount of error between current rotation and Turning.Setpoint
  if (gap <10)
  {
    TurningPID.SetTunings(Turning.consKp,Turning.consKi,Turning.consKd); // If little error then use conservative gains
  }
  else 
  {
    TurningPID.SetTunings(Turning.aggKp,Turning.aggKi,Turning.aggKd); //If error large then use aggresive gains
  }
  TurningPID.Compute();
  PID_cw(Turning.Output*(0.1615/0.027)*9.5493);  //Convert OmegaZ to RPM value and Turning.Setpoint to motor
  return TURN_90_CW;
}

STATE align_wall_mov() {
  Serial1.println("ALIGNING TO WALL BY STRAFING....");
  AlignPID.SetMode(AUTOMATIC);  //Turn on PID control
  Align.Setpoint = 28;
  Align.Input = (B_L_IR.mean() +F_L_IR.mean())/2;
  AlignPID.SetTunings(Align.consKp,Align.consKi,Align.consKd);
  AlignPID.Compute();
  if ((Align.Input > Align.Setpoint -2) && (Align.Input < Align.Setpoint + 2)) {// If parralel to wall at wanted distance
     if(Timer.started == false) {
      Serial1.println("HERE");
      Timer.start_time = millis();
      Timer.started = true;
      return ALIGN_WALL_MOV;
    }
     else{
      Timer.elapsed = millis() - Timer.start_time;
    }
     if (Timer.elapsed > 1000) { //If it has remained within expected position for more than 2sec
       //Serial1.println("DISTANCE ALIGNED....");
       stop();
       gz = 0;
       GB = 0;
       wall_hit = false;
       Timer.started = false;
       Timer.elapsed = 0;
       return PRE_MOV_ALIGN; 
    }
    else return ALIGN_WALL_MOV;
  }
  else  { //If current distance higher than distance wanted
    Serial1.println("STRAFING");
    double gap = abs(Align.Input - Align.Setpoint); //Calculate the amount of error between current rotation and Turning.Setpoint
    if (gap < 4)  AlignPID.SetTunings(Align.consKp,Align.consKi,Align.consKd); // If little error then use conservative gains
    else      AlignPID.SetTunings(Align.aggKp,Align.aggKi,Align.aggKd); //If error large then use aggresive gains
    Timer.elapsed = 0; //Resetting the timer
    PID_strafe(Align.Output*(0.1615/0.027)*9.5493); 
    return ALIGN_WALL_MOV;   
  }
}

STATE pre_mov_align() {
  Serial1.println("ALIGNING...");
  Orient.Setpoint = 0;
  OrientPID. SetMode(AUTOMATIC);
  OrientPID.SetTunings(Align.consKp,Align.consKi,Align.consKd);
  double sensor_distance = 12.7;

  if (abs(F_L_IR.mean() - B_L_IR.mean()) > 2) {  //If the front IR larger than back IR by notable amount
    Timer.elapsed = 0;
    Orient.Input = atan2(F_L_IR.mean() - B_L_IR.mean(), sensor_distance)*(180/3.14);
    OrientPID.Compute();
    PID_cw(Orient.Output*(0.1615/0.027)*9.5493);
    Serial1.println(Orient.Input);
    return PRE_MOV_ALIGN;
  }
  else {
      if(Timer.started == false) {
      Serial1.println("HERE");
      Timer.start_time = millis();
      Timer.started = true;
      return PRE_MOV_ALIGN;
    }
     else{
      Timer.elapsed = millis() - Timer.start_time;
    }
    if(Timer.elapsed >1000) {
    Orient.Input = 0; //Ignore small angle differences
    wall_hit = false;
    Timer.started = false;
    Timer.elapsed = 0;
    stop();
    return return_state;
    }
    else return PRE_MOV_ALIGN;
  }
}

STATE go_to_corner() {
    Serial1.println("GO TO INITIAL CORNER....");
    if (L_M_IR.mean() < 28 && R_M_IR.mean() < 28) {
      stop();
      return_state = PATHING;
      return TURN_90_CW; 
    }
    if (abs(B_L_IR.mean() - F_L_IR.mean()) > 1) {
      return_state = GO_TO_CORNER;
      return ALIGN_WALL_MOV;
    }
    else {
      forward();
      return GO_TO_CORNER;
    }
}

STATE pathing() {
  Check_Obstacle();

  if (Obstacle_Flag == 0) {
  Serial1.println("PATHING....");
  if(initial_reading_read == false) {
    Vx.Setpoint = 28;
    Vy.Setpoint = Sonar_read.getMedian() - 20;
    Orient.Setpoint = 0;
    initial_y = Sonar_read.getMedian();
    Vy.Input = 0;
    gz = 0;
    GB = 0;
    initial_reading_read = true;
  }
    if ((Vy.Input < Vy.Setpoint -1) || (Vy.Input > Vy.Setpoint  + 1)) { //hasnt hit the wall yet
      if (Timer.started == false) { //If timer hasnt start ->start the timer
        Serial1.print("Start timer");
        Timer.start_time = millis(); //record the time robot start moving
        Timer.started = true;
        return PATHING;
      }
      else if(Timer.started == true) {  //If robot has started moving            
        Timer.elapsed = millis() - Timer.start_time;  //record total elapsed time since start of movement
        Orient.Input = atan2(F_L_IR.mean()-B_L_IR.mean(),12.7);
        Vx.Input = (B_L_IR.mean() +F_L_IR.mean())/2;
        Vy.Input = initial_y - Sonar_read.getMedian();
        OrientPID.SetMode(AUTOMATIC);
        VxPID.SetMode(AUTOMATIC);
        VyPID.SetMode(AUTOMATIC);
        VyPID.SetTunings(Vy.aggKp,Vy.aggKi,Vy.aggKd);
        VxPID.SetTunings(Vx.aggKp,Vx.aggKi,Vx.aggKd);
        OrientPID.SetTunings(Orient.aggKp,Orient.aggKi,Orient.aggKd);  
        if(abs(Orient.Setpoint - Orient.Input) > 2){
          Serial1.println("turna");
          OrientPID.Compute();
          PID_cw(Orient.Output*(0.1615/0.027)*9.5493);
          Serial1.println(Orient.Output);
          return PATHING;
        }
        if(abs(Vx.Setpoint - Vx.Input) > 3){
          Serial1.print("strafa");
          VxPID.Compute();
          PID_strafe(Vx.Output*(0.1615/0.027)*9.5493);
          Serial1.println(Vx.Output);
          return PATHING;          
        } 
          if(abs(Vy.Setpoint - Vy.Input) > 3){
          //Serial1.println("mova");
          VyPID.Compute();
          PID_forward(Vy.Output*(0.1615/0.027)*9.5493);
          Serial1.println(Vy.Output);
          return PATHING;            
        }
      }
    }
    else {  //If wall has been hit
      wall_hit = false;   //reset wall_hit flag
      Timer.elapsed = millis() - Timer.start_time;  
      Timer.started = false;  //reset clock flag;
      stop(); //Stop the robot to prepare for rotation
      initial_reading_read = false;
    }
    if (Arena.corner_count == 4) {  //If wall has been hit and it is the fourth wall encountered
      int temp;
      Arena.Width = (Arena.Size[1] + Arena.Size[3])/2;  //Find the averaged value of an undetermined dimension of the arena
      Arena.Length = (Arena.Size[2] + Arena.Size[4])/2; //Find another averaged value of an undetermined dimension of the arena
      if (Arena.Width > Arena.Length) { // The longer averaged value would be the length and the other one would be the width of the arena
        temp = Arena.Width;
        Arena.Width = Arena.Length;
        Arena.Length = temp;
      }
      Arena.corner_count = 0;
      Arena.loop_count ++;
      return_state = PATHING; //Pass info about the state that called the rotation state to ensure accurate return point
      return TURN_90_CW;  //Rotate, align and repeat
    }
    else { //If wall has been hit and it has not been 4 walls
    
      Arena.Size[Arena.index] = 10 * Timer.elapsed;  //Calculate the distance of the previously travelled wall
      Arena.corner_count ++;            //increment counters
      Arena.index ++;
      return_state = PATHING;         //Pass info about the state that called the rotation state to ensure accurate return point
      return TURN_90_CW;  //Rotate, align and repeat
    }
  }
  else
  {
      return OBSTACLE_AVOIDANCE;
   }
  
}

STATE obstacle_avoidance()
{
  Serial1.println("Obstacle Avoidance...");


  if ( Obstacle_case == 1)
  {
    if (Obstacle_Flag == 1)
    {
      if (B_L_IR.mean() > (Global_Distance + 20) && F_L_IR.mean() > (Global_Distance + 20))
      {
        Obstacle_Flag = 2;
        forward();
        return OBSTACLE_AVOIDANCE;
      }
      else
      {
        strafe_right();
        Previous_Sonar = cm;
        return OBSTACLE_AVOIDANCE;
      }
    }
    else if (Obstacle_Flag == 2)
    {
      if (Previous_Sonar - cm > 40)
      {
        Obstacle_Flag = 3;
        strafe_left();
        return OBSTACLE_AVOIDANCE;
      }
      else
      {
        forward();
        return OBSTACLE_AVOIDANCE;
      }
    }
    else if (Obstacle_Flag == 3)
    {
      if (B_L_IR.mean() <= (Global_Distance) && F_L_IR.mean() <= (Global_Distance))
      {
        Obstacle_Flag = 0;
        Obstacle_case = 0;
        return PATHING;
      }
      else
      {
        strafe_left();
        return OBSTACLE_AVOIDANCE;
      }
    }
  }
  else if ( Obstacle_case == 2)
  {
    //Serial1.println("go into the case!!!!!!!!!");
    if (Obstacle_Flag == 1)
    {
      Serial1.println ("xxx");
      if (B_L_IR.mean() > (Global_Distance + 10) && F_L_IR.mean() > (Global_Distance + 10))
      {
        Obstacle_Flag = 2;
        forward();
        return OBSTACLE_AVOIDANCE;
      }
      else
      {
        strafe_right();
        Previous_Sonar = cm;
        return OBSTACLE_AVOIDANCE;
      }
    }
    else if (Obstacle_Flag == 2)
    {
      if (Previous_Sonar - cm > 40)
      {
        Obstacle_Flag = 3;
        strafe_left();
        return OBSTACLE_AVOIDANCE;
      }
      else
      {
        forward();
        return OBSTACLE_AVOIDANCE;
      }
    }
    else if (Obstacle_Flag == 3)
    {
      if (B_L_IR.mean() <= (Global_Distance) && F_L_IR.mean() <= (Global_Distance))
      {
        Obstacle_Flag = 0;
        Obstacle_case = 0;
        return PATHING;
      }
      else
      {
        strafe_left();
        return OBSTACLE_AVOIDANCE;
      }

    }
  }
  else if ( Obstacle_case == 4)
  {
    if (Obstacle_Flag == 1)
    {
      if (B_L_IR.mean() > (Global_Distance + 15) && F_L_IR.mean() > (Global_Distance + 15))
      {
        Obstacle_Flag = 2;
        forward();
        return OBSTACLE_AVOIDANCE;
      }
      else
      {
        strafe_right();
        Previous_Sonar = cm;
        return OBSTACLE_AVOIDANCE;
      }
    }
    else if (Obstacle_Flag == 2)
    {
      if (Previous_Sonar - cm > 40)
      {
        Obstacle_Flag = 3;
        strafe_left();
        return OBSTACLE_AVOIDANCE;
      }
      else
      {
        forward();
        return OBSTACLE_AVOIDANCE;
      }
    }
    else if (Obstacle_Flag == 3)
    {
      if (B_L_IR.mean() <= (Global_Distance) && F_L_IR.mean() <= (Global_Distance))
      {
        Obstacle_Flag = 0;
        Obstacle_case = 0;
        return PATHING;
      }
      else
      {
        strafe_left();
        return OBSTACLE_AVOIDANCE;
      }
    }
  }
  else if ( Obstacle_case == 3)
  {
    if (Obstacle_Flag == 1)
    {
      if (B_L_IR.mean() < 45 || F_L_IR.mean() < 45)
      {
        if (B_L_IR.mean() > (Global_Distance + 30) && F_L_IR.mean() > (Global_Distance + 30))
        {
          Obstacle_Flag = 2;
          forward();
          return OBSTACLE_AVOIDANCE;
        }
        else
        {
          strafe_right();
          Previous_Sonar = cm;
          return OBSTACLE_AVOIDANCE;
        }
      }
      else
      {
        if (B_L_IR.mean() > (Global_Distance - 10) && F_L_IR.mean() > (Global_Distance - 10))
        {
          Obstacle_Flag = 2;
          forward();
          return OBSTACLE_AVOIDANCE;
        }
        else
        {
          strafe_left();
          Previous_Sonar = cm;
          return OBSTACLE_AVOIDANCE;
        }
      }
    }
    else if (Obstacle_Flag == 2)
    {
      if (Previous_Sonar - cm > 40)
      {
        Obstacle_Flag = 3;
        strafe_left();
        return OBSTACLE_AVOIDANCE;
      }
      else
      {
        forward();
        return OBSTACLE_AVOIDANCE;
      }
    }
    else if (Obstacle_Flag == 3)
    {
      if (B_L_IR.mean() <= (Global_Distance) && F_L_IR.mean() <= (Global_Distance))
      {
        Obstacle_Flag = 0;
        Obstacle_case = 0;
        return PATHING;
      }
      else
      {
        strafe_left();
        return OBSTACLE_AVOIDANCE;
      }
    }
  }
  else if ( Obstacle_case == 5)
  {
    if (Obstacle_Flag == 1)
    {
      if (B_L_IR.mean() < 45 || F_L_IR.mean() < 45)
      {
        if (B_L_IR.mean() > (Global_Distance + 25) && F_L_IR.mean() > (Global_Distance + 25))
        {
          Obstacle_Flag = 2;
          forward();
          return OBSTACLE_AVOIDANCE;
        }
        else
        {
          strafe_right();
          Previous_Sonar = cm;
          return OBSTACLE_AVOIDANCE;
        }
      }
      else
      {
        if (B_L_IR.mean() > (Global_Distance - 15) && F_L_IR.mean() > (Global_Distance - 15))
        {
          Obstacle_Flag = 2;
          forward();
          return OBSTACLE_AVOIDANCE;
        }
        else
        {
          strafe_left();
          Previous_Sonar = cm;
          return OBSTACLE_AVOIDANCE;
        }
      }
    }
    else if (Obstacle_Flag == 2)
    {
      if (Previous_Sonar - cm > 40)
      {
        Obstacle_Flag = 3;
        strafe_left();
        return OBSTACLE_AVOIDANCE;
      }
      else
      {
        forward();
        return OBSTACLE_AVOIDANCE;
      }
    }
    else if (Obstacle_Flag == 3)
    {
      if (B_L_IR.mean() <= (Global_Distance) && F_L_IR.mean() <= (Global_Distance))
      {
        Obstacle_Flag = 0;
        Obstacle_case = 0;
        return PATHING;
      }
      else
      {
        strafe_left();
        return OBSTACLE_AVOIDANCE;
      }
    }
  }
}

STATE running() {
  Serial1.println("RUNNING....");
      stop(); 
  return RUNNING;
}

//I2Cfunction to set the register bit directly. 
int MPU9150_writeSensor(int addr,int data){
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission(true);

  return 1;
}

float GetSensorData(int sensorpin, int sensor_type) {
  float sensorDistance = 0;
  Voltage = analogRead(sensorpin) * (5.0 / 1023.0);
  switch (sensor_type) {
    case 1:
      sensorDistance = ((32.7038)*pow(Voltage,4)) + ((-209.1157)*pow(Voltage,3)) + ((497.2726)*pow(Voltage,2)) + ((-535.3244)*pow(Voltage,1)) + (240.2890);
      if (sensorDistance > 35) sensorDistance = 35.1;
      else if (sensorDistance < 10) sensorDistance = 9.9;
      return sensorDistance;
      break;
    case 2:
      //Calculation to convert Voltage to actual distance (-54.1019x^3 + 281.7959x^2 - 484.024x^1 +290x^0)
      sensorDistance = ((24.5779)*pow(Voltage,4)) + ((-168.0864)*pow(Voltage,3)) + ((427.7699)*pow(Voltage,2)) + ((-491.5899)*Voltage) + (235.0541);
      if (sensorDistance > 35) sensorDistance = 35.1;
      else if (sensorDistance < 10) sensorDistance = 9.9;
      return sensorDistance;
      break;
    case 3: 
       sensorDistance = ((1.863026246188062e+02)*pow(Voltage,6)) + ((-1.845169911582243e+03)*pow(Voltage,5)) + ((7.439179034295212e+03)*pow(Voltage,4)) + ((-1.562029764383641e+04)*pow(Voltage,3)) + ((1.803018750573619e+04)*pow(Voltage,2)) + ((-1.090301254801551e+04)*Voltage) + (2.773722282380848e+03);
       if (sensorDistance > 150) sensorDistance = 150.1;
       else if (sensorDistance < 20) sensorDistance = 19.9;
       return sensorDistance;
       break;
    case 4:
       sensorDistance = ((1.131231220206932e+02)*pow(Voltage,6)) + ((-1.122266913511645e+03)*pow(Voltage,5)) + ((4.511566959508631e+03)*pow(Voltage,4)) + ((-9.401835264002219e+03)*pow(Voltage,3)) + ((1.072940997535599e+04)*pow(Voltage,2)) + ((-6.414472644059954e+03)*pow(Voltage,1)) + (1.644374116252332e+03);
       if (sensorDistance > 150) sensorDistance = 150.1;
       else if (sensorDistance < 20) sensorDistance = 19.9;
       return sensorDistance;
       break;
  }
}

void echoCheck() { // If ping received, convert sonar reading to distance in cm
  if (sonar.check_timer()) cm = sonar.ping_result / US_ROUNDTRIP_CM;
}

void PrintData() { // Sensor ping cycle complete, do something with the results.
  // The following code would be replaced with your code that does something with the ping results.
  Serial1.print("Sonar: ");
  Serial1.print(Sonar_read.getMedian());
  Serial1.print("cm ");
  Serial1.print("L_M_IR: ");
  Serial1.print(L_M_IR.mean());
  Serial1.print("R_M_IR: ");
  Serial1.print(R_M_IR.mean());
  Serial1.print("B_L_IR: ");
  Serial1.print(B_L_IR.mean());
  Serial1.print("F_L_IR: ");
  Serial1.println(F_L_IR.mean());
  // display tab-separated values
//  Serial1.print(" gz;gyroZate;angle:\t");
//  Serial1.print(gz); Serial1.print("\t");
//  Serial1.print(gyroZrate); Serial1.print("\t");
//  Serial1.print(GB); Serial1.println("\t");
  }

void disable_motors()
{
  left_font_motor.detach();  // detach the servo on pin left_front to the servo object
  left_rear_motor.detach();  // detach the servo on pin left_rear to the servo object
  right_rear_motor.detach();  // detach the servo on pin right_rear to the servo object
  right_font_motor.detach();  // detach the servo on pin right_front to the servo object

  pinMode(left_front, Turning.Input);
  pinMode(left_rear, Turning.Input);
  pinMode(right_rear, Turning.Input);
  pinMode(right_front, Turning.Input);
}

void enable_motors()
{
  left_font_motor.attach(left_front);  // attaches the servo on pin left_front to the servo object
  left_rear_motor.attach(left_rear);  // attaches the servo on pin left_rear to the servo object
  right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to the servo object
  right_font_motor.attach(right_front);  // attaches the servo on pin right_front to the servo object
}

int RPM_to_speed_val(int RPM,int motor_num)
{

  if (RPM < -20) { // Dead band occurs between -20 to 20 RPM for all wheels
    if (motor_num == L_F_Motor) speed_val = 0.0006*pow(RPM,3)+0.071*pow(RPM,2)+4*RPM-17.9103; // Conversion for Left Front Motor
    else if (motor_num == L_R_Motor) speed_val = 0.0007*pow(RPM,3)+0.0996*pow(RPM,2)+5.7107*RPM+11.2634; // Conversion for Left Rear Motor
    else if (motor_num == R_F_Motor) speed_val = 0.001*pow(RPM,3)+0.1691*pow(RPM,2)+10.4426*RPM+118.611; // Conversion for Right Front Motor
    else if (motor_num == R_R_Motor) speed_val = 0.0008*pow(RPM,3)+0.126*pow(RPM,2)+7.9445*RPM+75.4544; // Conversion for Right Rear Motor
  }
  else if (RPM > 20) { 
    if (motor_num == L_F_Motor) speed_val = 0.0006*pow(RPM,3)-0.0765*pow(RPM,2)+4.6489*RPM-4.4044; // Conversion for Left Front Motor
    else if (motor_num == L_R_Motor) speed_val = 0.0008*pow(RPM,3)-0.1279*pow(RPM,2)+7.9078*RPM-71.7347; // Conversion for Left Rear Motor
    else if (motor_num == R_F_Motor) speed_val = 0.0009*pow(RPM,3)-0.1521*pow(RPM,2)+9.6613*RPM-111.7486; // Conversion for Right Front Motor
    else if (motor_num == R_R_Motor) speed_val = 0.0011*pow(RPM,3)-0.1901*pow(RPM,2)+12.1429*RPM-160.5558; // Conversion for Right Rear Motor
  }
  else speed_val = 0;
  return speed_val; 
}
void stop() //Stop
{
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

void PID_forward(double RPM_value)
{
  left_font_motor.writeMicroseconds(1500 + RPM_to_speed_val(RPM_value,L_F_Motor));
  left_rear_motor.writeMicroseconds(1500 + RPM_to_speed_val(RPM_value,L_R_Motor));
  right_rear_motor.writeMicroseconds(1500 - RPM_to_speed_val(RPM_value,R_R_Motor));
  right_font_motor.writeMicroseconds(1500 - RPM_to_speed_val(RPM_value,R_F_Motor));
}

void forward()
{
  left_font_motor.writeMicroseconds(1500 + RPM_to_speed_val(RPM,L_F_Motor));
  left_rear_motor.writeMicroseconds(1500 + RPM_to_speed_val(RPM,L_R_Motor));
  right_rear_motor.writeMicroseconds(1500 - RPM_to_speed_val(RPM,R_R_Motor));
  right_font_motor.writeMicroseconds(1500 - RPM_to_speed_val(RPM,R_F_Motor));
}

void reverse ()
{
  left_font_motor.writeMicroseconds(1500 - RPM_to_speed_val(RPM,L_F_Motor));
  left_rear_motor.writeMicroseconds(1500 - RPM_to_speed_val(RPM,L_R_Motor));
  right_rear_motor.writeMicroseconds(1500 + RPM_to_speed_val(RPM,R_R_Motor));
  right_font_motor.writeMicroseconds(1500 + RPM_to_speed_val(RPM,R_F_Motor));
}

void PID_ccw(double RPM_value)
{
  
  left_font_motor.writeMicroseconds(1500 - RPM_to_speed_val(RPM_value,L_F_Motor));
  left_rear_motor.writeMicroseconds(1500 - RPM_to_speed_val(RPM_value,L_R_Motor));
  right_rear_motor.writeMicroseconds(1500 - RPM_to_speed_val(RPM_value,R_R_Motor));
  right_font_motor.writeMicroseconds(1500 - RPM_to_speed_val(RPM_value,R_F_Motor));
}

void ccw ()
{
  left_font_motor.writeMicroseconds(1500 - RPM_to_speed_val(RPM,L_F_Motor));
  left_rear_motor.writeMicroseconds(1500 - RPM_to_speed_val(RPM,L_R_Motor));
  right_rear_motor.writeMicroseconds(1500 - RPM_to_speed_val(RPM,R_R_Motor));
  right_font_motor.writeMicroseconds(1500 - RPM_to_speed_val(RPM,R_F_Motor));
}

void PID_cw(double RPM_value)
{
  
  left_font_motor.writeMicroseconds(1500 + RPM_to_speed_val(RPM_value,L_F_Motor));
  left_rear_motor.writeMicroseconds(1500 + RPM_to_speed_val(RPM_value,L_R_Motor));
  right_rear_motor.writeMicroseconds(1500 + RPM_to_speed_val(RPM_value,R_R_Motor));
  right_font_motor.writeMicroseconds(1500 + RPM_to_speed_val(RPM_value,R_F_Motor));
}

void cw ()
{
  left_font_motor.writeMicroseconds(1500 + RPM_to_speed_val(RPM,L_F_Motor));
  left_rear_motor.writeMicroseconds(1500 + RPM_to_speed_val(RPM,L_R_Motor));
  right_rear_motor.writeMicroseconds(1500 + RPM_to_speed_val(RPM,R_R_Motor));
  right_font_motor.writeMicroseconds(1500 + RPM_to_speed_val(RPM,R_F_Motor));
}

void strafe_left ()
{
  left_font_motor.writeMicroseconds(1500 - RPM_to_speed_val(RPM,L_F_Motor));
  left_rear_motor.writeMicroseconds(1500 + RPM_to_speed_val(RPM,L_R_Motor));
  right_rear_motor.writeMicroseconds(1500 + RPM_to_speed_val(RPM,R_R_Motor));
  right_font_motor.writeMicroseconds(1500 - RPM_to_speed_val(RPM,R_F_Motor));
}

void PID_strafe (double RPM_value)
{
  left_font_motor.writeMicroseconds(1500 + RPM_to_speed_val(RPM_value,L_F_Motor));
  left_rear_motor.writeMicroseconds(1500 - RPM_to_speed_val(RPM_value,L_R_Motor));
  right_rear_motor.writeMicroseconds(1500 - RPM_to_speed_val(RPM_value,R_R_Motor));
  right_font_motor.writeMicroseconds(1500 + RPM_to_speed_val(RPM_value,R_F_Motor));
}

void strafe_right ()
{
  left_font_motor.writeMicroseconds(1500 + RPM_to_speed_val(RPM,L_F_Motor));
  left_rear_motor.writeMicroseconds(1500 - RPM_to_speed_val(RPM,L_R_Motor));
  right_rear_motor.writeMicroseconds(1500 - RPM_to_speed_val(RPM,R_R_Motor));
  right_font_motor.writeMicroseconds(1500 + RPM_to_speed_val(RPM,R_F_Motor));
}

void Check_Obstacle ()
{
if (Obstacle_Flag == 0 ) 
  {
    if (L_M_IR.mean() < 25)
    {
      if (R_M_IR.mean() < 25)
      {
        Obstacle_case=0;
      }
      else {
        if (cm < 15&&cm>1)
        {
          Obstacle_Flag = 1;
          Obstacle_case = 4;
        }
        else {
          Obstacle_Flag = 1;
          Obstacle_case = 2;
          //Serial1.println("Left case!!!");
        }
      }
    }
    else
    {
      if (R_M_IR.mean() < 25)
      {
        if (cm < 15&&cm>1)
        {
          Obstacle_Flag = 1;
          Obstacle_case = 5;
        }
        else {
          Obstacle_Flag = 1;
          Obstacle_case = 3;
        }

      }
      else {
        if (cm<15 && cm>1)
        {
          Obstacle_Flag = 1;
          Obstacle_case = 1;
        }
      }
    }
  }
}

