    //2127
    //27546
    #include <Adafruit_ICM20X.h>
    #include <Adafruit_ICM20948.h>
    #include <Adafruit_Sensor.h>
    #include <Wire.h>
    #include <VL53L0X.h>
    #include <ArduinoQueue.h>

    //_______TOF PARAMETERS___________________
    // address we will assign if dual sensor is present
    VL53L0X sensor;
    VL53L0X sensor2;
    VL53L0X sensor3;

    const int sensor1_int = 7; //green
    const int sensor2_int = 8; //yellow
    const int sensor3_int = 4; //purple

    const int trig = 2; //purple
    const int echo = 3; //orange

    //2x Blue A4, SDA
    //2x Orange A5 SCL

    const int enable_left = 5; //white
    const int enable_right = 6; //green

    const int LeftMotorForward = 9; //grey
    const int LeftMotorBackward = 10; //white

    const int RightMotorForward = 11; //Yellow
    const int RightMotorBackward = 12; //green

    double front_distance = 0;


    //_______IMU PARAMETERS__________________________
    Adafruit_ICM20948 icm;
    uint16_t measurement_delay_us = 65535; // Delay between measurements for testing
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t mag;
    sensors_event_t temp;

    double min_x, max_x, mid_x;
    double min_y, max_y, mid_y;
    double min_z, max_z, mid_z;

    //Magnetometer Variables
    const int period = 5000; 
    bool mag_init = false;

    int startMillis;

    //Forward move params
        float forward_intergral = 0;
    float forward_error = 999;
    float forward_kp = 30;
    float forward_ki = 0.06f;
    float forward_kd = 10;

    double drive_rot_kp = 0.4;
    double drive_rot_kd = 0.2; //7;

    double drive_side_kp = 0.12;
    double drive_side_kd = 0.12; //-4;

    float allowable_angle_error = 8;
    int allowable_front_error = 20;
    int allowable_side_error = 30;
    double pitch = 0;
    double dist_between_lidars = 140;
    double dist_from_wall = 0;

    double delta_side_error = 0;
    double delta_angle_error = 0;
    double prev_side_error = 0;
    double prev_angle_error = 0;

    double pit_time_max = 300;
    double pit_time = 0;
    bool pit_start = false;

    double completion_delay = 0;
    bool completion_delay_started = false;

    //Distance turn values
    int minimum_side_dist = 999;
    int minimum_front_dist = 999; //For now while testing

    //NANO 
    const int ledPin = LED_BUILTIN;

    //______CONTROL INPUT PARAMETERS_______________________
    int target_yaw = 10; //Same as box heading merge variable into one
    double angle_error;
    double angle = 0;
    double rotation_integral = 0; 

    //______CONTROL OUTPUT PARAMETERS______________________
    double steering;
    double forward;

    double LeftMotorSpeed;
    double RightMotorSpeed;

    double side_error = 0;

    double rear_lidar_distance;
    double forward_lidar_distance;

    double Prev_forward_lidar_distance = 0;
    double Prev_rear_lidar_distance = 0;

    double Delta_forward_lidar_distance = 0;
    double Delta_rear_lidar_distance = 0;
    //                                                    PIT    Gravel(T)   2x Gravel  2x sand     Sand(T)  Sand   Flat    Final
    //                                  0       1     2     3       4            5           6           7        8      9
    float front_distance_drive [11] = {285,   285,  285,   600,   550,         550,        500,        550,   1000,   1000,    1000,}; 
    float side_distance [11] =        {125,   125,  125,   125,   440,         440,        440,        440,    600,    600,     600,};
    float delay_list [11] =           {0,     0,    0,     0,     0,           0,          2000,       5000,      0,      0,      0,};
    //true = align, false = turn
    int align_or_turn [11]         =  {0,      0,   0,     0,     0,           0,           0,          0,        0,      0,      0,};

    int navigation_index = 0;
    bool done_forward = false;
    bool done_turn = false;
    bool done_align = false;

    //Align
    bool close_forward = false;

    bool one_wheel_turn = false;

    enum State{
        none,
        driving,
        turning,
        align,
        done
    };

    State state = driving;
      
    void setup() {

      Serial.begin(115200);
      Serial.println("Starting");
      

      Wire.begin();
      Wire.setClock(400000); // use 400 kHz I2C

      //Interface setup
      pinMode(ledPin, OUTPUT);

      pinMode(trig, OUTPUT);
      pinMode(echo, INPUT);

      //_________________MOTOR SETUP_________________
      setup_motors();

      //setup_imu();
      //________________SETUP TOF____________________
      setID();

      startMillis = millis();
      Serial.println("Startup Completed.");

      state = driving;
      //debug_imu();

      //navigation_index = 3;
    }

    void loop() {

      //_________ULTRASONIC LOOP FUNCTIONS________
      //i2c_scanner();
      read_dual_sensors();
      //log_tof_sensors();
      log_data();
      //run_ultrasonic();

      //_____________MOTOR LOOP FUNCTIONS_________
      //motor_test_forward();
      //motor_test();

      //____________CONTROL LOOP FUNCTIONS________
      navigate_compass();
      //forward = 255;
      //steering = 130;
      motor_control();
    }

    //+ steering left
    //- steering right

    //State 1: driving
    //State 2: turning
    //State 3: align
    //State 4: done

    void log_data(){
      Serial.println("");
      Serial.print(" Steering: ");  Serial.print(steering);
      Serial.print(" Forward: ");  Serial.print(forward);
      Serial.print(" Lead Range: ");  Serial.print(forward_lidar_distance);
      Serial.print(" Rear Range: ");  Serial.print(rear_lidar_distance);
      Serial.print(" State: ");  Serial.print(state);
      Serial.print(" Index: ");  Serial.print(navigation_index);
      Serial.print(" Angle Error: ");  Serial.print(angle_error);
      Serial.print(" Side Error: ");  Serial.print(side_error);
      Serial.print(" Forward dist: ");  Serial.print(front_distance);
      Serial.print(" Side diff: ");  Serial.print(forward_lidar_distance-rear_lidar_distance);
      Serial.print(" Target Front: ");  Serial.print(front_distance_drive[navigation_index]);
    }

    void navigate_compass(){ 
      log_data();
      
      /*if(done_align)
        delay(4000);
      done_align = exit_pit_turn();
      return;*/

      //testing function

      if(navigation_index == 9){
        state = done;
        return;
      }

      if(driving){
        if(done_forward){
          if(align_or_turn[navigation_index]){
            delay(200);
            state = align;
            done_forward = false;
          }
          else{
            delay(200);
            state = turning;
            done_forward = false;
          }
        }
        else{
          done_forward = forward_drive();
        }
      }

      //log_tof_sensors();
      //rotation_drive();
      //motor_test_forward();
      //forward_drive(); 

      //Check rotation error calculation
      if (state == turning) {
        if(done_turn){
          delay(200);
          state = driving;
          done_turn = false;
          navigation_index++;
        }
        else{
          done_turn = timed_turn();
        }
      }

      if (state == align) {
        if(done_align){
          delay(200);
          if(close_forward)
            state = turning;
          else
            state = driving;
          done_align = false;
        }
        else{
          done_align = exit_pit_turn();
        }
      }


    }

    void steering_control(double amount){
      steering = amount;
      motor_control();
    }

    void foward_control(double amount){
      forward = amount;
      motor_control();
    }

    void motor_control(){
      RightMotorSpeed = steering + forward;
      LeftMotorSpeed = -steering + forward;

      double max_motor_speed = 255;

      RightMotorSpeed = constrain(RightMotorSpeed, -max_motor_speed, max_motor_speed);
      LeftMotorSpeed = constrain(LeftMotorSpeed, -max_motor_speed, max_motor_speed);

      if(!one_wheel_turn){
        analogWrite(enable_left, abs(RightMotorSpeed)); //ENA pin
      }
      else{
        analogWrite(enable_left, abs(0)); 
      }
      analogWrite(enable_right, abs(LeftMotorSpeed)); //ENB pin

      if(RightMotorSpeed <= 0){
        digitalWrite(RightMotorForward, HIGH);
        digitalWrite(RightMotorBackward, LOW);
      }
      if(RightMotorSpeed >= 0){
        digitalWrite(RightMotorForward, LOW);
        digitalWrite(RightMotorBackward, HIGH);
      }

      if(LeftMotorSpeed <= 0){
        digitalWrite(LeftMotorForward, HIGH);
        digitalWrite(LeftMotorBackward, LOW);
      }
      if(LeftMotorSpeed >= 0){
        digitalWrite(LeftMotorForward, LOW);
        digitalWrite(LeftMotorBackward, HIGH);
      }
    }

    bool pulse_started = false;
    bool pulse_on = false;
    double pulse_time = 0;
    double motor_pulse_time = 50;
    //close forward or far foward
    bool exit_pit_turn() {
      angle_error = forward_lidar_distance-rear_lidar_distance;
      /*side_error = (rear_lidar_distance+forward_lidar_distance)/2 - side_distance[navigation_index];
      if (abs(forward_lidar_distance-rear_lidar_distance) < 40 && abs(side_error) < 110) {
        steering = 0;
        return true;
      }
      steering = -155;
      return false;*/


      //one_wheel_turn = true;
      if(!pulse_started){
        pulse_time = millis();
        pulse_started = true; 
        pulse_on != pulse_on;
      }
      if(millis() - pulse_time > motor_pulse_time){
        pulse_started = false;
      }
      else{
        if(pulse_on){
          steering = angle_error * drive_rot_kp;
          steering += steering/abs(steering) * 105;
        }
      }

      if(abs(angle_error) < 11){
        if(front_distance < front_distance_drive[navigation_index]){
          close_forward = true;
        }
        else{
            close_forward = false;
        }
        steering = 0;
        one_wheel_turn = false;
        return true;
      }     
      return false;
    }



    bool started = false;
    double time = 0;
    double timed_turn_time = 250; //450;
    bool timed_turn(){
      if(started){
        delay(timed_turn_time);
        steering = 0;
        started = false;
        return true;
      }
      if(!started){
          time = millis();
          started = true;

          steering = -175;
          if(navigation_index == 7){
            steering = -225;
          }
      }
      /*
      
      else{
        steering = 0;
        started = false;
        return true;
      }*/
      return false;
    }

    // bill
    // assuming we have distance from sensors
    /*
      d1 = lidar1
      d2 = lidar2
      l1 = dist from d1 to d2
    */


    //left is - steering 
    //Balance heading heading constant with maximising forward motion and keeping wall distances within parameters.
    bool forward_drive(){
      angle = atan2(dist_between_lidars, forward_lidar_distance-rear_lidar_distance);
      angle_error = forward_lidar_distance-rear_lidar_distance;
      delta_angle_error = angle_error - prev_angle_error;
      prev_angle_error = angle_error;

      //TODO use the ultrasonic
      //forward_error = forward_lidar_distance - front_distance_drive[navigation_index];
      
      double cur_side_dist = side_distance[navigation_index];
      side_error = (rear_lidar_distance+forward_lidar_distance)/2 - side_distance[navigation_index];
      delta_side_error = side_error - prev_side_error;
      prev_side_error = side_error;

      double forward_mod = 2.3; //2.3
      forward = 110*forward_mod;
      steering = angle_error * drive_rot_kp + delta_angle_error * drive_rot_kd + side_error * drive_side_kp + delta_side_error * drive_side_kd;
      steering *= forward_mod;

      if(front_distance < front_distance_drive[navigation_index]+150){ 
        forward = 110;
      }

      //In pit
      if(navigation_index == 3){
        //for testing
        // if(pit_start){
        //   forward = 0;
        //   return false;
        // }
        if(!pit_start && front_distance < 400){
          pit_time = millis();
          pit_start = true; 
        }
        if(millis() - pit_time > pit_time_max && abs(front_distance - front_distance_drive[navigation_index]) < 30 && pit_start){
          steering = 0;
          forward = 0;
          return true;
        }
      }
      else{
        if(!completion_delay_started){
          completion_delay = millis();
          completion_delay_started = true; 
        }
        if(millis() - completion_delay > delay_list[navigation_index] && completion_delay_started){
          if(front_distance < front_distance_drive[navigation_index]){ //&& angle_error < 15){
            steering = 0;
            forward = 0;
            completion_delay_started = false;
            return true;
          }
        }
        else{
          if(front_distance < front_distance_drive[navigation_index]){ 
            steering = 0;
            forward = 0;
            return true;
          }
        }
        //reached final position
      }
      
      if(steering > 3 && forward == 0)
          steering += steering/abs(steering) * 120;
      return false;
    }

    bool align_using_tof(){
      angle_error = forward_lidar_distance-rear_lidar_distance;
      steering = angle_error * drive_rot_kp;
      steering += steering/abs(steering) * 120;
      if(angle_error < 3)
        return true;
      return false;
    }

    void motor_test_forward(){
      
      //drive_left_motor(0);
      delay(2000000);
    }

    void setup_drive(){
      steering = 0;
      //rotation_intergral = 0;
      state = driving;
    }

    double turn_increment_timer = 3; //Hardcoded timer so the robot turns past the wall ideally
    double turn_time;
    void setup_turn(){
      turn_time = millis();
      state = turning;
      forward = 0;
    }



    void setup_motors(){
      // put your setup code here, to run once:
      pinMode(LeftMotorForward, OUTPUT);
      pinMode(LeftMotorBackward, OUTPUT);
      pinMode(RightMotorForward, OUTPUT);
      pinMode(RightMotorBackward, OUTPUT);

      pinMode(enable_left, OUTPUT); 
      pinMode(enable_right, OUTPUT);
    }

    void log_tof_sensors(){
      Serial.print("Lead Distance: ");
      Serial.println(forward_lidar_distance);

      Serial.print("Rear Distance: ");
      Serial.println(rear_lidar_distance);

      Serial.print("Front Distance: ");
      Serial.println(front_distance);
    }

    void setID() {
      pinMode(sensor1_int, OUTPUT);
      pinMode(sensor2_int, OUTPUT);
      pinMode(sensor3_int, OUTPUT);
      digitalWrite(sensor1_int, LOW);
      digitalWrite(sensor2_int, LOW);
      digitalWrite(sensor3_int, LOW);

      delay(500);
      Wire.begin();


      Serial.begin (115200);

      //SENSOR
      pinMode(sensor1_int, INPUT);
      delay(150);
      Serial.println("00");
      sensor.init(true);
      Serial.println("01");
      delay(100);
      sensor.setAddress((uint8_t)22);
      Serial.println("02");

      //SENSOR 2
      pinMode(sensor2_int, INPUT);
      delay(150);
      sensor2.init(true);
      Serial.println("03");
      delay(100);
      sensor2.setAddress((uint8_t)25);
      Serial.println("04");

      // SENSOR 3
      pinMode(sensor3_int, INPUT);
      delay(150);
      sensor3.init(true);
      Serial.println("05");
      delay(100);
      sensor3.setAddress((uint8_t)27);
      Serial.println("06");

      sensor.setTimeout(500);
      sensor2.setTimeout(500);
      sensor3.setTimeout(500);
    }
    
    void i2c_scanner()
    {
      byte error, address;
      int nDevices;
    
      Serial.println("Scanning...");
    
      nDevices = 0;
      for(address = 1; address < 127; address++ )
      {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
    
        if (error == 0)
        {
          Serial.print("I2C device found at address 0x");
          if (address<16)
            Serial.print("0");
          Serial.print(address,HEX);
          Serial.println("  !");
    
          nDevices++;
        }
        else if (error==4)
        {
          Serial.print("Unknown error at address 0x");
          if (address<16)
            Serial.print("0");
          Serial.println(address,HEX);
        }
        else{
           /*Serial.print("Error at address 0x");
          if (address<16)
            Serial.print("0");
          Serial.println(address,HEX);*/
        }
      }
      if (nDevices == 0)
        Serial.println("No I2C devices found\n");
      else
        Serial.println("done\n");
    
      delay(500);           // wait 5 seconds for next scan
    }

    void read_dual_sensors() {
      //CHECK DISTANCES
      if(navigation_index <= 3)
        forward_lidar_distance = (sensor2.readRangeSingleMillimeters()-85);
      else if(navigation_index <= 6)
        forward_lidar_distance = (sensor2.readRangeSingleMillimeters()-65);
      else if(navigation_index <= 9)
       forward_lidar_distance = (sensor2.readRangeSingleMillimeters()-15);
      //if(rear_lidar_distance > 2000)
      //  rear_lidar_distance = 0;

      delay(10);

      if(navigation_index <= 3)
        rear_lidar_distance = (sensor.readRangeSingleMillimeters()-60);
      else if(navigation_index <= 6)
        rear_lidar_distance = (sensor.readRangeSingleMillimeters()-85);
      else if(navigation_index <= 9)
        rear_lidar_distance = (sensor.readRangeSingleMillimeters()-140);

      delay(10);

      if(navigation_index <= 3)
        front_distance = (sensor3.readRangeSingleMillimeters());
      else if(navigation_index <= 6)
        front_distance = (sensor3.readRangeSingleMillimeters());
      else if(navigation_index <= 9)
        front_distance = (sensor3.readRangeSingleMillimeters());

      //if(forward_lidar_distance > 2000)
      //  forward_lidar_distance = 0;
      //delay(300);//can change to a lower time like 100
    }


    //Used for the initial setup of the robot
    bool both_tof_covered(){
      int min_range = 50;
      return forward_lidar_distance < min_range && rear_lidar_distance < min_range;
    }
