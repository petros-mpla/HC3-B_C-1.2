
//      @mpla
//      copyright 12024
//      CERN OHL-2

//  CERN has developed this licence to promote collaboration among
//  hardware designers and to provide a legal tool which supports the
//  freedom to use, study, modify, share and distribute hardware designs
//  and products based on those designs. Version 2 of the CERN Open
//  Hardware Licence comes in three variants: CERN-OHL-P (permissive); and
//  two reciprocal licences: CERN-OHL-W (weakly reciprocal) and this
//  licence, CERN-OHL-S (strongly reciprocal).


const float G = 6.67430e-11;             // (m^3 kg^-1 s^-2)
const float M_earth = 5.972e24;          // (kg)
const float R_earth = 6371000;           // (m)
const float rho = 1.225;                 // (kg/m^3)
const float drag_coeff = 2.2;
const float area = 0.01;                 // (m^2)
const float mass_satellite = 1.3;        // (kg)
const float communicationPower = 0.5;    // (LoRa)
const float sensorPower = 0.05;          // (50mW)
const float controlSystemPower = 0.2;    // (reaction wheels)
const float microcontrollerPower = 0.03; // (30mA)
const float MAGNETIC_THRESHOLD = 1.5; 
const float GYROSCOPE_THRESHOLD = 5.0; 
const float ACCELEROMETER_THRESHOLD = 0.5;

float altitude = 110000; //  LEO (m)
float velocity = 7800;   // (m/s)
float pos_x = 0, pos_y = R_earth + altitude; // Χ0
float vel_x = velocity, vel_y = 0;
float dt = 1; // (1s)
float magneticFieldX, magneticFieldY, magneticFieldZ;
float gyroX, gyroY, gyroZ;
float accelX, accelY, accelZ;

unsigned long communicationTime = 1800; 
unsigned long sensorTime = 300;         
unsigned long controlTime = 600;        
unsigned long microcontrollerTime = 3600; 

const float max_acceleration = 10.0; // m/s²
const float max_angular_velocity = 20.0; // rad/s
const float max_magnetic_field = 50.0;   // μT


float calculateEnergy(float power, unsigned long time) {
  return power * time; 
}

float atmospheric_density(float altitude) {
  if (altitude > 100000) return 0; 
  return rho * exp(-altitude / 8500.0); 
}


float gravitational_force(float altitude) {
  float r = R_earth + altitude;
  return G * M_earth * mass_satellite / (r * r);
}


float drag_force(float velocity, float altitude) {
  float density = atmospheric_density(altitude);
  return 0.5 * density * drag_coeff * area * velocity * velocity;
}


float accelerometer_reading(float acc_x, float acc_y) {
  return sqrt(acc_x * acc_x + acc_y * acc_y);
}


float gyroscope_reading(float angular_velocity) {
  return angular_velocity;
}


float magnetometer_reading(float altitude) {
  return 40.0 * exp(-altitude / 20000.0); 
}

void check_sensor_limits(float acceleration, float angular_velocity, float magnetic_field) {
  if (acceleration > max_acceleration) {
    Serial.println("a out of order");
  }
  if (angular_velocity > max_angular_velocity) {
    Serial.println("Ω-u out of order");
  }
  if (magnetic_field > max_magnetic_field) {
    Serial.println("T out of order");
  }
}

void update_position_and_velocity() {
  
  float r = sqrt(pos_x * pos_x + pos_y * pos_y);

  float F_gravity = gravitational_force(r - R_earth);

  float F_drag = drag_force(velocity, r - R_earth);

 
  float acc_x = -F_gravity * (pos_x / r) / mass_satellite;
  float acc_y = -F_gravity * (pos_y / r) / mass_satellite;

  
  acc_x -= (F_drag / mass_satellite) * (vel_x / velocity);
  acc_y -= (F_drag / mass_satellite) * (vel_y / velocity);

  
  vel_x += acc_x * dt;
  vel_y += acc_y * dt;

 
  pos_x += vel_x * dt;
  pos_y += vel_y * dt;

  
  velocity = sqrt(vel_x * vel_x + vel_y * vel_y);

  
  float acceleration = accelerometer_reading(acc_x, acc_y);
  float angular_velocity = gyroscope_reading(velocity / r);
  float magnetic_field = magnetometer_reading(r - R_earth);

  check_sensor_limits(acceleration, angular_velocity, magnetic_field);

  Serial.print("H: ");
  Serial.print(r - R_earth);
  Serial.print(" m, U: ");
  Serial.print(velocity);
  Serial.print(" m/s, a: ");
  Serial.print(acceleration);
  Serial.print(" m/s², T: ");
  Serial.print(magnetic_field);
  Serial.println(" μT");
}

void setup() {
  Serial.begin(9600);

  
  float energyCommunication = calculateEnergy(communicationPower, communicationTime);
  float energySensor = calculateEnergy(sensorPower, sensorTime);
  float energyControl = calculateEnergy(controlSystemPower, controlTime);
  float energyMicrocontroller = calculateEnergy(microcontrollerPower, microcontrollerTime);

 
  Serial.print("W-J:\n");
  Serial.print("Communication: ");
  Serial.print(energyCommunication);
  Serial.println(" J");

  Serial.print("Sensors: ");
  Serial.print(energySensor);
  Serial.println(" J");

  Serial.print("GSN: ");
  Serial.print(energyControl);
  Serial.println(" J");

  Serial.print("Micro Controler: ");
  Serial.print(energyMicrocontroller);
  Serial.println(" J");


  float totalEnergy = energyCommunication + energySensor + energyControl + energyMicrocontroller;
  Serial.print("F-J: ");
  Serial.print(totalEnergy);
  Serial.println(" J");
}

void loop() {
  
  magneticFieldX = analogRead(A0);  
  magneticFieldY = analogRead(A1);  
  magneticFieldZ = analogRead(A2);  

  gyroX = analogRead(A3);  
  gyroY = analogRead(A4);  
  gyroZ = analogRead(A5);  

  accelX = analogRead(A6); 
  accelY = analogRead(A7); 
  accelZ = analogRead(A8); 

  
  if (abs(magneticFieldX) > MAGNETIC_THRESHOLD || abs(magneticFieldY) > MAGNETIC_THRESHOLD || abs(magneticFieldZ) > MAGNETIC_THRESHOLD) {
    Serial.println("T error");
    
  }

  if (abs(gyroX) > GYROSCOPE_THRESHOLD || abs(gyroY) > GYROSCOPE_THRESHOLD || abs(gyroZ) > GYROSCOPE_THRESHOLD) {
    Serial.println("R error");
    
  }

  if (abs(accelX) > ACCELEROMETER_THRESHOLD || abs(accelY) > ACCELEROMETER_THRESHOLD || abs(accelZ) > ACCELEROMETER_THRESHOLD) {
    Serial.println("a error");
    
  }

  delay(1000); 
}