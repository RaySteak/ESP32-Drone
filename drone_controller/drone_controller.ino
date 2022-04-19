#define INCLUDE_vTaskSuspend 1
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <WiFi.h>
#include "soc/rtc_wdt.h"

// motor pins and chanels
#define FRONT_LEFT_PIN 25 //CW
#define FRONT_RIGHT_PIN 26 //CCW
#define REAR_LEFT_PIN 27 //CW
#define REAR_RIGHT_PIN 14 //CCW

#define FRONT_LEFT 0 //CW, blue wire
#define FRONT_RIGHT 1 //CCW, red wire
#define REAR_LEFT 2 //CW, black wire
#define REAR_RIGHT 3 //CCW, orange wire

// frequenmcy for pwm (must be between 50 and 60)
#define PWM_FREQ 55

// loop time in microseconds
#define LOOP_TIME_us 5000

// mpu interrupt pin
#define MPU_INTERRUPT 19

// indexes in the vector of the 3 axis of rotation
#define YAW 0
#define PITCH 1
#define ROLL 2

// indexes for lines of pid matrix
#define PID_P 0
#define PID_I 1
#define PID_D 2

// control scaling constants
#define SPIN_SCALE 10.f

// WiFi ssid and password
const char* ssid     = "ESP32-Drone";
const char* password = "incercatisamahackati";
WiFiServer server(42069);
WiFiClient client;

// current command values
float joystick_throttle;
float spin;
float pitch_angle;
float pitch_strength;

// calibration and gyro loops
hw_timer_t *calibration_timer = NULL, *gyro_timer = NULL;
SemaphoreHandle_t calibration_sem = NULL, gyro_sem = NULL;
TaskHandle_t calibration_handle, gyro_handle;

void IRAM_ATTR drone_calibrate_on_timer()
{
  xSemaphoreGive(calibration_sem);
}

void IRAM_ATTR sample_gyro_on_interrupt()
{
  xSemaphoreGive(gyro_sem);
}

// everything gyro-related
MPU6050 accelgyro;
const int16_t acc_off[3] = {-5909, 5955, 8661};
const int16_t gyr_off[3] = {-27, -25, -15};
Quaternion quat;
VectorFloat grav;

float angles[3] = {0.f, 0.f, 0.f};
float desired_angles[3] = {0.f, 0.f, 0.f};
float angle_errors[3];
float angle_errors_sum[3] = {0.f, 0.f, 0.f};
float angle_errors_prev[3] = {0.f, 0.f, 0.f};

const float kpid [3][3] = {{0.1f, 0.1f, 0.1f},  // P: yaw, pitch, roll
                           {0.1f, 0.1f, 0.1f},  // I: yaw, pitch, roll
                           {0.1f, 0.1f, 0.1f}}; // D: yaw, pitch, roll

int FIFO_packet_size;
uint8_t *FIFO_buffer;

// util functions

inline uint16_t throttle_to_pwm(double ms)
{
  return uint16_t((ms + 1.0) / 1000.0 * double(PWM_FREQ) * 65535.0);
}

void setup()
{
  Wire.begin();
    
  Serial.begin(38400);

  //Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  //Serial.println("Testing device connections...");
  //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // use the code below to change accel/gyro offset values
  
  /*Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
  Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
  Serial.print("\n");*/

  // FLIGHT SETUP CODE
  ledcSetup(FRONT_LEFT, 55, 16); // canal 0, frecventa 55, 16 biti rezolutie
  ledcSetup(FRONT_RIGHT, 55, 16); // canal 1, frecventa 55, 16 biti rezolutie
  ledcSetup(REAR_LEFT, 55, 16); // canal 2, frecventa 55, 16 biti rezolutie
  ledcSetup(REAR_RIGHT, 55, 16); // canal 3, frecventa 55, 16 biti rezolutie
    
  ledcAttachPin(FRONT_LEFT_PIN, FRONT_LEFT);
  ledcAttachPin(FRONT_RIGHT_PIN, FRONT_RIGHT);
  ledcAttachPin(REAR_LEFT_PIN, REAR_LEFT);
  ledcAttachPin(REAR_RIGHT_PIN, REAR_RIGHT);
    
  uint16_t startup_freq = throttle_to_pwm(0.0);
  uint16_t zero_freq = throttle_to_pwm(0.0);
  ledcWrite(FRONT_LEFT, startup_freq);
  ledcWrite(FRONT_RIGHT, startup_freq);
  ledcWrite(REAR_LEFT, startup_freq);
  ledcWrite(REAR_RIGHT, startup_freq);
  /*delay(3000);
  ledcWrite(FRONT_LEFT, throttle_to_pwm(0.27 , 55));
  delay(2000);
  ledcWrite(FRONT_LEFT, throttle_to_pwm(0.00 , 55));
  ledcWrite(FRONT_RIGHT, throttle_to_pwm(0.27 , 55));
  delay(2000);
  ledcWrite(FRONT_RIGHT, throttle_to_pwm(0.00 , 55));
  ledcWrite(REAR_LEFT, throttle_to_pwm(0.27 , 55));
  delay(2000);
  ledcWrite(REAR_LEFT,throttle_to_pwm(0.00 , 55));
  ledcWrite(REAR_RIGHT, throttle_to_pwm(0.27 , 55));
  delay(2000);
  ledcWrite(REAR_RIGHT, throttle_to_pwm(0.00 , 55));*/
  ledcWrite(FRONT_LEFT, zero_freq);
  ledcWrite(FRONT_RIGHT, zero_freq);
  ledcWrite(REAR_LEFT, zero_freq);
  ledcWrite(REAR_RIGHT, zero_freq);

  // Wi-Fi AP
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  server.begin();

  // semaphores
  calibration_sem = xSemaphoreCreateBinary();
  gyro_sem = xSemaphoreCreateBinary();

  // create calibration task
  //xTaskCreatePinnedToCore(network, "network", 5000, NULL, 0, &network_handle, 0);
  xTaskCreate(drone_calibrate, "drone_calibrate", 5000, NULL, 0, &calibration_handle);
  xTaskCreate(sample_gyro, "gyro_sample", 5000, NULL, 0, &gyro_handle);

  // 8kHz looptime would be ideal, to match modern speed controllers
  // and it can't be achieved because the MPU6050 acceleration sensor sampling rate
  // is 1kHz, even though gyro sampling rate is 8kHz. But it gets worse, doing our
  // own fusion of gyro and acceleration samples would be a pain so we will use
  // the sensor's DMP which only has a sample rate of 200hZ but provides us
  // with already processed data
  accelgyro.dmpInitialize();
  
  Serial.println("Setting internal sensor offsets...");
  accelgyro.setXAccelOffset(acc_off[0]);
  accelgyro.setYAccelOffset(acc_off[1]);
  accelgyro.setZAccelOffset(acc_off[2]);
  accelgyro.setXGyroOffset(gyr_off[0]);
  accelgyro.setYGyroOffset(gyr_off[1]);
  accelgyro.setZGyroOffset(gyr_off[2]);
  
  accelgyro.PrintActiveOffsets();
  
  accelgyro.setDMPEnabled(true);
  FIFO_packet_size = accelgyro.dmpGetFIFOPacketSize();
  FIFO_buffer = (uint8_t *)malloc(FIFO_packet_size * sizeof(uint8_t));
  byte stat = accelgyro.getIntStatus();
  
  // We want 200Hz looptime, so a loop every 5ms 
  // 80MHz base timer frequency, divided by 80 gives 1MHz freq, so each increment will be 1us
  calibration_timer = timerBegin(0, 80, true); // timer 0, 80 prescaler, count up
  timerAttachInterrupt(calibration_timer, &drone_calibrate_on_timer, true);
  timerAlarmWrite(calibration_timer, LOOP_TIME_us, true); // 5ms looptime
  timerAlarmEnable(calibration_timer);

  attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT), sample_gyro_on_interrupt, RISING);
  /*gyro_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(gyro_timer, &sample_gyro_on_timer, true);
  timerAlarmWrite(gyro_timer, SAMPLE_RATE_us, true);
  timerAlarmEnable(gyro_timer);*/
}

void drone_calibrate(void *param)
{
  int i = 0;
  
  float angle_errors_delta[3];
  float actual_angles[3];
  float pid[3];
  while(true)
  {
    xSemaphoreTake(calibration_sem, portMAX_DELAY);
    
    angle_errors[YAW] = desired_angles[YAW] - angles[YAW];
    angle_errors[PITCH] = desired_angles[PITCH] - angles[PITCH];
    angle_errors[ROLL] = desired_angles[ROLL] - angles[ROLL];

    angle_errors_sum[YAW] += angle_errors[YAW];
    angle_errors_sum[PITCH] += angle_errors[PITCH];
    angle_errors_sum[ROLL] += angle_errors[ROLL];

    angle_errors_delta[YAW] = angle_errors[YAW] - angle_errors_prev[YAW];
    angle_errors_delta[PITCH] = angle_errors[PITCH] - angle_errors_prev[PITCH];
    angle_errors_delta[ROLL] = angle_errors[ROLL] - angle_errors_prev[ROLL];

    pid[YAW] = kpid[YAW][PID_P] * angle_errors[YAW] + kpid[YAW][PID_I] * angle_errors_sum[YAW] + kpid[YAW][PID_D] * angle_errors_delta[YAW];
    pid[PITCH] = kpid[PITCH][PID_P] * angle_errors[PITCH] + kpid[PITCH][PID_I] * angle_errors_sum[PITCH] + kpid[PITCH][PID_D] * angle_errors_delta[PITCH];
    pid[ROLL] = kpid[ROLL][PID_P] * angle_errors[ROLL] + kpid[ROLL][PID_I] * angle_errors_sum[ROLL] + kpid[ROLL][PID_D] * angle_errors_delta[ROLL];

    angle_errors_prev[YAW] = angle_errors[YAW];
    angle_errors_prev[PITCH] = angle_errors[PITCH];
    angle_errors_prev[ROLL] = angle_errors[ROLL];

    float throttle_front_left = joystick_throttle - pid[YAW] + pid[PITCH] + pid[ROLL];
    float throttle_front_right = joystick_throttle + pid[YAW] + pid[PITCH] - pid[ROLL];
    float throttle_rear_left = joystick_throttle + pid[YAW] - pid[PITCH] + pid[ROLL];
    float throttle_rear_right = joystick_throttle - pid[YAW] - pid[PITCH] - pid[ROLL];

    throttle_front_left = max(0.f, min(throttle_front_left, 1.f));
    throttle_front_right = max(0.f, min(throttle_front_right, 1.f));
    throttle_rear_left = max(0.f, min(throttle_rear_left, 1.f));
    throttle_rear_right = max(0.f, min(throttle_rear_right, 1.f));

    if(i == 100)
    {
    Serial.println(String(throttle_front_left) + " " + String(throttle_front_right) + " " + String(throttle_rear_left) + " " + String(throttle_rear_right));
    i = 0;
    }
    i++;
    
    /*ledcWrite(FRONT_LEFT, throttle_to_pwm(throttle_front_left));
    ledcWrite(FRONT_RIGHT, throttle_to_pwm(throttle_front_right));
    ledcWrite(REAR_LEFT, throttle_to_pwm(throttle_rear_left));
    ledcWrite(REAR_RIGHT, throttle_to_pwm(throttle_rear_right));*/

    ledcWrite(FRONT_LEFT, throttle_to_pwm(joystick_throttle));
    ledcWrite(FRONT_RIGHT, throttle_to_pwm(joystick_throttle));
    ledcWrite(REAR_LEFT, throttle_to_pwm(joystick_throttle));
    ledcWrite(REAR_RIGHT, throttle_to_pwm(joystick_throttle));
  }
}

void sample_gyro(void *param)
{
  int i = 0;
  float aux_angles[3];
  bool first_measurement = true;
  float reference_angles[3];
  float measured_angles[3];
  
  while(true)
  {
    xSemaphoreTake(gyro_sem, portMAX_DELAY);
    byte stat = accelgyro.getIntStatus();
    
    while(accelgyro.getFIFOCount() < FIFO_packet_size)
      ;
    //Serial.println("FIFOUL E " + String(accelgyro.getFIFOCount()));
    accelgyro.getFIFOBytes(FIFO_buffer, FIFO_packet_size);
    accelgyro.dmpGetQuaternion(&quat, FIFO_buffer);
    accelgyro.dmpGetGravity(&grav, &quat);
    accelgyro.dmpGetYawPitchRoll(aux_angles, &quat, &grav);
    measured_angles[YAW] = aux_angles[YAW];
    measured_angles[PITCH] = aux_angles[ROLL];
    measured_angles[ROLL] = -aux_angles[PITCH];
    if(first_measurement)
    {
      reference_angles[YAW] = measured_angles[YAW];
      reference_angles[PITCH] = measured_angles[PITCH];
      reference_angles[ROLL] = measured_angles[ROLL];
      first_measurement = false;
    }
    angles[YAW] = measured_angles[YAW] - reference_angles[YAW];
    angles[PITCH] = measured_angles[PITCH] - reference_angles[PITCH];
    angles[ROLL] = measured_angles[ROLL] - reference_angles[ROLL];
    /*accelgyro.getAcceleration(&ax, &ay, &az);
    accelgyro.getRotation(&gx, &gy, &gz);
    Serial.print(transf_acc(ax)); Serial.print("\t");
    Serial.print(transf_acc(ay)); Serial.print("\t");
    Serial.print(transf_acc(az)); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);*/
    i++;
    if(i == 100)
    {
    Serial.print("Unghiurile: ");
    Serial.print(String(angles[YAW]) + " ");
    Serial.print(String(angles[PITCH]) + " ");
    Serial.println(String(angles[ROLL]));
    i = 0;
    }
  }
}

bool command_complete = false;
String command = "";

void loop()
{
  if(!client.connected())
  {
    client = server.available();
    command_complete = false;
    command = "";
  }
  else
  {
    if(client.available())
    {
      char c = client.read();
      //Serial.write(c);
      command += c;
      if(c == '\n')
        command_complete = true;
    }
    if(command_complete)
    {
      //Serial.println(command);
      int pos = command.indexOf(": ") + 2;
      String nr = command.substring(pos);
      int param = nr.toInt();

      if(command.startsWith("Throttle"))
      {
        joystick_throttle = param / 100.0;
      }
      else if(command.startsWith("Spin"))
      {
        spin = param / 50.0;
        desired_angles[YAW] += spin / SPIN_SCALE;
      }
      else
      {
        int pos2 = command.indexOf(", ") + 2;
        String nr2 = nr.substring(pos2);
        int param2 = nr2.toInt();
        pitch_angle = param;
        pitch_strength = param2;
      }
      command = "";
      command_complete = false;
    }
  }
}
