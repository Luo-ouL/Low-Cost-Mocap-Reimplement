#include <esp_now.h>
#include <WiFi.h>
#include <PID_v1_bc.h>
#include <ArduinoJson.h>
#include <sbus.h>

#define MAX_VEL 100
#define Z_GAIN 0.6
#define ROTOR_RADIUS 0.0225
unsigned long lastLoopTime = micros();
unsigned long lastSbusSend = micros();
unsigned long lastPing;
unsigned long timeArmed;
float loopFrequency = 2000.0;
float sbusFrequency = 50.0;
bool armed = false;

int Pitch_Trim = 0, Roll_Trim = 0, Throttle_Trim = -50, Yaw_Trim = 0;
double groundEffectCoef = 400, groundEffectOffset = 0; // the groundEffectCoef is useless in this code

bfs::SbusTx sbus_tx(&Serial1, 33, 32, true, false);
bfs::SbusData data;

double xPosSetpoint, xPos;
double yPosSetpoint, yPos;
double zPosSetpoint, zPos;
double yawPosSetpoint, yawPos, yawPosOutput;

double xVelSetpoint, xVel, xVelOutput;
double yVelSetpoint, yVel, yVelOutput;
double zVelSetpoint, zVel, zVelOutput;

double xyPosKp = 1, xyPosKi = 0.2, xyPosKd = 0.02;
double zPosKp = 1.5, zPosKi = 0.1, zPosKd = 0.02;
double yawPosKp = 0.3, yawPosKi = 0.1, yawPosKd = 0.05;

double xyVelKp = 0.5, xyVelKi = 0.2, xyVelKd = 0.1;
double zVelKp = 0.3, zVelKi = 0.06, zVelKd = 0.06;

PID xPosPID(&xPos, &xVelSetpoint, &xPosSetpoint, xyPosKp, xyPosKi, xyPosKd, DIRECT);
PID yPosPID(&yPos, &yVelSetpoint, &yPosSetpoint, xyPosKp, xyPosKi, xyPosKd, DIRECT);
PID zPosPID(&zPos, &zVelSetpoint, &zPosSetpoint, zPosKp, zPosKi, zPosKd, DIRECT);
//PID yawPosPID(&yawPos, &yawPosOutput, &yawPosSetpoint, yawPosKp, yawPosKi, yawPosKd, DIRECT);
PID yawPosPID(&yawPos, &yawPosOutput, &yawPosSetpoint, yawPosKp, yawPosKi, yawPosKd, P_ON_E, DIRECT, YAW_ON);

PID xVelPID(&xVel, &xVelOutput, &xVelSetpoint, xyVelKp, xyVelKi, xyVelKd, DIRECT);
PID yVelPID(&yVel, &yVelOutput, &yVelSetpoint, xyVelKp, xyVelKi, xyVelKd, DIRECT);
PID zVelPID(&zVel, &zVelOutput, &zVelSetpoint, zVelKp, zVelKi, zVelKd, DIRECT);

JsonDocument json;
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  DeserializationError err = deserializeJson(json, (char*)incomingData);
  if (err) {
    Serial.println("failed to parse json");
    return;
  }

  if (!json["armed"].isNull()) {
    if (json["armed"]) {
      armed = true;
      timeArmed = millis();
    }
    else {
      armed = false;
    }
  } else if (!json["setpoint"].isNull()) {
    xPosSetpoint = json["setpoint"][0];
    yPosSetpoint = json["setpoint"][1];
    zPosSetpoint = json["setpoint"][2];
    yawPosSetpoint = json["setpoint"][3];
  } else if (!json["pid"].isNull()) {
    xPosPID.SetTunings(json["pid"][0], json["pid"][1], json["pid"][2]);
    yPosPID.SetTunings(json["pid"][0], json["pid"][1], json["pid"][2]);
    zPosPID.SetTunings(json["pid"][3], json["pid"][4], json["pid"][5]);
    yawPosPID.SetTunings(json["pid"][6], json["pid"][7], json["pid"][8]);
    xVelPID.SetTunings(json["pid"][9], json["pid"][10], json["pid"][11]);
    yVelPID.SetTunings(json["pid"][9], json["pid"][10], json["pid"][11]);
    zVelPID.SetTunings(json["pid"][12], json["pid"][13], json["pid"][14]);
  } else if (!json["trim"].isNull()) {
    Pitch_Trim = json["trim"][0];
    Roll_Trim = json["trim"][1];
    Throttle_Trim = json["trim"][2];
    Yaw_Trim = json["trim"][3];
  } else if (!json["groundEff"].isNull()) {
    groundEffectCoef = json["groundEff"][0];
    groundEffectOffset = json["groundEff"][1];
  } else if (!json["pos"].isNull() && !json["vel"].isNull()) {
    xPos = json["pos"][0];
    yPos = json["pos"][1];
    zPos = json["pos"][2];
    yawPos = json["pos"][3];
    xVel = json["vel"][0];
    yVel = json["vel"][1];
    zVel = json["vel"][2];
  }
  lastPing = micros();
}

void resetPid(PID &pid, double min, double max) {
  pid.SetOutputLimits(0.0, 1.0);
  pid.SetOutputLimits(-1.0, 0.0);
  pid.SetOutputLimits(min, max);
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Initialize ESP-NOW protocol
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initializaion failed.");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("ESP-NOW receiver launch, waiting for data.");

  // Initialize PID controllers
  xPosPID.SetMode(AUTOMATIC);
  yPosPID.SetMode(AUTOMATIC);
  zPosPID.SetMode(AUTOMATIC);
  yawPosPID.SetMode(AUTOMATIC);
  xVelPID.SetMode(AUTOMATIC);
  yVelPID.SetMode(AUTOMATIC);
  zVelPID.SetMode(AUTOMATIC);

  xPosPID.SetSampleTime(0.5);
  yPosPID.SetSampleTime(0.5);
  zPosPID.SetSampleTime(0.5);
  yawPosPID.SetSampleTime(0.5);
  xVelPID.SetSampleTime(0.5);
  yVelPID.SetSampleTime(0.5);
  zVelPID.SetSampleTime(0.5);

  xPosPID.SetOutputLimits(-MAX_VEL, MAX_VEL);
  yPosPID.SetOutputLimits(-MAX_VEL, MAX_VEL);
  zPosPID.SetOutputLimits(-MAX_VEL, MAX_VEL);
  yawPosPID.SetOutputLimits(-1, 1);
  xVelPID.SetOutputLimits(-1, 1);
  yVelPID.SetOutputLimits(-1, 1);
  zVelPID.SetOutputLimits(-1, 1);

  // SBUS test
  sbus_tx.Begin();

  data.failsafe = false;
  data.ch17 = true;
  data.ch18 = true;
  data.lost_frame = false;

  for (int i = 500; i > 172; i --) {
    for (int j = 0; j < 16; j ++) {
      data.ch[j] = i;
    }
    Serial.println(i);
    sbus_tx.data(data);
    sbus_tx.Write();
  }

  // Acquire start time
  lastPing = micros();
  lastLoopTime = micros();
  lastSbusSend = micros();
}

void loop() {
  while (micros() - lastLoopTime < 1e6 / loopFrequency) {yield();}
  lastLoopTime = micros();

  if (micros() - lastPing > 2e6) {
    armed = false;
  }

  if (armed) {
    data.ch[4] = 1800;
    xPosPID.Compute();
    yPosPID.Compute();
    zPosPID.Compute();
    yawPosPID.Compute();

    xVelPID.Compute();
    yVelPID.Compute();
    zVelPID.Compute();
  } else {
    data.ch[4] = 172;
    resetPid(xPosPID, -MAX_VEL, MAX_VEL);
    resetPid(yPosPID, -MAX_VEL, MAX_VEL);
    resetPid(zPosPID, -MAX_VEL, MAX_VEL);
    resetPid(yawPosPID, -1, 1);
    resetPid(xVelPID, -1, 1);
    resetPid(yVelPID, -1, 1);
    resetPid(zVelPID, -1, 1);
  }

  int Pitch_PWM = 992 + (xVelOutput*cos(yawPos)+yVelOutput*sin(yawPos)) * 811 + Pitch_Trim;  // Pitch
  int Roll_PWM = 992 + (xVelOutput*sin(yawPos)-yVelOutput*cos(yawPos)) * 811 + Roll_Trim;  // Roll
  int Throttle_PWM = 992 + (Z_GAIN * zVelOutput * 811) + Throttle_Trim; // Throttle
  int Yaw_PWM = 992 - (yawPosOutput * 811) + Yaw_Trim;  // Yaw

  Throttle_PWM = armed && millis() - timeArmed > 100 ? Throttle_PWM : 172;

  data.ch[0] = Roll_PWM;
  data.ch[1] = Pitch_PWM;
  data.ch[2] = Throttle_PWM;
  data.ch[3] = Yaw_PWM;

  if (micros() - lastSbusSend > 1e6 / sbusFrequency) {
    lastSbusSend = micros();
    sbus_tx.data(data);
    sbus_tx.Write();
  }
}