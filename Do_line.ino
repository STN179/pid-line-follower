// ================= PIN =================
int sensorPin[5] = {36, 39, 34, 35, 32};

#define ENA 25
#define IN1 26
#define IN2 27
#define ENB 33
#define IN3 14
#define IN4 12

// ================= PID =================
float Kp = 28;
float Ki = 0;
float Kd = 10;

float P, I, D;
int error = 0;
int last_error = 0;
float PID_value = 0;

int baseSpeed = 80;

// ============ PWM HANDLE ============
int pwmA;
int pwmB;

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 5; i++) pinMode(sensorPin[i], INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pwmA = ledcAttach(ENA, 1000, 8);
  pwmB = ledcAttach(ENB, 1000, 8);
}

// ================= STOP =================
void stopMotors() {
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ================= READ SENSOR =================
void readSensors() {
  int s[5];
  for (int i = 0; i < 5; i++) {
    s[i] = digitalRead(sensorPin[i]);   // 1 = line đen
  }

  if      (s[0]==0 && s[1]==0 && s[2]==0 && s[3]==0 && s[4]==1) error = 4;
  else if (s[0]==0 && s[1]==0 && s[2]==0 && s[3]==1 && s[4]==1) error = 3;
  else if (s[0]==0 && s[1]==0 && s[2]==0 && s[3]==1 && s[4]==0) error = 2;
  else if (s[0]==0 && s[1]==0 && s[2]==1 && s[3]==1 && s[4]==0) error = 1;
  else if (s[0]==0 && s[1]==0 && s[2]==1 && s[3]==0 && s[4]==0) error = 0;
  else if (s[0]==0 && s[1]==1 && s[2]==1 && s[3]==0 && s[4]==0) error = -1;
  else if (s[0]==0 && s[1]==1 && s[2]==0 && s[3]==0 && s[4]==0) error = -2;
  else if (s[0]==1 && s[1]==1 && s[2]==0 && s[3]==0 && s[4]==0) error = -3;
  else if (s[0]==1 && s[1]==0 && s[2]==0 && s[3]==0 && s[4]==0) error = -4;
}

// ================= PID =================
void calculatePID() {
  P = error;
  I += error;
  D = error - last_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  PID_value = constrain(PID_value, -80, 80);

  last_error = error;
}

// ================= MOTOR =================
void motorControl() {

  // ===== GIẢM TỐC KHI CUA GẮT =====
  int dynamicSpeed = baseSpeed;
  if (abs(error) >= 3) {
    dynamicSpeed = 60;   // tốc độ chậm khi cua gắt
  }

  int leftSpeed  = dynamicSpeed + PID_value;
  int rightSpeed = dynamicSpeed - PID_value;

  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  ledcWrite(ENA, leftSpeed);
  ledcWrite(ENB, rightSpeed);

  // chạy tiến
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}


// ================= LOOP =================
void loop() {
  int sum = 0;
  for (int i = 0; i < 5; i++) {
    sum += digitalRead(sensorPin[i]);
  }

  // Nếu nhấc xe (11111) → dừng motor
  if (sum == 5) {
    stopMotors();
    return;
  }

  readSensors();
  calculatePID();
  motorControl();
}
