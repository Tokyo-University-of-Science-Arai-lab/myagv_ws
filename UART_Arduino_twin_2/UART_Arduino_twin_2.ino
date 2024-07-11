long val1;
long val2;

//motor1(左)
#define ENCODER1_A_PIN 2
#define ENCODER1_B_PIN 4
#define INA1 12
#define INB1 13
#define PWM1 11
//motor2(右)
#define ENCODER2_A_PIN 3
#define ENCODER2_B_PIN 5
#define INA2 7
#define INB2 8
#define PWM2 9

String cmds[2];

//-------------------------------------------------------
//入力されたモータ制御値の変換
//概要：dataの文字を区切って配列に入れる
//引数：data=区切りたい文字列 delimiter=区切り文字 dst=区切った文字を入れる配列
//戻り値：配列の数
//-------------------------------------------------------
int split(String data, char delimiter, String* dst) {
  int index = 0;
  int datalength = data.length();

  for (int i = 0; i < datalength; i++) {
    char tmp = data.charAt(i);
    if (tmp == delimiter) {
      index++;
      if (index >= 2) return -1;
    } else {
      dst[index] += tmp;
    }
  }
  return (index + 1);
}

//-------------------------------------------------------
//エンコーダ関連
volatile long encoderValue1 = 0;
volatile long encoderValue2 = 0;
unsigned long previousMillis = 0;
int ppr_val = 11;
float gear_ratio = 43.5;

//関数定義
void encoder1_func() {
  int A1_state = digitalRead(ENCODER1_A_PIN);
  int B1_state = digitalRead(ENCODER1_B_PIN);
  if (A1_state == B1_state) {
    encoderValue1++; // counter clockwise
  } else {
    encoderValue1--; // clockwise
  }
}

void encoder2_func() {
  int A2_state = digitalRead(ENCODER2_A_PIN);
  int B2_state = digitalRead(ENCODER2_B_PIN);
  if (A2_state == B2_state) {
    encoderValue2++; // counter clockwise
  } else {
    encoderValue2--; // clockwise
  }
}

void setup() {
  Serial.begin(9600);
  //ピンの設定
  pinMode(INA1, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INB2, OUTPUT);
  pinMode(PWM2, OUTPUT);

  //エンコーダ関連
  pinMode(ENCODER1_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER1_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_A_PIN), encoder1_func, RISING);
  pinMode(ENCODER2_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER2_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A_PIN), encoder2_func, RISING);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    int index = split(data, ',', cmds);
    if (index == 2) {
      val2= cmds[0].toInt(); //変更
      val1= cmds[1].toInt(); //変更
      cmds[0] = ""; // クリア
      cmds[1] = ""; // クリア
    }
  }

  analogWrite(PWM1, abs(val1));
  analogWrite(PWM2, abs(val2));
  digitalWrite(INA1, val1 >= 0 ? HIGH : LOW);
  digitalWrite(INB1, val1 < 0 ? HIGH : LOW);
  digitalWrite(INA2, val2 >= 0 ? LOW : HIGH); //HIGH:LOW
  digitalWrite(INB2, val2 < 0 ? LOW : HIGH);  //HIGH:LOW

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > 1000) {
    previousMillis = currentMillis;

    noInterrupts();
    float rpm_val_1 = (float)(encoderValue1 * 60 / ppr_val / gear_ratio);
    float rpm_val_2 = (float)(-1 * encoderValue2 * 60 / ppr_val / gear_ratio);
    encoderValue1 = 0;
    encoderValue2 = 0;
    interrupts();

    Serial.print(rpm_val_1);
    Serial.print(",");
    Serial.println(rpm_val_2);
  }
}
