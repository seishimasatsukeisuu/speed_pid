#include <Arduino.h>
#include "SpeedPID.h"

const uint8_t DIR_PIN = 22;
const uint8_t PWM_PIN = 23;

const int8_t Motor_Channel = 0;   // pwmチャンネル番号(espのledc)
const int8_t Sign_Correction = 1; // 符号補正、モーターの取り付け方向によって+-を変える

const uint8_t ENCODER_A_PIN = 25;
const uint8_t ENCODER_B_PIN = 26;

volatile int32_t count = 0;
volatile uint8_t prev_state = 0;

double rpm_filtered = 0;

const double target_rpm = 60;  // 目標rpm値
const int16_t PWM_LIMIT = 255; // pwmの最大値

// PID制御器(Kp(比例), Ki(積分), Kd(微分), pwm出力制限)
SpeedPID speed_pid_1(1.5, 3., 0., -PWM_LIMIT, PWM_LIMIT);

const double ENC_COUNTS_PER_REV = 4096.0 * 2.; // エンコーダ1回転あたりのカウント数
const double GEAR_RATIO = 1.;                  // ギア比(補正係数)実際の回転数に変換するため

void Motor(int8_t dirPin, int8_t pwmPin, int sign, int pwm_signed)
{
  bool is_dir = HIGH;
  int duty = sign * pwm_signed; // 符号補正
  // 回転方向決定
  if (duty > 0)
  {
    is_dir = HIGH;
  }
  else
  {
    is_dir = LOW;
  }
  digitalWrite(dirPin, is_dir);
  ledcWrite(pwmPin, abs(duty));
}

unsigned long last = micros(); // 前回の時刻を記録

void IRAM_ATTR Encodercounter()
{
  uint8_t a = digitalRead(ENCODER_A_PIN);
  uint8_t b = digitalRead(ENCODER_B_PIN);

  uint8_t curr_state = (a << 1) | b;

  uint8_t transition = (prev_state << 2) | curr_state; //+1-1を決める

  // 正しい遷移のみカウント
  switch (transition)
  {
  // 正回転
  case 0b0001:
  case 0b0111:
  case 0b1110:
  case 0b1000:
    count++;
    break;

  // 逆回転
  case 0b0010:
  case 0b0100:
  case 0b1101:
  case 0b1011:
    count--;
    break;

  default:
    // ノイズなど（無視）
    break;
  }
  prev_state = curr_state;
}

void setup()
{
  Serial.begin(115200);

  pinMode(ENCODER_A_PIN, INPUT);
  pinMode(ENCODER_B_PIN, INPUT);

  // 初期状態を読む
  uint8_t a = digitalRead(ENCODER_A_PIN);
  uint8_t b = digitalRead(ENCODER_B_PIN);
  prev_state = (a << 1) | b;

  // 両方の変化で割り込み
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), Encodercounter, CHANGE); // rising = 立ち上がりを検出
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), Encodercounter, CHANGE);
  pinMode(DIR_PIN, OUTPUT);
  ledcSetup(Motor_Channel, 12800, 8);    // 周波数：12.8kHz, 分解能：8bit
  ledcAttachPin(PWM_PIN, Motor_Channel); // ピンにpwm割り当て

  last = micros(); // 時刻リセット
}

const long CONTROL_CYCLE = 20000; // 制御周期：20000μs = 20ms
long prev_count_1 = 0;            // 前回のエンコーダー値

void loop()
{
  // サンプリング制御(一定周期でだけ実行)
  unsigned long now = micros();
  if (now - last < CONTROL_CYCLE)
    return;

  // dt = 0.005秒
  last += CONTROL_CYCLE;
  double dt = CONTROL_CYCLE * 1e-6;

  // 現在のカウント取得
  noInterrupts();
  long c1 = count;
  interrupts();
  // 増分(速度に対応)
  const long dc1 = c1 - prev_count_1;
  // 更新
  prev_count_1 = c1;

  // 回転数計算
  double rpm = dc1 * GEAR_RATIO / ENC_COUNTS_PER_REV / dt * 60;

  // フィルタ
  double alpha = 0.2; // 0〜1（小さいほどなめらか）
  rpm_filtered = alpha * rpm + (1 - alpha) * rpm_filtered;

  // PID制御
  double output_1 = speed_pid_1.update(target_rpm, rpm_filtered, dt);

  Motor(DIR_PIN, Motor_Channel, Sign_Correction, (int)(lround(output_1)));

  // デバック出力
  Serial.printf("rpm:%f filtered:%f target:%f c1:%d pwm:%f\n", rpm, rpm_filtered, target_rpm, c1, output_1);
}
