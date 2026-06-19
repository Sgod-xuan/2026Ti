#include <MsTimer2.h>
#include <Wire.h>            // MPU6050 I2C 通信

/*==========================================================
 * 巡线小车 —— 完整控制程序
 * 硬件：LF04 四路红外传感器 + 双电机编码器闭环
 *       HC-SR04 超声波测距 + MPU6050 陀螺仪转向
 *
 * 状态流转：
 *   WAIT(0) ──按下D4按钮──> RUN(1) ──四路全黑──>
 *   MEASURE1(3) ──测距完──> TURN(4) ──转90°完──>
 *   REVERSE(6) ──四路全黑──> MEASURE2(5) ──测距完──> STOP(2)
 *
 * LF04 传感器输出逻辑：
 *   检测到黑线 → LOW  (0)
 *   检测到白底 → HIGH (1)
 *
 * 接线：
 *   MPU6050  SDA→A4   SCL→A5   VCC→3.3V  GND→GND
 *   HC-SR04  Trig→D12  Echo→D13
 *   HC-05    TXD→D0   RXD→D1   VCC→5V    GND→GND（上传代码时拔掉）
 *   按钮      一端→D4  另一端→GND（INPUT_PULLUP，按下低电平触发）
 *==========================================================*/

/*----------------------------------------------------------
 * 一、引脚定义
 *----------------------------------------------------------*/

// LF04 传感器（从左到右：S1 S2 S3 S4）
#define S1_PIN  A0
#define S2_PIN  A1
#define S3_PIN  A2
#define S4_PIN  A3

// 左电机驱动
const uint8_t L_AIN1 = 9;
const uint8_t L_AIN2 = 6;

// 右电机驱动
const uint8_t R_AIN1 = 10;
const uint8_t R_AIN2 = 5;

// 左电机编码器
#define L_ENCODER_A 7
#define L_ENCODER_B 2         // 外部中断 0

// 右电机编码器
#define R_ENCODER_A 8
#define R_ENCODER_B 3         // 外部中断 1

// HC-SR04
#define TRIG_PIN    12
#define ECHO_PIN    13

// 启动按钮
#define BTN_PIN     4    // 低电平触发，内部上拉

// MPU6050 I2C 地址
#define MPU6050_ADDR 0x68

/*----------------------------------------------------------
 * 二、巡线 PD 参数
 *----------------------------------------------------------*/
float BASE_SPEED = 10;
float LINE_KP    = 2.0;
float LINE_KD    = 5.0;

/*----------------------------------------------------------
 * 三、电机 PI 参数
 *----------------------------------------------------------*/
float L_KP = 50, L_KI = 10;
float R_KP = 50, R_KI = 10;

/*----------------------------------------------------------
 * 四、死区补偿
 *----------------------------------------------------------*/
const int PWM_RESTRICT   = 255;
const int L_FWD_DEADZONE = 30;
const int L_REV_DEADZONE = 38;
const int R_FWD_DEADZONE = 29;
const int R_REV_DEADZONE = 23;

/*----------------------------------------------------------
 * 五、转向参数
 *----------------------------------------------------------*/
float TURN_SPEED   = 8.0;    // 原地转向时单轮速度（脉冲数/10ms）
float TARGET_ANGLE = 90.0;   // 目标转向角度（度）
float ANGLE_KP     = 0.15;   // 转向比例系数，接近目标时减速
float ANGLE_THRESH = 0.8;    // 到达目标角度的误差容限（度）

/*----------------------------------------------------------
 * 六、超声波参数
 *----------------------------------------------------------*/
#define SOUND_SPEED  0.017   // 0.034cm/us / 2（往返）

/*----------------------------------------------------------
 * 七、状态机定义
 *----------------------------------------------------------*/
#define STATE_WAIT      0    // 等待启动：按下 D4 按钮
#define STATE_RUN       1    // 巡线中
#define STATE_STOP      2    // 全部任务完成，停车
#define STATE_MEASURE1  3    // 第一次测距
#define STATE_TURN      4    // 左转 90°
#define STATE_MEASURE2  5    // 第二次测距
#define STATE_REVERSE   6    // 测距前倒车

int carState = STATE_WAIT;

/*----------------------------------------------------------
 * 八、运行时变量
 *----------------------------------------------------------*/

// 传感器
bool s1, s2, s3, s4;
int  lastError = 0;

// 电机目标转速
volatile float TargetL = 0;
volatile float TargetR = 0;

// 编码器计数
volatile int CountL = 0;
volatile int CountR = 0;

// 电机速度与 PWM
int VelocityL = 0, VelocityR = 0;
int valueL    = 0, valueR    = 0;

// PI 内部状态
float L_PWM = 0, L_Bias = 0, L_LastBias = 0;
float R_PWM = 0, R_Bias = 0, R_LastBias = 0;

// MPU6050
float gyroZ_offset = 0;      // 陀螺仪 Z 轴零偏（上电校准）
float currentAngle = 0;      // 当前累积偏航角（度）
float turnStartAngle = 0;    // 开始转向时的角度基准
unsigned long lastGyroTime = 0;

// 测距结果
float dist1 = 0;
float dist2 = 0;


/*==========================================================
 * setup
 *==========================================================*/
void setup()
{
  Serial.begin(9600);
  Serial.println("/**** 巡线小车启动 ****/");

  // 传感器引脚
  pinMode(S1_PIN, INPUT);
  pinMode(S2_PIN, INPUT);
  pinMode(S3_PIN, INPUT);
  pinMode(S4_PIN, INPUT);

  // 编码器引脚
  pinMode(L_ENCODER_A, INPUT);
  pinMode(L_ENCODER_B, INPUT);
  pinMode(R_ENCODER_A, INPUT);
  pinMode(R_ENCODER_B, INPUT);

  // 电机驱动引脚
  pinMode(L_AIN1, OUTPUT);
  pinMode(L_AIN2, OUTPUT);
  pinMode(R_AIN1, OUTPUT);
  pinMode(R_AIN2, OUTPUT);

  // 启动按钮引脚（内部上拉，按下为LOW）
  pinMode(BTN_PIN, INPUT_PULLUP);

  // 超声波引脚
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Serial 同时作为蓝牙通信和调试输出
  // 接线：HC-05 TXD→D0(RX)，RXD→D1(TX)，上传代码时拔掉蓝牙模块

  // MPU6050 初始化
  Wire.begin();
  mpuInit();
  mpuCalibrate();   // 静止校准零偏，约1秒

  lastGyroTime = micros();

  // 编码器外部中断
  attachInterrupt(0, READ_ENCODER_L, CHANGE);
  attachInterrupt(1, READ_ENCODER_R, CHANGE);

  // 10ms 定时器：电机 PI 控制
  MsTimer2::set(10, motorControl);
  MsTimer2::start();
}

/*==========================================================
 * loop：状态机主控
 *==========================================================*/
void loop()
{
  // 每次loop都更新陀螺仪角度积分
  updateAngle();

  // 读取红外传感器
  s1 = !digitalRead(S1_PIN);
  s2 = !digitalRead(S2_PIN);
  s3 = !digitalRead(S3_PIN);
  s4 = !digitalRead(S4_PIN);

  bool allBlack = s1 && s2 && s3 && s4;

  // ---------- STATE_WAIT：等待按钮触发 ----------
  if (carState == STATE_WAIT)
  {
    TargetL = 0;
    TargetR = 0;

    if (digitalRead(BTN_PIN) == LOW)   // 按钮按下
    {
      Serial.println("按钮触发，开始巡线！");

      // 等待松手，防止抖动或长按导致重复触发
      while (digitalRead(BTN_PIN) == LOW) delay(10);
      delay(50);   // 松手后再消抖

      TargetL = BASE_SPEED;
      TargetR = BASE_SPEED;

      delay(600);
      carState = STATE_RUN;
    }

    delay(10);
    return;
  }

  // ---------- STATE_RUN：巡线中 ----------
  if (carState == STATE_RUN)
  {
    if (allBlack)
    {
      // 到达终点十字，停车，进入第一次测距
      TargetL = 0;
      TargetR = 0;
      Serial.println("检测到终点十字，停车！");
      delay(300);   // 等待小车完全停稳
      carState = STATE_MEASURE1;
      delay(10);
      return;
    }

    // PD 巡线计算
    int error = calcError();
    float correction = LINE_KP * error + LINE_KD * (error - lastError);
    lastError = error;

    float leftTarget  = constrain(BASE_SPEED + correction, -BASE_SPEED*2, BASE_SPEED*2);
    float rightTarget = constrain(BASE_SPEED - correction, -BASE_SPEED*2, BASE_SPEED*2);

    TargetL = leftTarget;
    TargetR = rightTarget;

    delay(10);
    return;
  }

  // ---------- STATE_MEASURE1：第一次测距 ----------
  if (carState == STATE_MEASURE1)
  {
    TargetL = 0;
    TargetR = 0;
    dist1 = getDistance();

    // 串口调试输出
    Serial.print("第一次测距（正前方）：");
    if (dist1 < 0) Serial.println("超出范围");
    else { Serial.print(dist1 , 1); Serial.println(" cm"); }

    // 蓝牙发送
    Serial.print("DIST1:");
    if (dist1 < 0) Serial.println("OUT_OF_RANGE");
    else { Serial.print(dist1 , 1); Serial.println("cm"); }

    // 记录当前角度作为转向基准
    turnStartAngle = currentAngle;
    carState = STATE_TURN;
    delay(200);
    return;
  }


  // ---------- STATE_TURN：右转 90° ----------
  if (carState == STATE_TURN)
  {
    float turned = currentAngle - turnStartAngle;   // 已转过的角度

    if (turned <= -(TARGET_ANGLE - ANGLE_THRESH))
    {
      // 到达目标角度，停车
      TargetL = 0;
      TargetR = 0;
      Serial.println("转向完成！");
      delay(300);   // 等待完全停稳
      carState = STATE_REVERSE;
    }
    else
    {
      // 比例减速：越接近目标角度转得越慢，防止过冲
      float remaining = TARGET_ANGLE - abs(turned);
      float speed = constrain(remaining * ANGLE_KP, 2.0, TURN_SPEED);

      // 左转：左轮后退，右轮前进（原地转）
      TargetL =  speed;
      TargetR = -speed;
    }

    delay(10);
    return;
  }


  if (carState == STATE_REVERSE)
  {
    if (allBlack)
    {
      // 到达终点十字，停车，进入第二次测距
      TargetL = 0;
      TargetR = 0;
      Serial.println("检测到终点十字，停车！");
      delay(300);   // 等待小车完全停稳
      carState = STATE_MEASURE2;
      delay(10);
      return;
    }
    else
    {
      TargetL = -TURN_SPEED;
      TargetR = -TURN_SPEED;
    }
    delay(10);
    return;

  }
  // ---------- STATE_MEASURE2：第二次测距 ----------
  if (carState == STATE_MEASURE2)
  {
    TargetL = 0;
    TargetR = 0;
    dist2 = getDistance();

    // 串口调试输出
    Serial.print("第二次测距（左转90°后）：");
    if (dist2 < 0) Serial.println("超出范围");
    else { Serial.print(dist2, 1); Serial.println(" cm"); }

    // 蓝牙发送第二次测距
    Serial.print("DIST2:");
    if (dist2 < 0) Serial.println("OUT_OF_RANGE");
    else { Serial.print(dist2, 1); Serial.println("cm"); }

    // 蓝牙发送汇总
    Serial.println("===== TASK DONE =====");
    Serial.print("FORWARD:");
    if (dist1 < 0) Serial.println("OUT_OF_RANGE");
    else { Serial.print(dist1, 1); Serial.println("cm"); }
    Serial.print("LEFT:");
    if (dist2 < 0) Serial.println("OUT_OF_RANGE");
    else { Serial.print(dist2, 1); Serial.println("cm"); }
    Serial.println("=====================");

    carState = STATE_STOP;
    delay(10);
    return;
  }

  // ---------- STATE_STOP：全部完成，停车 ----------
  if (carState == STATE_STOP)
  {
    TargetL = 0;
    TargetR = 0;
    delay(10);
    return;
  }
}

/*==========================================================
 * 计算偏差
 *==========================================================*/
int calcError()
{
  if (!s1 && !s2 && !s3 && !s4) return lastError;
  if (s1 && !s2 && !s3 && !s4)  return -2;
  if (s2 && !s3)                 return -1;
  if (s2 && s3)                  return  0;
  if (!s2 && s3)                 return  1;
  if (s4 && !s1 && !s2 && !s3)  return  2;
  return lastError;
}

/*==========================================================
 * HC-SR04 测距
 *==========================================================*/
float getDistance()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  if (duration == 0) return -1;
  return duration * SOUND_SPEED;
}

/*==========================================================
 * MPU6050 初始化
 *==========================================================*/
void mpuInit()
{
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);   // PWR_MGMT_1
  Wire.write(0x00);   // 唤醒
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);   // GYRO_CONFIG
  Wire.write(0x00);   // ±250°/s 量程
  Wire.endTransmission();
}

/*==========================================================
 * 陀螺仪零偏校准（静止放置，采样100次取平均）
 *==========================================================*/
void mpuCalibrate()
{
  Serial.println("陀螺仪校准中，请保持静止...");
  long sum = 0;
  for (int i = 0; i < 100; i++)
  {
    sum += readGyroZ_raw();
    delay(10);
  }
  gyroZ_offset = sum / 100.0;
  Serial.print("校准完成，零偏："); Serial.println(gyroZ_offset);
}

/*==========================================================
 * 读取陀螺仪 Z 轴原始值
 *==========================================================*/
int16_t readGyroZ_raw()
{
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x47);   // GYRO_ZOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 2, true);
  int16_t raw = (Wire.read() << 8) | Wire.read();
  return raw;
}

/*==========================================================
 * 更新偏航角（积分陀螺仪角速度）
 * 在 loop() 每帧调用，时间间隔越短越精确
 *==========================================================*/
void updateAngle()
{
  unsigned long now = micros();
  float dt = (now - lastGyroTime) / 1000000.0;   // 转换为秒
  lastGyroTime = now;

  int16_t raw = readGyroZ_raw();
  // ±250°/s 量程对应灵敏度 131 LSB/(°/s)
  float gyroZ_dps = (raw - gyroZ_offset) / 131.0;

  // 积分：角速度 × 时间 = 角度增量
  // 过滤微小抖动，避免静止时漂移积累
  if (abs(gyroZ_dps) > 0.5)
    currentAngle += gyroZ_dps * dt;
}

/*==========================================================
 * 编码器中断服务函数
 *==========================================================*/
void READ_ENCODER_L()
{
  if (digitalRead(L_ENCODER_A) == LOW)
    CountL += (digitalRead(L_ENCODER_B) == LOW) ? -1 : 1;
  else
    CountL += (digitalRead(L_ENCODER_B) == LOW) ?  1 : -1;
}

void READ_ENCODER_R()
{
  if (digitalRead(R_ENCODER_A) == LOW)
    CountR += (digitalRead(R_ENCODER_B) == LOW) ?  1 : -1;
  else
    CountR += (digitalRead(R_ENCODER_B) == LOW) ? -1 :  1;
}

/*==========================================================
 * 10ms 定时器中断：双电机 PI 调节
 *==========================================================*/
void motorControl()
{
  VelocityL = CountL; CountL = 0;
  VelocityR = CountR; CountR = 0;

  valueL = Incremental_PI(VelocityL, TargetL, L_KP, L_KI, L_PWM, L_Bias, L_LastBias);
  valueR = Incremental_PI(VelocityR, TargetR, R_KP, R_KI, R_PWM, R_Bias, R_LastBias);

  Set_PWM_L(valueL);
  Set_PWM_R(valueR);
}

/*==========================================================
 * 增量式 PI 控制器
 *==========================================================*/
int Incremental_PI(int Encoder, float Target,
                   float Kp, float Ki,
                   float &pwm, float &bias, float &lastBias)
{
  bias     = Target - Encoder;
  pwm     += Kp * (bias - lastBias) + Ki * bias;
  pwm      = constrain(pwm, -PWM_RESTRICT, PWM_RESTRICT);
  lastBias = bias;
  return (int)pwm;
}

/*==========================================================
 * PWM 输出函数
 *==========================================================*/
void Set_PWM_L(int val)
{
  if (val > 0)
  {
    val = constrain(val + L_FWD_DEADZONE, 0, PWM_RESTRICT);
    analogWrite(L_AIN1, val);
    digitalWrite(L_AIN2, LOW);
  }
  else if (val == 0)
  {
    digitalWrite(L_AIN1, LOW);
    digitalWrite(L_AIN2, LOW);
  }
  else
  {
    val = constrain(-val + L_REV_DEADZONE, 0, PWM_RESTRICT);
    analogWrite(L_AIN2, val);
    digitalWrite(L_AIN1, LOW);
  }
}

void Set_PWM_R(int val)
{
  if (val > 0)
  {
    val = constrain(val + R_FWD_DEADZONE, 0, PWM_RESTRICT);
    analogWrite(R_AIN1, val);
    digitalWrite(R_AIN2, LOW);
  }
  else if (val == 0)
  {
    digitalWrite(R_AIN1, LOW);
    digitalWrite(R_AIN2, LOW);
  }
  else
  {
    val = constrain(-val + R_REV_DEADZONE, 0, PWM_RESTRICT);
    analogWrite(R_AIN2, val);
    digitalWrite(R_AIN1, LOW);
  }
}
