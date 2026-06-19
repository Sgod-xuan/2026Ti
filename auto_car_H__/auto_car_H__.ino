/*==========================================================
 * 自动行驶小车 —— H题完整控制程序
 * 2024 年全国大学生电子设计竞赛
 * 任务路径：
 *   TASK 1: A→B（直行 100cm）
 *   TASK 2: A→B→弧(BC)→C→D→弧(DA)→A  顺时针一圈
 *   TASK 3: A→C→弧(CB)→B→D→弧(DA)→A  对角穿越一圈
 *   TASK 4: 按TASK3路径连续4圈
 *
 * 直行（空白路面）控制策略：
 *   - 启动时记录当前航向角作为目标
 *   - 陀螺仪实时检测偏差，差速修正
 *   - 编码器累计脉冲数到达目标距离后停止
 *   - 若中途传感器检测到弧线，提前切换巡线模式
 *
 * 传感器逻辑（LF04）：
 *   黑线→LOW → 取反后 true
 *   白底→HIGH → 取反后 false
 *
 * 引脚分配：
 *   LF04    S1→A0  S2→A1  S3→A2  S4→A3
 *   L电机   AIN1→9  AIN2→6
 *   R电机   AIN1→10 AIN2→5
 *   L编码器 A→7  B→2(INT0)
 *   R编码器 A→8  B→3(INT1)
 *   MPU6050 SDA→A4  SCL→A5
 *   蜂鸣器  →D11（低电平触发有源蜂鸣器）
 *   LED     →D12
 *   按钮    →D4（INPUT_PULLUP，按下低电平）
 *==========================================================*/

#include <MsTimer2.h>
#include <Wire.h>

/*----------------------------------------------------------
 * ★ 任务选择：修改此宏切换要求 1 / 2 / 3 / 4
 *----------------------------------------------------------*/
#define TASK  2

/*----------------------------------------------------------
 * 引脚定义
 *----------------------------------------------------------*/
#define S1_PIN       A0
#define S2_PIN       A1
#define S3_PIN       A2
#define S4_PIN       A3

const uint8_t L_AIN1 = 9;
const uint8_t L_AIN2 = 6;
const uint8_t R_AIN1 = 10;
const uint8_t R_AIN2 = 5;

#define L_ENCODER_A  7
#define L_ENCODER_B  2   // INT0
#define R_ENCODER_A  8
#define R_ENCODER_B  3   // INT1

#define BUZZER_PIN   11
#define LED_PIN      12
#define BTN_PIN      4

#define MPU6050_ADDR 0x68

/*----------------------------------------------------------
 * ★ 场地参数（请根据实测标定后修改）
 *
 * ENCODER_PULSES_PER_CM：
 *   标定方法：让小车直行100cm，记录左轮累计脉冲数，除以100即得
 *   典型值：编码器400脉冲/转、轮径6.5cm → 约19.6脉冲/cm
 *
 * DIAG_TURN_ANGLE：
 *   场地水平100cm、竖直120cm，对角线方向角 = arctan(100/120) ≈ 39.8°
 *   小车初始朝向为水平（A→B方向），需顺时针转 39.8° 才朝向C
 *   若小车初始朝向与AB不同，需相应调整
 *----------------------------------------------------------*/
#define ENCODER_PULSES_PER_CM  20.0f

#define DIST_AB      100.0f           // A→B 直线距离 cm
#define DIST_CD      100.0f           // C→D 直线距离 cm
#define DIST_DIAG    156.2f           // A→C 对角线距离 cm（sqrt(100²+120²)）

#define TARGET_AB    ((long)(DIST_AB   * ENCODER_PULSES_PER_CM))
#define TARGET_CD    ((long)(DIST_CD   * ENCODER_PULSES_PER_CM))
#define TARGET_DIAG  ((long)(DIST_DIAG * ENCODER_PULSES_PER_CM))

#define DIAG_TURN_ANGLE  39.8f        // A点出发前需顺时针转的角度（朝向C）

/*----------------------------------------------------------
 * PD 巡线参数（弧线段）
 *----------------------------------------------------------*/
float BASE_SPEED  = 10.0f;
float LINE_KP     = 2.0f;
float LINE_KD     = 5.0f;

/*----------------------------------------------------------
 * 直行参数（空白路面段）
 *----------------------------------------------------------*/
float STRAIGHT_SPEED  = 10.0f;
float STRAIGHT_KP     = 0.08f;   // 航向偏差修正系数（越大修正越猛）

/*----------------------------------------------------------
 * 电机 PI 参数
 *----------------------------------------------------------*/
float L_KP = 50, L_KI = 10;
float R_KP = 50, R_KI = 10;

/*----------------------------------------------------------
 * 死区补偿
 *----------------------------------------------------------*/
const int PWM_RESTRICT   = 255;
const int L_FWD_DEADZONE = 30;
const int L_REV_DEADZONE = 38;
const int R_FWD_DEADZONE = 29;
const int R_REV_DEADZONE = 23;

/*----------------------------------------------------------
 * 转向参数（陀螺仪原地转向）
 *----------------------------------------------------------*/
float TURN_SPEED   = 7.0f;
float ANGLE_KP     = 0.15f;
float ANGLE_THRESH = 1.0f;    // 误差容限（度）

/*----------------------------------------------------------
 * 声光提示时长
 *----------------------------------------------------------*/
#define BEEP_MS  500

/*----------------------------------------------------------
 * 状态机枚举
 *----------------------------------------------------------*/
enum CarState {
  ST_WAIT = 0,

  // 声光提示（通用，完成后跳 nextState）
  ST_BEEP,

  // 陀螺仪原地转向（通用，完成后跳 nextState）
  ST_TURN_GYRO,

  // 直行（走空白路面，完成后触发 BEEP 再跳 nextState）
  ST_STRAIGHT,

  // 弧线巡线（完成后触发 BEEP 再跳 nextState）
  ST_ARC,

  // 最终停车
  ST_STOP
};

CarState carState = ST_WAIT;

/*----------------------------------------------------------
 * 下一步状态（供 ST_BEEP / ST_TURN_GYRO 使用）
 *----------------------------------------------------------*/
CarState nextState = ST_STOP;

/*----------------------------------------------------------
 * 运行时变量
 *----------------------------------------------------------*/

// 传感器
bool s1, s2, s3, s4;
int  lastError = 0;

// 电机目标速度
volatile float TargetL = 0;
volatile float TargetR = 0;

// 编码器（motorControl读后清零）
volatile int CountL = 0;
volatile int CountR = 0;

// 速度与 PWM
int VelocityL = 0, VelocityR = 0;
int valueL = 0, valueR = 0;

// PI 内部状态
float L_PWM = 0, L_Bias = 0, L_LastBias = 0;
float R_PWM = 0, R_Bias = 0, R_LastBias = 0;

// 陀螺仪
float gyroZ_offset   = 0;
float currentAngle   = 0;
float turnStartAngle  = 0;
float targetTurnAngle = 0;
unsigned long lastGyroTime = 0;

// 直行
volatile long straightPulseL = 0;  // 正向脉冲累计
long          straightTarget  = 0;
float         straightHeading = 0;

// 声光提示
unsigned long beepStart = 0;
bool finalBeepDone = false;

// 圈数（要求4）
int lapCount = 0;

/*----------------------------------------------------------
 * 多步任务队列（最大16步）
 * 每步记录：当前段类型 + 结束后进入的状态
 *----------------------------------------------------------*/

// 段类型
#define SEG_STRAIGHT  0   // 直行
#define SEG_ARC       1   // 弧线巡线
#define SEG_TURN      2   // 转向

struct Segment {
  int   type;
  long  param_l;    // 直行：目标脉冲；转向：角度*100（保留两位小数）
  float param_f;    // 转向时用的角度（度）
};

#define MAX_SEGS 32
Segment segs[MAX_SEGS];
int segTotal = 0;
int segIdx   = 0;

// 添加直行段
void addStraight(long pulses) {
  if (segTotal >= MAX_SEGS) return;
  segs[segTotal].type    = SEG_STRAIGHT;
  segs[segTotal].param_l = pulses;
  segTotal++;
}

// 添加弧线段
void addArc() {
  if (segTotal >= MAX_SEGS) return;
  segs[segTotal].type = SEG_ARC;
  segTotal++;
}

// 添加转向段（正=左转逆时针，负=右转顺时针）
void addTurn(float angle) {
  if (segTotal >= MAX_SEGS) return;
  segs[segTotal].type    = SEG_TURN;
  segs[segTotal].param_f = angle;
  segTotal++;
}

// 开始执行某一段
void startSegment(int idx);

// 前进到下一段（或结束）
void advanceToNext() {
  segIdx++;
  if (segIdx >= segTotal) {
    // 所有段完成
    lapCount++;
    Serial.print("第 "); Serial.print(lapCount); Serial.println(" 圈完成");

#if TASK == 4
    if (lapCount < 4) {
      // 重新跑一圈
      segIdx = 0;
      startSegment(0);
      return;
    }
#endif
    carState = ST_STOP;
  } else {
    startSegment(segIdx);
  }
}

/*==========================================================
 * setup
 *==========================================================*/
void setup()
{
  Serial.begin(9600);
  Serial.println("/* 自动行驶小车 H题 */");

  pinMode(S1_PIN, INPUT);
  pinMode(S2_PIN, INPUT);
  pinMode(S3_PIN, INPUT);
  pinMode(S4_PIN, INPUT);

  pinMode(L_ENCODER_A, INPUT);
  pinMode(L_ENCODER_B, INPUT);
  pinMode(R_ENCODER_A, INPUT);
  pinMode(R_ENCODER_B, INPUT);

  pinMode(L_AIN1, OUTPUT); pinMode(L_AIN2, OUTPUT);
  pinMode(R_AIN1, OUTPUT); pinMode(R_AIN2, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);  // 低电平触发，默认关
  digitalWrite(LED_PIN, LOW);

  pinMode(BTN_PIN, INPUT_PULLUP);

  Wire.begin();
  mpuInit();
  mpuCalibrate();
  lastGyroTime = micros();

  attachInterrupt(0, READ_ENCODER_L, CHANGE);
  attachInterrupt(1, READ_ENCODER_R, CHANGE);

  MsTimer2::set(10, motorControl);
  MsTimer2::start();

  // 构建任务序列
  buildTaskSequence();

  Serial.println("等待按钮...");
}

/*----------------------------------------------------------
 * 根据 TASK 宏构建任务段序列
 *----------------------------------------------------------*/
void buildTaskSequence()
{
  segTotal = 0;
  segIdx   = 0;

#if TASK == 1
  /*
   * 要求1：A → B 直行
   * 起始朝向：小车头朝B方向（水平向右）
   */
  addStraight(TARGET_AB);

#elif TASK == 2
  /*
   * 要求2：顺时针一圈
   * A→B（直行）→ B→C（右侧下弧线巡线）→ C→D（直行）→ D→A（左侧上弧线巡线）
   *
   * 注：B处到C是沿弧线走，C处到D直行时小车已转过180°，
   *     因此直行方向与AB段反向。此处假设小车到C时已自然朝向D方向，
   *     若有偏差可在C处增加转向段修正。
   */
  addStraight(TARGET_AB);    // A→B
  addArc();                  // B→C 弧线
  addStraight(TARGET_CD);    // C→D
  addArc();                  // D→A 弧线

#elif TASK == 3 || TASK == 4
  /*
   * 要求3/4：对角穿越一圈
   * 在A点先顺时针转 DIAG_TURN_ANGLE° 朝向C，
   * 直行到C，沿弧到B，在B点转向朝D，
   * 直行到D，沿弧回A。
   *
   * B→D段方向：B在右上、D在左下，从B出发需顺时针转
   *            (180° - DIAG_TURN_ANGLE)° 使车头朝D方向
   *            实际角度需根据弧线完成后的车头朝向来定，
   *            建议实测修正。
   */
  addTurn(-DIAG_TURN_ANGLE);    // A处顺时针转身朝C（-表示顺时针/右转）
  addStraight(TARGET_DIAG);     // A→C
  addArc();                     // C→B 弧线（反向绕弧）
  addTurn(-(180.0f - DIAG_TURN_ANGLE)); // B处转向朝D（顺时针约140.2°）
  addStraight(TARGET_DIAG);     // B→D
  addArc();                     // D→A 弧线
#endif
}

/*----------------------------------------------------------
 * 开始执行第 idx 段
 *----------------------------------------------------------*/
void startSegment(int idx)
{
  if (idx >= segTotal) { carState = ST_STOP; return; }

  Segment &seg = segs[idx];

  Serial.print("▶ 段 "); Serial.print(idx);
  Serial.print(" 类型="); Serial.println(seg.type);

  switch (seg.type)
  {
    case SEG_STRAIGHT:
      // 重置直行计数
      noInterrupts();
      straightPulseL = 0;
      interrupts();
      straightTarget  = seg.param_l;
      straightHeading = currentAngle;
      carState = ST_STRAIGHT;
      break;

    case SEG_ARC:
      lastError = 0;
      carState  = ST_ARC;
      TargetL   = BASE_SPEED;
      TargetR   = BASE_SPEED;
      break;

    case SEG_TURN:
      turnStartAngle  = currentAngle;
      targetTurnAngle = seg.param_f;
      carState = ST_TURN_GYRO;
      break;
  }
}

/*==========================================================
 * loop
 *==========================================================*/
void loop()
{
  updateAngle();

  s1 = !digitalRead(S1_PIN);
  s2 = !digitalRead(S2_PIN);
  s3 = !digitalRead(S3_PIN);
  s4 = !digitalRead(S4_PIN);

  bool allBlack = (s1 && s2 && s3 && s4);
  bool hasLine  = (s1 || s2 || s3 || s4);

  /* ---- ST_WAIT ---- */
  if (carState == ST_WAIT)
  {
    TargetL = TargetR = 0;
    if (digitalRead(BTN_PIN) == LOW)
    {
      while (digitalRead(BTN_PIN) == LOW) delay(10);
      delay(50);
      lapCount = 0;
      finalBeepDone = false;
      segIdx = 0;
      Serial.println("出发！");
      startSegment(0);
    }
    delay(10);
    return;
  }

  /* ---- ST_BEEP：声光500ms后跳 nextState ---- */
  if (carState == ST_BEEP)
  {
    TargetL = TargetR = 0;
    if (beepStart == 0)
    {
      beepStart = millis();
      digitalWrite(BUZZER_PIN, LOW);
      digitalWrite(LED_PIN, HIGH);
      Serial.println("[声光提示]");
    }
    if (millis() - beepStart >= BEEP_MS)
    {
      beepStart = 0;
      digitalWrite(BUZZER_PIN, HIGH);
      digitalWrite(LED_PIN, LOW);
      // nextState == 255 是哨兵值，表示"前进到下一段"
      if ((int)nextState == 255) {
        advanceToNext();
      } else {
        carState = nextState;
      }
    }
    delay(5);
    return;
  }

  /* ---- ST_TURN_GYRO：原地转向到目标角度 ---- */
  if (carState == ST_TURN_GYRO)
  {
    float turned = currentAngle - turnStartAngle;
    bool reached = (targetTurnAngle >= 0)
                   ? (turned >= targetTurnAngle - ANGLE_THRESH)
                   : (turned <= targetTurnAngle + ANGLE_THRESH);

    if (reached)
    {
      TargetL = TargetR = 0;
      Serial.println("转向完成");
      delay(150);
      // 转向完成后直接前进到下一段（不需要声光提示）
      advanceToNext();
    }
    else
    {
      float remaining = abs(targetTurnAngle) - abs(turned);
      float speed = constrain(remaining * ANGLE_KP, 2.0f, TURN_SPEED);

      if (targetTurnAngle > 0) {
        TargetL = -speed; TargetR = +speed;  // 左转
      } else {
        TargetL = +speed; TargetR = -speed;  // 右转
      }
    }
    delay(5);
    return;
  }

  /* ---- ST_STRAIGHT：走空白路面 ---- */
  if (carState == ST_STRAIGHT)
  {
    // 优先检测：若传感器发现弧线，说明到达终点附近，提前切换
    if (hasLine)
    {
      Serial.println("直行检测到弧线，切换巡线");
      // 直接声光提示，然后跳下一段（弧线段）
      nextState = ST_BEEP;  // 将在BEEP里跳 advanceToNext()
      // 实际上我们需要BEEP完后进入下一段，用特殊处理
      beepStart = 0;
      nextState = (CarState)255;  // 哨兵，BEEP完成后调用 advanceToNext
      carState  = ST_BEEP;
      TargetL   = TargetR = 0;
      delay(5);
      return;
    }

    // 到达目标脉冲
    if (straightPulseL >= (unsigned long)straightTarget)
    {
      TargetL = TargetR = 0;
      Serial.print("直行完成，脉冲=");
      Serial.println(straightPulseL);
      delay(100);
      beepStart = 0;
      nextState = (CarState)255;  // 哨兵
      carState  = ST_BEEP;
      return;
    }

    // 陀螺仪航向保持
    float headingErr = currentAngle - straightHeading;
    float corr = headingErr * STRAIGHT_KP * STRAIGHT_SPEED;
    TargetL = STRAIGHT_SPEED - corr;
    TargetR = STRAIGHT_SPEED + corr;

    delay(5);
    return;
  }

  /* ---- ST_ARC：弧线巡线 ---- */
  if (carState == ST_ARC)
  {
    // 全黑：到达顶点
    if (allBlack)
    {
      TargetL = TargetR = 0;
      Serial.println("到达顶点（全黑）");
      delay(150);
      beepStart = 0;
      nextState = (CarState)255;  // 哨兵
      carState  = ST_BEEP;
      return;
    }

    // 全白：弧线丢失，保持上次修正缓慢前进
    if (!hasLine)
    {
      float corr = LINE_KP * lastError;
      TargetL = BASE_SPEED * 0.6f + corr;
      TargetR = BASE_SPEED * 0.6f - corr;
      delay(5);
      return;
    }

    // 正常 PD 巡线
    int error = calcError();
    float correction = LINE_KP * error + LINE_KD * (error - lastError);
    lastError = error;

    TargetL = constrain(BASE_SPEED + correction, -BASE_SPEED * 2, BASE_SPEED * 2);
    TargetR = constrain(BASE_SPEED - correction, -BASE_SPEED * 2, BASE_SPEED * 2);

    delay(5);
    return;
  }

  /* ---- ST_STOP ---- */
  if (carState == ST_STOP)
  {
    TargetL = TargetR = 0;
    if (!finalBeepDone)
    {
      finalBeepDone = true;
      for (int i = 0; i < 3; i++) {
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(LED_PIN, HIGH);
        delay(200);
        digitalWrite(BUZZER_PIN, HIGH);
        digitalWrite(LED_PIN, LOW);
        delay(150);
      }
      Serial.println("===== 任务完成 =====");
    }
    delay(50);
    return;
  }
}

/*==========================================================
 * BEEP 完成后的特殊处理（nextState == 255 时调用 advanceToNext）
 * 注意：255 强转 CarState 只是用作哨兵，不是真实状态
 * 实际上 ST_BEEP 里检测到 255 时调用 advanceToNext()
 *
 * 修改 ST_BEEP 块以支持哨兵：
 * （已在上方 ST_BEEP 代码中用 nextState==255 → advanceToNext 实现）
 *==========================================================*/

// 重新实现 ST_BEEP 以支持哨兵——在loop中直接处理，见上文
// 此处无需额外代码

/*==========================================================
 * 编码器中断
 *==========================================================*/
void READ_ENCODER_L()
{
  int delta;
  if (digitalRead(L_ENCODER_A) == LOW)
    delta = (digitalRead(L_ENCODER_B) == LOW) ? -1 : 1;
  else
    delta = (digitalRead(L_ENCODER_B) == LOW) ?  1 : -1;

  CountL += delta;
  if (delta > 0) straightPulseL += delta;  // 仅正向计距
}

void READ_ENCODER_R()
{
  if (digitalRead(R_ENCODER_A) == LOW)
    CountR += (digitalRead(R_ENCODER_B) == LOW) ?  1 : -1;
  else
    CountR += (digitalRead(R_ENCODER_B) == LOW) ? -1 :  1;
}

/*==========================================================
 * 10ms 定时中断：电机 PI
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

int Incremental_PI(int Encoder, float Target,
                   float Kp, float Ki,
                   float &pwm, float &bias, float &lastBias)
{
  bias     = Target - Encoder;
  pwm     += Kp * (bias - lastBias) + Ki * bias;
  pwm      = constrain(pwm, -(float)PWM_RESTRICT, (float)PWM_RESTRICT);
  lastBias = bias;
  return (int)pwm;
}

void Set_PWM_L(int val)
{
  if (val > 0) {
    val = constrain(val + L_FWD_DEADZONE, 0, PWM_RESTRICT);
    analogWrite(L_AIN1, val); digitalWrite(L_AIN2, LOW);
  } else if (val == 0) {
    digitalWrite(L_AIN1, LOW); digitalWrite(L_AIN2, LOW);
  } else {
    val = constrain(-val + L_REV_DEADZONE, 0, PWM_RESTRICT);
    analogWrite(L_AIN2, val); digitalWrite(L_AIN1, LOW);
  }
}

void Set_PWM_R(int val)
{
  if (val > 0) {
    val = constrain(val + R_FWD_DEADZONE, 0, PWM_RESTRICT);
    analogWrite(R_AIN1, val); digitalWrite(R_AIN2, LOW);
  } else if (val == 0) {
    digitalWrite(R_AIN1, LOW); digitalWrite(R_AIN2, LOW);
  } else {
    val = constrain(-val + R_REV_DEADZONE, 0, PWM_RESTRICT);
    analogWrite(R_AIN2, val); digitalWrite(R_AIN1, LOW);
  }
}

/*==========================================================
 * 计算传感器偏差
 *==========================================================*/
int calcError()
{
  if (!s1 && !s2 && !s3 && !s4) return lastError;
  if ( s1 && !s2 && !s3 && !s4) return -3;
  if ( s1 &&  s2 && !s3 && !s4) return -2;
  if (!s1 &&  s2 && !s3 && !s4) return -1;
  if (!s1 &&  s2 &&  s3 && !s4) return  0;
  if (!s1 && !s2 &&  s3 && !s4) return  1;
  if (!s1 && !s2 &&  s3 &&  s4) return  2;
  if (!s1 && !s2 && !s3 &&  s4) return  3;
  return 0;
}

/*==========================================================
 * MPU6050
 *==========================================================*/
void mpuInit()
{
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B); Wire.write(0x00);
  Wire.endTransmission();
}

void mpuCalibrate()
{
  Serial.println("陀螺仪校准，请静止...");
  long sum = 0;
  for (int i = 0; i < 100; i++) { sum += readGyroZ_raw(); delay(10); }
  gyroZ_offset = sum / 100.0f;
  Serial.print("零偏："); Serial.println(gyroZ_offset);
}

int16_t readGyroZ_raw()
{
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 2, true);
  return (int16_t)((Wire.read() << 8) | Wire.read());
}

void updateAngle()
{
  unsigned long now = micros();
  float dt = (now - lastGyroTime) / 1000000.0f;
  lastGyroTime = now;
  int16_t raw = readGyroZ_raw();
  float dps = (raw - gyroZ_offset) / 131.0f;
  if (abs(dps) > 0.5f) currentAngle += dps * dt;
}
