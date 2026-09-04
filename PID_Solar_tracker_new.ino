/*
 * ============================================================
 *  DUAL AXIS SOLAR TRACKER - KONTROL PID
 *  "Kontrol PID pada Sistem Dual Axis Solar Tracker Berbasis
 *   ESP32 untuk Meningkatkan Akurasi Pelacakan Matahari
 *   pada Panel Surya 20 WP"
 * ============================================================
 *  Komponen:
 *    - ESP32
 *    - 4x Modul LDR (AO)
 *    - 2x Motor Driver BTS7960
 *
 *  Mapping LDR:
 *    LDR1 = Kiri Atas  | LDR2 = Kanan Atas
 *    LDR3 = Kiri Bawah | LDR4 = Kanan Bawah
 *
 *  Pengujian: 06.00 - 18.00 WIB
 * ============================================================
 */

// ============================================================
//  [1] PIN DEFINITIONS
// ============================================================
#define LDR1_PIN  34    // Kiri  Atas
#define LDR2_PIN  32    // Kanan Atas
#define LDR3_PIN  35    // Kiri  Bawah
#define LDR4_PIN  33    // Kanan Bawah

#define MOTOR_X_RPWM  25
#define MOTOR_X_LPWM  26
#define MOTOR_X_R_EN  27
#define MOTOR_X_L_EN  14

#define MOTOR_Y_RPWM  18
#define MOTOR_Y_LPWM  19
#define MOTOR_Y_R_EN  21
#define MOTOR_Y_L_EN  22

// ============================================================
//  [2] PARAMETER SISTEM
//  *** UBAH NILAI INI SAAT PROSES TUNING ***
// ============================================================
float Kp_X = 1.3,  Ki_X = 0.5,  Kd_X = 0.09;
float Kp_Y = 0.6,  Ki_Y = 0.12,  Kd_Y = 0.1;

#define DEADBAND          90
#define PWM_MIN           15    // PERBAIKAN: MIN harus < MAX
#define PWM_MAX           40    // PERBAIKAN: MAX harus > MIN

// --- KICK-START (mengatasi gesekan statis/stiction saat motor mulai bergerak dari diam) ---
// PENTING: PWM_RESOLUTION=8bit -> rentang PWM sebenarnya 0-255, bukan 0-100!
// PWM_MIN/PWM_MAX di atas (10-20) sangat kecil, kemungkinan besar tidak cukup untuk
// MULAI memutar motor dari diam (meski cukup untuk MEMPERTAHANKAN gerakan).
// WAJIB DIUJI: naikkan KICKSTART_PWM sedikit demi sedikit sampai motor pasti bergerak dari diam.
#define KICKSTART_PWM          90    // PWM pulsa awal (0-255) - MULAI dari sini, naikkan jika masih diam
#define KICKSTART_DURATION_MS  200   // durasi pulsa kick-start (ms)
// --- ADAPTIF ERROR MAX (menggantikan ERROR_MAX tetap) ---
#define ERROR_MAX_INIT     800    // Tebakan awal, akan menyesuaikan sendiri saat runtime
#define ERROR_MAX_MIN      300    // Batas bawah, agar mapping PWM tidak terlalu sensitif
#define ERROR_MAX_DECAY    0.999  // Makin dekat ke 1 = makin lambat "melupakan" puncak error lama

float errorMaxX_adaptive = ERROR_MAX_INIT;
float errorMaxY_adaptive = ERROR_MAX_INIT;
// --- ADAPTIF ANTI-OSILASI (deadband melebar saat motor sering balik arah) ---
#define OSC_DECREASE       0.80   // damping dikalikan ini tiap kali motor balik arah
#define OSC_RECOVER        1.03   // damping pulih perlahan tiap siklus arah stabil
#define DAMPING_MIN        0.30   // batas bawah damping -> deadband bisa melebar ~3.3x
#define INTEGRAL_LIMIT    1000.0
#define SAMPLE_TIME_MS    1000
#define SOFTSTART_STEP    8

// LEDC ESP32 core v3.x — tidak pakai channel eksplisit
#define PWM_FREQ          1000   // Hz
#define PWM_RESOLUTION    8      // bit → 0-255

// ============================================================
//  [3] VARIABEL GLOBAL
// ============================================================
float errorX = 0, prevErrorX = 0, integralX = 0, outputX = 0;
float errorY = 0, prevErrorY = 0, integralY = 0, outputY = 0;

int currentPWM_X = 0;
int currentPWM_Y = 0;

unsigned long lastTime = 0;
float dt = 0;

float dampingX = 1.0, dampingY = 1.0;
int   lastDirX = 0,   lastDirY = 0;

int ldr1Val, ldr2Val, ldr3Val, ldr4Val;

// --- Variabel Filter LDR ---
// Lapis 1: rata-rata 10 sampel (di dalam readLDR)
// Lapis 2: spike filter — buang perubahan mendadak > SPIKE_LIMIT
// Lapis 3: Low Pass Filter — haluskan perubahan gradual
float filtered1 = 0, filtered2 = 0, filtered3 = 0, filtered4 = 0;
#define LPF_ALPHA     0.1    // 0.0-1.0 | kecil=halus lambat, besar=cepat noise
#define SPIKE_LIMIT   200    // max perubahan per iterasi yang dianggap valid


// ============================================================
//  [4] SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("============================================");
  Serial.println("  Dual Axis Solar Tracker PID - ESP32");
  Serial.println("  Pengujian: 06.00 - 18.00 WIB");
  Serial.println("============================================");

  // ✅ PERBAIKAN: ESP32 core v3.x gunakan ledcAttach() langsung
  // Format: ledcAttach(pin, freq, resolution)
  ledcAttach(MOTOR_X_RPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR_X_LPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR_Y_RPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR_Y_LPWM, PWM_FREQ, PWM_RESOLUTION);

  // Enable BTS7960
  pinMode(MOTOR_X_R_EN, OUTPUT); digitalWrite(MOTOR_X_R_EN, HIGH);
  pinMode(MOTOR_X_L_EN, OUTPUT); digitalWrite(MOTOR_X_L_EN, HIGH);
  pinMode(MOTOR_Y_R_EN, OUTPUT); digitalWrite(MOTOR_Y_R_EN, HIGH);
  pinMode(MOTOR_Y_L_EN, OUTPUT); digitalWrite(MOTOR_Y_L_EN, HIGH);

  stopMotorX();
  stopMotorY();

  analogReadResolution(12);
  lastTime = millis();
  delay(500);
  Serial.println("System Ready.");
  Serial.println();
}

// ============================================================
//  [5] LOOP UTAMA
// ============================================================
void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastTime >= SAMPLE_TIME_MS) {
    dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    if (dt <= 0) return;

    readLDR();
    computeError();

    outputX = computePID(errorX, integralX, prevErrorX,
                         dt, Kp_X, Ki_X, Kd_X);
    outputY = computePID(errorY, integralY, prevErrorY,
                         dt, Kp_Y, Ki_Y, Kd_Y);

    driveMotorX(outputX);
    driveMotorY(outputY);

    printMonitor();
    printPlotter();
  }
}

// ============================================================
//  [6] BACA LDR + LOW PASS FILTER + INVERT
//
//  Rumus LPF:
//  filtered = (alpha × nilai_baru) + ((1 - alpha) × nilai_sebelumnya)
//
//  Contoh dengan alpha=0.2:
//  filtered = (0.2 × bacaan_baru) + (0.8 × filtered_lama)
//  → 80% nilai lama + 20% nilai baru = perubahan halus & stabil
// ============================================================
void readLDR() {
  static bool firstRead = true;   // supaya sampel pertama tidak dianggap "spike" (filtered masih 0)

  long s1=0, s2=0, s3=0, s4=0;

  // Rata-rata 5 sampel (filter noise ADC)
  for (int i = 0; i < 5; i++) {
    s1 += analogRead(LDR1_PIN);
    s2 += analogRead(LDR2_PIN);
    s3 += analogRead(LDR3_PIN);
    s4 += analogRead(LDR4_PIN);
    delayMicroseconds(500);
  }

  // Invert: gelap=0, terang=4095
  float raw1 = 4095 - (s1 / 5);
  float raw2 = 4095 - (s2 / 5);
  float raw3 = 4095 - (s3 / 5);
  float raw4 = 4095 - (s4 / 5);

  if (firstRead) {
    // Inisialisasi pertama kali: langsung pakai nilai mentah, tidak ada "nilai lama" utk dibandingkan
    filtered1 = raw1;
    filtered2 = raw2;
    filtered3 = raw3;
    filtered4 = raw4;
    firstRead = false;
  } else {
    // Spike filter (lapis 2): kalau lompatan dari nilai sebelumnya > SPIKE_LIMIT,
    // dianggap noise/gangguan mendadak -> abaikan, pakai nilai filter terakhir yang valid
    if (abs(raw1 - filtered1) > SPIKE_LIMIT) raw1 = filtered1;
    if (abs(raw2 - filtered2) > SPIKE_LIMIT) raw2 = filtered2;
    if (abs(raw3 - filtered3) > SPIKE_LIMIT) raw3 = filtered3;
    if (abs(raw4 - filtered4) > SPIKE_LIMIT) raw4 = filtered4;

    // Low Pass Filter (lapis 3)
    filtered1 = (LPF_ALPHA * raw1) + ((1.0 - LPF_ALPHA) * filtered1);
    filtered2 = (LPF_ALPHA * raw2) + ((1.0 - LPF_ALPHA) * filtered2);
    filtered3 = (LPF_ALPHA * raw3) + ((1.0 - LPF_ALPHA) * filtered3);
    filtered4 = (LPF_ALPHA * raw4) + ((1.0 - LPF_ALPHA) * filtered4);
  }

  // Simpan ke variabel integer untuk dipakai PID & monitor
  ldr1Val = (int)filtered1;
  ldr2Val = (int)filtered2;
  ldr3Val = (int)filtered3;
  ldr4Val = (int)filtered4;
}

// ============================================================
//  [7] HITUNG ERROR
//  ErrorX = (kanan) - (kiri)  → + = gerak kanan, - = gerak kiri
//  ErrorY = (bawah) - (atas)  → + = gerak turun, - = gerak naik
// ============================================================
void computeError() {
  errorX = (float)((ldr1Val + ldr3Val) - (ldr2Val + ldr4Val));
  errorY = (float)((ldr3Val + ldr4Val) - (ldr1Val + ldr2Val));
}

// ============================================================
//  [8] HITUNG PID
// ============================================================
float computePID(float error, float &integral, float &prevError,
                 float dt, float kp, float ki, float kd) {
  if (dt <= 0) return 0;

  if (abs(error) < DEADBAND) {
    integral  = 0;
    prevError = 0;
    return 0;
  }

  float P  = kp * error;

  integral += error * dt;
  integral  = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float I   = ki * integral;

  float D   = kd * ((error - prevError) / dt);

  float output = P + I + D;
  prevError    = error;

  return output;
}

// ============================================================
//  [9] DRIVE MOTOR X
// ============================================================
void driveMotorX(float output) {
  float absErr = abs(errorX);
  int dirX = (output > 0) ? 1 : ((output < 0) ? -1 : 0);

  if (dirX != 0) {
    if (lastDirX != 0 && dirX != lastDirX) {
      dampingX *= OSC_DECREASE;
      if (dampingX < DAMPING_MIN) dampingX = DAMPING_MIN;
    } else {
      dampingX *= OSC_RECOVER;
      if (dampingX > 1.0) dampingX = 1.0;
    }
    lastDirX = dirX;
  }
  float effectiveDeadbandX = DEADBAND / dampingX;

  if (absErr <= effectiveDeadbandX || abs(output) < 1.0) {
    stopMotorX();
    return;
  }

  if (absErr > errorMaxX_adaptive) {
    errorMaxX_adaptive = absErr;
  } else {
    errorMaxX_adaptive *= ERROR_MAX_DECAY;
    if (errorMaxX_adaptive < ERROR_MAX_MIN) errorMaxX_adaptive = ERROR_MAX_MIN;
  }

  absErr = constrain(absErr, effectiveDeadbandX, errorMaxX_adaptive);
  int targetPWM = map((long)absErr, (long)effectiveDeadbandX, (long)errorMaxX_adaptive,
                       (long)PWM_MIN, (long)PWM_MAX);
  targetPWM = constrain(targetPWM, PWM_MIN, PWM_MAX);

  if (currentPWM_X == 0) {
    // Motor dari kondisi DIAM -> kasih pulsa kick-start dulu utk lawan gesekan statis
    if (output > 0) {
      ledcWrite(MOTOR_X_RPWM, KICKSTART_PWM);
      ledcWrite(MOTOR_X_LPWM, 0);
    } else {
      ledcWrite(MOTOR_X_RPWM, 0);
      ledcWrite(MOTOR_X_LPWM, KICKSTART_PWM);
    }
    delay(KICKSTART_DURATION_MS);
    currentPWM_X = targetPWM;   // lanjut langsung ke target setelah kick
  } else if (currentPWM_X < targetPWM) {
    currentPWM_X = min(currentPWM_X + SOFTSTART_STEP, targetPWM);
  } else {
    currentPWM_X = targetPWM;
  }

  if (output > 0) {
    ledcWrite(MOTOR_X_RPWM, currentPWM_X);
    ledcWrite(MOTOR_X_LPWM, 0);
  } else {
    ledcWrite(MOTOR_X_RPWM, 0);
    ledcWrite(MOTOR_X_LPWM, currentPWM_X);
  }
}
// ============================================================
//  [10] DRIVE MOTOR Y
// ============================================================
void driveMotorY(float output) {
  float absErr = abs(errorY);
  int dirY = (output > 0) ? 1 : ((output < 0) ? -1 : 0);

  if (dirY != 0) {
    if (lastDirY != 0 && dirY != lastDirY) {
      dampingY *= OSC_DECREASE;
      if (dampingY < DAMPING_MIN) dampingY = DAMPING_MIN;
    } else {
      dampingY *= OSC_RECOVER;
      if (dampingY > 1.0) dampingY = 1.0;
    }
    lastDirY = dirY;
  }
  float effectiveDeadbandY = DEADBAND / dampingY;

  if (absErr <= effectiveDeadbandY || abs(output) < 1.0) {
    stopMotorY();
    return;
  }

  if (absErr > errorMaxY_adaptive) {
    errorMaxY_adaptive = absErr;
  } else {
    errorMaxY_adaptive *= ERROR_MAX_DECAY;
    if (errorMaxY_adaptive < ERROR_MAX_MIN) errorMaxY_adaptive = ERROR_MAX_MIN;
  }

  absErr = constrain(absErr, effectiveDeadbandY, errorMaxY_adaptive);
  int targetPWM = map((long)absErr, (long)effectiveDeadbandY, (long)errorMaxY_adaptive,
                       (long)PWM_MIN, (long)PWM_MAX);
  targetPWM = constrain(targetPWM, PWM_MIN, PWM_MAX);

  if (currentPWM_Y == 0) {
    // Motor dari kondisi DIAM -> kasih pulsa kick-start dulu utk lawan gesekan statis
    if (output > 0) {
      ledcWrite(MOTOR_Y_RPWM, KICKSTART_PWM);
      ledcWrite(MOTOR_Y_LPWM, 0);
    } else {
      ledcWrite(MOTOR_Y_RPWM, 0);
      ledcWrite(MOTOR_Y_LPWM, KICKSTART_PWM);
    }
    delay(KICKSTART_DURATION_MS);
    currentPWM_Y = targetPWM;   // lanjut langsung ke target setelah kick
  } else if (currentPWM_Y < targetPWM) {
    currentPWM_Y = min(currentPWM_Y + SOFTSTART_STEP, targetPWM);
  } else {
    currentPWM_Y = targetPWM;
  }

  if (output > 0) {
    ledcWrite(MOTOR_Y_RPWM, currentPWM_Y);
    ledcWrite(MOTOR_Y_LPWM, 0);
  } else {
    ledcWrite(MOTOR_Y_RPWM, 0);
    ledcWrite(MOTOR_Y_LPWM, currentPWM_Y);
  }
}
// ============================================================
//  [11] STOP MOTOR
// ============================================================
void stopMotorX() {
  ledcWrite(MOTOR_X_RPWM, 0);
  ledcWrite(MOTOR_X_LPWM, 0);
  currentPWM_X = 0;
}

void stopMotorY() {
  ledcWrite(MOTOR_Y_RPWM, 0);
  ledcWrite(MOTOR_Y_LPWM, 0);
  currentPWM_Y = 0;
}

// ============================================================
//  [13] SERIAL MONITOR
//  Hanya tampilkan: LDR1-4, Error X&Y, Output PID X&Y
// ============================================================
void printMonitor() {

  // Arah Motor X
  String arahX;
  if (abs(outputX) < 1.0)  arahX = "STOP";
  else if (outputX > 0)    arahX = "KANAN";
  else                     arahX = "KIRI";

  // Arah Motor Y
  String arahY;
  if (abs(outputY) < 1.0)  arahY = "STOP";
  else if (outputY > 0)    arahY = "BAWAH";
  else                     arahY = "ATAS";

  Serial.printf(
    "LDR1:%4d | LDR2:%4d | LDR3:%4d | LDR4:%4d | ErrX:%7.1f | ErrY:%7.1f | PID_X:%7.1f | PID_Y:%7.1f | Arah Motor X:%s | Arah Motor Y:%s\n",
    ldr1Val, ldr2Val, ldr3Val, ldr4Val,
    errorX, errorY,
    outputX, outputY,
    arahX.c_str(), arahY.c_str()
  );
}

// ============================================================
//  [14] SERIAL PLOTTER (Tools → Serial Plotter Arduino IDE)
//  Grafik real-time untuk dokumentasi skripsi
// ============================================================
void printPlotter() {
  Serial.print("ErrorX:"); Serial.print(errorX);   Serial.print("\t");
  Serial.print("ErrorY:"); Serial.print(errorY);   Serial.print("\t");
  Serial.print("PID_X:");  Serial.print(outputX);  Serial.print("\t");
  Serial.print("PID_Y:");  Serial.println(outputY);
}
