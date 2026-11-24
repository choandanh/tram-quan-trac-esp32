# tram-quan-trac-esp32
// ============================================================
// TRAM QUAN TRAC ESP32 + BLYNK + ENCODER
// RELAY KICH HIGH - CA 4 JUMPER DAT O H
//
// CHAN KET NOI:
// DHT11 DATA      -> GPIO23
// OLED SDA        -> GPIO21
// OLED SCL        -> GPIO22
// Encoder CLK     -> GPIO25
// Encoder DT      -> GPIO26
// Encoder SW      -> GPIO27
// FC-28 AO        -> GPIO34
// MS-CDS05 AO     -> GPIO35
// Relay IN1       -> GPIO16 / RX2 - quat
// Relay IN2       -> GPIO17 / TX2 - phun suong
// Relay IN3       -> GPIO18       - bom
// Relay IN4       -> GPIO19       - den
//
// BLYNK:
// V0  Nhiet do do duoc
// V1  Do am khong khi do duoc
// V2  Do am dat tuong doi
// V3  Anh sang tuong doi
// V4  Trang thai lenh quat
// V5  Trang thai lenh phun
// V6  Trang thai lenh bom
// V7  Trang thai lenh den
// V8  Che do: 0 AUTO, 1 MANUAL
// V9  Lenh quat
// V10 Lenh phun
// V11 Lenh bom
// V12 Lenh den
// V13 Nguong nhiet do: 0..100 C
// V14 Nguong do am KK: 0..100 %
// V15 Nguong do am dat: 0..100 %
// V16 Nguong anh sang: 0..100 %
//
// Nguong ESP32 la nguong chinh.
// Khi ket noi lai, ESP32 gui nguong da luu len Blynk.
// Chinh nguong tren app khi thiet bi Online.
// Slider nen bat Send on release.
//
// V4..V7 la trang thai lenh phan mem,
// khong phai phan hoi do duoc tu tai.
// ============================================================

// ============================================================
// 1. DIEN THONG TIN BLYNK VA WIFI TAI DAY
// ============================================================

#define BLYNK_TEMPLATE_ID   "TMPL6lA3e2-km"
#define BLYNK_TEMPLATE_NAME "Tram Quan Trac ESP32."
#define BLYNK_AUTH_TOKEN    "85TzEUZ87-kolQhwkDdbuk1x4aZ72-zE"

#define BLYNK_PRINT Serial

#include <Arduino.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <DHT.h>
#include <Preferences.h>
#include <math.h>

const char* TEN_WIFI = "TP-Link_C37C";
const char* MAT_KHAU_WIFI = "58126608";

// ============================================================
// 2. KHAI BAO CHAN
// ============================================================

#define DHT_PIN 23
#define DHT_TYPE DHT11

#define OLED_SDA 21
#define OLED_SCL 22

#define ENC_CLK 25
#define ENC_DT  26
#define ENC_SW  27

#define SOIL_PIN  34
#define LIGHT_PIN 35

#define RELAY_QUAT 16
#define RELAY_PHUN 17
#define RELAY_BOM  18
#define RELAY_DEN  19

enum Kenh {
  QUAT = 0,
  PHUN = 1,
  BOM  = 2,
  DEN  = 3
};

const int SO_KENH = 4;

const int PIN_RELAY[SO_KENH] = {
  RELAY_QUAT,
  RELAY_PHUN,
  RELAY_BOM,
  RELAY_DEN
};

const char* TEN_TAI[SO_KENH] = {
  "QUAT",
  "PHUN SUONG",
  "MAY BOM",
  "DEN LED"
};

// ============================================================
// 3. CAU HINH RELAY
// ============================================================

// CA 4 JUMPER O H.
const int RELAY_TAT = LOW;
const int RELAY_BAT = HIGH;

// Ban nay dieu khien AUTO/MANUAL that.
const bool CHE_DO_THU_TAI = false;
const bool CHO_PHEP_RELAY_THAT = true;

// Muon thu tung kenh bang Serial:
// CHE_DO_THU_TAI = true
// CHO_PHEP_RELAY_THAT = false

const unsigned long THU_TAI_TOI_DA_MS = 3000UL;
const unsigned long THU_TAI_MO_KHOA_MS = 60000UL;
const unsigned long BOM_TOI_DA_MS = 5000UL;

bool daMoKhoaThuTai = false;
int kenhDangThu = -1;

unsigned long lucMoKhoaThuTai = 0;
unsigned long lucBatDauThuTai = 0;

bool relayDangBat[SO_KENH] = {
  false, false, false, false
};

// ============================================================
// 4. HIEU CHUAN CAM BIEN
// ============================================================

// Cac moc lay theo code ban dau.
// Can hieu chuan lai neu thay cam bien/nguon cap.

const int SOIL_DRY = 4095;
const int SOIL_WET = 1450;

const int LIGHT_DARK = 4095;
const int LIGHT_BRIGHT = 40;

// ============================================================
// 5. THIET BI
// ============================================================

DHT dht(DHT_PIN, DHT_TYPE);

U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(
  U8G2_R0,
  U8X8_PIN_NONE
);

Preferences prefs;

// ============================================================
// 6. DU LIEU CAM BIEN VA NGUONG
// ============================================================

float nhietDo = NAN;
float doAmKK = NAN;

int soilADC = 0;
int soilPercent = 0;

int lightADC = 0;
int lightPercent = 0;

// Thu tu:
// 0 nhiet do, 1 do am KK, 2 do am dat, 3 anh sang.
int nguong[SO_KENH] = {
  30, 60, 30, 50
};

const int NGUONG_MIN = 0;
const int NGUONG_MAX = 100;

const char* KHOA_NVS[SO_KENH] = {
  "temp",
  "hum",
  "soil",
  "lightPct"
};

const char* TEN_NGUONG[SO_KENH] = {
  "Nhiet do",
  "Do am KK",
  "Do am dat",
  "Anh sang"
};

// Do tre tranh dong/ngat lien tuc gan nguong.
const float TRE_NHIET_C = 1.0f;
const int TRE_AM_KK = 3;
const int TRE_AM_DAT = 3;
const int TRE_ANH_SANG = 5;

// ============================================================
// 7. TRANG THAI DIEU KHIEN
// ============================================================

bool autoQuat = false;
bool autoPhun = false;
bool autoBom = false;
bool autoDen = false;

bool cheDoManual = false;

bool lenhTay[SO_KENH] = {
  false, false, false, false
};

bool trangThaiYeuCau[SO_KENH] = {
  false, false, false, false
};

unsigned long lucBatBomAuto = 0;
bool bomAutoDaKhoa = false;

unsigned long lucBatBomManual = 0;
bool bomManualDangChay = false;
bool bomManualDaKhoa = false;

bool veNgay = true;
bool canGuiTrangThaiNgay = true;

// ============================================================
// 8. OLED VA ENCODER
// ============================================================

enum Trang {
  TRANG_CHINH,
  MENU,
  CHINH_NGUONG,
  XEM_THIET_BI
};

Trang trang = TRANG_CHINH;

int mucChon = 0;
int mucDangSua = 0;
int giaTriSua = 0;

int trangThaiAB = 0;
int tichLuyXung = 0;

int nutDocTruoc = HIGH;
int nutOnDinh = HIGH;

unsigned long lucDoiNut = 0;
const unsigned long CHONG_DOI_NUT_MS = 20UL;

// ============================================================
// 9. THOI GIAN
// ============================================================

unsigned long lanDocDHT = 0;
unsigned long lanDocAnalog = 0;
unsigned long lanVeOLED = 0;

unsigned long lanThuWiFi = 0;
unsigned long lanThuBlynk = 0;
unsigned long lanGuiBlynk = 0;
unsigned long lanGuiTrangThaiNgay = 0;

const unsigned long CHU_KY_DHT_MS = 2200UL;
const unsigned long CHU_KY_ANALOG_MS = 1000UL;
const unsigned long CHU_KY_OLED_MS = 200UL;
const unsigned long CHU_KY_GUI_BLYNK = 5000UL;
const unsigned long CHU_KY_THU_LAI = 10000UL;

bool daCauHinhBlynk = false;

// ============================================================
// 10. KHAI BAO HAM
// ============================================================

void capNhatDieuKhien();
void capNhatChanRelay();
void guiTrangThaiBlynk();
void guiNguongBlynk();
void luuNguong(int muc, int giaTri);

// ============================================================
// 11. HAM CO BAN
// ============================================================

bool thongTinHopLe() {
  return
    String(BLYNK_TEMPLATE_ID).length() > 0 &&
    String(BLYNK_TEMPLATE_ID) != "DIEN_TEMPLATE_ID" &&
    String(BLYNK_AUTH_TOKEN).length() > 0 &&
    String(BLYNK_AUTH_TOKEN) != "DIEN_AUTH_TOKEN_MOI" &&
    String(TEN_WIFI).length() > 0 &&
    String(TEN_WIFI) != "DIEN_TEN_WIFI" &&
    String(MAT_KHAU_WIFI) != "DIEN_MAT_KHAU_WIFI";
}

bool dangManualThucTe() {
  return cheDoManual && Blynk.connected();
}

int docADC(int pin) {
  long tong = 0;

  for (int i = 0; i < 10; i++) {
    tong += analogRead(pin);
    delayMicroseconds(300);
  }

  return (int)(tong / 10);
}

int tinhDoAmDat(int adc) {
  float phanTram =
    (float)(SOIL_DRY - adc) * 100.0f /
    (SOIL_DRY - SOIL_WET);

  return constrain((int)roundf(phanTram), 0, 100);
}

int tinhAnhSang(int adc) {
  float phanTram =
    (float)(LIGHT_DARK - adc) * 100.0f /
    (LIGHT_DARK - LIGHT_BRIGHT);

  return constrain((int)roundf(phanTram), 0, 100);
}

bool layTrangThaiHienThi(int kenh) {
  if (CHE_DO_THU_TAI || CHO_PHEP_RELAY_THAT) {
    return relayDangBat[kenh];
  }

  return trangThaiYeuCau[kenh];
}

// ============================================================
// 12. XUAT GPIO RELAY
// ============================================================

void capNhatChanRelay() {
  unsigned long now = millis();

  bool khoaThuConHieuLuc =
    daMoKhoaThuTai &&
    now - lucMoKhoaThuTai < THU_TAI_MO_KHOA_MS;

  bool lanThuConHieuLuc =
    kenhDangThu >= 0 &&
    kenhDangThu < SO_KENH &&
    now - lucBatDauThuTai < THU_TAI_TOI_DA_MS;

  bool dauRaMoi[SO_KENH] = {
    false, false, false, false
  };

  if (CHE_DO_THU_TAI) {
    if (khoaThuConHieuLuc && lanThuConHieuLuc) {
      dauRaMoi[kenhDangThu] = true;
    }
  } else if (CHO_PHEP_RELAY_THAT) {
    for (int i = 0; i < SO_KENH; i++) {
      dauRaMoi[i] = trangThaiYeuCau[i];
    }
  }

  // Tat cac kenh can tat truoc.
  for (int i = 0; i < SO_KENH; i++) {
    if (!dauRaMoi[i]) {
      digitalWrite(PIN_RELAY[i], RELAY_TAT);
    }
  }

  for (int i = 0; i < SO_KENH; i++) {
    if (dauRaMoi[i]) {
      digitalWrite(PIN_RELAY[i], RELAY_BAT);
    }

    if (relayDangBat[i] != dauRaMoi[i]) {
      relayDangBat[i] = dauRaMoi[i];
      veNgay = true;
      canGuiTrangThaiNgay = true;
    }
  }
}

// ============================================================
// 13. THU RELAY QUA SERIAL
// ============================================================

void inHuongDanThu() {
  Serial.println();
  Serial.println("A: mo khoa thu 60 giay");
  Serial.println("1: quat, 2: phun, 3: bom, 4: den");
  Serial.println("Moi kenh bat khoang 3 giay");
  Serial.println("0 hoac S: tat tat ca va khoa");
}

void dungThuTai() {
  kenhDangThu = -1;
  daMoKhoaThuTai = false;

  capNhatChanRelay();
  veNgay = true;
}

void kiemTraHetGioThu() {
  unsigned long now = millis();

  if (
    kenhDangThu >= 0 &&
    now - lucBatDauThuTai >= THU_TAI_TOI_DA_MS
  ) {
    int kenhVuaThu = kenhDangThu;
    kenhDangThu = -1;

    capNhatChanRelay();

    Serial.print("[THU] DA TAT ");
    Serial.println(TEN_TAI[kenhVuaThu]);

    veNgay = true;
  }

  if (
    daMoKhoaThuTai &&
    now - lucMoKhoaThuTai >= THU_TAI_MO_KHOA_MS
  ) {
    dungThuTai();
    Serial.println("[THU] HET GIO. DA KHOA.");
  }
}

void xuLyThuTaiQuaSerial() {
  if (!CHE_DO_THU_TAI) {
    return;
  }

  kiemTraHetGioThu();

  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '0' || c == 'S' || c == 's') {
      dungThuTai();
      Serial.println("[THU] STOP.");
    } else if (c == 'A' || c == 'a') {
      dungThuTai();

      daMoKhoaThuTai = true;
      lucMoKhoaThuTai = millis();

      Serial.println("[THU] MO KHOA. NHAP 1..4.");
      veNgay = true;
    } else if (c >= '1' && c <= '4') {
      kiemTraHetGioThu();

      if (!daMoKhoaThuTai) {
        Serial.println("[THU] NHAP A TRUOC.");
        continue;
      }

      kenhDangThu = -1;
      capNhatChanRelay();

      kenhDangThu = c - '1';
      lucBatDauThuTai = millis();

      capNhatChanRelay();

      Serial.print("[THU] BAT ");
      Serial.println(TEN_TAI[kenhDangThu]);

      veNgay = true;
    } else if (c == 'H' || c == 'h' || c == '?') {
      inHuongDanThu();
    }
  }

  kiemTraHetGioThu();
}

// ============================================================
// 14. DIEU KHIEN AUTO
// ============================================================

void capNhatTrangThaiAUTO() {
  // DHT11 doc loi: tat quat va phun trong AUTO.
  if (isnan(nhietDo) || isnan(doAmKK)) {
    autoQuat = false;
    autoPhun = false;
  } else {
    float nguongTatQuat = nguong[QUAT] - TRE_NHIET_C;

    if (nguongTatQuat < 0.0f) {
      nguongTatQuat = 0.0f;
    }

    // Nhiet do cao hon nguong: bat quat.
    if (!autoQuat && nhietDo > nguong[QUAT]) {
      autoQuat = true;
    } else if (
      autoQuat &&
      nhietDo <= nguongTatQuat
    ) {
      autoQuat = false;
    }

    int nguongTatPhun = min(
      100,
      nguong[PHUN] + TRE_AM_KK
    );

    // Do am KK thap hon nguong: bat phun.
    if (!autoPhun && doAmKK < nguong[PHUN]) {
      autoPhun = true;
    } else if (
      autoPhun &&
      doAmKK >= nguongTatPhun
    ) {
      autoPhun = false;
    }
  }

  int nguongTatDen = min(
    100,
    nguong[DEN] + TRE_ANH_SANG
  );

  // Anh sang thap hon nguong: bat den.
  if (!autoDen && lightPercent < nguong[DEN]) {
    autoDen = true;
  } else if (
    autoDen &&
    lightPercent >= nguongTatDen
  ) {
    autoDen = false;
  }

  int nguongTatBom = min(
    100,
    nguong[BOM] + TRE_AM_DAT
  );

  // Dat du am: tat bom va xoa khoa.
  if (soilPercent >= nguongTatBom) {
    autoBom = false;
    bomAutoDaKhoa = false;
  }

  // MANUAL khong tich luy thoi gian bom AUTO.
  if (dangManualThucTe()) {
    autoBom = false;
    return;
  }

  // Dat kho: bat bom neu chua bi khoa.
  if (
    !autoBom &&
    !bomAutoDaKhoa &&
    soilPercent < nguong[BOM]
  ) {
    autoBom = true;
    lucBatBomAuto = millis();
  }

  // Moi dot bom AUTO toi da 5 giay.
  if (
    autoBom &&
    millis() - lucBatBomAuto >= BOM_TOI_DA_MS
  ) {
    autoBom = false;
    bomAutoDaKhoa = true;

    Serial.println(
      "[AUTO] BOM HET 5 GIAY. CHO DAT DU AM DE XOA KHOA."
    );

    veNgay = true;
  }
}

// ============================================================
// 15. CHON AUTO / MANUAL
// ============================================================

void capNhatDieuKhien() {
  capNhatTrangThaiAUTO();

  bool trangThaiCu[SO_KENH];

  for (int i = 0; i < SO_KENH; i++) {
    trangThaiCu[i] = trangThaiYeuCau[i];
  }

  if (dangManualThucTe()) {
    trangThaiYeuCau[QUAT] = lenhTay[QUAT];
    trangThaiYeuCau[PHUN] = lenhTay[PHUN];
    trangThaiYeuCau[DEN] = lenhTay[DEN];

    if (!lenhTay[BOM]) {
      trangThaiYeuCau[BOM] = false;
      bomManualDangChay = false;
      bomManualDaKhoa = false;
    } else if (bomManualDaKhoa) {
      trangThaiYeuCau[BOM] = false;
    } else {
      if (!bomManualDangChay) {
        bomManualDangChay = true;
        lucBatBomManual = millis();
      }

      if (
        millis() - lucBatBomManual >= BOM_TOI_DA_MS
      ) {
        trangThaiYeuCau[BOM] = false;
        bomManualDangChay = false;
        bomManualDaKhoa = true;

        Serial.println(
          "[MANUAL] BOM HET 5 GIAY. OFF ROI ON DE CHAY LAI."
        );

        veNgay = true;
      } else {
        trangThaiYeuCau[BOM] = true;
      }
    }
  } else {
    trangThaiYeuCau[QUAT] = autoQuat;
    trangThaiYeuCau[PHUN] = autoPhun;
    trangThaiYeuCau[BOM] = autoBom;
    trangThaiYeuCau[DEN] = autoDen;

    if (bomManualDangChay && lenhTay[BOM]) {
      bomManualDaKhoa = true;
    }

    bomManualDangChay = false;
  }

  for (int i = 0; i < SO_KENH; i++) {
    if (trangThaiCu[i] != trangThaiYeuCau[i]) {
      veNgay = true;
      canGuiTrangThaiNgay = true;
    }
  }

  capNhatChanRelay();
}

// ============================================================
// 16. LUU VA DONG BO NGUONG
// ============================================================

void guiNguongBlynk() {
  if (!Blynk.connected()) {
    return;
  }

  Blynk.virtualWrite(V13, nguong[QUAT]);
  Blynk.virtualWrite(V14, nguong[PHUN]);
  Blynk.virtualWrite(V15, nguong[BOM]);
  Blynk.virtualWrite(V16, nguong[DEN]);
}

void luuNguong(int muc, int giaTri) {
  if (muc < 0 || muc >= SO_KENH) {
    return;
  }

  giaTri = constrain(
    giaTri,
    NGUONG_MIN,
    NGUONG_MAX
  );

  // Chi ghi NVS khi gia tri thay doi.
  if (nguong[muc] != giaTri) {
    nguong[muc] = giaTri;
    prefs.putInt(KHOA_NVS[muc], giaTri);
  }

  // Dong bo man hinh dang chinh bang encoder.
  if (trang == CHINH_NGUONG && mucDangSua == muc) {
    giaTriSua = nguong[muc];
  }

  capNhatDieuKhien();
  veNgay = true;

  // V13..V16 tuong ung muc 0..3.
  if (Blynk.connected()) {
    Blynk.virtualWrite(13 + muc, nguong[muc]);
  }

  Serial.print("[NGUONG] ");
  Serial.print(TEN_NGUONG[muc]);
  Serial.print(" = ");
  Serial.println(nguong[muc]);
}

// ============================================================
// 17. NHAN LENH BLYNK
// ============================================================

BLYNK_CONNECTED() {
  // Xoa lenh tay cu khi ket noi lai.
  for (int i = 0; i < SO_KENH; i++) {
    lenhTay[i] = false;
  }

  bomManualDangChay = false;
  bomManualDaKhoa = false;

  Blynk.virtualWrite(V9, 0);
  Blynk.virtualWrite(V10, 0);
  Blynk.virtualWrite(V11, 0);
  Blynk.virtualWrite(V12, 0);

  // Nguong da luu tren ESP32 duoc gui len app.
  guiNguongBlynk();

  // Chi dong bo che do tu Cloud.
  Blynk.syncVirtual(V8);

  capNhatDieuKhien();

  canGuiTrangThaiNgay = true;
  veNgay = true;

  Serial.println("[BLYNK] ONLINE. DA DONG BO NGUONG.");
}

BLYNK_WRITE(V8) {
  cheDoManual = (param.asInt() == 1);

  capNhatDieuKhien();
  veNgay = true;

  Serial.println(
    cheDoManual ? "[BLYNK] MANUAL" : "[BLYNK] AUTO"
  );
}

BLYNK_WRITE(V9) {
  lenhTay[QUAT] = (param.asInt() == 1);
  capNhatDieuKhien();
}

BLYNK_WRITE(V10) {
  lenhTay[PHUN] = (param.asInt() == 1);
  capNhatDieuKhien();
}

BLYNK_WRITE(V11) {
  lenhTay[BOM] = (param.asInt() == 1);

  if (!lenhTay[BOM]) {
    bomManualDangChay = false;
    bomManualDaKhoa = false;
  }

  capNhatDieuKhien();
}

BLYNK_WRITE(V12) {
  lenhTay[DEN] = (param.asInt() == 1);
  capNhatDieuKhien();
}

BLYNK_WRITE(V13) {
  luuNguong(QUAT, param.asInt());
}

BLYNK_WRITE(V14) {
  luuNguong(PHUN, param.asInt());
}

BLYNK_WRITE(V15) {
  luuNguong(BOM, param.asInt());
}

BLYNK_WRITE(V16) {
  luuNguong(DEN, param.asInt());
}

// ============================================================
// 18. GUI DU LIEU BLYNK
// ============================================================

void guiTrangThaiBlynk() {
  if (!Blynk.connected()) {
    return;
  }

  Blynk.virtualWrite(V4, (int)layTrangThaiHienThi(QUAT));
  Blynk.virtualWrite(V5, (int)layTrangThaiHienThi(PHUN));
  Blynk.virtualWrite(V6, (int)layTrangThaiHienThi(BOM));
  Blynk.virtualWrite(V7, (int)layTrangThaiHienThi(DEN));

  canGuiTrangThaiNgay = false;
  lanGuiTrangThaiNgay = millis();
}

void guiDuLieuBlynk() {
  if (!Blynk.connected()) {
    return;
  }

  if (!isnan(nhietDo)) {
    Blynk.virtualWrite(V0, nhietDo);
  }

  if (!isnan(doAmKK)) {
    Blynk.virtualWrite(V1, doAmKK);
  }

  Blynk.virtualWrite(V2, soilPercent);
  Blynk.virtualWrite(V3, lightPercent);

  guiTrangThaiBlynk();
}

void xuLyKetNoiBlynk() {
  if (!daCauHinhBlynk) {
    return;
  }

  unsigned long now = millis();

  if (WiFi.status() != WL_CONNECTED) {
    if (now - lanThuWiFi >= CHU_KY_THU_LAI) {
      lanThuWiFi = now;
      WiFi.reconnect();

      Serial.println("[WIFI] THU KET NOI LAI.");
    }

    return;
  }

  if (!Blynk.connected()) {
    if (now - lanThuBlynk >= CHU_KY_THU_LAI) {
      lanThuBlynk = now;

      bool coRelayDangBat = false;

      for (int i = 0; i < SO_KENH; i++) {
        if (relayDangBat[i]) {
          coRelayDangBat = true;
        }
      }

      unsigned long thoiGianCho =
        coRelayDangBat ? 100UL : 500UL;

      Blynk.connect(thoiGianCho);

      if (Blynk.connected()) {
        lanGuiBlynk = millis() - CHU_KY_GUI_BLYNK;
      }
    }
  }

  if (Blynk.connected()) {
    Blynk.run();
  }
}

void xuLyGuiBlynk() {
  if (!Blynk.connected()) {
    return;
  }

  unsigned long now = millis();

  if (now - lanGuiBlynk >= CHU_KY_GUI_BLYNK) {
    lanGuiBlynk = now;
    guiDuLieuBlynk();
  } else if (
    canGuiTrangThaiNgay &&
    now - lanGuiTrangThaiNgay >= 1000UL
  ) {
    guiTrangThaiBlynk();
  }
}

// ============================================================
// 19. ENCODER
// ============================================================

void xuLyXoay(int huong) {
  if (trang == MENU) {
    mucChon += huong;

    if (mucChon > 5) {
      mucChon = 0;
    }

    if (mucChon < 0) {
      mucChon = 5;
    }

    veNgay = true;
  } else if (trang == CHINH_NGUONG) {
    giaTriSua = constrain(
      giaTriSua + huong,
      NGUONG_MIN,
      NGUONG_MAX
    );

    veNgay = true;
  }
}

void xuLyNhan() {
  if (trang == TRANG_CHINH) {
    mucChon = 0;
    trang = MENU;
  } else if (trang == MENU) {
    if (mucChon == 5) {
      trang = TRANG_CHINH;
    } else if (mucChon == 4) {
      trang = XEM_THIET_BI;
    } else {
      mucDangSua = mucChon;
      giaTriSua = nguong[mucDangSua];
      trang = CHINH_NGUONG;
    }
  } else if (trang == CHINH_NGUONG) {
    luuNguong(mucDangSua, giaTriSua);

    mucChon = mucDangSua;
    trang = MENU;
  } else if (trang == XEM_THIET_BI) {
    mucChon = 4;
    trang = MENU;
  }

  veNgay = true;
}

void docEncoder() {
  static const int8_t bangXung[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
  };

  int clk = digitalRead(ENC_CLK);
  int dt = digitalRead(ENC_DT);

  int abMoi = (clk << 1) | dt;

  if (abMoi != trangThaiAB) {
    int chiSo = (trangThaiAB << 2) | abMoi;

    tichLuyXung += bangXung[chiSo];
    trangThaiAB = abMoi;

    if (tichLuyXung >= 4) {
      xuLyXoay(1);
      tichLuyXung = 0;
    } else if (tichLuyXung <= -4) {
      xuLyXoay(-1);
      tichLuyXung = 0;
    }
  }

  int nutHienTai = digitalRead(ENC_SW);
  unsigned long now = millis();

  if (nutHienTai != nutDocTruoc) {
    lucDoiNut = now;
  }

  if (
    now - lucDoiNut >= CHONG_DOI_NUT_MS &&
    nutHienTai != nutOnDinh
  ) {
    nutOnDinh = nutHienTai;

    if (nutOnDinh == LOW) {
      xuLyNhan();
    }
  }

  nutDocTruoc = nutHienTai;
}

// ============================================================
// 20. HIEN THI OLED
// ============================================================

void veMucMenu(int muc) {
  if (muc >= 0 && muc < SO_KENH) {
    oled.print(TEN_NGUONG[muc]);
    oled.print(": ");
    oled.print(nguong[muc]);

    if (muc == QUAT) {
      oled.print(" C");
    } else {
      oled.print("%");
    }
  } else if (muc == 4) {
    oled.print("Xem thiet bi");
  } else if (muc == 5) {
    oled.print("Quay lai");
  }
}

void hienThiOLED() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x12_tf);

  if (trang == TRANG_CHINH) {
    oled.drawStr(12, 10, "TRAM QUAN TRAC");
    oled.drawHLine(0, 13, 128);

    oled.setCursor(0, 25);
    oled.print("Nhiet do: ");

    if (isnan(nhietDo)) {
      oled.print("--");
    } else {
      oled.print(nhietDo, 1);
    }

    oled.print(" C");

    oled.setCursor(0, 37);
    oled.print("Do am KK: ");

    if (isnan(doAmKK)) {
      oled.print("--");
    } else {
      oled.print(doAmKK, 1);
    }

    oled.print("%");

    oled.setCursor(0, 49);
    oled.print("Am dat: ");
    oled.print(soilPercent);
    oled.print("%");

    oled.setCursor(0, 61);
    oled.print("Anh sang: ");
    oled.print(lightPercent);
    oled.print("%");
  } else if (trang == MENU) {
    oled.drawStr(5, 11, "CAI DAT NGUONG");
    oled.drawHLine(0, 14, 128);

    int mucDau = constrain(mucChon - 1, 0, 3);

    for (int i = 0; i < 3; i++) {
      int muc = mucDau + i;

      oled.setCursor(0, 29 + i * 13);
      oled.print(muc == mucChon ? "> " : "  ");

      veMucMenu(muc);
    }
  } else if (trang == CHINH_NGUONG) {
    oled.drawStr(5, 11, "CHINH NGUONG");
    oled.drawHLine(0, 14, 128);

    oled.setCursor(0, 28);
    oled.print(TEN_NGUONG[mucDangSua]);

    oled.setFont(u8g2_font_9x18B_tf);
    oled.setCursor(25, 49);
    oled.print(giaTriSua);

    if (mucDangSua == QUAT) {
      oled.print(" C");
    } else {
      oled.print(" %");
    }

    oled.setFont(u8g2_font_6x12_tf);
    oled.drawStr(0, 62, "Nhan SW de LUU");
  } else if (trang == XEM_THIET_BI) {
    const char* tieuDe;

    if (CHE_DO_THU_TAI) {
      if (kenhDangThu >= 0) {
        tieuDe = "DANG THU RELAY";
      } else if (daMoKhoaThuTai) {
        tieuDe = "THU: DA MO KHOA";
      } else {
        tieuDe = "THU: DANG KHOA";
      }
    } else if (!CHO_PHEP_RELAY_THAT) {
      tieuDe = "DIEU KHIEN MO PHONG";
    } else if (dangManualThucTe()) {
      tieuDe = "DIEU KHIEN MANUAL";
    } else {
      tieuDe = "DIEU KHIEN AUTO";
    }

    oled.drawStr(0, 11, tieuDe);
    oled.drawHLine(0, 14, 128);

    const char* nhan[SO_KENH] = {
      "Quat: ",
      "Phun suong: ",
      "Bom: ",
      "Den: "
    };

    for (int i = 0; i < SO_KENH; i++) {
      oled.setCursor(0, 26 + i * 12);
      oled.print(nhan[i]);

      bool bomDangKhoa =
        i == BOM &&
        !CHE_DO_THU_TAI &&
        !layTrangThaiHienThi(BOM) &&
        (
          (dangManualThucTe() && bomManualDaKhoa) ||
          (!dangManualThucTe() && bomAutoDaKhoa)
        );

      if (bomDangKhoa) {
        oled.print("KHOA");
      } else {
        oled.print(
          layTrangThaiHienThi(i) ? "BAT" : "TAT"
        );
      }
    }
  }

  oled.sendBuffer();
}

// ============================================================
// 21. DOC CAM BIEN VA IN SERIAL
// ============================================================

void docDHT() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();

  if (!isnan(t) && !isnan(h)) {
    nhietDo = t;
    doAmKK = h;
  } else {
    nhietDo = NAN;
    doAmKK = NAN;

    Serial.println("[DHT11] LOI DOC CAM BIEN.");
  }
}

void docCamBienAnalog() {
  soilADC = docADC(SOIL_PIN);
  soilPercent = tinhDoAmDat(soilADC);

  lightADC = docADC(LIGHT_PIN);
  lightPercent = tinhAnhSang(lightADC);
}

void inDuLieuSerial() {
  Serial.print("Nhiet: ");
  Serial.print(nhietDo, 1);

  Serial.print(" C | Am KK: ");
  Serial.print(doAmKK, 1);

  Serial.print("% | Dat: ");
  Serial.print(soilPercent);

  Serial.print("% | Sang: ");
  Serial.print(lightPercent);
  Serial.print("%");

  Serial.print(" | Nguong T/H/S/L: ");

  for (int i = 0; i < SO_KENH; i++) {
    if (i > 0) {
      Serial.print("/");
    }

    Serial.print(nguong[i]);
  }

  if (CHE_DO_THU_TAI) {
    Serial.print(" | THU");
  } else if (!CHO_PHEP_RELAY_THAT) {
    Serial.print(" | MO PHONG");
  } else if (dangManualThucTe()) {
    Serial.print(" | MANUAL");
  } else {
    Serial.print(" | AUTO");
  }

  Serial.print(" | Q/P/B/D: ");

  for (int i = 0; i < SO_KENH; i++) {
    if (i > 0) {
      Serial.print("/");
    }

    Serial.print(layTrangThaiHienThi(i) ? 1 : 0);
  }

  Serial.println();
}

// ============================================================
// 22. SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

  // Dat relay ve tat ngay khi setup bat dau.
  // Trang thai trong giai doan reset phu thuoc phan cung.
  for (int i = 0; i < SO_KENH; i++) {
    digitalWrite(PIN_RELAY[i], RELAY_TAT);
    pinMode(PIN_RELAY[i], OUTPUT);
    digitalWrite(PIN_RELAY[i], RELAY_TAT);
  }

  capNhatChanRelay();

  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);

  trangThaiAB =
    (digitalRead(ENC_CLK) << 1) |
    digitalRead(ENC_DT);

  nutDocTruoc = digitalRead(ENC_SW);
  nutOnDinh = nutDocTruoc;

  dht.begin();

  Wire.begin(OLED_SDA, OLED_SCL);
  oled.setI2CAddress(0x3C * 2);
  oled.begin();
  Wire.setClock(400000);

  analogReadResolution(12);
  analogSetPinAttenuation(SOIL_PIN, ADC_11db);
  analogSetPinAttenuation(LIGHT_PIN, ADC_11db);

  pinMode(SOIL_PIN, INPUT);
  pinMode(LIGHT_PIN, INPUT);

  prefs.begin("tramqt", false);

  nguong[QUAT] = constrain(
    prefs.getInt("temp", 30),
    NGUONG_MIN,
    NGUONG_MAX
  );

  nguong[PHUN] = constrain(
    prefs.getInt("hum", 60),
    NGUONG_MIN,
    NGUONG_MAX
  );

  nguong[BOM] = constrain(
    prefs.getInt("soil", 30),
    NGUONG_MIN,
    NGUONG_MAX
  );

  // Tuong thich gia tri anh sang da luu trong code cu.
  int lightSaved = prefs.getInt("lightPct", -1);

  if (lightSaved < 0) {
    int oldLightADC = prefs.getInt("light", 2000);

    nguong[DEN] = tinhAnhSang(oldLightADC);
    prefs.putInt("lightPct", nguong[DEN]);
  } else {
    nguong[DEN] = constrain(
      lightSaved,
      NGUONG_MIN,
      NGUONG_MAX
    );
  }

  docCamBienAnalog();
  capNhatDieuKhien();

  hienThiOLED();

  lanVeOLED = millis();
  lanDocAnalog = millis();

  Serial.println();
  Serial.println("================================");
  Serial.println("TRAM QUAN TRAC ESP32");
  Serial.println("RELAY HIGH - CA 4 JUMPER O H");
  Serial.println("V8=0 AUTO; V8=1 MANUAL");
  Serial.println("V13: nguong nhiet do");
  Serial.println("V14: nguong do am khong khi");
  Serial.println("V15: nguong do am dat");
  Serial.println("V16: nguong anh sang");
  Serial.println("CA 4 NGUONG CHINH TU 0 DEN 100");

  if (CHE_DO_THU_TAI) {
    inHuongDanThu();
  } else if (CHO_PHEP_RELAY_THAT) {
    Serial.println("AUTO/MANUAL DIEU KHIEN THAT.");
    Serial.println("LENH SERIAL A, 1..4, 0 DANG TAT.");
  } else {
    Serial.println("DANG MO PHONG.");
  }

  Serial.println("================================");

  daCauHinhBlynk = thongTinHopLe();

  if (daCauHinhBlynk) {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(TEN_WIFI, MAT_KHAU_WIFI);

    Blynk.config(BLYNK_AUTH_TOKEN);

    lanThuWiFi = millis();
    lanThuBlynk = millis() - CHU_KY_THU_LAI;

    Serial.println("[WIFI] BAT DAU KET NOI.");
  } else {
    Serial.println("[OFFLINE] CHUA DIEN DU WIFI/BLYNK.");
    Serial.println("AUTO VA ENCODER VAN HOAT DONG.");
  }
}

// ============================================================
// 23. LOOP
// ============================================================

void loop() {
  xuLyThuTaiQuaSerial();
  docEncoder();

  capNhatDieuKhien();

  xuLyKetNoiBlynk();
  xuLyThuTaiQuaSerial();

  unsigned long now = millis();

  if (now - lanDocDHT >= CHU_KY_DHT_MS) {
    lanDocDHT = now;
    docDHT();
  }

  now = millis();

  if (now - lanDocAnalog >= CHU_KY_ANALOG_MS) {
    lanDocAnalog = now;

    docCamBienAnalog();
    capNhatDieuKhien();
    inDuLieuSerial();
  }

  capNhatDieuKhien();

  if (CHE_DO_THU_TAI) {
    kiemTraHetGioThu();
  }

  xuLyGuiBlynk();

  now = millis();

  if (veNgay || now - lanVeOLED >= CHU_KY_OLED_MS) {
    veNgay = false;
    lanVeOLED = now;

    hienThiOLED();
  }
}
