/* =========================================================
   smart-SGT (Smart Tower) — Arduino Uno R4 WiFi
   ── 이번 수정본에서 고친 것 ────────────────────────────
   ① 낙상(5초) 미작동 원인 수정 : HuskyLens 인식이 순간순간 끊겨도
      FALL_GAP(0.7초) 안에 다시 잡히면 '계속 쓰러진 것'으로 이어서 셈
   ② DC모터 : 부팅 자가진단에서 1초 강제 회전 + 매 루프 상태 재적용
      + 시리얼/대시보드에서 TEST_MOTOR 명령으로 즉시 확인 가능
   ③ 교실 전등 : 실내 인원 1명 이상일 때만 ON (아무도 없으면 OFF)
   ④ TFT : 부팅 자가진단 컬러 테스트 + RST핀/속도 옵션 정리
   ⑤ 대시보드 연동 : HuskyLens 연결상태(hl1/hl2)를 함께 전송,
      2초마다 #DBG 진단줄 출력(시리얼 모니터에서 원인 바로 확인)
   ========================================================= */
// ★ 터치 사용 여부 (0 = 터치 끄고 A0를 TFT RESET으로 사용 — 화면부터 살리기)
#define USE_TOUCH 0

#include <Wire.h>
#include <SPI.h>
#include "HUSKYLENS.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#if USE_TOUCH
  #include <XPT2046_Touchscreen.h>
#endif
#include "DHT.h"
#include "Arduino_LED_Matrix.h"
#include <WiFiS3.h>
#include <WiFiSSLClient.h>

// ---------- 이 단말이 설치된 교실 ID (대시보드 CLASSES의 id와 동일하게) ----------
#define CLASS_ID  "1-1"
#define SUB_ARDUINO_ADDR 8      // 안내방송용 두 번째 아두이노 I2C 주소

// ---------- 핀 ----------
#define US1_TRIG  2
#define US1_ECHO  3
#define US2_TRIG  4
#define US2_ECHO  5
#define DHTPIN    6
#define BUZZER    7      // 능동 피에조
#define MOTOR     8      // 환기 DC모터 (반드시 트랜지스터 + 별도 5V)
#define SERVO_PIN 9      // 창문 서보
#define TFT_DC    10
// A0 : USE_TOUCH 1이면 터치 CS, 0이면 TFT RESET 전용핀
#define TFT_CS    A1
#define MQ2       A2
#define LIGHT     A3     // 흰색 LED = 교실 전등
#if USE_TOUCH
  #define T_CS    A0
  #define TFT_RST -1     // 터치를 쓰면 TFT RESET은 보드 RESET핀에 연결
#else
  #define TFT_RST A0     // ★ 터치 대신 A0를 TFT RESET으로 사용 (TFT의 RESET선을 A0로 옮기세요)
#endif
//  SPI: D11(MOSI)/D12(MISO)/D13(SCK) — TFT+터치 공유 / T_IRQ 미연결 / TFT의 SD_ 핀은 연결하지 않음
//  I2C: A4(SDA)/A5(SCL) → HuskyLens#1 + BH1750 + 방송용 아두이노 / Serial1(D0/D1): HuskyLens#2

// ---------- 설정 ----------
#define SELFTEST     1     // 1이면 부팅할 때 TFT·LED·부저·모터 자가진단 실행
#define CAL_MODE     0     // 1이면 터치 보정 모드 (USE_TOUCH 1일 때만)
#define TFT_SPI_HZ   8000000  // 화면이 하얗기만 하면 4000000까지 낮춰보세요
#define DHTTYPE      DHT11
#define LUX_DARK     100
#define GAS_BAD      400
#define GAS_SEVERE   700
#define US_NEAR      20
#define FALL_HOLD    1000  // 쓰러진 채 이만큼(1초) 움직임이 없으면 도움요청 → 텔레그램 발송
#define FALL_GAP      700  // ★ 인식이 잠깐 끊겨도 이 시간 안에 다시 잡히면 이어서 셈
#define FALL_ID      2     // HuskyLens#2에서 '쓰러짐(obj2)'으로 학습한 ID
#define LIGHT_USE_CAMERA 0 // 0 = 출입 카운트로만 전등 제어 (1이면 카메라가 봐도 ON)
#define USE_BH1750       0 // 조도센서 사용 안 함 (1로 바꾸면 사용)
#define BUZZER_ACTIVE 1

// ---------- WiFi / 텔레그램 ----------
const char* WIFI_SSID = "여기에_WiFi이름";
const char* WIFI_PASS = "여기에_WiFi비번";
const char* BOT_TOKEN = "여기에_봇토큰";
const char* CHAT_ID   = "여기에_ChatID";
bool wifiOK=false;

HUSKYLENS huskyFire;     // #1 화재(색 인식) — I2C
HUSKYLENS huskyPeople;   // #2 사람(서있음/쓰러짐) — Serial1
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);
#if USE_TOUCH
  XPT2046_Touchscreen ts(T_CS);
#endif
DHT dht(DHTPIN, DHTTYPE);
ArduinoLEDMatrix matrix;

int   g_ppl=0, g_gas=0, g_level=0, cctvMode=0, g_inside=0, g_temp=0, g_humi=0;
float g_lux=0;
int   g_box[5][4];
bool  g_fan=false, g_win=true, g_alarm=false, g_light=false;
bool  g_camfire=false, g_fall=false, g_help=false, g_fallRaw=false;
bool  hl1ok=false, hl2ok=false;
unsigned long helpT=0, fallStart=0, fallSeen=0, dhtT=0, touchLock=0, dbgT=0, motorTestUntil=0;
int   lastLevel=-1, lastPpl=-1, lastGas=-1, lastTH=-1, lastMatrix=-1;
bool  us1_prev=false, us2_prev=false;
unsigned long us1_t=0, us2_t=0;
bool  sentFire=false, sentFall=false, sentHelp=false;

// 내장 12x8 매트릭스 : 정상=전부 꺼짐 / 긴급=전부 켜짐(점멸)
const uint32_t FR_OFF[3]  = {0x00000000,0x00000000,0x00000000};
const uint32_t FR_FIRE[3] = {0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF};

// ===== BH1750 =====
#define BH1750_ADDR 0x23
void bh1750Begin(){ Wire.beginTransmission(BH1750_ADDR); Wire.write(0x10); Wire.endTransmission(); }
float bh1750Lux(){
  if (Wire.requestFrom(BH1750_ADDR, 2) != 2) return 0;
  uint8_t hi=Wire.read(), lo=Wire.read();
  return (((uint16_t)hi<<8)|lo)/1.2f;
}

// ===== 부저 / 서보 =====
void buzzerOn(){  if(BUZZER_ACTIVE) digitalWrite(BUZZER,HIGH); else tone(BUZZER,2300); }
void buzzerOff(){ if(BUZZER_ACTIVE) digitalWrite(BUZZER,LOW);  else noTone(BUZZER); }
void servoWrite(int angle){
  int pulse=map(angle,0,180,500,2500);
  for(int i=0;i<20;i++){ digitalWrite(SERVO_PIN,HIGH); delayMicroseconds(pulse); digitalWrite(SERVO_PIN,LOW); delay(20); }
}

// ===== 방송용 아두이노에 신호 (1=화재, 2=쓰러짐, 0=해제) =====
void speakerCmd(uint8_t code){
  Wire.beginTransmission(SUB_ARDUINO_ADDR);
  Wire.write(code);
  Wire.endTransmission();
}

// ===== 초음파 (등 맞대고 서로 반대 방향으로 붙인 배치 지원) =====
//  US1 = 바깥(복도)을 보는 센서 / US2 = 안(교실)을 보는 센서
//  들어올 때 : 복도쪽(US1)에 먼저 보였다가 → 교실쪽(US2)에 보임  → 입장 +1
//  나갈 때   : 교실쪽(US2) 먼저 → 복도쪽(US1)                     → 퇴장 -1
#define US_DETECT_MAX 90      // 이 거리(cm) 안에 사람이 들어오면 '감지'로 봄
#define BLOCK_MARGIN  20      // 벽까지 거리가 잡히는 경우, 기준선보다 이만큼 가까우면 감지
#define SEQ_MS        3000    // 두 센서를 차례로 지나는 데 허용하는 시간
#define COOL_MS       900     // 한 번 세고 나서 쉬는 시간(중복 카운트 방지)

long baseA=999, baseB=999;    // 아무도 없을 때 거리(부팅 시 자동 측정, 벽 없으면 999)
long dA=999, dB=999;
unsigned long coolUntil=0;

long usDist(int trig,int echo){
  digitalWrite(trig,LOW); delayMicroseconds(2);
  digitalWrite(trig,HIGH); delayMicroseconds(10); digitalWrite(trig,LOW);
  long dur=pulseIn(echo,HIGH,25000); if(dur==0) return 999; return dur*0.034/2;
}
void calibrateUS(){           // 문간에 아무도 없는 상태에서 기준 거리 측정
  long sa=0,sb=0; int na=0,nb=0;
  for(int i=0;i<9;i++){
    long a=usDist(US1_TRIG,US1_ECHO); if(a<400){sa+=a;na++;} delay(45);
    long b=usDist(US2_TRIG,US2_ECHO); if(b<400){sb+=b;nb++;} delay(45);
  }
  baseA = na? sa/na : 999;
  baseB = nb? sb/nb : 999;
  Serial.print("#US 기준거리 A(바깥)="); Serial.print(baseA);
  Serial.print("cm  B(안쪽)=");          Serial.print(baseB);
  Serial.print("cm  감지범위=");          Serial.print(US_DETECT_MAX); Serial.println("cm 이내");
}
bool detected(long d,long base){
  if(d>=400) return false;                          // 에코 없음 = 아무것도 없음
  if(d < US_DETECT_MAX) return true;                // 감지 범위 안에 들어옴
  if(base<400 && d < base-BLOCK_MARGIN) return true; // 벽이 보이는 경우: 기준선보다 가까움
  return false;
}
void updateEntry(){
  dA = usDist(US1_TRIG,US1_ECHO); delay(15);        // 번갈아 측정(서로 간섭 방지)
  dB = usDist(US2_TRIG,US2_ECHO);
  bool a=detected(dA,baseA), b=detected(dB,baseB);
  bool aRise = a && !us1_prev, bRise = b && !us2_prev;
  unsigned long now=millis();

  if(aRise){
    if(us2_t && now-us2_t<SEQ_MS && now>coolUntil){  // 안쪽 먼저 → 바깥 = 퇴장
      if(g_inside>0) g_inside--;
      us1_t=0; us2_t=0; coolUntil=now+COOL_MS;
    } else us1_t=now;
  }
  if(bRise){
    if(us1_t && now-us1_t<SEQ_MS && now>coolUntil){  // 바깥 먼저 → 안쪽 = 입장
      g_inside++;
      us1_t=0; us2_t=0; coolUntil=now+COOL_MS;
    } else us2_t=now;
  }
  if(us1_t && now-us1_t>SEQ_MS) us1_t=0;             // 혼자 지나간 신호는 만료
  if(us2_t && now-us2_t>SEQ_MS) us2_t=0;
  us1_prev=a; us2_prev=b;
}

// ===== HuskyLens =====
void readFire(){                       // #1 : 학습한 색(불)이 보이면 화재
  g_camfire=false;
  hl1ok = huskyFire.request();
  if(hl1ok){
    while(huskyFire.available()){
      HUSKYLENSResult r=huskyFire.read();
      if(r.command==COMMAND_RETURN_BLOCK) g_camfire=true;
    }
  }
}
int readPeople(){                      // #2 : 사람 위치 + 쓰러짐(ID2)
  int n=0; bool fallNow=false;
  hl2ok = huskyPeople.request();
  if(hl2ok){
    while(huskyPeople.available() && n<5){
      HUSKYLENSResult r=huskyPeople.read();
      if(r.command==COMMAND_RETURN_BLOCK){
        if(r.ID==FALL_ID) fallNow=true;
        g_box[n][0]=r.xCenter; g_box[n][1]=r.yCenter; g_box[n][2]=r.width; g_box[n][3]=r.height; n++;
      }
    }
  }
  g_fallRaw = fallNow;
  unsigned long now=millis();
  if(fallNow) fallSeen=now;
  // ★ 인식이 깜빡여도 FALL_GAP 안에 다시 보이면 '계속 쓰러진 상태'로 이어서 카운트
  bool still = (fallSeen && now-fallSeen < FALL_GAP);
  if(still){ if(fallStart==0) fallStart=now; g_fall = (now-fallStart >= FALL_HOLD); }
  else { fallStart=0; fallSeen=0; g_fall=false; }
  return n;
}

// ---------- 화재 판정 (불꽃 + 가스 동시) ----------
#define FIRE_REQUIRE_BOTH 1   // 1 = 불꽃(색)과 가스가 함께 잡혀야 '화재'. 0이면 하나만 잡혀도 화재
#define GAS_FIRE     400      // 화재로 인정할 가스 최소값
#define FIRE_WINDOW  3000     // 불꽃과 가스가 이 시간(ms) 안에 겹치면 '동시'로 인정
unsigned long camSeen=0;
bool g_camRecent=false, g_gasFire=false;

int computeLevel(){
  unsigned long now=millis();
  if(g_camfire) camSeen=now;
  g_camRecent = (camSeen && now-camSeen < FIRE_WINDOW);   // 불꽃이 방금 보였는가
  g_gasFire   = (g_gas >= GAS_FIRE);                       // 가스가 화재 수준인가
#if FIRE_REQUIRE_BOTH
  if(g_camRecent && g_gasFire) return 2;                   // ★ 둘 다 → 화재(긴급)
  if(g_camRecent || g_gas >= GAS_BAD) return 1;            // 하나만 → 주의
  return 0;
#else
  if(g_camRecent || g_gas >= GAS_SEVERE) return 2;
  if(g_gas >= GAS_BAD) return 1;
  return 0;
#endif
}
void applyOutputs(int lv){
  g_fan   = (lv>=1);
  g_win   = (lv==0); servoWrite(g_win?90:0);
  g_alarm = (lv>=2); if(g_alarm) buzzerOn(); else buzzerOff();
}

// ===== 텔레그램 메시지 (한글은 미리 URL 인코딩된 문자열) =====
const char* TG_FIRE_A = "%F0%9F%94%A5%20%5Bsmart-SGT%5D%20%ED%99%94%EC%9E%AC%20%EB%B0%9C%EC%83%9D%0A%EA%B5%90%EC%8B%A4%3A%20";
const char* TG_FIRE_B = "%0A%EB%B6%88%EA%BD%83%EA%B3%BC%20%EA%B0%80%EC%8A%A4%EA%B0%80%20%EB%8F%99%EC%8B%9C%EC%97%90%20%EA%B0%90%EC%A7%80%EB%90%98%EC%97%88%EC%8A%B5%EB%8B%88%EB%8B%A4.%0A%EC%A6%89%EC%8B%9C%20%EB%8C%80%ED%94%BC%ED%95%98%EA%B3%A0%20119%EC%97%90%20%EC%8B%A0%EA%B3%A0%ED%95%98%EC%84%B8%EC%9A%94.%0A%EC%86%8C%EB%B0%A9%EC%84%9C%20%EC%B0%BE%EA%B8%B0%3A%20https%3A%2F%2Fmap.kakao.com%2F%3Fq%3D%EC%86%8C%EB%B0%A9%EC%84%9C%0A%EC%9D%91%EA%B8%89%EC%8B%A4%20%EC%B0%BE%EA%B8%B0%3A%20https%3A%2F%2Fmap.kakao.com%2F%3Fq%3D%EC%9D%91%EA%B8%89%EC%8B%A4";
const char* TG_FALL_A = "%F0%9F%9A%A8%20%5Bsmart-SGT%5D%20%EB%82%99%EC%83%81%20%EA%B0%90%EC%A7%80%0A%EA%B5%90%EC%8B%A4%3A%20";
const char* TG_FALL_B = "%0A%EC%93%B0%EB%9F%AC%EC%A7%84%20%EC%B1%84%201%EC%B4%88%20%EC%9D%B4%EC%83%81%20%EC%9B%80%EC%A7%81%EC%9E%84%EC%9D%B4%20%EC%97%86%EC%8A%B5%EB%8B%88%EB%8B%A4.%0A%EC%A6%89%EC%8B%9C%20%ED%99%95%EC%9D%B8%EC%9D%B4%20%ED%95%84%EC%9A%94%ED%95%98%EB%A9%B0%20119%20%EC%8B%A0%EA%B3%A0%EA%B0%80%20%ED%95%84%EC%9A%94%ED%95%A0%20%EC%88%98%20%EC%9E%88%EC%8A%B5%EB%8B%88%EB%8B%A4.%0A%EC%A3%BC%EB%B3%80%20%EC%9D%91%EA%B8%89%EC%8B%A4%3A%20https%3A%2F%2Fmap.kakao.com%2F%3Fq%3D%EC%9D%91%EA%B8%89%EC%8B%A4";
const char* TG_HELP_A = "%F0%9F%86%98%20%5Bsmart-SGT%5D%20%EB%8F%84%EC%9B%80%20%EC%9A%94%EC%B2%AD%0A%EA%B5%90%EC%8B%A4%3A%20";
const char* TG_HELP_B = "%0A%EA%B5%90%EC%8B%A4%20%EB%8B%A8%EB%A7%90%EC%97%90%EC%84%9C%20%EB%8F%84%EC%9B%80%20%EC%9A%94%EC%B2%AD%20%EB%B2%84%ED%8A%BC%EC%9D%B4%20%EB%88%8C%EB%A0%B8%EC%8A%B5%EB%8B%88%EB%8B%A4.%20%ED%99%95%EC%9D%B8%20%EB%B0%94%EB%9E%8D%EB%8B%88%EB%8B%A4.";
String tgMsg(const char* a, const char* b){ return String(a) + CLASS_ID + String(b); }

void tgSend(String msg){
  if(!wifiOK) return;
  WiFiSSLClient c;
  if(c.connect("api.telegram.org",443)){
    String url="/bot"+String(BOT_TOKEN)+"/sendMessage?chat_id="+String(CHAT_ID)+"&text="+msg;
    c.print("GET "+url+" HTTP/1.1\r\nHost: api.telegram.org\r\nConnection: close\r\n\r\n");
    unsigned long t=millis(); while(c.connected() && millis()-t<3000){ while(c.available()) c.read(); } c.stop();
  }
}

// ===== TFT + 터치 =====
#define BTN_Y 250
const char* levelText(int lv){ return lv==2?"FIRE!":lv==1?"CAUTION":"SAFE"; }
uint16_t levelColor(int lv){ return lv==2?ILI9341_RED:lv==1?ILI9341_ORANGE:ILI9341_GREEN; }
const char* modeText(int m){ return m==2?"LOCK":m==1?"REC":"PRIVACY"; }
void drawRow(int y,const char* label){ tft.setTextColor(0x8410); tft.setTextSize(1); tft.setCursor(12,y); tft.print(label); }

void drawButtons(bool helpActive){
  tft.fillRect(0, BTN_Y, 118, 320-BTN_Y, 0x3186);
  tft.setTextColor(ILI9341_WHITE); tft.setTextSize(1); tft.setCursor(15, BTN_Y+15); tft.print("CCTV MODE");
  tft.setTextColor(0x07FF); tft.setTextSize(2); tft.setCursor(35, BTN_Y+35); tft.print(modeText(cctvMode));
  tft.fillRect(122, BTN_Y, 118, 320-BTN_Y, helpActive?ILI9341_GREEN:ILI9341_RED);
  tft.setTextColor(ILI9341_WHITE); tft.setTextSize(2); tft.setCursor(140, BTN_Y+25);
  tft.print(helpActive?"SENT!":"HELP");
}
void drawTFTStatic(){
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE); tft.setTextSize(2); tft.setCursor(10,10); tft.print("smart-SGT");
  tft.setTextColor(0x8410); tft.setTextSize(1); tft.setCursor(10,32); tft.print("Classroom Safety System");
  tft.drawFastHLine(0,46,240,0x4208);
  tft.setTextColor(ILI9341_YELLOW); tft.setTextSize(2); tft.setCursor(10,115); tft.print("IN ROOM:");
  drawRow(150,"Temp / Humi:");
  drawRow(180,"Gas Air:");
  drawRow(210,"Light:");
  drawButtons(false);
}
void updateTFT(){
  if(g_level!=lastLevel){
    tft.fillRect(0,52,240,48,levelColor(g_level));
    tft.setTextColor(ILI9341_WHITE); tft.setTextSize(3); tft.setCursor(15,64); tft.print(levelText(g_level));
    lastLevel=g_level;
  }
  if(g_inside!=lastPpl){
    tft.fillRect(120,110,110,30,ILI9341_BLACK);
    tft.setTextColor(ILI9341_CYAN); tft.setTextSize(3); tft.setCursor(120,112);
    tft.print(g_inside); tft.setTextSize(2); tft.print(" P");
    lastPpl=g_inside;
  }
  if(g_temp*100+g_humi!=lastTH){
    tft.fillRect(120,148,110,20,ILI9341_BLACK);
    tft.setTextColor(ILI9341_WHITE); tft.setTextSize(2); tft.setCursor(120,148);
    tft.print(g_temp); tft.print("C / "); tft.print(g_humi); tft.print("%");
    lastTH=g_temp*100+g_humi;
  }
  if(g_gas!=lastGas){
    tft.fillRect(120,178,110,20,ILI9341_BLACK);
    tft.setTextColor(ILI9341_WHITE); tft.setTextSize(2); tft.setCursor(120,178); tft.print(g_gas);
    lastGas=g_gas;
  }
  tft.fillRect(120,208,110,20,ILI9341_BLACK);
  tft.setTextSize(2); tft.setTextColor(g_light?ILI9341_GREEN:0x8410);
  tft.setCursor(120,208); tft.print(g_light?"ON":"OFF");
}
void mapTouch(int rawX,int rawY,int &sx,int &sy){   // ⚠ CAL_MODE로 raw값 확인 후 숫자 조정
  sx = map(rawX, 200, 3900, 0, 240);
  sy = map(rawY, 200, 3900, 0, 320);
}
void handleTouch(){
#if !USE_TOUCH
  return;
#else
  if(!ts.touched()) return;
  if(millis()-touchLock<400) return;
  TS_Point p=ts.getPoint(); int sx,sy; mapTouch(p.x,p.y,sx,sy);
  touchLock=millis();
  if(sy < BTN_Y) return;
  if(sx < 118){ cctvMode=(cctvMode+1)%3; drawButtons(false); }
  else { g_help=true; helpT=millis(); drawButtons(true); }
#endif
}

// ===== 내장 매트릭스 (긴급 시 전체 점멸) =====
void updateMatrix(int lv){
  static unsigned long mt=0; static bool on=false;
  if(lv==2){ if(millis()-mt>400){ mt=millis(); on=!on; matrix.loadFrame(on?FR_FIRE:FR_OFF); } lastMatrix=2; }
  else if(lastMatrix!=lv){ lastMatrix=lv; matrix.loadFrame(FR_OFF); }
}

// ===== 부팅 자가진단 (배선 확인용) =====
void selfTest(){
  tft.fillScreen(ILI9341_RED);   delay(350);
  tft.fillScreen(ILI9341_GREEN); delay(350);
  tft.fillScreen(ILI9341_BLUE);  delay(350);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE); tft.setTextSize(2); tft.setCursor(10,20); tft.print("SELF TEST");
  tft.setTextSize(1); tft.setTextColor(0x07FF);
  tft.setCursor(10,60); tft.print("LED ...");    for(int i=0;i<3;i++){ digitalWrite(LIGHT,HIGH); delay(180); digitalWrite(LIGHT,LOW); delay(180); }
  tft.setCursor(10,80); tft.print("BUZZER ...");  buzzerOn(); delay(250); buzzerOff();
  tft.setCursor(10,100); tft.print("MOTOR 1s ..."); digitalWrite(MOTOR,HIGH); delay(1000); digitalWrite(MOTOR,LOW);
  tft.setCursor(10,120); tft.print("MATRIX ...");  matrix.loadFrame(FR_FIRE); delay(500); matrix.loadFrame(FR_OFF);
  delay(400);
}

// ===== USB / 대시보드 =====
String getState(){
  const char* lvStr = g_level==2?"emergency":g_level==1?"caution":"safe";
  String j="{\"class\":\""; j+=CLASS_ID; j+="\"";
  j+=",\"ppl\":"; j+=g_ppl;
  j+=",\"inside\":"; j+=g_inside;
  j+=",\"box\":[";
  for(int i=0;i<g_ppl && i<5;i++){ j+="["; j+=g_box[i][0]; j+=","; j+=g_box[i][1]; j+=","; j+=g_box[i][2]; j+=","; j+=g_box[i][3]; j+="]"; if(i<g_ppl-1 && i<4) j+=","; }
  j+="]";
  j+=",\"lux\":";j+=String(g_lux,0); j+=",\"air\":";j+=g_gas; j+=",\"temp\":";j+=g_temp; j+=",\"humi\":";j+=g_humi;
  j+=",\"mode\":";j+=cctvMode; j+=",\"level\":\"";j+=lvStr; j+="\"";
  j+=",\"fan\":";j+=(g_fan?1:0); j+=",\"window\":";j+=(g_win?1:0); j+=",\"alarm\":";j+=(g_alarm?1:0);
  j+=",\"light\":";j+=(g_light?1:0); j+=",\"fall\":";j+=(g_fall?1:0); j+=",\"help\":";j+=(g_help?1:0);
  j+=",\"hl1\":";j+=(hl1ok?1:0); j+=",\"hl2\":";j+=(hl2ok?1:0);
  j+=",\"cam\":";j+=(g_camRecent?1:0); j+=",\"gasfire\":";j+=(g_gasFire?1:0);
  j+="}"; return j;
}
int setActuator(String cmd){
  if      (cmd=="WIN_OPEN")   { servoWrite(90); g_win=true; }
  else if (cmd=="WIN_CLOSE")  { servoWrite(0);  g_win=false; }
  else if (cmd=="FAN_ON")     { g_fan=true; }
  else if (cmd=="FAN_OFF")    { g_fan=false; }
  else if (cmd=="ALARM_ON")   { buzzerOn();  g_alarm=true; }
  else if (cmd=="ALARM_OFF")  { buzzerOff(); g_alarm=false; }
  else if (cmd=="LIGHT_ON")   { digitalWrite(LIGHT,HIGH); }
  else if (cmd=="LIGHT_OFF")  { digitalWrite(LIGHT,LOW); }
  else if (cmd=="RESET_CNT")  { g_inside=0; }
  else if (cmd=="TEST_MOTOR") { motorTestUntil=millis()+2000; }   // 모터 2초 강제 회전
  else if (cmd=="TEST_LED")   { for(int i=0;i<6;i++){ digitalWrite(LIGHT,i%2); delay(200);} }
  else if (cmd=="CAL_US")     { calibrateUS(); }                 // 초음파 기준거리 다시 측정
  return 1;
}

// ===== 터치 보정 모드 =====
void calLoop(){
#if USE_TOUCH
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE); tft.setTextSize(2); tft.setCursor(10,10); tft.print("TOUCH CAL");
  tft.setTextSize(1); tft.setTextColor(0x8410); tft.setCursor(10,40); tft.print("Touch 4 corners - note raw x,y");
  while(true){
    if(ts.touched()){
      TS_Point p=ts.getPoint();
      tft.fillRect(0,80,240,60,ILI9341_BLACK);
      tft.setTextColor(0x07FF); tft.setTextSize(2); tft.setCursor(10,90);
      tft.print("X:"); tft.print(p.x); tft.print(" Y:"); tft.print(p.y);
      Serial.print("RAW X:"); Serial.print(p.x); Serial.print("  Y:"); Serial.println(p.y);
    }
    delay(120);
  }
#endif
}

void setup(){
  Serial.begin(115200); Serial.setTimeout(20);

  // SPI 장치 CS를 먼저 비활성(HIGH)으로 고정 — TFT 백화 방지
#if USE_TOUCH
  pinMode(T_CS,OUTPUT);   digitalWrite(T_CS,HIGH);
#endif
  pinMode(TFT_CS,OUTPUT); digitalWrite(TFT_CS,HIGH);

  pinMode(US1_TRIG,OUTPUT); pinMode(US1_ECHO,INPUT);
  pinMode(US2_TRIG,OUTPUT); pinMode(US2_ECHO,INPUT);
  pinMode(BUZZER,OUTPUT); pinMode(MOTOR,OUTPUT); pinMode(LIGHT,OUTPUT);
  digitalWrite(MOTOR,LOW); digitalWrite(LIGHT,LOW); buzzerOff();
  pinMode(SERVO_PIN,OUTPUT); servoWrite(90); g_win=true;

  SPI.begin();
  tft.begin(TFT_SPI_HZ);
  tft.setRotation(0);
#if USE_TOUCH
  ts.begin(); ts.setRotation(0);
#endif
  matrix.begin();

  if(CAL_MODE){ calLoop(); }
  if(SELFTEST){ selfTest(); }

  drawTFTStatic();

  Wire.begin();
  huskyFire.begin(Wire);
  huskyFire.writeAlgorithm(ALGORITHM_COLOR_RECOGNITION);          // #1 불 = 색 인식
  Serial1.begin(9600);
  huskyPeople.begin(Serial1);
  huskyPeople.writeAlgorithm(ALGORITHM_OBJECT_CLASSIFICATION);    // #2 서있음/쓰러짐
#if USE_BH1750
  bh1750Begin();
#endif
  dht.begin();
  calibrateUS();          // 문간에 아무도 없는 상태에서 기준거리 자동 측정

  if(String(WIFI_SSID)!="여기에_WiFi이름"){
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    for(int i=0;i<20 && WiFi.status()!=WL_CONNECTED;i++) delay(500);
    wifiOK=(WiFi.status()==WL_CONNECTED);
  }
}

void loop(){
  // 1) 카메라 · 센서 · 출입
  readFire();
  g_ppl = readPeople();
  updateEntry();
#if USE_BH1750
  g_lux = bh1750Lux();
#endif
  g_gas = analogRead(MQ2);
  if(millis()-dhtT>2000){ dhtT=millis(); float t=dht.readTemperature(), h=dht.readHumidity();
    if(!isnan(t)) g_temp=(int)t; if(!isnan(h)) g_humi=(int)h; }

  // 2) TFT 터치 (HELP / CCTV 모드)
  handleTouch();
  if(g_help && millis()-helpT>3000){ g_help=false; drawButtons(false); }

  // 3) 위험도 판정 + 방송 아두이노 신호
  int lv=computeLevel();
  if(lv!=g_level){ g_level=lv; applyOutputs(lv); speakerCmd(lv>=2 ? 1 : 0); }
  updateMatrix(lv);

  // 쓰러짐이 확정되는 순간(1초 지속) 방송 신호 2 전송
  static bool lastFallState=false;
  if(g_fall!=lastFallState){ if(g_fall) speakerCmd(2); lastFallState=g_fall; }

  // 4) 교실 전등 : 실내에 사람이 1명 이상일 때만 ON
  bool present = (g_inside>=1);
#if LIGHT_USE_CAMERA
  present = present || (g_ppl>=1);
#endif
  g_light = present;
  digitalWrite(LIGHT, g_light?HIGH:LOW);

  // 5) 환기 모터 (자동 g_fan + TEST_MOTOR 강제회전) — 매 루프 상태 재적용
  bool motorOn = g_fan || (millis() < motorTestUntil);
  digitalWrite(MOTOR, motorOn?HIGH:LOW);

  // 6) TFT 갱신
  updateTFT();

  // 7) 텔레그램 (각 상황당 1회)
  if(lv==2 && !sentFire){ tgSend(tgMsg(TG_FIRE_A,TG_FIRE_B)); sentFire=true; } if(lv<2) sentFire=false;
  if(g_fall && !sentFall){ tgSend(tgMsg(TG_FALL_A,TG_FALL_B)); sentFall=true; } if(!g_fall) sentFall=false;
  if(g_help && !sentHelp){ tgSend(tgMsg(TG_HELP_A,TG_HELP_B)); sentHelp=true; } if(!g_help) sentHelp=false;

  // 8) USB 시리얼 (명령 수신 / 상태 송신 / 진단)
  if(Serial.available()){ String c=Serial.readStringUntil('\n'); c.trim(); if(c.length()) setActuator(c); }
  static unsigned long lastTx=0;
  if(millis()-lastTx>500){ lastTx=millis(); Serial.println(getState()); }

  if(millis()-dbgT>2000){ dbgT=millis();
    Serial.print("#DBG HL1(fire)="); Serial.print(hl1ok?"OK":"NO");
    Serial.print(" flame="); Serial.print(g_camRecent?1:0);
    Serial.print(" gasFire="); Serial.print(g_gasFire?1:0);
    Serial.print(" lv="); Serial.print(g_level);
    Serial.print(" | HL2(people)="); Serial.print(hl2ok?"OK":"NO");
    Serial.print(" ppl="); Serial.print(g_ppl);
    Serial.print(" fallRaw="); Serial.print(g_fallRaw?1:0);
    Serial.print(" fallHold="); Serial.print(fallStart? (millis()-fallStart) : 0);
    Serial.print("ms fall="); Serial.print(g_fall?1:0);
    Serial.print(" | US A="); Serial.print(dA); Serial.print("cm");
    Serial.print(" B="); Serial.print(dB); Serial.print("cm");
    Serial.print(" inside="); Serial.print(g_inside);
    Serial.print(" gas="); Serial.print(g_gas);
    Serial.print(" temp="); Serial.print(g_temp);
    Serial.print(" light="); Serial.print(g_light?1:0);
    Serial.print(" fan="); Serial.println(g_fan?1:0);
  }

  delay(60);
}
