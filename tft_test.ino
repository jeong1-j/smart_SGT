/* =========================================================
   TFT 단독 테스트 — 백화(하얀 화면) 원인 찾기용 (3.2" / 2.4" SPI 모듈)
   ● 이 스케치는 TFT만 씁니다. HuskyLens·터치·센서 전부 뺐습니다.
   ● 다른 부품 선은 전부 뽑고, TFT 선만 꽂은 상태로 업로드하세요.
   ● 시리얼 모니터 115200 으로 열어서 진단값을 확인하세요.

   [배선]  CS→A1, DC→D10, RESET→A0, SDI(MOSI)→D11,
          SDO(MISO)→D12, SCK→D13, LED→3.3V, VCC→5V, GND→GND
          ⚠ SD_ 로 시작하는 핀과 터치(T_) 핀은 전부 뽑아두세요.
          ⚠ 5V→3.3V 보호: SCK·SDI·CS·DC·RESET 5개 선에 1kΩ을 직렬로 넣으면 훨씬 안정적입니다.
   ========================================================= */
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

#define TFT_CS   A1
#define TFT_DC   10
#define TFT_RST  A0        // ★ TFT의 RESET선을 A0에 꽂으세요 (백화의 가장 흔한 원인)
// 터치선(T_CS 등)은 이 테스트에선 전부 뽑아두세요

#define SPI_HZ   4000000   // 4MHz로 아주 느리게 시작 (되면 8000000 → 24000000)

Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

void diag(){
  Serial.println(F("---- 드라이버 칩 확인 (가장 중요) ----"));
  Serial.print(F("ID4 (0xD3): "));
  for(uint8_t i=1;i<=3;i++){ Serial.print(F("0x")); Serial.print(tft.readcommand8(0xD3,i),HEX); Serial.print(F(" ")); }
  Serial.println();
  Serial.println(F("  0x0 0x93 0x41  -> ILI9341 (이 코드 그대로 사용 가능)"));
  Serial.println(F("  0x0 0x94 0x86  -> ILI9486 (480x320, 다른 라이브러리 필요)"));
  Serial.println(F("  0x0 0x94 0x88  -> ILI9488 (480x320, 다른 라이브러리 필요)"));
  Serial.println(F("  0x0 0x77 0x96  -> ST7796  (480x320, 다른 라이브러리 필요)"));
  Serial.println(F("  전부 0x0/0xFF  -> 통신 안 됨 (배선/전압) 또는 SPI가 아닌 병렬 모듈"));
  Serial.println();
  Serial.print(F("해상도: ")); Serial.print(tft.width()); Serial.print(F(" x ")); Serial.println(tft.height());
  Serial.println(F("  240 x 320 이 맞습니다 (3.2\" ILI9341도 동일)"));
  Serial.println();
  Serial.println(F("---- TFT 진단값 ----"));
  Serial.print(F("Power Mode  0x")); Serial.println(tft.readcommand8(ILI9341_RDMODE), HEX);
  Serial.print(F("MADCTL      0x")); Serial.println(tft.readcommand8(ILI9341_RDMADCTL), HEX);
  Serial.print(F("Pixel Fmt   0x")); Serial.println(tft.readcommand8(ILI9341_RDPIXFMT), HEX);
  Serial.print(F("Image Fmt   0x")); Serial.println(tft.readcommand8(ILI9341_RDIMGFMT), HEX);
  Serial.print(F("Self Diag   0x")); Serial.println(tft.readcommand8(ILI9341_RDSELFDIAG), HEX);
  Serial.println(F("→ 전부 0x0 또는 0xFF 면 디스플레이와 통신이 안 되는 것(배선·전압 문제)"));
  Serial.println(F("→ 0x9C, 0x48, 0x5, 0x0, 0xC0 같은 값이 섞여 나오면 통신은 되는 것"));
}

void setup(){
  Serial.begin(115200);
  delay(600);
  Serial.println(F("\n== TFT 단독 테스트 시작 =="));

  SPI.begin();
  tft.begin(SPI_HZ);
  tft.setRotation(0);

  diag();

  // 색 채우기 테스트
  const uint16_t cols[5] = {ILI9341_RED, ILI9341_GREEN, ILI9341_BLUE, ILI9341_YELLOW, ILI9341_BLACK};
  const char*   names[5] = {"RED","GREEN","BLUE","YELLOW","BLACK"};
  for(int i=0;i<5;i++){
    Serial.print(F("화면 채움: ")); Serial.println(names[i]);
    tft.fillScreen(cols[i]);
    delay(700);
  }

  // 글자·도형 테스트
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE); tft.setTextSize(3);
  tft.setCursor(15, 30);  tft.print("TFT OK");
  tft.setTextColor(ILI9341_CYAN); tft.setTextSize(2);
  tft.setCursor(15, 75);  tft.print("smart-SGT");
  tft.setTextColor(0x8410); tft.setTextSize(1);
  tft.setCursor(15, 105); tft.print("display test passed");
  tft.drawRect(10, 130, 220, 80, ILI9341_GREEN);
  tft.fillRect(20, 140, 60, 60, ILI9341_RED);
  tft.fillCircle(150, 170, 30, ILI9341_YELLOW);

  Serial.println(F("== 테스트 끝 =="));
  Serial.println(F("화면에 'TFT OK' 가 보이면 성공."));
  Serial.println(F("아직 하얗다면 위의 ID4 값을 확인하세요:"));
  Serial.println(F("  값이 나온다  -> 통신은 됨. SPI_HZ를 2000000으로 낮춰 재시도"));
  Serial.println(F("  전부 0/FF   -> 배선 문제. CS와 DC를 바꿔 꽂았는지 먼저 확인"));
}

void loop(){
  // 1초마다 사각형 색을 바꿔 화면이 살아있는지 확인
  static uint8_t k=0;
  const uint16_t c[4]={ILI9341_RED, ILI9341_GREEN, ILI9341_BLUE, ILI9341_MAGENTA};
  tft.fillRect(20, 230, 200, 30, c[k++ % 4]);
  delay(1000);
}
