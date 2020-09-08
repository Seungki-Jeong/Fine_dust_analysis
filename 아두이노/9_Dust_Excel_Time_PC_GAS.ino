/*
 *  이 소스는 에듀이노(Eduino)에 의해서 번역, 수정, 작성되었고 소유권 또한 에듀이노의 것입니다. 
 *  소유권자의 허락을 받지 않고 무단으로 수정, 삭제하여 배포할 시 법적인 처벌을 받을 수 있습니다. 
 *  
 *  에듀이노 미세먼지 키트 기본예제 
 *  - 미세먼지 키트 활용예제 -
 *  미세먼지 측정 센서,온습도센서의 값을 LCD 1602(I2C)와 RGB LED 모듈로 출력하는 예제 코드입니다.
 *    
 */
 //GAS 센서 MQ-135 사용

 // PLX-DAQ v2.11 사용법
 // 1. 엑셀파일(PLX-DAQ-v2.11.xlsm) 실행
 // 2. (실행여부 묻는 창 뜰경우) 실행
 // 3. (UI가 뜨지 않는 경우) Open PLX DAQ UI 실행
 // 4. 아두이노 연결한 포트 확인 후 Port: 에 번호 입력.
 // 5. 데이터 기록할 Sheet 선택 (Sheet name to post to:)
 // 6. 아두이노 프로그램의 시리얼 모니터 끄기(열어뒀을 경우 해당하며 엑셀 실행불가.)
 // 7. Connect
 // 8. Display direct debug => 선택하여 기록상태 실시간 확인.

#include <Wire.h>                        // i2C 통신을 위한 라이브러리
#include <LiquidCrystal_I2C.h>        // LCD 1602 I2C용 라이브러리
#include <DHT.h>                      // 온습도 센서 사용을 위한 라이브러리

// GAS Sensor MQ-135 Start

/// The load resistance on the board----------------------------
#define RLOAD 10.0
/// Calibration resistance at atmospheric CO2 level
#define RZERO 76.63
/// Parameters for calculating ppm of CO2 from sensor resistance
#define PARA 116.6020682
#define PARB 2.769034857

// Gas Sensor MQ-135 Finish-------------------------------------

#define DHTPIN A1 // 온습도 센서 핀 지정
#define DHTTYPE DHT11 // DHT 타입 지정
DHT dht(DHTPIN, DHTTYPE); // DHT11의 타입, 핀을 dht로 지정

int dust_sensor = A0;   // 미세먼지 핀 설정

int rgb_red = 5;    // rgb 핀 빨간색 핀
int rgb_green = 6;  // rgb핀 녹색색 핀
int rgb_blue = 7;  // rgb핀 파란색 핀

float dust_value = 0;  // 센서에서 입력받은 미세먼지 값
float dustDensityug = 0;  // ug/m^3 값을 계산

int sensor_led = 12;      // 미세먼지 센서 안에 있는 적외선 led 핀 번호
int sampling = 280;       // 적외선 led를 키고, 센서값을 읽어들여 미세먼지를 측정하는 샘플링 시간
int waiting = 40;    
float stop_time = 9680;   // 센서를 구동하지 않는 시간

double five_dust[5] = {0};
double recent_dust = 0, total_dust = 0;

LiquidCrystal_I2C lcd(0x27,16,2);     // 접근주소: 0x3F or 0x27 1602 Display

 byte humi[8] = {     // 물컵모양 출력
  0b00000,
  0b10001,
  0b10001,
  0b10001,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
};
 byte temp[8] = {     // 온도계 모양 출력
  0b00100,
  0b01010,
  0b01010,
  0b01010,
  0b01010,
  0b10001,
  0b11111,
  0b01110,
};
 byte char_temp[8] = {     // 온도 단위 출력
  0b10000,
  0b00110,
  0b01001,
  0b01000,
  0b01000,
  0b01000,
  0b01001,
  0b00110,
};

void setup(){
  lcd.init();                     // LCD 초기화
  lcd.backlight();                // 백라이트 켜기
  lcd.createChar(1, temp);          // 온도계모양 출력
  lcd.createChar(2, humi);          // 물컵 모양 출력
  lcd.createChar(3, char_temp);     // 온도 단위 출력
  pinMode(sensor_led,OUTPUT);     // 미세먼지 적외선 led를 출력으로 설정
  pinMode(4, OUTPUT);
  
  pinMode(rgb_red, OUTPUT);     // 3색 LED 모듈 출력으로 설정, 붉은색
  pinMode(rgb_green, OUTPUT);   // 녹색
  pinMode(rgb_blue, OUTPUT);    // 파란색
  
  Serial.begin(9600);            // 시리얼 모니터 시작, 속도는 9600  

  Serial.println("CLEARDATA");
  Serial.println("LABEL,TIME,TIMER,Temperature,Humidity,Dust,GAS[PPM]");
  //LABEL 이후에 작성되는 ROW 항목들은 10가지 작성 가능.
  //https://forums.parallax.com/discussion/136500/basic-plx-daq-operation
}


void loop(){
  
  delay(1000);
  
  digitalWrite(sensor_led, LOW);    // LED 켜기
  delayMicroseconds(sampling);      // 샘플링해주는 시간. 

  
  int count=0;
  dust_value = analogRead(dust_sensor); // 센서 값 읽어오기
    
  delayMicroseconds(waiting);       // 너무 많은 데이터 입력을 피해주기 위해 잠시 멈춰주는 시간. 
  
  digitalWrite(sensor_led, HIGH);   // LED 끄기
  delayMicroseconds(stop_time);     // LED 끄고 대기  
  
  recent_dust = (0.17 * (dust_value * (5.0 / 1024)) - 0.1) * 1000;    // 미세먼지 값 계산
  five_dust[4] = recent_dust;   // 새로운 미세먼지 값 입력
  total_dust = five_dust[4];               // 5개의 미세먼지 값을 저장할 변수
  
  for(int i=0; i<4; i++)
  {
    total_dust += five_dust[i];
    five_dust[i] = five_dust[i+1];  // 0~4번째까지 미세먼지 값 저장을 위해 4번째 배열 비워주기
  }

  if(five_dust[0] != 0)
  {
    dustDensityug = total_dust / 5;
    //Serial.print("Dust Density [ug/m^3]: ");            // 시리얼 모니터에 미세먼지 값 출력    
    //Serial.println(dustDensityug);
  }

//GAS Coding
  int GAS_VAL = analogRead(2);
  GAS_VAL = (1023./(float)GAS_VAL) * 5. - 1.* RLOAD;
  float Resistance;
  Resistance = GAS_VAL;

  float PPM;
  PPM = PARA * pow((Resistance/RZERO), -PARB);
//
 
  int humi = dht.readHumidity();
  int temp = dht.readTemperature();
  
    //Serial.print("humidity:");          // ‘시리얼 플로터’ 사용위해 이부분 주석 필요
    //Serial.print(humi);                  // 습도값 출력
    //Serial.print("\t temperature:");       // ‘시리얼 플로터’ 사용위해 이부분 주석 필요
    //Serial.println(temp);                  // 온도값 출력 
   
 //    Serial.println("LABEL,TIME,TIMER,Temperature,Humidity,Dust,GAS[PPM]"); 형식의 시리얼 출력단 내용
  Serial.print("DATA,TIME,");  //반드시 "DATA,TIME,"이라고 작성해야만 PLX-DAQ 상에서 시간을 적어줌. 아두이노 시리얼상에는 시간 표시되지 않음.
  Serial.print("TIMER,");  //초기화 후 경과한 시간 표시.
  //Serial.print("DATE");
  //Serial.print(",");
  //Serial.print("t");   //t에 저장된 시간 불러오기.
  //Serial.print(",");
  Serial.print(temp);
  Serial.print(",");
  Serial.print(humi);
  Serial.print(",");
  Serial.print(dustDensityug, 4); //미세먼지 농도 소수점 4째자리까지 출력
  Serial.print(",");
  Serial.println(PPM,1);
  
  Serial.println();
  
  lcd.setCursor(0,0);             // 1번째, 1라인  
  lcd.write(byte(1));             // 온도계 출력
  lcd.setCursor(2,0);             // 3번째, 1라인
  lcd.print((int)temp);           // 온도 출력
  lcd.setCursor(5,0);              // 6번째 1라인
  lcd.write(byte(3));             // 온도 단위 출력
  
  lcd.setCursor(8,0);             // 9번째, 1라인
  lcd.write(byte(2));             // 물컵 출력
  lcd.setCursor(10,0);            // 11번째, 1라인
  lcd.print(humi);                // 습도 출력
  lcd.setCursor(13,0);            // 15번째, 1라인
  lcd.print("%");                 // % 출력
  
  lcd.setCursor(0,1);             // 1번째, 2라인
  lcd.print("F.Dust");            // fine dust 글자 출력
  lcd.setCursor(7,1);             // 6번째, 2라인
  lcd.print(dustDensityug);       // 미세먼지 출력
  lcd.setCursor(11,1);
  lcd.print("ug/m3");

  
  if(dustDensityug <= 30.0)       // 대기 중 미세먼지가 좋음 일때 파란색 출력
     light(0, 0, 255);
  else if(30.0 < dustDensityug && dustDensityug <= 80.0)     // 대기 중 미세먼지가 보통 일때 녹색 출력
     light(0, 255, 0);    
  else if (80.0 < dustDensityug && dustDensityug <= 150.0)    // 대기 중 미세먼지가 나쁨 일때 노란색 출력
     light(255, 80, 1);
  else                                                     // 대기 중 미세먼지가 매우 나쁨 일때 빨간색 출력
     light(255, 0, 0);

  
   
  delay(1000);
}

void light(int a, int b, int c){
  analogWrite(rgb_red, a);
  analogWrite(rgb_green, b);
  analogWrite(rgb_blue, c);    
}

void printDigits(int digits){
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}
