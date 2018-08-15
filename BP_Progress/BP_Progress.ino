#include <WOW_Protocol.h>

/*################# 프로토콜의 ID 정의 #################*/
#define PROTOCOL_ID_BP		0x15		 //BP 모듈의 ID
/*####################################################*/

/*################# 프로토콜의 CMD 정의 #################*/
#define PROTOCOL_CMD_BP_REQUEST		0xA0 //BP 데이터 요청
#define PROTOCOL_CMD_BP_RESPONSE	0xB0 //BP 데이터 응답
#define PROTOCOL_CMD_BP_ERROR		0xB1 //오류 사항 응답
/*#####################################################*/

/*################# 프로토콜의 MID 정의 #################*/
#define PROTOCOL_MID_BP_SYS_DATA	0x80 //BP의 SYS 데이터
#define PROTOCOL_MID_BP_DIA_DATA	0x81 //BP의 DIA 데이터
/*#####################################################*/

#define DEBUG_SERIAL	Serial
#define BLE_SERIAL		Serial1

WPacketBase bleTxPacket; //BLE 송신 패킷 객체 생성

#define BLE_SEND_INTERVAL			3000 //BLE 데이터를 전송하는 간격 설정

//디바이스의 전원 제어 포트 정의
#define POWER_CONTROL  A4 // 전원 제어 핀
#define POWER_SWITCH_STATE  A5 // 전원 스위치 상태 핀

// 상태 출력 LED 포트 정의
#define LOWBAT_LED 13 // LOW BAT. LED 핀

/*################# BLE 통신 상태 LED 관련 정의 #################*/
enum STATUS_LED_CONTOL_TYPE{ //LED의 제어유형을 정의한다.
	LED_CONTROL_NORMAL,
	LED_CONTROL_TOGGLE
};
#define LED_ON_MAX_COUNT	80
/**
 * LED의 제어정보를 담은 구조체이다.
 */
typedef struct{
	char enable;	//LED가 켜지는 것을 활성화하는 변수
	int on_count;	//LED가 켜지는 시간을 카운트하는 변수
	char type;		//제어 유형을 저장하는 변수
	char isOn;		//현재 LED가 켜져있는지 저장하는 변수
}led_data_t;

led_data_t led_data; //BLE 통신 LED 관련정보를 선언한다.

#define BT_LED_PIN			12
/*#############################################################*/

#define DATA_INPUT_PIN		A1
#define PUMP_SWITCH_PIN		4
#define VALVE_SWITCH_PIN	5
#define PUMP_MOTOR_PIN		10
#define VALVE_SOL_PIN		11

#define PRESSURE_OFFSET		0		//

/*
 * 2SMPP-02	Span Voltage : 31mV@37kPa
 * OP-AMP gain : 101
 * 1kPa = 7.5mmHg
 * blood_pressure(mmHg) = ADC * (5/1024)*1000*37/31/101*7.5
 */	
#define GAIN_BP				0.43f

#define MAX_SAMPLE_SIZE		128

int sampleData[MAX_SAMPLE_SIZE];

unsigned int guiSampleDataCount = 0;
int giPeakValue = 0;

int giSysData; //(global int)수축 혈압을 저장하는 전역변수
int giDiaData; //(global int)이완 혈압을 저장하는 전역변수

void enableLed();
void ledProcess();
void sendBleData();

/**
 * Low Pass Filter의 줄임말로, 고주파(잡음이 심한 파형)에서
 * 저주파(잡음이 적은 파형)를 필터링하여 추출한다.
 * @param Input             입력 값
 * @param Output            출력 값
 * @param SamplingFrequency 샘플링 주파수
 * @param CutOffFrequency   차단할 주파수
 * @param PastInput         이전 입력 값
 * @param PastOutput        이전 출력 값
 */
void LPF(float *Input, float *Output, float SamplingFrequency, float CutOffFrequency, float *PastInput, float *PastOutput)
{
	float a1,b0,b1,w0;
	
	w0 = 2*3.14*CutOffFrequency;
	a1 = (w0 - 2*SamplingFrequency)/(2*SamplingFrequency + w0);
	b0 = w0/(2*SamplingFrequency + w0);
	b1 = b0;
	
	*Output = b0*(*Input) + b1*(*PastInput) - a1*(*PastOutput);
	*PastOutput = *Output;
	*PastInput = *Input;
}

int detect_peak(
        const int*   data, /* the data */ 
        int          data_count, /* row count of data */ 
        int          delta, /* delta used for distinguishing peaks */
        int& mx, //
        int& mn //
        )
{
    int     i;
    int		mx_pos = 0;
    int		mn_pos = 0;
    int		is_detecting_emi = 0;
	int		peakDetect;
	
    mx = data[0];
    mn = data[0];
	
    for(i = 1; i < data_count; ++i)
    {
        if(data[i] > mx)
        {
            mx_pos = i;
            mx = data[i];
        }
        if(data[i] < mn)
        {
            mn_pos = i;
            mn = data[i];
        }

        if(is_detecting_emi &&
                data[i] <= (mx - delta + 2))
        {
            is_detecting_emi = 0;
            return 2;
        }
        else if( (!is_detecting_emi) && (data[i] >= mn + delta) )
        {
            is_detecting_emi = 1;
            
            mx = data[mn_pos];
            peakDetect = mx;
            mx_pos = mn_pos;
        }
    }
    return 0;
}

void setup() {
	DEBUG_SERIAL.begin(115200);
	BLE_SERIAL.begin(115200);
	pinMode(POWER_SWITCH_STATE, INPUT); 
	
	pinMode(POWER_CONTROL, OUTPUT); 
	pinMode(LOWBAT_LED, OUTPUT);
	
	digitalWrite(POWER_CONTROL, HIGH);
	pinMode(BT_LED_PIN, OUTPUT);
	
	pinMode(PUMP_SWITCH_PIN, INPUT);
	pinMode(VALVE_SWITCH_PIN, INPUT);
	pinMode(PUMP_MOTOR_PIN, OUTPUT);
	pinMode(VALVE_SOL_PIN, OUTPUT);
	delay(3000);
}

void loop() {
	static unsigned long int prevBleSendMillis = millis();
	static char workingFlag = 0;

	if(digitalRead(PUMP_SWITCH_PIN))
	{
		workingFlag = 1;
		delay(300);
	}

	if(workingFlag & (0x01 << 0))
	{
		bpMeasurementProcess();
		workingFlag = 0;
	}

	//일정 시간 간격으로 BLE 데이터를 전송한다.
	if( (millis() - prevBleSendMillis) > BLE_SEND_INTERVAL)
	{
		bleTxPacket.setID(PROTOCOL_ID_BP);
		bleTxPacket.setCMD(PROTOCOL_CMD_BP_RESPONSE);
		bleTxPacket.clearPayload(); //payload 데이터를 전부 제거한다.
		bleTxPacket.addPayload((unsigned char)PROTOCOL_MID_BP_SYS_DATA, giSysData);
		bleTxPacket.addPayload((unsigned char)PROTOCOL_MID_BP_DIA_DATA, giDiaData);
		bleTxPacket.calcLRC(); //설정된 정보를 기준으로 LRC를 계산한다.
		sendBleData();
		prevBleSendMillis = millis();
	}
	
	powerLedProcess(); //Power와 관련된 처리를 진행한다.
	ledProcess(); //BLE LED 관련 처리를 진행한다.
}

float getPressure()
{
	float result;
	result = (float)analogRead(DATA_INPUT_PIN);
	//ADC로 측정된 공압 데이터를 수은주밀리그램으로 환상한다.
	//결과값 = ADC 값 * 수은주밀리그램 변환 계수 * 공압 오차
	result = result * GAIN_BP + PRESSURE_OFFSET;
	return result;
}

void bpMeasurementProcess()
{
	static float pastInput = 0, pastOutput = 0;
	float lpfResult = 0;
	unsigned long int prevPumpingMillis = 0;
	int rawData;
	float fValue;
	int mx, mn;
	char peakCount = 0;
	long prevNoPeakMillis = 0, currNoPeakCount = 0;
	char enSensingFlag = 0;
	int targetPumpingValue;
	float sysResult = 0, diaResult = -1;
	int peakThreshold = 5;
	
	digitalWrite(VALVE_SOL_PIN, LOW);
	analogWrite(PUMP_MOTOR_PIN, 255);
	guiSampleDataCount = 0;
	pastInput = 0;
	pastOutput = 0;
	targetPumpingValue = 100;
	do //커프가 팔에 딱 붙여서 감쌀 수 있게 커프에 공압을 넣어준다.
	{
		fValue = getPressure(); //공압값 측정(단위 : mmHg 수은주밀리그램)
		LPF(&fValue, &lpfResult, 20.0, 0.1, &pastInput, &pastOutput); //Low Pass Filter로 노이즈를 최소화한다.
		delay(5);
	}while(lpfResult < targetPumpingValue); //현재압력이 목표압력과 같아지도록 공압을 계속 넣는다.
	analogWrite(PUMP_MOTOR_PIN, 0); //현재압력과 목표압력이 같아졌으면 Pump 모터를 정지한다.
	delay(500);
	
	while(1)
	{
		fValue = getPressure(); //공압값 측정(단위 : mmHg 수은주밀리그램)
		LPF(&fValue, &lpfResult, 20.0, 0.1, &pastInput, &pastOutput); //Low Pass Filter로 노이즈를 최소화한다.

		//공압을 넣는 시간이 300ms 이상이고, 맥박을 측정하는 프로세스를 허용하는 플래그가 0일 때
		if( (millis() - prevPumpingMillis > 300) && enSensingFlag == 0)
		{
			analogWrite(PUMP_MOTOR_PIN, 0); //Pump 모터를 정지한다.
			enSensingFlag = 1; //공압 센서를 읽는 프로세스를 시작할 수 있게 한다.
			guiSampleDataCount = 0; //Sample 데이터를 수집하는 카운트를 0으로 초기화한다.
			delay(100); //Pump 모터 정지 후 공압 안정을 위하여 100ms를 대기한다.
			prevNoPeakMillis = millis(); //맥박을 감지못한 시간을 측정하기 위하여 시간값을 초기화한다.
		}
		
		if( enSensingFlag > 0 ) //맥박을 측정하는 프로세스가 허용되어있을 때
		{
			Serial.print(lpfResult);
			Serial.println();

			//Sample 데이터를 먼저 MAX_SAMPLE_SIZE만큼 모아두고, 맥박을 감지하기 때문에
			//먼저 MAX_SAMPLE_SIZE만큼 데이터를 수집한다.
			if(guiSampleDataCount < MAX_SAMPLE_SIZE)
			{
				sampleData[ guiSampleDataCount++ ] = lpfResult; //Sample 데이터를 수집한다.
			}
			//만약 Sample 데이터가 전부 모였으면 맥박을 감지한다.
			else
			{
				//detect_peak 함수를 이용하여 맥박이 감지되었는지 확인하고, 결과값을 detectResult 변수에 저장한다.
				char detectResult = detect_peak(sampleData, MAX_SAMPLE_SIZE, peakThreshold, mx, mn);
				
				if( detectResult == 2 ) //만약 맥박이 감지되었으면
				{
					//
					//peakCount 변수의 값을 먼저 1증가 시키고, 만약 맥박이 두번 연속 감지되었으면 
					//해당 압력에 맥박이 확실하게 감지되고 있는것으로 판별한다.
					if(++peakCount >= 2)
					{
						if(mx > 130) //만약 맥박의 수치가 130이면, 맥박의 확인 감도를 60으로 올린다.
							peakThreshold = 40;

						sysResult = max(sysResult, mn); //수축혈압을 현재 측정중인 최대값으로 갱신한다.
						//현재 맥박이 처음 감지되어 이완혈압이 설정 안되어있다면, 현재 공압을 이완혈압으로 저장한다.
						if(diaResult == -1)
						{
							diaResult = mx; //
						}
						
						enSensingFlag = 0; //맥박 감지 프로세스를 비허용한다.
						peakCount = 0; //맥박 감지 횟수 카운트를 초기화한다.
						analogWrite(PUMP_MOTOR_PIN, 255); //Pump 모터를 동작시켜 공압을 증가시킨다.
						prevPumpingMillis = millis(); //Pump 모터동작 시간을 초기화한다.
					}
					prevNoPeakMillis = millis();
					Serial.println("\n####################################peak!####################################");
					
					guiSampleDataCount = 0; //Sample 데이터 카운트를 초기화한다. 
				}
				
				for(int i=0; i<(MAX_SAMPLE_SIZE - 1); i++)
				{
					sampleData[i] = sampleData[i + 1]; //데이터를 한칸씩 이동하여 오래된 데이터를 지운다.
				}
				sampleData[(MAX_SAMPLE_SIZE - 1)] = lpfResult; //새로운 데이터를 추가한다.
			}
			//만약 맥박이 2.3초 이상 측정이 안되었을 시 공압을 늘린다.
			if((millis() - prevNoPeakMillis) > 2300)
			{
				analogWrite(PUMP_MOTOR_PIN, 255); //모터를 동작시킨다.
				prevPumpingMillis = millis();
				enSensingFlag = 0; //맥박 감지 프로세스를 비허용한다.
				peakCount = 0; //맥박 감지 횟수 카운트를 초기화한다.
				guiSampleDataCount = 0; //Sample 데이터 카운트를 초기화한다.
				if(diaResult != -1) //만약 이완혈압이 측정되었을 시
				{
					printResult(sysResult, diaResult); //수축혈압, 이완혈압을 출력한다.
					analogWrite(PUMP_MOTOR_PIN, 0); //모터를 정지한다.
					digitalWrite(VALVE_SOL_PIN, HIGH); //밸브를 열어 커프내의 공압을 뺀다.
					break; //혈압 측정 while문을 빠져나간다.
				}
				prevNoPeakMillis = millis();
			}
		}
		// 만약 압력이 과도하게 들어갔을 경우
		if(targetPumpingValue >= 400)
		{
			Serial.print("over pressure");
			printResult(sysResult, diaResult);
			analogWrite(PUMP_MOTOR_PIN, 0); //모터를 정지한다.
			digitalWrite(VALVE_SOL_PIN, HIGH);  //밸브를 열어 커프내의 공압을 뺀다.
			break; //혈압 측정 while문을 빠져나간다.
		}
		delay(5);
	}
}

void printResult(int sysRaw, int diaRaw)
{
	Serial.print("SYS : ");
	Serial.print(sysRaw);
	Serial.print(", DIA : ");
	Serial.print(diaRaw);
	Serial.println();

	giSysData = sysRaw;
	giDiaData = diaRaw;
}

/**
 * 전원 스위치에 대한 기능을 정의한 함수
 */
void powerLedProcess()
{
  static unsigned long int currTime = 0, prevTime = 0; 
  static char ledControlFlag = 0; // LED 제어를 위한 플래그 변수

  currTime = millis();
  int powerSwitchValue = analogRead(POWER_SWITCH_STATE); 
  if(powerSwitchValue>20)
  {
    if( (currTime - prevTime) > 1000)
    {
      ledControlFlag = !ledControlFlag;
     
      digitalWrite(LOWBAT_LED, ledControlFlag); 
      digitalWrite(POWER_CONTROL, LOW);
      delay(80);
    }
    else
    {
      digitalWrite(LOWBAT_LED, LOW);
    }
  }
 
  else
  {
    digitalWrite(LOWBAT_LED, HIGH);
    prevTime = currTime;
    digitalWrite(POWER_CONTROL, HIGH);
  }
}

/**
 * @brief
 * LED를 활성화 시키는 함수이다.
 * LED를 활성화하게 되면, ledProcess에서 자동적으로 일정시간동안 LED를 키고, 끄게한다.
 */
void enableLed()
{
	if(led_data.enable == 0) //LED가 비활성화 되어있을 때
	{
		led_data.enable = 1; //LED를 활성화 시킨다.
		led_data.on_count = 0; //시간 카운트를 초기화한다.
		led_data.type = LED_CONTROL_NORMAL; //제어 타입을 기본으로 설정한다.
	}
	else //이미 LED가 활성화 되어있을 때
	{
		led_data.enable = 1; //LED를 활성화 시킨다.
		led_data.on_count = 0; //시간 카운트를 초기화한다.
		led_data.type = LED_CONTROL_TOGGLE; //제어 타입을 토글로 설정한다.
	}
}


/**
 * @brief
 * LED를 제어하는 작업을 수행하는 함수이다.
 * led_data_t 타입의 enable 변수의 값에 따라 led를 일정 시간 키고, 끌 수 있다.
 */
void ledProcess()
{
	if(led_data.enable == 1)
	{
		if(led_data.type == LED_CONTROL_NORMAL)
		{
			led_data.isOn = 0;
		}
		else
		{
			led_data.isOn ^= 1; //값을 반전시킨다.
		}
		if(led_data.on_count++ > LED_ON_MAX_COUNT) //카운트 값이 임계값 초과되었는지 확인 후 카운트 값 증가
		{
			led_data.enable = 0;
			led_data.on_count = 0;
		}
	}
	else
	{
		led_data.isOn = 1;
	}
	digitalWrite(BT_LED_PIN, led_data.isOn);
}

void sendBleData()
{
	enableLed();
	for(int i=0; i<bleTxPacket.getPacketLength(); i++)
	{
		unsigned char dd = bleTxPacket.at(i);
		BLE_SERIAL.write(dd);
	}
}
