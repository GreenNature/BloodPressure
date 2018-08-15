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

/*################# 디바이스의 전원 제어 포트 정의 #################*/
#define POWER_EN_PIN	A4
#define POWER_KEY_PIN	A5
/*###############################################################*/

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

#define BLOOD_OFFSET		0

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
        int& mx,
        int& mn
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
                data[i] <= mx - delta)
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
	pinMode(POWER_EN_PIN, OUTPUT);
	
	pinMode(POWER_KEY_PIN, INPUT);
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(BT_LED_PIN, OUTPUT);
	digitalWrite(POWER_EN_PIN, HIGH);
	
	pinMode(PUMP_SWITCH_PIN, INPUT);
	pinMode(VALVE_SWITCH_PIN, INPUT);
	pinMode(PUMP_MOTOR_PIN, OUTPUT);
	pinMode(VALVE_SOL_PIN, OUTPUT);
	delay(3000);
}

void loop() {
	static unsigned long int prevBleSendMillis = millis();
	static float pastInput = 0, pastOutput = 0;
	static char workingFlag = 0;
	float lpfResult = 0;
	unsigned long int prevPumpingMillis = 0;
	int rawData;
	float fValue;
	int mx, mn;
	char peakCount = 0;
	long prevNoPeakCount = 0, currNoPeakCount = 0;
	char enSensingFlag = 0;
	int targetPumpingValue = 0;
	float sysResult = 0, diaResult = 10000;
	int peakThreshold = 5;

	if(digitalRead(PUMP_SWITCH_PIN))
	{
		workingFlag = 1;
		delay(300);
	}

	if(workingFlag & (0x01 << 0))
	{
		digitalWrite(VALVE_SOL_PIN, LOW);
		digitalWrite(PUMP_MOTOR_PIN, HIGH);
		guiSampleDataCount = 0;
		targetPumpingValue = 100;
		do //팔에 맞게 커프에 공압을 넣어준다.
		{
			fValue = getPressure(); //공압값 측정(단위 : mmHg 수은주밀리그램)
			LPF(&fValue, &lpfResult, 20.0, 0.1, &pastInput, &pastOutput);
		}while(lpfResult < targetPumpingValue);
		digitalWrite(PUMP_MOTOR_PIN, LOW);

		delay(500);
		analogWrite(PUMP_MOTOR_PIN, 255);
		
		while(1)
		{
			currNoPeakCount = millis();
			fValue = getPressure(); //공압값 측정(단위 : mmHg 수은주밀리그램)
			LPF(&fValue, &lpfResult, 20.0, 0.1, &pastInput, &pastOutput);
			
			if( (millis() - prevPumpingMillis > 300) && enSensingFlag == 0)
			{
				analogWrite(PUMP_MOTOR_PIN, 0); 
				enSensingFlag = 1;
				guiSampleDataCount = 0;
				delay(100);
				prevNoPeakCount = currNoPeakCount;
			}
			
			if( enSensingFlag > 0 )
			{
				Serial.print(guiSampleDataCount);
				Serial.print(",");
				Serial.print(lpfResult);
				Serial.print(",");
				Serial.println(mx-mn);
				
				if(guiSampleDataCount < MAX_SAMPLE_SIZE)
				{
					sampleData[ guiSampleDataCount++ ] = lpfResult;
				}
				else
				{
					char detectResult = detect_peak(sampleData, MAX_SAMPLE_SIZE, 5, mx, mn);
					if( detectResult == 2 )
					{
						//Serial.print("\ndistinction : ");
						//Serial.println(mx - mn);
						if(mx - mn > peakThreshold)
						{
							if(++peakCount >= 2)
							{
								analogWrite(PUMP_MOTOR_PIN, 255);
								prevPumpingMillis = millis();
								enSensingFlag = 0;
								peakCount = 0;

								if(mx > 130)
									peakThreshold = 60;

								sysResult = max(sysResult, mn);
								if(diaResult == 10000)
									diaResult = mn;
							}
							prevNoPeakCount = currNoPeakCount;
							Serial.println("\n####################################peak!####################################");
						}
						guiSampleDataCount = 0;
					}
					
					for(int i=0; i<(MAX_SAMPLE_SIZE - 1); i++)
					{
						sampleData[i] = sampleData[i + 1];
					}
					sampleData[(MAX_SAMPLE_SIZE - 1)] = lpfResult;
				}
				//만약 맥박이 2.3초 이상 측정이 안되었을 시 공압을 늘린다.
				if((currNoPeakCount - prevNoPeakCount) > 2300)
				{
					analogWrite(PUMP_MOTOR_PIN, 255); //모터를 동작시킨다.
					prevPumpingMillis = millis();
					enSensingFlag = 0;
					peakCount = 0;
					guiSampleDataCount = 0;
					if(diaResult != 10000)
					{
						printResult(sysResult, diaResult);
						analogWrite(PUMP_MOTOR_PIN, 0);
						digitalWrite(VALVE_SOL_PIN, HIGH);
						break;
					}
					prevNoPeakCount = currNoPeakCount;
				}
			}
			// 만약 압력이 과도하게 들어갔을 경우
			if(targetPumpingValue >= 400)
			{
				Serial.print("over pressure");
				printResult(sysResult, diaResult);
				analogWrite(PUMP_MOTOR_PIN, 0);
				digitalWrite(VALVE_SOL_PIN, HIGH);
				break;
			}
			delay(5);
		}
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
	result = result * GAIN_BP + BLOOD_OFFSET;
	return result;
}

void bpMeasurementProcess()
{
	
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

void powerLedProcess()
{
	static unsigned long int buttonCurr = 0, buttonPrev = 0;
	static char ledFlag = 0;

	buttonCurr = millis();
	int powerAnlogValue = analogRead(POWER_KEY_PIN);
	if(powerAnlogValue>20)
	{
		if( (buttonCurr - buttonPrev) > 1000)
		{
			static char ledFlag = 0;
			digitalWrite(LED_BUILTIN, ledFlag^=1);
			digitalWrite(POWER_EN_PIN, LOW);
			delay(80);
		}
		else
		{
			digitalWrite(LED_BUILTIN, LOW);
		}
	}
	else
	{
		digitalWrite(LED_BUILTIN, HIGH);
		buttonPrev = buttonCurr;
		digitalWrite(POWER_EN_PIN, HIGH);
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
