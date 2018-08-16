#define DEBUG_SERIAL	Serial

//디바이스의 전원 제어 포트 정의
#define POWER_CONTROL  A4 // 전원 제어 핀
#define POWER_SWITCH_STATE  A5 // 전원 스위치 상태 핀

// 상태 출력 LED 포트 정의
#define LOWBAT_LED 13 // LOW BAT. LED 핀

#define DATA_INPUT_PIN		A1	//공압이 입력되는 핀을 정의한다.
#define START_SWITCH_PIN	4	//혈압 측정을 시작하는 스위치의 핀을 정의한다.
#define PUMP_MOTOR_PIN		10	//공압펌프 모터의 핀을 정의한다.
#define VALVE_SOL_PIN		11	//솔레노이드 밸브의 핀을 정의한다.

int giPressureOffset = 0;	//공압의 오차 보정 수치를 설정하는 변수

/*
 * ADC 측정 값에서 수은주밀리미터로 변환하기 위해 곱하는 계수 값이다.
 * 2SMPP-02	Span Voltage : 31mV@37kPa
 * OP-AMP gain : 101
 * 1kPa = 7.5mmHg
 * blood_pressure(mmHg) = ADC*5/1023/101*1000*37/31*7.5
 */	
#define GAIN_BP				0.43f

#define MAX_SAMPLE_SIZE		128 //맥박 파형의 데이터를 저장하는 배열의 크기를 정의
int sampleData[MAX_SAMPLE_SIZE]; //맥박 파형 데이터를 저장하는 배열

unsigned int guiSampleDataCount = 0; //맥박 파형의 데이터 개수를 저장하는 변수

void enableLed();
void ledProcess();
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

/**
 * 파형 데이터에서 피크값을 검출하는 함수이다.
 * @param data			파형 데이터가 저장되어있는 배열을 입력한다.
 * @param data_count	파형 데이터의 개수를 입력한다.
 * @param delta			피크를 검출하기 위한 변화량의 기준을 입력한다.
 * 						마루와 골사이의 차이가 이 변화량 이상이어야지만 피크 값이 있다고 판단한다.
 * @param mx			파형 데이터 중 최대 값을 저장하는 변수 포인터를 입력한다.
 * @param mn			파형 데이터 중 최소 값을 저장하는 변수 포인터를 입력한다.
 * @return				입력 받은 데이터에서 peak가 감지안되었다면, 0을 리턴한다.
 * 						만약 peak가 감지되었다면 2를 리턴한다.
 */
int detect_peak(
        const int*   data,
        int          data_count,
        int          delta,
        int& mx,
        int& mn
        )
{
    int     i; //데이터 배열을 탐색하기 위한 index를 저장하는 변수
    int		mx_pos = 0; //파형 데이터 중 최대 값의 위치를 저장하는 변수
    int		mn_pos = 0; //파형 데이터 중 최소 값의 위치를 저장하는 변수
    int		is_detecting_emi = 0; //파형이 상승하였는지 확인하는 변수
	
    mx = data[0]; //파형의 최대 값을 우선 첫 번째 데이터로 적용한다.
    mn = data[0]; //파형의 최소 값을 우선 첫 번째 데이터로 적용한다.
	
    for(i = 1; i < data_count; ++i)
    {
        if(data[i] > mx) //만약 data[i]의 값이 현재까지 확인된 최대 값 보다 클 경우
        {
            mx_pos = i; //최대 값이 있는 index를 현재 index로 설정한다.
            mx = data[i]; //최대 값을 data[i]의 값으로 갱신한다.
        }
        if(data[i] < mn) //만약 data[i]의 값이 현재까지 확인된 최소 값 보다 작을 경우
        {
            mn_pos = i; //최대 값이 있는 index를 현재 index로 설정한다.
            mn = data[i];  //최대 값을 data[i]의 값으로 갱신한다.
        }

        if(is_detecting_emi && //만약 파형이 delta 값을 넘어 상승한 상태이고,
                data[i] <= (mx - (delta / 2))) //파형의 데이터가 다시 내려가 마루를 만들며, delta의 1/2 지점보다 하강하였을 때
        {
            is_detecting_emi = 0; //파형이 상승하였는지 확인하는 변수의 값을 0으로 설정한다.
            return 2; //Peak를 감지하였으므로 2를 리턴한다.
        }
        else if( (!is_detecting_emi) && (data[i] >= mn + delta) ) //만약 파형의 데이터가 delta 값을 넘어 상승하였을 때
        {
            is_detecting_emi = 1; //파형의 데이터가 상승하였음을 저장한다.

            // 다음 데이터 확인 때 우연히 잠시 낮아진 데이터를 보고 피크 값으로 확인하지 않기 위하여
            // mx의 값을 mn 값으로 잠시 낮추어 데이터 재확인을 할 수 있게 한다.
            mx = data[mn_pos];
            mx_pos = mn_pos;
        }
    }
    return 0;
}

void setup() {
	DEBUG_SERIAL.begin(115200);
	pinMode(POWER_SWITCH_STATE, INPUT); 
	
	pinMode(POWER_CONTROL, OUTPUT); 
	pinMode(LOWBAT_LED, OUTPUT);
	
	digitalWrite(POWER_CONTROL, HIGH);
	
	pinMode(START_SWITCH_PIN, INPUT);
	pinMode(PUMP_MOTOR_PIN, OUTPUT);
	pinMode(VALVE_SOL_PIN, OUTPUT);
	delay(3000);
}

void loop() {
	if(digitalRead(START_SWITCH_PIN)) //만약 측정 시작 버튼이 눌렸을 경우
	{
		bpMeasurementProcess(); //혈압을 측정하는 프로세스를 시작한다.
		delay(300);
	}

	powerLedProcess(); //Power와 관련된 처리를 진행한다.
}

/**
 * 혈압 값을 리턴하는 함수
 * 리턴되는 혈압 값은 ADC 값에서 mmHg 값으로 변환되서 리턴된다.
 * @return 혈압 값(mmHg)
 */
float getPressure()
{
	float result;
	result = (float)analogRead(DATA_INPUT_PIN);
	//ADC로 측정된 공압 데이터를 수은주밀리그램으로 환상한다.
	//결과값 = ADC 값 * 수은주밀리미터 변환 계수 * 공압 오차
	result = result * GAIN_BP + giPressureOffset;
	return result;
}

/**
 * 혈압 측정을 진행하는 함수
 * 진행되는 과정은 크게 아래와 같다.
 * 1. 커프가 팔에 딱 붙여서 감쌀 수 있게 커프에 공압을 넣어준다.
 * 2. 공압을 측정하면서 맥박이 감지되는지 확인한다.
 * 3. 만약 맥박이 2.3초 동안 감지안되었으면, 공압펌프를 동작시켜 공압을 넣어준다.
 * 4. 공압을 넣기 시작한 300ms후에 공압펌프를 멈추고, 다시 공압을 측정한다.
 * 5. 만약 맥박이 두 번 감지되었으면 해당 공압을 이완혈압으로 저장한다. 그리고 공압펌프를 동작시켜 공압을 넣어준다.
 * 6. 공압을 넣기 시작한 300ms후에 공압펌프를 멈추고, 다시 공압을 측정한다.
 * 7. 만약 맥박이 두 번 감지되었고, 이완혈압이 측정되어있으면 해당 공압 값을 수축혈압으로 저장한다. 그리고 공압펌프를 동작시켜 공압을 넣어준다.
 * 8. 6, 7번 과정을 반복 후 만약 2.3초 동안 맥박이 감지 안되었을 때 이완혈압이 측정되어있으면 혈압 측정을 완료한다.
 */
void bpMeasurementProcess()
{
	float pastInput = 0, pastOutput = 0; //LPF에서 이전 값을 저장하기 위한 변수
	float fValue; //현재 센서 값을 저장하기 위한 변수
	float lpfResult = 0; //LPF 결과 값을 저장하는 변수
	unsigned long int prevPumpingMillis = 0; //공압을 넣는 시간을 체크하기 위하여, 공압을 넣기 시작한 시간을 저장하는 변수
	unsigned long int prevNoPeakMillis = 0; //맥박이 감지안된 시간을 체크하기 위하여, 체크를 시작한 시간을 저장하는 변수
	int mx, mn; //공압 파형에서 최대 값과 최소 값을 저장하는 변수
	char peakCount = 0; //공압 파형에서 peak 감지 횟수를 저장하는 변수
	char enSensingFlag = 0; //공압 파형을 측정을 활성화하는 변수
	int targetPumpingValue; //목표 공압을 저장하는 변수 
	float sysResult = 0, diaResult = -1; //수축혈압, 이완혈압을 저장하는 변수
	int peakThreshold = 7; //공압 파형에서 맥박이 뛰었을 때의 파형이 올라가는 변화량 기준을 저장하는 변수

	/*
	 * 공압의 오차값을 보정하기 위하여 공압을 넣지 않은 상태의 값을 60에서 뺀다.
	 * 60에서 빼는 이유는 사람의 최소 혈압의 기준이 60이기 때문에
	 * 맥박이 감지되는 시점을 60에서 시작하여 측정한다.
	 */
	giPressureOffset = 60 - getPressure(); //공압값 측정(단위 : mmHg 수은주밀리그램)
	
	digitalWrite(VALVE_SOL_PIN, LOW); //밸브를 닫아 공기가 빠져나가지 않게 한다.
	analogWrite(PUMP_MOTOR_PIN, 255); //공압펌프를 동작시킨다.
	guiSampleDataCount = 0; //데이터 저장 카운트를 초기화한다.
	pastInput = 0; //LPF에서 사용하는 이전 값을 저장하기 위한 변수를 초기화한다.
	pastOutput = 0; //LPF에서 사용하는 이전 값을 저장하기 위한 변수를 초기화한다.
	//목표 공압수치를 100으로 하여 공압이 100이 될 때 까지 펌프가 동작할 수 있게 한다.
	//단, 모터가 공압을 넣고있을 때에는 넣지 않고있을 때보다 높은 수치가 뜨므로, 100을 넣어서 멈추면 60정도가 측정된다.
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
		fValue = getPressure(); //공압값 측정(단위 : mmHg 수은주밀리미터)
		LPF(&fValue, &lpfResult, 20.0, 0.1, &pastInput, &pastOutput); //Low Pass Filter로 노이즈를 최소화한다.

		//공압을 넣기 시작한 지 300ms가 지났으며, 맥박을 측정하는 프로세스를 허용하는 플래그가 0일 때
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
			DEBUG_SERIAL.print(lpfResult); //현재 공압값을 시리얼 모니터에 출력한다.
			DEBUG_SERIAL.println();

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
					/*
					 * 자그만한 움직임 때문에 맥박을 잘못 인식하는 경우를 없에기 위하여
					 * 피크 감지가 두 번되어야 해당 압력에 맥박이 확실하게 감지되고 있는것으로 판별한다.
					 */
					if(++peakCount >= 2)
					{
						/*
						 * 만약 맥박의 수치가 130이면, 맥박의 확인 감도를 40으로 올린다.
						 * 기계식 혈압계는 소리로 혈관의 막힘을 측정하지만, 공압은 혈관을 다 막아도 그 막은 뒤쪽의 압력이 측정되기 때문에
						 * 130mmHg 이후에는 혈압 파형의 인식 기준 값을 높이며, 정상 혈압을 가진 사람은 해당 지점에서 기준 값을 안넘지만,
						 * 혈압이 높은 사람은 이완과 수축의 압력 차이가 많이 나기 때문에 130mmHg 이후에도 측정을 지속할 수 있다.
						 */
						if(mx > 130)
							peakThreshold = 40;

						//현재 맥박이 처음 감지되어 이완혈압이 설정 안되어있다면, 현재 공압을 이완혈압으로 저장한다.
						if(diaResult == -1)
						{
							diaResult = (mx+mn) / 2; //혈압 파형의 골과 마루의 평균 값을 이완혈압으로 저장한다.
						}
						else
						{
							sysResult = (mx+mn) / 2; //혈압 파형의 골과 마루의 평균 값을 수축혈압으로 저장한다.	
						}
						
						enSensingFlag = 0; //맥박 감지 프로세스를 비허용한다.
						peakCount = 0; //맥박 감지 횟수 카운트를 초기화한다.
						analogWrite(PUMP_MOTOR_PIN, 255); //Pump 모터를 동작시켜 공압을 증가시킨다.
						prevPumpingMillis = millis(); //Pump 모터동작 시간을 초기화한다.
					}
					prevNoPeakMillis = millis(); //맥박을 감지못한 시간을 측정하기 위하여 시간값을 초기화한다.
					DEBUG_SERIAL.println("\n####peak!####");
					
					guiSampleDataCount = 0; //Sample 데이터 카운트를 초기화한다. 
				}
				else //만약 맥박이 감지안되었을 때
				{
					for(int i=0; i<(MAX_SAMPLE_SIZE - 1); i++)
					{
						sampleData[i] = sampleData[i + 1]; //데이터를 한칸씩 이동하여 오래된 데이터를 지운다.
					}
					sampleData[(MAX_SAMPLE_SIZE - 1)] = lpfResult; //새로운 데이터를 추가한다.
				}
			}
			//만약 맥박이 2.3초 이상 측정이 안되었을 시 공압을 늘린다.
			if((millis() - prevNoPeakMillis) > 2300)
			{
				analogWrite(PUMP_MOTOR_PIN, 255); //모터를 동작시킨다.
				prevPumpingMillis = millis(); //Pump 모터동작 시간을 초기화한다.
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
				prevNoPeakMillis = millis(); //맥박을 감지못한 시간을 측정하기 위하여 시간값을 초기화한다.
			}
		}
		// 만약 압력이 과도하게 들어갔을 경우
		if(targetPumpingValue >= 400)
		{
			DEBUG_SERIAL.print("over pressure");
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
	DEBUG_SERIAL.print("SYS : ");
	DEBUG_SERIAL.print(sysRaw);
	DEBUG_SERIAL.print(", DIA : ");
	DEBUG_SERIAL.print(diaRaw);
	DEBUG_SERIAL.println();
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
