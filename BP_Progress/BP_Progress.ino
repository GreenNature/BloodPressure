#define POWER_EN_PIN	A4
#define POWER_KEY_PIN	A5

#define DATA_INPUT_PIN		A1
#define PUMP_SWITCH_PIN		4
#define VALVE_SWITCH_PIN	5
#define PUMP_MOTOR_PIN		10
#define VALVE_SOL_PIN		11

#define BLOOD_OFFSET		7

#define MAX_SAMPLE_SIZE		128

int sampleData[MAX_SAMPLE_SIZE];

unsigned int guiSampleDataCount = 0;
int giPeakValue = 0;

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
                data[i] < mx - delta)
        {
            is_detecting_emi = 0;

            //i = mx_pos - 1;
            //mn = data[mx_pos];
            peakDetect = mn;
            mn_pos = mx_pos;
            return 2;
        }
        else if( (!is_detecting_emi) && (data[i] > mn + delta) )
        {
            is_detecting_emi = 1;
            
            //i = mn_pos - 1;
            mx = data[mn_pos];
            peakDetect = mx;
            mx_pos = mn_pos;
        }
#define DBG_NUM	2
#if DBG_NUM==0
		Serial.print(data[i]);
		Serial.println();
#elif DBG_NUM==1
        Serial.print("i : ");
        Serial.print(i);
		Serial.print(", data : ");
		Serial.print(data[i]);
		Serial.print(", mx : ");
		Serial.print(mx);
		Serial.print(", mn : ");
		Serial.print(mn);
		Serial.print(", mx_pos : ");
		Serial.print(mx_pos);
		Serial.print(", mn_pos : ");
		Serial.print(mn_pos);
		Serial.print(", is_de : ");
		Serial.print(is_detecting_emi);
		Serial.println();
#endif
    }
    return 0;
}

void setup() {
	Serial.begin(115200);
	Serial1.begin(115200);
	pinMode(PUMP_SWITCH_PIN, INPUT);
	pinMode(VALVE_SWITCH_PIN, INPUT);
	pinMode(PUMP_MOTOR_PIN, OUTPUT);
	pinMode(VALVE_SOL_PIN, OUTPUT);
	pinMode(POWER_EN_PIN, OUTPUT);
	pinMode(POWER_KEY_PIN, INPUT);
	digitalWrite(POWER_EN_PIN, HIGH);
	delay(3000);
	Serial1.print("Battery : ");
	Serial1.println(analogRead(A0));
}

void loop() {
	static unsigned long int buttonCurr = 0, buttonPrev = 0;
	static float pastInput = 0, pastOutput = 0;
	static char workingFlag = 0;
	float lpfResult = 0;
	int rawData;
	float fValue;
	int mx, mn;
	char peakCount = 0;
	long prevNoPeakCount = 0, currNoPeakCount = 0;
	char enSensingFlag = 0;
	int goalPumpingValue = 0;
	int sysResult = 0, diaResult = 10000;
	int peakThreshold = 5;

	buttonCurr = millis();
	int powerAnlogValue = analogRead(POWER_KEY_PIN);
	if(powerAnlogValue>20)
	{
		if( (buttonCurr - buttonPrev) > 1000)
		{
			static char ledFlag = 0;
			digitalWrite(13, ledFlag^=1);
			digitalWrite(POWER_EN_PIN, LOW);
			delay(80);
		}
		else
		{
			digitalWrite(13, LOW);
		}
	}
	else
	{
		digitalWrite(13, HIGH);
		buttonPrev = buttonCurr;
		digitalWrite(POWER_EN_PIN, HIGH);
	}

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
		goalPumpingValue = 220;
		do //적당히 조여준다.
		{
			rawData = analogRead(DATA_INPUT_PIN);
			fValue = (float)rawData;
			LPF(&fValue, &lpfResult, 20.0, 0.1, &pastInput, &pastOutput);
		}while(lpfResult < goalPumpingValue);
		digitalWrite(PUMP_MOTOR_PIN, LOW);

		delay(500);
		analogWrite(PUMP_MOTOR_PIN, 255);
		
		while(1)
		{
			currNoPeakCount = millis();
			rawData = analogRead(DATA_INPUT_PIN);
			fValue = (float)rawData;
			LPF(&fValue, &lpfResult, 20.0, 0.1, &pastInput, &pastOutput);
			if(lpfResult >= goalPumpingValue && enSensingFlag == 0)
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
				Serial.println();
				
				if(guiSampleDataCount < MAX_SAMPLE_SIZE)
				{
					sampleData[ guiSampleDataCount++ ] = lpfResult;
				}
				else
				{
					char detectResult = detect_peak(sampleData, MAX_SAMPLE_SIZE, 15, mx, mn);
					if( detectResult == 2 )
					{
						//Serial.print("\ndistinction : ");
						//Serial.println(mx - mn);
						if(mx - mn > peakThreshold)
						{
							if(++peakCount >= 2)
							{
								analogWrite(PUMP_MOTOR_PIN, 255);
								goalPumpingValue += 15;
								enSensingFlag = 0;
								peakCount = 0;

								if(mn > 250)
									peakThreshold = 100;

								sysResult = max(sysResult, mn);
								if(diaResult == 10000)
									diaResult = mn + (mx - mn);
							}
							prevNoPeakCount = currNoPeakCount;
							Serial.println("\n####################################peak!####################################");
						}
						guiSampleDataCount = 0;
					}
					
					//guiSampleDataCount = 0;
					for(int i=0; i<(MAX_SAMPLE_SIZE - 1); i++)
					{
						sampleData[i] = sampleData[i + 1];
					}
					sampleData[(MAX_SAMPLE_SIZE - 1)] = lpfResult;
				}

				if((currNoPeakCount - prevNoPeakCount) > 2300)
				{
					analogWrite(PUMP_MOTOR_PIN, 255);
					goalPumpingValue += 15;
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
			if(goalPumpingValue >= 800)
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
}

void printResult(int sysRaw, int diaRaw)
{
	sysRaw = (int)(((float)sysRaw / 550.0) * 258.57) + BLOOD_OFFSET;
	diaRaw = (int)(((float)diaRaw / 550.0) * 258.57) + BLOOD_OFFSET;
	Serial.print("SYS : ");
	Serial.print(sysRaw);
	Serial.print(", DIA : ");
	Serial.print(diaRaw);
	Serial.println();

	Serial1.print("SYS : ");
	Serial1.print(sysRaw);
	Serial1.print(", DIA : ");
	Serial1.print(diaRaw);
}


