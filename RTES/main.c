#include <LPC214x.h>

#define EEPROM_Addr 0xA0 //device address
#define I2Cwrite 0x00	 //LSB bit 0 (write)
#define I2Cread 0x01	 //LSB bit 1 (read)

#define I2C_ENABLE 1 << 6 //I2C Enable bit
#define I2C_START 1 << 5  //Start Bit
#define I2C_STOP 1 << 4	  //Stop Bit
#define I2C_SI 1 << 3	  //I2C interrupt flag
#define I2C_AACK 1 << 2	  //assert ACK flag

#define PID_KP 2.0f
#define PID_KI 0.01f
#define PID_KD 0.25f

#define PID_TAU 0.02f

#define PID_LIM_MIN 0.0f
#define PID_LIM_MAX 255.0f

#define PID_LIM_MIN_INT -255.0f
#define PID_LIM_MAX_INT 255.0f

#define SAMPLE_TIME_S 0.1f

unsigned char status = 0;

typedef struct
{
	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Derivative low-pass filter time constant */
	float tau;

	/* Output limits */
	float limMin;
	float limMax;

	/* Integrator limits */
	float limMinInt;
	float limMaxInt;

	/* Sample time (in seconds) */
	float T;

	/* Controller "memory" */
	float integrator;
	float prevError; /* Required for integrator */
	float differentiator;
	float prevMeasurement; /* Required for differentiator */

	/* Controller output */
	float out;
} PID;

void PIDController_Init(PID *pid)
{

	/* Clear controller variables */
	pid->integrator = 0.0f;
	pid->prevError = 0.0f;

	pid->differentiator = 0.0f;
	pid->prevMeasurement = 0.0f;

	pid->out = 0.0f;
}

float PIDController_Update(PID *pid, float setpoint, float measurement, int flag)
{

	/*
	* Error signal
	*/
	float error, proportional;
	if (flag == 0)
	{
		error = setpoint - measurement;
	}
	else
	{
		error = measurement - setpoint;
	}

	/*
	* Proportional
	*/
	proportional = pid->Kp * error;

	/*
	* Integral
	*/
	pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

	/* Anti-wind-up via integrator clamping */
	if (pid->integrator > pid->limMaxInt)
	{

		pid->integrator = pid->limMaxInt;
	}
	else if (pid->integrator < pid->limMinInt)
	{

		pid->integrator = pid->limMinInt;
	}

	/*
	* Derivative (band-limited differentiator)
	*/

	pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement) /* Note: derivative on measurement, therefore minus sign in front of equation! */
							+ (2.0f * pid->tau - pid->T) * pid->differentiator) /
						  (2.0f * pid->tau + pid->T);

	/*
	* Compute output and apply limits
	*/
	pid->out = proportional + pid->integrator + pid->differentiator;

	if (pid->out > pid->limMax)
	{

		pid->out = pid->limMax;
	}
	else if (pid->out < pid->limMin)
	{

		pid->out = pid->limMin;
	}

	/* Store error and measurement for later use */
	pid->prevError = error;
	pid->prevMeasurement = measurement;

	/* Return controller output */
	return pid->out;
}

__irq void Interrupt(void)
{
	char rx;
	int iir_value;
	unsigned char status;

	if (PWMIR & 0x0001)
	{
		PWMIR = 0x0001;
	}

	if (PWMIR & 0x0004)
	{
		PWMIR = 0x0004;
	}

	iir_value = U0IIR;
	while (!(iir_value & 0x01))
		;
	if (iir_value & 0x00000004)
	{
		rx = U0RBR;
	}
	U0THR = rx;
	while ((U0LSR & 0x40) == 0)
		;

	VICVectAddr = 0x00;
}

void UART0_Init(void)
{
	PINSEL0 |= 0x00000005;
	U0LCR = 0x83;
	U0DLM = 0x00;
	U0DLL = 0x62;
	U0LCR = 0x03;
}

void UART0_Write(unsigned char value)
{
	while (!(U0LSR & 0x20))
		;
	U0THR = value;
}

void UART0_Write_Text(char *msg)
{
	while (*msg)
	{
		UART0_Write(*msg);
		msg++;
	}
}

unsigned char UART0_Read(void)
{
	while (!(U0LSR & 0x01))
		;
	return (U0RBR);
}

int Randoms(int lower, int upper)
{
	int num = (rand() % (upper - lower + 1)) + lower;
	return num;
}

void Delay_ms(unsigned long times)
{
	unsigned long i, j;
	for (j = 0; j < times; j++)
		for (i = 0; i < 7500; i++)
			;
}

void I2CInit(void)
{
	PINSEL0 |= 0x00000050;

	I2C0CONCLR = I2C_ENABLE | I2C_START | I2C_STOP | I2C_SI | I2C_AACK;

	I2C0SCLH = 0x12C;
	I2C0SCLL = 0x12C;

	I2C0CONSET = I2C_ENABLE;
}

void I2CStart(void)
{
	unsigned int status;
	I2C0CONCLR = (I2C_START | I2C_STOP | I2C_SI | I2C_AACK);
	I2C0CONSET = (I2C_ENABLE);
	I2C0CONSET = (I2C_START);
	while (!((status = I2C0CONSET) & I2C_SI))
		;
}

void I2CStop(void)
{
	unsigned int status;
	I2C0CONCLR = I2C_START | I2C_SI | I2C_AACK;
	I2C0CONSET = I2C_STOP;
}

void I2Csend(unsigned char data)
{
	unsigned int status;
	I2C0DAT = data;
	I2C0CONCLR = I2C_START | I2C_STOP;
	I2C0CONCLR = I2C_SI;
	while (!((status = I2C0CONSET) & I2C_SI))
		;
}

unsigned char I2Cget(void)
{
	unsigned char data;
	unsigned int status;

	I2C0CONCLR = I2C_START | I2C_STOP;
	I2C0CONCLR = I2C_SI;
	I2C0CONSET = I2C_AACK;
	while (!((status = I2C0CONSET) & I2C_SI))
		;
	data = I2C0DAT;
	return data;
}

void ADC0_Init(void)
{
	AD0CR = 1 << 21;
	AD0CR = 0 << 21;
	PCONP = (PCONP & 0x001817BE) | (1UL << 12);
	PINSEL1 |= 0x00400000;

	AD0CR = 0x00200401;
}

unsigned int ADC0_Read(void)
{
	unsigned long adc_data;

	AD0CR |= 1UL << 24;
	do
	{
		adc_data = AD0GDR;
	} while (!(adc_data & 0x80000000));

	AD0CR &= ~0x01000000;

	adc_data = adc_data >> 6;
	adc_data = adc_data & 0x3FF;
	adc_data = adc_data * 3300;
	adc_data = adc_data / 1023;

	return (adc_data);
}

int main()
{
	int write_array[1] = {26};
	int read_array[1], adc_data, temp;
	unsigned char val[4];
	int i;

	PID tempPID = {PID_KP, PID_KI, PID_KD,
				   PID_TAU,
				   PID_LIM_MIN, PID_LIM_MAX,
				   PID_LIM_MIN_INT, PID_LIM_MAX_INT,
				   SAMPLE_TIME_S};
	PIDController_Init(&tempPID);

	PINSEL0 |= 0x00000005;
	PINSEL0 |= (1 << 15);

	VICVectAddr0 = (unsigned)Interrupt;
	VICVectCntl0 = (0x00000020 | 8);
	VICIntEnable = VICIntEnable | 0x00000100;
	VICIntSelect = VICIntSelect | 0x00000000;

	UART0_Init();
	I2CInit();
	ADC0_Init();

	UART0_Write_Text("********* 19ELC301 Mini Project **********\n\n\r");
	//UART0_Write_Text("Initialization done. \n\r\n");

	UART0_Write_Text("\r\nWriting Data.....\r\n");
	I2CStart();
	I2Csend(EEPROM_Addr | I2Cwrite);
	I2Csend(0x13);
	I2Csend(0x49);
	for (i = 0; i < sizeof(write_array) / sizeof(int); i++)
	{
		I2Csend(write_array[i]);
	}
	I2CStop();

	for (i = 0; i < sizeof(write_array) / sizeof(int); i++)
	{
		sprintf(val, "%d", write_array[i]);
		UART0_Write_Text(val);
		UART0_Write_Text("\r\n");
	}

	UART0_Write_Text("\r\nReading.....\r\n");
	I2CStart();
	I2Csend(EEPROM_Addr | I2Cwrite);
	I2Csend(0x13);
	I2Csend(0x49);
	I2CStop();
	I2CStart();
	I2Csend(EEPROM_Addr | I2Cread);
	for (i = 0; i < sizeof(read_array) / sizeof(int); i++)
	{
		read_array[i] = I2Cget();
	}
	I2CStop();

	for (i = 0; i < sizeof(read_array) / sizeof(int); i++)
	{
		sprintf(val, "%d", read_array[i]);
		UART0_Write_Text(val);
		UART0_Write_Text("\r\n");
	}

	PWMPR = 12 - 1;
	PWMMR0 = 10000;
	PWMMR2 = 100;
	PWMMCR = (1 << 1);
	PWMLER = (1 << 0) | (1 << 4) | (1 << 2);
	PWMPCR = (1 << 10) | (1 << 12);
	PWMTCR = (1 << 1);
	PWMTCR = (1 << 0) | (1 << 3);

	temp = read_array[0];

	while (1)
	{
		adc_data = ADC0_Read() / 10;

		UART0_Write_Text("\r\n");

		sprintf(val, "T: %d", adc_data);
		UART0_Write_Text(val);

		PWMMR2 = PIDController_Update(&tempPID, temp, adc_data, 1) * 100;
		PWMLER = (1 << 2);

		Delay_ms(100);
	}

	return 0;
}
