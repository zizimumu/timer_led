#include "AFE4400.h"
#include "delay.h"
/**************
PA1 -- ADCready
PA2 -- CS
PA3 -- RST
PA5 -- SCK
PA5 -- MISO
PA7 -- MOSI
******************/
#define AFEXX_CS_PIN 	GPIO_Pin_2
#define AFEXX_RST_PIN 	GPIO_Pin_3
#define AFEXX_GPIO 		GPIOA
#define AFEXX_CLOCK 	RCC_APB2Periph_GPIOA

#define AFEXX_CS_HIGH()	 AFEXX_GPIO->BSRR = AFEXX_CS_PIN
#define AFEXX_CS_LOW()	 AFEXX_GPIO->BRR = AFEXX_CS_PIN

#define AFEXX_RST_HIGH()	 AFEXX_GPIO->BSRR = AFEXX_RST_PIN
#define AFEXX_RST_LOW()	 AFEXX_GPIO->BRR = AFEXX_RST_PIN




#define zeros           6
#define poles           6
#define GAIN            2.659066659e+05
#define sampleRate      500
#define windowTime      3
#define STAGES          3

#define PACKET_LED_PORT_ADDRESS         __MSP430_BASEADDRESS_PORT1_R__
#define PACKET_LED_PORT_NUMBER          GPIO_PORT_P1
#define PACKET_LED_PIN                  GPIO_PIN4


unsigned char ScopeArra[16];

//Parameters for digital bandpass filter
//const int BITS = 27;
const int TRUNC_BITS = 25;
//const int KEEP_BITS = 27;

//const  long SCALE = 33554432;
//const double SOS [3][6] = {
//	{1,  0,  -1,  1,  -1.9725464130456409,  0.97381705538051955},
//	{1,  0,  -1,  1,  -1.9950869055123981,  0.99513037435061724},
//	{1,  0,  -1,  1,  -1.9688341956177746,  0.96906741719379641}};

const double s [4] = {0.015585406745858222, 0.015585406745858222, 0.015466291403101799, 1.0};


const long mx = 67108863;
const long mn = -67108864;


double spO2 = 0; //Variable to store calculated SpO2 value


uint32_t voltageCodeCounter = 0; //Counter for how many times DRDY is received

//Variables for heart rate measurement
int transitions[2] = {-1,-1};



long previousValue = 0;
long currentValue = 0;

//Variables for SpO2 measurement
long maxIRValue = 0;
long minIRValue = 1000000000;
long maxRedValue = 0;
long minRedValue = 1000000000;

int pulseStarted = 0; //Flag to indicate if at least 1 pulse is completed

unsigned char voltageCodeBuffer[20]; //Buffer to hold voltage code values to be sent over BLE
int bufferCounter = 0; //Counter for indexing into voltage code buffer

char filtered = 1; //Flag set to choose between filtered and unfiltered streams

unsigned int heartRate = 0;
unsigned int heartRate2 = 10;
unsigned int heartReport = 10;
unsigned int pulseOx = 0;

const  long SCALE = 33554432;
const double SOS [3][6] = {
	{1,  0,  -1,  1,  -1.9725464130456409,  0.97381705538051955},
	{1,  0,  -1,  1,  -1.9950869055123981,  0.99513037435061724},
	{1,  0,  -1,  1,  -1.9688341956177746,  0.96906741719379641}};




static long filter(long sample);
static unsigned int calcHeartRate(int start, int end);
static unsigned int calcPulseOx(long maxIR, long minIR, long maxRed, long minRed);
static void writeRegister(unsigned char address, unsigned long data);
static uint32_t readRegister(unsigned char address);
static void toggleChipSelect(void );
static void resetAFE4400(void );
static void enableWrite(void );
static void enableRead(void );
static uint32_t collectRED(void );
static uint32_t collectIR(void );
static long collectIRMinusAMBIR(void );


/**
* @brief Bandpass filter between 0.5 and 3Hz
*
* @param  sample a long
*
* @return  long
*/

static long filter(long sample)
{
//    const long b_int [3][3] = {
//	{(long) (SOS [0][0] * SCALE), (long) (SOS [0][1] * SCALE), (long) (SOS [0][2] * SCALE)},
//	{(long) (SOS [1][0] * SCALE), (long) (SOS [1][1] * SCALE), (long) (SOS [1][2] * SCALE)},
//	{(long) (SOS [2][0] * SCALE), (long) (SOS [2][1] * SCALE), (long) (SOS [2][2] * SCALE)}};
//
//    const long a_int [3][3] = {
//	{(long) (SOS [0][3] * SCALE), (long) (SOS [0][4] * SCALE), (long) (SOS [0][5] * SCALE)},
//	{(long) (SOS [1][3] * SCALE), (long) (SOS [1][4] * SCALE), (long) (SOS [1][5] * SCALE)},
//	{(long) (SOS [2][3] * SCALE), (long) (SOS [2][4] * SCALE), (long) (SOS [2][5] * SCALE)}};

	long b_int [3][3] = {0,0,0,0,0,0,0,0,0};
	long a_int [3][3] = {0,0,0,0,0,0,0,0,0};

	long s_int[4] = {0,0,0,0};

	int k;
	int j;


    static long dly [STAGES][2] = {{0,0}, {0,0}, {0,0}};
    long result, wn;
    long mysample = sample;
    long wa1, wa2, wa3;
    long wb1, wb2, wb3;

    int i;

	for(k=0; k<3; k++)
	{
		for(j=0; j<3;j++)
		{
			b_int[k][j] = (long) (SOS [k][j] * SCALE);
			a_int[k][j] = (long) (SOS [k][j+3] * SCALE);
		}
	}

//    long s_int [4] = {
//	(long) (s[0] * SCALE), (long) (s[1] * SCALE), (long) (s[2] * SCALE), (long) (s[3] * SCALE)};

	

	for(j=0;j<4;j++)
	{
		s_int[j] = (long) (s[j] * SCALE);
	}

  

    for (i = 0; i < STAGES; i++)
    {
            //2nd-order LCCDE code
            //(eqn 8)

            wa1 =  ((long long)mysample * s_int[i]) >> (TRUNC_BITS);
            wa2 = ((long long)a_int[i][1] * dly[i][0]) >> TRUNC_BITS;
            wa3 = ((long long)a_int[i][2] * dly[i][1]) >> TRUNC_BITS;
            wn = wa1 - wa2 - wa3;
          
            //(eqn 9)
            wb1 = ((long long)b_int[i][0] * wn) >> TRUNC_BITS;
            wb2 = ((long long)b_int[i][1] * dly[i][0]) >> TRUNC_BITS;
            wb3 = ((long long)b_int[i][2] * dly[i][1]) >> TRUNC_BITS;

            result = wb1 + wb2 + wb3;
           
            //Update filter buffers for stage i
            dly[i][1] = dly[i][0];
            dly[i][0] = wn;
            mysample = result; //in case we have to loop again


    }
    
 return (long)result;

}

/**
* @brief Calculate heart rate given start and end of pulse
*
* @param  start an int
*
* @param  end an int
*
* @return  unsigned int
*/

static unsigned int calcHeartRate(int start, int end)
{
  int pulseLength = 0;
  double tempHeartRate = 0;
  int heartRate = 0;
  
  if(start > end) end += 3000; //In case end index of pulse wrapped around 6 -second window
  pulseLength = end - start; //Calculate length of pulse based on start and end indices

  //Check if this is reasonable pulse length
  if((pulseLength >= sampleRate/4) && (pulseLength < sampleRate*3))   // 125 -- 1500  : 4 - 0.3Hz
  {
    
    tempHeartRate = (60.00 * sampleRate)/ pulseLength;
    
    tempHeartRate *= 100; //Multiply by 100 to maintain 2 decimal points when casting to integer
    
    heartRate = (int)tempHeartRate; //Cast down to integer to send over BLE
    
    return heartRate;
  }
  
  return 0; //Return 0 for invalid pulse length

}

/**
* @brief Calculate SpO2 given max and min Red and IR values
*
* @param  maxIR a long
*
* @param  minIR a long
*
* @param  maxRed a long
*
* @param  minRed a long
*
* @return  unsigned int
*/

static unsigned int calcPulseOx(long maxIR, long minIR, long maxRed, long minRed)
{
  double irDC = minIR;
  double irAC = maxIR - minIR;
  double redDC = minRed;
  double redAC = maxRed - minRed;
  double tempSpO2;
  int pulseOx;
  
  double ratio = (redAC/redDC)/(irAC/irDC);
  
  spO2 = 110 - 25*ratio;
  
  tempSpO2 = 100*spO2; //Multiply by 100 to maintain 2 decimal points when casting down to integer
  
  pulseOx = (int)tempSpO2; //Cast to int to transfer over BLE
  
  return pulseOx;
  
}








/**
* @brief  Writes to a register on the AFE4300
*
* @param  address an unsigned character
* @param  data an unsigned long
*
* @return  None
*/
static void writeRegister(unsigned char address, unsigned long data)
{
  unsigned char firstByte = (unsigned char)(data >> 16);
  unsigned char secondByte = (unsigned char)(data >> 8);
  unsigned char thirdByte = (unsigned char)data;
  
  //Specify address of register to be written to
  SPI_UserReadWriteByte(address);

  delay_us(5);
 
  //Send 3 bytes to be written
  SPI_UserReadWriteByte(firstByte);

  delay_us(5);
  SPI_UserReadWriteByte(secondByte);

  delay_us(5);
  SPI_UserReadWriteByte(thirdByte);
  delay_us(5);
  
  toggleChipSelect();
}

/**
* @brief  Reads from a register on the AFE4300 and returns the 24-bit value
*
* @param  address an unsigned character
*
* @return  spiReceive a long
*/
static uint32_t readRegister(unsigned char address)
{
 
  uint32_t spiReceive;
  unsigned char spiReceiveFirst;
  unsigned char spiReceiveSecond;
  unsigned char spiReceiveThird;


  SPI_UserReadWriteByte(address);

  delay_us(5);
 
  //Send 3 bytes to be written
  spiReceiveFirst = SPI_UserReadWriteByte(0x00);

  delay_us(5);
  spiReceiveSecond = SPI_UserReadWriteByte(0x00);

  delay_us(5);
  spiReceiveThird = SPI_UserReadWriteByte(0x00);
  delay_us(5);

  //Combine the three received bytes into a signed long
  spiReceive = (((uint32_t)spiReceiveFirst )<< 8);
  spiReceive |= spiReceiveSecond;
  spiReceive = spiReceive << 8;
  spiReceive |= spiReceiveThird;
  
  toggleChipSelect();
  
  return spiReceive;
}


/* @brief  Toggles Chips Select (CS)
*
* @param  None
*
* @return  None
*/
static void toggleChipSelect(void )
{
  //Pull CS high to block MISO
	AFEXX_CS_HIGH();
 	delay_us(5);
   //Pull CS low for next instruction
	AFEXX_CS_LOW();

}


static void resetAFE4400(void )
{
	//Reset the device
	AFEXX_RST_LOW();
	delay_us(500);

	//Pull Reset high to return to normal mode
	AFEXX_RST_HIGH();
	AFEXX_CS_LOW();
	
	delay_us(500);
	writeRegister(CONTROL0, 0x000008);
	

}

/** 
* @brief  Sets register write mode of AFE4400
*
* @param  None
*
* @return  None
*/
static void enableWrite(void )
{
  writeRegister(CONTROL0, 0x000000);
}


/** 
* @brief  Sets register read mode of AFE4400
*
* @param  None
*
* @return  None
*/
static void enableRead(void )
{
  writeRegister(CONTROL0, 0x000001);
}




/** 
* @brief  Collects the various data values and stores them in arrays
*
* @details Reads from Red, Ambient Red, IR, Ambient IR, Red - Ambient & IR - Ambient registers
*
* @param  None
*
* @return  None
*/


/**
* @brief Read the red value in the register
*
* @param  None
*
* @return  long
*/
static uint32_t collectRED(void )
{
  return readRegister(REDVALUE);
}

/**
* @brief Read the IR value in the register
*
* @param  None
*
* @return  long
*/
static uint32_t collectIR(void )
{
  return readRegister(IRVALUE);
}
        
/**
* @brief Read the IR-AMBIENT value in the register
*
* @param  None
*
* @return  long
*/
static long collectIRMinusAMBIR(void )
{
  return readRegister(IRMINUSAMBIRVALUE);
}


/** 
* @brief  Initializes the AFE4400 device
*
* @param  None
*
* @return  None
*/
void initAFE4400(void)
{
  
  resetAFE4400();
  
  enableWrite();
  
  writeRegister(REDSTARTCOUNT,0x001770);  //sets the start timing value for the LED2 signal sample  : 6000
  writeRegister(REDENDCOUNT,0x001DAF);    //7599
  writeRegister(REDLEDSTARTCOUNT,0x001770); //sets the start timing value for when the LED2 signal turns on: 6000
  writeRegister(REDLEDENDCOUNT,0x001DB0);	//7600
  writeRegister(AMBREDSTARTCOUNT,0x000000); //sets the start timing value for the ambient LED2 signal sample
  writeRegister(AMBREDENDCOUNT,0x00063F);   //1599
  
  writeRegister(IRSTARTCOUNT,0x0007D0);   //sets the start timing value for the LED1 signal sample: 2000
  writeRegister(IRENDCOUNT,0x000E0F);     //3599
  writeRegister(IRLEDSTARTCOUNT,0x0007D0);//sets the start timing value for when the LED1 signal turns on: 2000
  writeRegister(IRLEDENDCOUNT,0x000E10);  //3600
  writeRegister(AMBIRSTARTCOUNT,0x000FA0);//400
  writeRegister(AMBIRENDCOUNT,0x0015DF);	//5599
  
  writeRegister(REDCONVSTART,0x000002);	// 2
  writeRegister(REDCONVEND,0x0007CF);	//1999
  writeRegister(AMBREDCONVSTART,0x0007D2);//2002
  writeRegister(AMBREDCONVEND,0x000F9F);//3999
  writeRegister(IRCONVSTART,0x000FA2);//4002
  writeRegister(IRCONVEND,0x00176F);//5999
  writeRegister(AMBIRCONVSTART,0x001772);//6002
  writeRegister(AMBIRCONVEND,0x001F3F);//7999
  
  writeRegister(ADCRESETSTCOUNT0,0x000000);
  writeRegister(ADCRESETENDCOUNT0,0x000000);
  writeRegister(ADCRESETSTCOUNT1,0x0007D0);//2000
  writeRegister(ADCRESETENDCOUNT1,0x0007D0);//2000
  writeRegister(ADCRESETSTCOUNT2,0x000FA0);//400
  writeRegister(ADCRESETENDCOUNT2,0x000FA0);//400
  writeRegister(ADCRESETSTCOUNT3,0x001770);//6000
  writeRegister(ADCRESETENDCOUNT3,0x001770);//6000
  
  writeRegister(PRPCOUNT,0x001F3F);    //7999
  writeRegister(CONTROL1,0x000107);		//263
  writeRegister(TIAGAIN,0x000000);
  writeRegister(TIA_AMB_GAIN,0x000000);
  writeRegister(LEDCNTRL,0x011414); //LED current = 5mA
  
  writeRegister(CONTROL2,0x000000);
  writeRegister(MODE,0x000000);
 
  enableRead(); //Enable reading again to read out data

  DebugPrint("AFE Init done\r\n");
  

}


void AFEXX_IO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(	AFEXX_CLOCK, ENABLE );	
	/* Configure I/O for Flash Chip select */
	GPIO_InitStructure.GPIO_Pin = AFEXX_CS_PIN|AFEXX_RST_PIN;  //SPI CS , rst
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //¸´ÓÃÍÆÍìÊä³ö
	GPIO_Init(AFEXX_GPIO, &GPIO_InitStructure);
	
	AFEXX_CS_HIGH();
}


void AFEXX_processData(void )
{
	uint32_t index;
	uint32_t irSample;
	uint32_t redSample;
	long filtIRSample;

	

     voltageCodeCounter++;
    
    //Start calculating data after skipping 2 seconds,ADCready is received per 2ms
    if(voltageCodeCounter > 1000)
    {
       index = voltageCodeCounter - 1001;
       
       //Read Red and IR values from AFE4400
       irSample = collectIR();
       redSample = collectRED();
       
       //Filter the IR value
       filtIRSample = filter(irSample);


       //Find max and min IR and Red values in the current pulse
       if(pulseStarted)
       {
         if(irSample > maxIRValue) maxIRValue = irSample;
         if(irSample < minIRValue) minIRValue = irSample;
         if(redSample > maxRedValue) maxRedValue = redSample;
         if(redSample < minRedValue) minRedValue = redSample;
       }

       previousValue = currentValue;
       currentValue = filtIRSample;
       
       //If there was a transition from negative to positive in the filtered data
       if(previousValue < 0 && currentValue > 0)
       {
         if(transitions[0] == -1) transitions[0] = index; //If this is the first transition
         else if(transitions[1] == -1) //If this is the second transition
         {
           transitions[1] = index;
           pulseStarted = 1; //Set pulse started flag only after 2 confirmed transitions
         }
         else
         {
           //Keep indices of last two transitions to estimate length of pulse
           transitions[0] = transitions[1];
           transitions[1] = index;
           
           //Call signal functions to calculate heart rate and SpO2
           heartRate = calcHeartRate(transitions[0], transitions[1]);
           pulseOx = calcPulseOx(maxIRValue, minIRValue, maxRedValue, minRedValue);
           
           
           //Reset maximum and minimum defaults for comparison in next cycle
           maxIRValue = 0;
           minIRValue = 1000000000;
           maxRedValue = 0;
           minRedValue = 1000000000;
         }
       }


	   
	   ScopeArra[0] =(unsigned char) filtIRSample;
	   ScopeArra[1] =(unsigned char) (filtIRSample>>8);
	   ScopeArra[2] = (unsigned char)heartRate;
	   ScopeArra[3] = (unsigned char)(heartRate>>8);
	   SendToScope(ScopeArra);
      
    }
    
    //Build a filter history for the initial few samples
    else
    {
      filter(collectIRMinusAMBIR());
    }
}

