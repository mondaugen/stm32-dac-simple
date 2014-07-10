/* Get Cirrus DAC to beep using STM32F4 */

/* Includes ------------------------------------------------------------------*/
#include <math.h> 
#include <stdlib.h> 
/* This has to be included before core_cm4 because it tells it our interrupt
 * table */
#include "stm32f4xx.h"
#include <core_cm4.h> 
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h" 
#include "synths.h" 

/* In combination with
 * PLLI2S_N  =  192
 * PLLI2S_R  =  5
 * these will achieve a sampling rate of 48000
 */
#define I2SDIV      12
#define I2SODD      1 
#define SAMPLING_RATE SYNTHS_SR
#define RANDOM_SEED 1000 

#define NUM_SYNTHS 3

/* Play sine tone at 440 Hz */
#define SINE_TONE_FREQ 220 

/* generates a sine tone at a specific frequency and sample rate */
#define sineTone_MACRO(phase, freq, sr) \
    sin(2 * M_PI * (phase)); phase += freq / sr; while (phase > 1.) { phase -= 1.; };

/* map a a floating point value in between -1 and 1 to uint16_t */
#define FLT_TO_UINT16(x) ((uint16_t)(0xffff*((x + 1.) * 0.5)))

/* map a a floating point value in between -1 and 1 to int16_t */
#define FLT_TO_INT16(x) ((int16_t)(0x7fff*x))


/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup IO_Toggle
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
GPIO_InitTypeDef    GPIO_InitStructure;
I2C_InitTypeDef     I2C_InitStruct;

void Delay(__IO uint32_t nCount);
double sineTone(double *phase, double freq, double sr);


/* Interrupt definition */
extern void SPI3_IRQHandler(void);
/* Another interrupt def */
extern void EXTI0_IRQHandler(void);

// this slave address belongs to the STM32F4-Discovery board's 
// CS43L22 Audio DAC
// connect PD4 to VDD in order to get the DAC out of reset and test the I2C
// interface
#define CS43L22_ADDRESS 0x4A // the slave address (example)

/* the synth */
WaveArraySynth wasL, wasR;
/* Pointers to functions that generate sound, for each channel */
double (*pfdv_soundL)(void);
double (*pfdv_soundR)(void);

static int16_t stupidSaw(int16_t *phase)
{
    *phase = *phase + 1;
    return *phase;
}


static double WaveArraySynth_soundL(void)
{
    return wasL.waveArrayTick(&wasL);
}

static double WaveArraySynth_soundR(void)
{
    return wasR.waveArrayTick(&wasR);
}

static double simpleSineL(void)
{
    static double phase;
    return sineTone(&phase,220.5,SAMPLING_RATE) * 0.99;
//    return squareTone(&phase,220.5,SAMPLING_RATE);
//    return sawTone(&phase,((double)SAMPLING_RATE) / ((double)(1l << 16)),SAMPLING_RATE);
}

static double simpleSineR(void)
{
    static double phase;
    return sineTone(&phase,220.5,SAMPLING_RATE) * 0.99;
//    return squareTone(&phase,220.5,SAMPLING_RATE);
//    return sawTone(&phase,220.5,SAMPLING_RATE);
}

static double simpleADCRead(void)
{
//    return (double)ADC_GetConversionValue(ADC1) / 0x1000 - 0.5;
    static int lr = 0;
    lr = 1-lr;
    if (lr) {
        GPIO_SetBits(GPIOC,GPIO_Pin_3);
    } else {
        GPIO_ResetBits(GPIOC,GPIO_Pin_3);
    }
    return 0;
}

void I2C1_init(void){
	
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;
	
	// enable APB1 peripheral clock for I2C1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	// enable clock for SCL and SDA pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	/* setup SCL and SDA pins
	 * You can connect the I2C1 functions to two different
	 * pins:
	 * 1. SCL on PB6 or PB8  
	 * 2. SDA on PB7 or PB9
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_9; // we are going to use PB6 and PB9
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;			// set pins to alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// set GPIO speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;			// set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// enable pull up resistors
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// init GPIOB
	
	// Connect I2C1 pins to AF  
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);	// SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1); // SDA
	
	// configure I2C1 
	I2C_InitStruct.I2C_ClockSpeed = 100000; 		// 100kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;			// I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;	// 50% duty cycle --> standard
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;			// own address, not relevant in master mode
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;		// disable acknowledge when reading (can be changed later on)
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
	// enable I2C1
	I2C_Cmd(I2C1, ENABLE);

	I2C_Init(I2C1, &I2C_InitStruct);				// init I2C1
	
}

void I2S_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct, GPIOA_InitStruct;
    I2S_InitTypeDef I2S3_InitStruct;
    /* Enable APB1 peripheral clock for I2S3 (SPI3) */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3 | RCC_APB1Periph_I2C1, ENABLE);
    /* Enable AHB1 peripheral clock for GPIOC */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    /* Enable AHB1 peripheral clock for GPIOA */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /* Init I2S PLL */
    RCC_PLLI2SCmd(ENABLE);

    /* Setup MCLK, SCLK and SDIN pins */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_10 | GPIO_Pin_12;
    /* set to use alternate function */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 
    /* Just chose the fastest speed */
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 
    /* open/drain, line only has to be pulled low? using the same as for I2C*/
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    /* Enable pull up resistors, also don't know why, using the same as I2C */
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    /* initialize GPIOC */
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Setup LRCK/AINx pin */
    GPIOA_InitStruct.GPIO_Pin = GPIO_Pin_4;
    /* set to use alternate function */
    GPIOA_InitStruct.GPIO_Mode = GPIO_Mode_AF; 
    /* Just chose the fastest speed */
    GPIOA_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 
    /* open/drain, line only has to be pulled low? using the same as for I2C*/
    GPIOA_InitStruct.GPIO_OType = GPIO_OType_PP; /* this has to be push/pull, open/drain didn't work */
    /* Enable pull up resistors, also don't know why, using the same as I2C */
    GPIOA_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    /* initialize GPIOC */
    GPIO_Init(GPIOA, &GPIOA_InitStruct);
    
    /* Connect I2S3 (SPI3) pins to GPIO's Alternate Function */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_SPI3); /* MCLK */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3); /* SCLK */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3); /* SDIN */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI3); /* LRCK/AINx */

    /* Configure I2S3 (SPI3) */
    SPI_I2S_DeInit(SPI3);
    I2S3_InitStruct.I2S_Mode = I2S_Mode_MasterTx;
    I2S3_InitStruct.I2S_Standard = I2S_Standard_Phillips;
    I2S3_InitStruct.I2S_DataFormat = I2S_DataFormat_16b;
    I2S3_InitStruct.I2S_MCLKOutput = I2S_MCLKOutput_Enable;
    I2S3_InitStruct.I2S_AudioFreq = SAMPLING_RATE;
    I2S3_InitStruct.I2S_CPOL = I2S_CPOL_Low; /* I don't know, low I guess. */
    I2S_Init(SPI3, &I2S3_InitStruct);

    /* Enable transmitter interrupt in NVIC */
    NVIC_EnableIRQ(SPI3_IRQn);

    /* Enable transmit buffer empty interrupt */
    SPI_ITConfig(SPI3, SPI_I2S_IT_TXE, ENABLE);


}


/* This function issues a start condition and 
 * transmits the slave address + R/W bit
 * 
 * Parameters:
 * 		I2Cx --> the I2C peripheral e.g. I2C1
 * 		address --> the 7 bit slave address
 * 		direction --> the transmission direction can be:
 * 						I2C_Direction_Tranmitter for Master transmitter mode
 * 						I2C_Direction_Receiver for Master receiver
 */
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction){
	// wait until I2C1 is not busy any more
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
  
	// Send I2C1 START condition 
	I2C_GenerateSTART(I2Cx, ENABLE);
	  
	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
		
	// Send slave Address for write 
	I2C_Send7bitAddress(I2Cx, address, direction);
	  
	/* wait for I2Cx EV6, check if 
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */ 
	if(direction == I2C_Direction_Transmitter){
		while(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)
                == ERROR);
	}
	else if(direction == I2C_Direction_Receiver){
		while(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)
                == ERROR);
	}
}

/* This function transmits one byte to the slave device
 * Parameters:
 *		I2Cx --> the I2C peripheral e.g. I2C1 
 *		data --> the data byte to be transmitted
 */
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
	// wait for I2C1 EV8 --> last byte is still being transmitted (last byte in SR, buffer empty), next byte can already be written
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING));
	I2C_SendData(I2Cx, data);
}

/* This function reads one byte from the slave device 
 * and acknowledges the byte (requests another byte)
 */
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx){
	// enable acknowledge of received data
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

/* This function reads one byte from the slave device
 * and doesn't acknowledge the received data 
 * after that a STOP condition is transmitted
 */
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx){
	// disable acknowledge of received data
	// nack also generates stop condition after last byte received
	// see reference manual for more info
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	I2C_GenerateSTOP(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

/* This function issues a stop condition and therefore
 * releases the bus
 */
void I2C_stop(I2C_TypeDef* I2Cx){
	
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	// Send I2C1 STOP Condition after last byte has been transmitted
	I2C_GenerateSTOP(I2Cx, ENABLE);
}

/* configure LEDs to show some information */
void led_init()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    GPIO_InitTypeDef GPIOD_InitStructure;
    GPIOD_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12;
    GPIOD_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIOD_InitStructure.GPIO_OType = GPIO_OType_PP; /* What happens with open/drain? */
    GPIOD_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIOD_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIOD_InitStructure);
}

void I2C_send_bytes(I2C_TypeDef* I2Cx, uint8_t address, uint8_t *bytes, size_t numBytes)
{
    I2C_start(I2Cx, address, I2C_Direction_Transmitter);
    while (numBytes-- > 0) {
        I2C_write(I2Cx, *bytes++);
    }
    I2C_stop(I2Cx);
}

/* just gets the bytes, this has to be initializes with the CS43L22 by writing
 * the address of the register you want to read from, stopping and then by
 * calling this command and reading */
void I2C_read_bytes(I2C_TypeDef* I2Cx, uint8_t address, uint8_t *bytes, size_t numBytes)
{
    I2C_start(I2Cx, address, I2C_Direction_Receiver);
    while (numBytes-- > 1) {
        *bytes++ = I2C_read_ack(I2Cx); /* read but don't signal last byte */
    }
    *bytes++ = I2C_read_nack(I2Cx);    /* signal last byte */
    I2C_stop(I2Cx);
}

/* read register from address map. If numBytes > 1, sets 7th bit of map high to read
 * contiguous registers */
void CS43L22_read_bytes(I2C_TypeDef* I2Cx, uint8_t map, uint8_t *bytes, size_t numBytes)
{
    if (numBytes > 1) {
        map |= 0x80;
    }
    I2C_send_bytes(I2Cx, CS43L22_ADDRESS<<1, &map, 1);
    I2C_read_bytes(I2Cx, CS43L22_ADDRESS<<1, bytes, numBytes);
}


/* CS43L22 needs reset pin high to work. Enable I2S before calling this so that
 * the clock validation is meaningful. */
void CS43L22_init(I2C_TypeDef* I2Cx)
{
    uint8_t buffer[2]; /* holds the commands */

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    GPIO_InitTypeDef GPIOD_InitStructure;
    GPIOD_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIOD_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIOD_InitStructure.GPIO_OType = GPIO_OType_PP; /* What happens with open/drain? */
    GPIOD_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIOD_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIOD_InitStructure);

    /* bring RESET low then high */
    GPIO_ResetBits(GPIOD, GPIO_Pin_4);
    GPIO_SetBits(GPIOD, GPIO_Pin_4);

    /* wait until serial clock ready */
    uint32_t delaycount = 1000000;
    while(delaycount-- > 0);
    /*
    do {
        CS43L22_read_bytes(I2Cx, 0x2e, buffer, 1);
    } while (buffer[0] & 0x40);
    */

    /* see CS4L22 data sheet for register descriptions. First byte sent is
     * address, second is value to be written into register */

    /* turn off (Power Ctl 1) */
    buffer[0] = 0x02; buffer[1] = 0x01;
    I2C_send_bytes(I2Cx, CS43L22_ADDRESS << 1, buffer, 2);

    /* initialization sequence from CS43L22 datasheet p. 32 */
    buffer[0] = 0x00; buffer[1] = 0x99;
    I2C_send_bytes(I2Cx, CS43L22_ADDRESS << 1, buffer, 2);
    buffer[0] = 0x47; buffer[1] = 0x80;
    I2C_send_bytes(I2Cx, CS43L22_ADDRESS << 1, buffer, 2);
    buffer[0] = 0x32; buffer[1] = 0x80;
    I2C_send_bytes(I2Cx, CS43L22_ADDRESS << 1, buffer, 2);
    buffer[0] = 0x32; buffer[1] = 0x00;
    I2C_send_bytes(I2Cx, CS43L22_ADDRESS << 1, buffer, 2);
    buffer[0] = 0x00; buffer[1] = 0x00;
    I2C_send_bytes(I2Cx, CS43L22_ADDRESS << 1, buffer, 2);
    /* should be set up now, power ctl 1 is still 'off' though*/

}

void GPIO3_config(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef GPIOC_InitStructure;
    GPIOC_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIOC_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIOC_InitStructure.GPIO_OType = GPIO_OType_PP; /* What happens with open/drain? */
    GPIOC_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIOC_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIOC_InitStructure);
}

void rng_init(void)
{
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
    RNG_Cmd(ENABLE);
}
 
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
        system_stm32f4xx.c file
     */

    uint8_t buffer[8];
    size_t i;
    WaveArray *waL, *waR;

    waL = WaveArray_new(NUM_SYNTHS);
    waR = WaveArray_new(NUM_SYNTHS);

    /* Initialize internal random number generator */
    rng_init();
    /* wait for number to be ready */
    while (!RNG_GetFlagStatus(RNG_FLAG_DRDY));

    srand48(RNG_GetRandomNumber());

    for (i = 0; i < NUM_SYNTHS; i++) {
        waL->waves[i].freq = drand48() * 2000;
        waL->waves[i].amp  = 1. / NUM_SYNTHS;
    }

    for (i = 0; i < NUM_SYNTHS; i++) {
        waR->waves[i].freq = drand48() * 2000;
        waR->waves[i].amp  = 1. / NUM_SYNTHS;
    }

    WaveArraySynth_init_sines(&wasL, waL);
    WaveArraySynth_init_sines(&wasR, waR);

    /*
    pfdv_soundL = &WaveArraySynth_soundL;
    pfdv_soundR = &WaveArraySynth_soundR;
    */
    /*
    pfdv_soundL = &simpleSineL;
    pfdv_soundR = &simpleSineR;
    */
    pfdv_soundL = &simpleADCRead;
    pfdv_soundR = &simpleADCRead;

    GPIO_InitTypeDef ADC_GPIO_InitStruct;
    ADC_InitTypeDef ADC_InitStruct;
    ADC_CommonInitTypeDef ADC_CommonInitStruct;

    /* ADC interface clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    /* Enable ADC GPIO clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /* Configure GPIO for ADC */
    ADC_GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
    ADC_GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
    ADC_GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &ADC_GPIO_InitStruct);

    ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStruct);

    /* Configure ADC */
    ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStruct.ADC_ScanConvMode = DISABLE;
    ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStruct.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1,&ADC_InitStruct);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_3Cycles);

    /* Enable ADC */
    ADC_Cmd(ADC1,ENABLE);

    ADC_ContinuousModeCmd(ADC1,ENABLE);

    ADC_SoftwareStartConv(ADC1);

    GPIO3_config();

	I2C1_init(); // initialize I2C peripheral

    /* init I2S */
    I2S_init();

    /* start I2S */
    I2S_Cmd(SPI3, ENABLE);

    CS43L22_init(I2C1); // initialize dac periph

    led_init(); // init leds

    /* configure for I2S */
    buffer[0] = 0x06;
    buffer[1] = 0x07;
    I2C_send_bytes(I2C1, CS43L22_ADDRESS << 1, buffer, 2);

    /* configure MCLK */
    buffer[0] = 0x07;
    buffer[1] = 0x00; /* synchronously retimed... */
    I2C_send_bytes(I2C1, CS43L22_ADDRESS << 1, buffer, 2);

    /* configure headphones and speaker */
    buffer[0] = 0x04;
    buffer[1] = 0xaa;
    I2C_send_bytes(I2C1, CS43L22_ADDRESS << 1, buffer, 2);

    /* set volume and gang all volume controls */
    buffer[0] = 0x0d;
    buffer[1] = 0x70;
    I2C_send_bytes(I2C1, CS43L22_ADDRESS << 1, buffer, 2);

    /* set CIRRUS to power on state */
    buffer[0] = 0x02;
    buffer[1] = 0x9e;
    I2C_send_bytes(I2C1, CS43L22_ADDRESS << 1, buffer, 2);

	while(1);
	return 0;

}

/**
  * @brief  Delay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */
void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}

void EXTI0_IRQHandler(void)
{
    NVIC_ClearPendingIRQ(EXTI0_IRQn);
    NVIC_DisableIRQ(EXTI0_IRQn);
    GPIOD->ODR ^=  GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
    EXTI->PR = 0x00000001;
    NVIC_EnableIRQ(EXTI0_IRQn);
}

int16_t simpleTone()
{
    static int16_t state = 0x7fff;
    state *= -1;
    return state;
}
    

/* Figures out why was called then remedies. So far probably just fills the
 * buffer with values to transmit */
void SPI3_IRQHandler(void)
{
    uint16_t data;
    /* Check that transmit buffer empty */
    if (SPI3->SR & (uint32_t)SPI_SR_TXE) {
        /* If so, fill with data */
        if (SPI3->SR & (uint32_t)SPI_SR_CHSIDE) {
            data = FLT_TO_INT16(pfdv_soundR());
            SPI_I2S_SendData(SPI3,data);

        } else {
            data = FLT_TO_INT16(pfdv_soundL());
            SPI_I2S_SendData(SPI3,data);
        }
    }

}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
