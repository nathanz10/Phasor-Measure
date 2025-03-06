// ADC.c:  Shows how to use the 14-bit ADC.  This program
// measures the voltage from some pins of the EFM8LB1 using the ADC.
//
// (c) 2008-2023, Jesus Calvino-Fraga
//

#include <stdio.h>
#include <stdlib.h>
#include <EFM8LB1.h>
#include <math.h>

// ~C51~  

#define SYSCLK 72000000L
#define BAUDRATE 115200L
#define SARCLK 18000000L

#define ZERO_CROSS_PIN P0_0  // Alias for P0.0
#define ZERO_CROSS_PIN_PHASOR P0_1
#define REF_peak P2_2
#define PHA_peak P2_3

#define LCD_RS P1_7
// #define LCD_RW Px_x // Not used in this code. Connect to GND
#define LCD_E P2_0
#define LCD_D4 P1_3
#define LCD_D5 P1_2
#define LCD_D6 P1_1
#define LCD_D7 P1_0
#define CHARS_PER_LINE 16

unsigned int period_ticks = 0;
unsigned int quarter_period_ticks = 0;
//unsigned char overflow_count;


void Timer3us(unsigned char us);

void waitms(unsigned int ms);

void LCD_pulse(void)
{
    LCD_E = 1;
    Timer3us(40);
    LCD_E = 0;
}

void LCD_byte(unsigned char x)
{
    // The accumulator in the C8051Fxxx is bit-addressable
    ACC = x; // Send high nibble
    LCD_D7 = ACC_7;
    LCD_D6 = ACC_6;
    LCD_D5 = ACC_5;
    LCD_D4 = ACC_4;
    LCD_pulse();
    Timer3us(40);

    ACC = x; // Send low nibble
    LCD_D7 = ACC_3;
    LCD_D6 = ACC_2;
    LCD_D5 = ACC_1;
    LCD_D4 = ACC_0;
    LCD_pulse();
}

void WriteCommand(unsigned char x)
{
    LCD_RS = 0;
    LCD_byte(x);
    waitms(5);
}

void WriteData(unsigned char x)
{
    LCD_RS = 1;
    LCD_byte(x);
    waitms(2);
}

void LCD_4BIT(void)
{
    LCD_E = 0; // Resting state of LCD's enable is zero
    // LCD_RW = 0; // Not needed in this program, connect to GND
    waitms(20);

    // Ensure LCD starts in 8-bit mode before switching to 4-bit mode
    WriteCommand(0x33);
    WriteCommand(0x33);
    WriteCommand(0x32); // Switch to 4-bit mode

    // Configure LCD settings
    WriteCommand(0x28); // 4-bit mode, 2-line display, 5x8 font
    WriteCommand(0x0C); // Display ON, cursor OFF
    WriteCommand(0x01); // Clear screen command (takes some time)
    waitms(20); // Wait for the clear screen command to finish
}

void LCDprint(char *string, unsigned char line, bit clear)
{
    int j;

    // Set cursor position
    WriteCommand(line == 2 ? 0xC0 : 0x80);
    waitms(5);

    // Print the string
    for (j = 0; string[j] != 0; j++) 
        WriteData(string[j]);

    // Clear the rest of the line if required
    if (clear) 
        for (; j < CHARS_PER_LINE; j++) 
            WriteData(' ');
}

char _c51_external_startup (void)
{
	// Disable Watchdog with key sequence
	SFRPAGE = 0x00;
	WDTCN = 0xDE; //First key
	WDTCN = 0xAD; //Second key
  
	VDM0CN=0x80;       // enable VDD monitor
	RSTSRC=0x02|0x04;  // Enable reset on missing clock detector and VDD

	#if (SYSCLK == 48000000L)	
		SFRPAGE = 0x10;
		PFE0CN  = 0x10; // SYSCLK < 50 MHz.
		SFRPAGE = 0x00;
	#elif (SYSCLK == 72000000L)
		SFRPAGE = 0x10;
		PFE0CN  = 0x20; // SYSCLK < 75 MHz.
		SFRPAGE = 0x00;
	#endif
	
	#if (SYSCLK == 12250000L)
		CLKSEL = 0x10;
		CLKSEL = 0x10;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 24500000L)
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 48000000L)	
		// Before setting clock to 48 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x07;
		CLKSEL = 0x07;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 72000000L)
		// Before setting clock to 72 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x03;
		CLKSEL = 0x03;
		while ((CLKSEL & 0x80) == 0);
	#else
		#error SYSCLK must be either 12250000L, 24500000L, 48000000L, or 72000000L
	#endif
	
	P0MDOUT |= 0x10; // Enable UART0 TX as push-pull output
	XBR0     = 0x01; // Enable UART0 on P0.4(TX) and P0.5(RX)                     
	XBR1     = 0X00;
	XBR2     = 0x40; // Enable crossbar and weak pull-ups

	// Configure Uart 0
	#if (((SYSCLK/BAUDRATE)/(2L*12L))>0xFFL)
		#error Timer 0 reload value is incorrect because (SYSCLK/BAUDRATE)/(2L*12L) > 0xFF
	#endif
	SCON0 = 0x10;
	TH1 = 0x100-((SYSCLK/BAUDRATE)/(2L*12L));
	TL1 = TH1;      // Init Timer1
	TMOD &= ~0xf0;  // TMOD: timer 1 in 8-bit auto-reload
	TMOD |=  0x20;                       
	TR1 = 1; // START Timer1
	TI = 1;  // Indicate TX0 ready
  	
	return 0;
}



void InitADC (void)
{
	SFRPAGE = 0x00;
	ADEN=0; // Disable ADC
	
	ADC0CN1=
		(0x2 << 6) | // 0x0: 10-bit, 0x1: 12-bit, 0x2: 14-bit
        (0x0 << 3) | // 0x0: No shift. 0x1: Shift right 1 bit. 0x2: Shift right 2 bits. 0x3: Shift right 3 bits.		
		(0x0 << 0) ; // Accumulate n conversions: 0x0: 1, 0x1:4, 0x2:8, 0x3:16, 0x4:32
	
	ADC0CF0=
	    ((SYSCLK/SARCLK) << 3) | // SAR Clock Divider. Max is 18MHz. Fsarclk = (Fadcclk) / (ADSC + 1)
		(0x0 << 2); // 0:SYSCLK ADCCLK = SYSCLK. 1:HFOSC0 ADCCLK = HFOSC0.
	
	ADC0CF1=
		(0 << 7)   | // 0: Disable low power mode. 1: Enable low power mode.
		(0x1E << 0); // Conversion Tracking Time. Tadtk = ADTK / (Fsarclk)
	
	ADC0CN0 =
		(0x0 << 7) | // ADEN. 0: Disable ADC0. 1: Enable ADC0.
		(0x0 << 6) | // IPOEN. 0: Keep ADC powered on when ADEN is 1. 1: Power down when ADC is idle.
		(0x0 << 5) | // ADINT. Set by hardware upon completion of a data conversion. Must be cleared by firmware.
		(0x0 << 4) | // ADBUSY. Writing 1 to this bit initiates an ADC conversion when ADCM = 000. This bit should not be polled to indicate when a conversion is complete. Instead, the ADINT bit should be used when polling for conversion completion.
		(0x0 << 3) | // ADWINT. Set by hardware when the contents of ADC0H:ADC0L fall within the window specified by ADC0GTH:ADC0GTL and ADC0LTH:ADC0LTL. Can trigger an interrupt. Must be cleared by firmware.
		(0x0 << 2) | // ADGN (Gain Control). 0x0: PGA gain=1. 0x1: PGA gain=0.75. 0x2: PGA gain=0.5. 0x3: PGA gain=0.25.
		(0x0 << 0) ; // TEMPE. 0: Disable the Temperature Sensor. 1: Enable the Temperature Sensor.

	ADC0CF2= 
		(0x0 << 7) | // GNDSL. 0: reference is the GND pin. 1: reference is the AGND pin.
		(0x1 << 5) | // REFSL. 0x0: VREF pin (external or on-chip). 0x1: VDD pin. 0x2: 1.8V. 0x3: internal voltage reference.
		(0x1F << 0); // ADPWR. Power Up Delay Time. Tpwrtime = ((4 * (ADPWR + 1)) + 2) / (Fadcclk)
	
	ADC0CN2 =
		(0x0 << 7) | // PACEN. 0x0: The ADC accumulator is over-written.  0x1: The ADC accumulator adds to results.
		(0x0 << 0) ; // ADCM. 0x0: ADBUSY, 0x1: TIMER0, 0x2: TIMER2, 0x3: TIMER3, 0x4: CNVSTR, 0x5: CEX5, 0x6: TIMER4, 0x7: TIMER5, 0x8: CLU0, 0x9: CLU1, 0xA: CLU2, 0xB: CLU3

	ADEN=1; // Enable ADC
}

// Uses Timer3 to delay <us> micro-seconds. 
void Timer3us(unsigned char us)
{
	unsigned char i;               // usec counter
	
	// The input for Timer 3 is selected as SYSCLK by setting T3ML (bit 6) of CKCON0:
	CKCON0|=0b_0100_0000;
	
	TMR3RL = (-(SYSCLK)/1000000L); // Set Timer3 to overflow in 1us.
	TMR3 = TMR3RL;                 // Initialize Timer3 for first overflow
	
	TMR3CN0 = 0x04;                 // Sart Timer3 and clear overflow flag
	for (i = 0; i < us; i++)       // Count <us> overflows
	{
		while (!(TMR3CN0 & 0x80));  // Wait for overflow
		TMR3CN0 &= ~(0x80);         // Clear overflow indicator
	}
	TMR3CN0 = 0 ;                   // Stop Timer3 and clear overflow flag
}

void waitms (unsigned int ms)
{
	unsigned int j;
	unsigned char k;
	for(j=0; j<ms; j++)
		for (k=0; k<4; k++) Timer3us(250);
}

#define VDD 3.3035 // The measured value of VDD in volts

void InitPinADC (unsigned char portno, unsigned char pinno)
{
	unsigned char mask;
	
	mask=1<<pinno;

	SFRPAGE = 0x20;
	switch (portno)
	{
		case 0:
			P0MDIN &= (~mask); // Set pin as analog input
			P0SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 1:
			P1MDIN &= (~mask); // Set pin as analog input
			P1SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 2:
			P2MDIN &= (~mask); // Set pin as analog input
			P2SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		default:
		break;
	}
	SFRPAGE = 0x00;
}

unsigned int ADC_at_Pin(unsigned char pin)
{
	ADC0MX = pin;   // Select input from pin
	ADINT = 0;
	ADBUSY = 1;     // Convert voltage at the pin
	while (!ADINT); // Wait for conversion to complete
	return (ADC0);
}

float Volts_at_Pin(unsigned char pin)
{
	 return ((ADC_at_Pin(pin)*VDD)/0b_0011_1111_1111_1111);
}

/*void InitTimer0(void){
 
    TMOD&=0B_1111_1100; // Clear all the configuration bits of timer 0, but leave the bits of timer 1 unchanged
    TMOD|=0B_0000_0001; // Configure timer 0, but do nothing to timer 1
    TR0 = 0;       // Ensure Timer 0 is stopped initially
}*/

void InitTimer0(void)
{
    SFRPAGE = 0x00; // Ensure correct SFR page
    TR0 = 0;        // Stop Timer 0
    CKCON0 &= ~0x08; // Ensure Timer 0 uses SYSCLK / 12
    TMOD = (TMOD & 0xF0) | 0x01; // Set Timer 0 as 16-bit mode
    TH0 = 0; TL0 = 0; // Reset Timer 0
    TF0 = 0; // Clear overflow flag
}


/*unsigned int measurePeriod(){
    TR0 = 0;  // Stop Timer 0
    TH0 = 0; TL0 = 0; // Reset Timer 0

    while (ZERO_CROSS_PIN == 1); // Wait for signal to go LOW
    while (ZERO_CROSS_PIN == 0); // Wait for signal to go HIGH

    TR0 = 1;  // Start Timer 0

    while (ZERO_CROSS_PIN == 1); // Wait for signal to go LOW again

    TR0 = 0;  // Stop Timer 0

    return (TH0 << 8) | TL0; // Return 16-bit timer value
}*/

/*unsigned int halfPeriod(){

	//printf("1\n");
	
	TR0=0; // Stop timer 0
	
	//printf("SFRPAGE before TMOD: %X\n", SFRPAGE);

	//TMOD=0B_0000_0001; // Set timer 0 as 16-bit timer
	//printf("12\n");
	TH0=0; TL0=0; // Reset the timer
	//printf("13\n");
	
	
	while (ZERO_CROSS_PIN==1); // Wait for the signal to be zero
	while (ZERO_CROSS_PIN==0); // Wait for the signal to be one
	
	
	
	//printf("3\n");
	TR0=1; // Start timing
	//printf("3\n");
	while (ZERO_CROSS_PIN==1); // Wait for the signal to be zero
	
	//printf("4\n");
	TR0=0; // Stop timer 0
	// [TH0,TL0] is half the period in multiples of 12/CLK, so:
	
	//printf("Measured half-period: %u\n", (TH0 * 0x100 + TL0) * 2);
	//printf("Half-Period: %u ticks, Frequency: %.2f Hz\n", (TH0 * 0x100 + TL0) * 2, (float)SYSCLK / (((TH0 * 0x100 + TL0) * 2) * 24));


	return(TH0*0x100+TL0)*2;
	
}*/

/*
void WaitQuarterPeriod(unsigned int period)
{
    unsigned int delay_ticks = period / 4;
    
    TR0 = 0;  // Stop Timer 0
    TH0 = 0; TL0 = 0;  // Reset Timer 0

    TR0 = 1;  // Start Timer 0

    while (((TH0 << 8) | TL0) < delay_ticks);  // Wait for Period/4

    TR0 = 0;  // Stop Timer 0
}*/


/*void WaitQuarterPeriod(unsigned int period)
{
    unsigned int delay_ticks = period / 4;
    unsigned int timer_value = 0;
    unsigned int overflow_count = 0;

    TR0 = 0;  // Stop Timer 0
    TH0 = 0; TL0 = 0;  // Reset Timer 0
    TF0 = 0;  // Clear Timer Overflow Flag

    TR0 = 1;  // Start Timer 0

    while (timer_value < delay_ticks)
    {
        if (TF0)  // Check if overflow occurred
        {
            TF0 = 0;  // Clear Timer Overflow Flag
            overflow_count++;  // Track overflows
        }

        // Read current timer value
        timer_value = (overflow_count * 65536) + ((TH0 << 8) | TL0);
    }

    TR0 = 0;  // Stop Timer 0
}
*/

/*unsigned int phaseDif()
{
    
    TR0 = 0;  // Stop Timer 0
    TH0 = 0; TL0 = 0; // Reset Timer 0
	overflow_count = 0;
	
    while (ZERO_CROSS_PIN == 1){ // Wait for reference signal (P0.0) to go LOW
    	if(TF0){
    		TF0 = 0;
    		overflow_count++;
    	}
    }
    while (ZERO_CROSS_PIN == 0){ // Wait for reference signal to go HIGH
    	if(TF0){
    		TF0 = 0;
    		overflow_count++;
    	}
    }

    TR0 = 1;  // Start Timer 0
	overflow_count = 0;
	
	while (ZERO_CROSS_PIN_PHASOR == 1){ // Wait for phasor (P0.1) to go LOW
    	if(TF0){
    		TF0 = 0;
    		overflow_count++;
    	}
    }
    while (ZERO_CROSS_PIN_PHASOR == 0){ // Wait for phasor (P0.1) to go HIGH
    	if(TF0){
    		TF0 = 0;
    		overflow_count++;
    	}
    }

    TR0 = 0;  // Stop Timer 0

    return overflow_count*0x10000L+TH0*0x100+TL0; // Return 16-bit timer value
}
float calculatePhaseShift(unsigned int period_ticks, unsigned int delta_ticks) {
    //float time_per_tick = 12.0 / SYSCLK;  // Time per tick in seconds
    //float period_time = (float)period_ticks * time_per_tick;  // Convert period to seconds
    //float delta_time = delta_ticks * time_per_tick;    // Convert delta t to seconds
    
    // Compute phase shift in degrees
    //float phase_shift = (delta_time / period_time) * 360.0;
    float phase_shift = ((float)period_ticks / (float)delta_ticks) * 360.0;
    
    return phase_shift;  // Return phase shift in degrees
}*/

float calculatePhaseDifference(unsigned long period_ticks)
{
    unsigned int overflow_count = 0;
    unsigned long delta_ticks;
    float phase_diff;

    // Ensure Timer 0 is reset
    TR0 = 0;
    TH0 = 0;
    TL0 = 0;
    TF0 = 0;
    overflow_count = 0;

    // **Wait for Reference Signal Zero Crossing**
    while (ZERO_CROSS_PIN == 1) // Wait for signal to go LOW
    {
        if (TF0)
        {
            TF0 = 0;
            overflow_count++;
        }
    }
    while (ZERO_CROSS_PIN == 0) // Wait for signal to go HIGH
    {
        if (TF0)
        {
            TF0 = 0;
            overflow_count++;
        }
    }

    // **Start Timer for Phase Measurement**
    TR0 = 1;

    // **Wait for Phasor Signal Zero Crossing**
    while (ZERO_CROSS_PIN_PHASOR == 1) // Wait for signal to go LOW
    {
        if (TF0)
        {
            TF0 = 0;
            overflow_count++;
        }
    }
    while (ZERO_CROSS_PIN_PHASOR == 0) // Wait for signal to go HIGH
    {
        if (TF0)
        {
            TF0 = 0;
            overflow_count++;
        }
    }

    // **Stop Timer**
    TR0 = 0;

    // **Calculate Phase Difference**
    delta_ticks = (overflow_count * 65536L) + (TH0 * 256L) + TL0;
    phase_diff = ((float)delta_ticks / (float)period_ticks) * 360.0;

    // **Adjust Phase Difference to -180° to 180°**
    if (phase_diff > 180.0)
    {
        phase_diff -= 360.0;
    }

    return phase_diff;
}




void main (void)
{

	unsigned long T;
	float v[2];
	char buffer [16];
	float period;
	float frequency;
	unsigned char overflow_count;
	
	//unsigned int delta_ticks;
	float phase_shift;
	float peakRMS;
	float phaRMS;
	float printVal;
	

    waitms(500); // Give PuTTy a chance to start before sending
	printf("\x1b[2J"); // Clear screen using ANSI escape sequence.
	
	printf ("ADC test program\n"
	        "File: %s\n"
	        "Compiled: %s, %s\n\n",
	        __FILE__, __DATE__, __TIME__);
	
	InitPinADC(2, 2); // Configure P2.2 as analog input
	InitPinADC(2, 3); // Configure P2.3 as analog input
	InitPinADC(2, 4); // Configure P2.4 as analog input
	InitPinADC(2, 5); // Configure P2.5 as analog input
    InitADC();
    InitTimer0();
    LCD_4BIT();

	while(1)
	{
	    // Read 14-bit value from the pins configured as analog inputs
		v[0] = Volts_at_Pin(QFP32_MUX_P2_2);
		v[1] = Volts_at_Pin(QFP32_MUX_P2_3);
		//v[2] = Volts_at_Pin(QFP32_MUX_P2_4);
		//v[3] = Volts_at_Pin(QFP32_MUX_P2_5);
		//printf ("V@P2.2=%7.5fV, V@P2.3=%7.5fV, V@P2.4=%7.5fV, V@P2.5=%7.5fV\r\n", v[0], v[1], v[2], v[3]);

		
		
			TL0=0;
			TH0=0;
			overflow_count=0;
			TF0=0;
			
			while (ZERO_CROSS_PIN == 1){
				if(TF0){
    				TF0 = 0;
    				overflow_count++;
    			}
    		}
			while (ZERO_CROSS_PIN == 0){
				if(TF0){
    				TF0 = 0;
    				overflow_count++;
    			}
    		}
			TR0=1; // Start Timer/Counter 0
			overflow_count=0;
			while (ZERO_CROSS_PIN==1){
				if(TF0){
    				TF0 = 0;
    				overflow_count++;
    			}
    		}
    		/*while (ZERO_CROSS_PIN == 0){
				if(TF0){
    				TF0 = 0;
    				overflow_count++;
    			}
    		}*/
			TR0=0; // Stop Timer/Counter 0
			T=2.0*(overflow_count*0x10000L+TH0*0x100L+TL0);
			printf("\rperiod ticks: %lu \n", T);
			
			frequency = ((float)SYSCLK / (12.0 * (float)T));
		
			printf("\rf=%lfHz\n", frequency);
			
			sprintf(buffer, "%fhz", frequency);
			LCDprint(buffer, 1, 1);
		
			period = 1.0 / frequency; 
			
			printf("\rT=%lfs\n", period);
			
			
			
			while (ZERO_CROSS_PIN == 1);
			while (ZERO_CROSS_PIN == 0);
			//WaitQuarterPeriod(period);
			waitms((int)(period*250.0));
		
			peakRMS = Volts_at_Pin(QFP32_MUX_P2_2)/1.4142;//Volts_at_Pin(REF_peak); //p2.2
			printf("\nVrms(REF): %f V\n", peakRMS);
			
			sprintf(buffer, "%.3fVr", peakRMS);
			LCDprint(buffer, 2, 1);
			
			waitms(10);
			
			while (ZERO_CROSS_PIN_PHASOR == 1);
			while (ZERO_CROSS_PIN_PHASOR == 0);
			waitms((int)(period*250.0));
			
			phaRMS = Volts_at_Pin(QFP32_MUX_P2_3)/1.4142;//Volts_at_Pin(PHA_peak); //p2.3
			printf("\nVrms(PHA): %f V\n", phaRMS);
			
			sprintf(buffer, "%fVp", phaRMS);
			LCDprint(buffer, 2, 8);
			
			
			
			
			
			
			
			
			// Call function after measuring period
			phase_shift = calculatePhaseDifference(T);
			printVal = 2.0 - phase_shift;
			
			printf("\nPhase Shift: %f°\n", printVal);
			
			// Display on LCD
			sprintf(buffer, "P: %fd", printVal);
			LCDprint(buffer, 1, 8);

			
			
			/*delta_ticks = phaseDif();
			printf("\r delta ticks: %u \n",delta_ticks);
			printf("\rperiod ticks: %lu \n", T);
			phase_shift = calculatePhaseShift(T, (float)delta_ticks);
			printf("\nphase shift: %f °\n", phase_shift);*/
		
		
		waitms(500);
	 }  
}	