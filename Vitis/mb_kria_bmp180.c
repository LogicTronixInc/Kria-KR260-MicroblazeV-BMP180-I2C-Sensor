/** 
 * @brief Temperature acquisition with I2C sensor BMP180
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include "platform.h"
#include "xil_printf.h"
#include "xiic.h"

// Base address of the AXI I2C IP
#define AXI_IIC_BASE_ADDR		XPAR_AXI_IIC_0_BASEADDR 

// Address of the I2C sensor on the I2C bus
#define IIC_SLAVE_ADDR			0x77 

// Registers addresses
#define ID_REG_ADDR				0xD0
#define CONTROL_REG_ADDR		0xF4
#define CALIB_REG_START_ADDR	0xAA

// Calibration data registers
#define CAL_AC1_REG		0xAA
#define CAL_AC2_REG		0xAC
#define CAL_AC3_REG		0xAE
#define CAL_AC4_REG		0xB0
#define CAL_AC5_REG		0xB2
#define CAL_AC6_REG		0xB4
#define CAL_B1_REG		0xB6
#define CAL_B2_REG		0xB8
#define CAL_MB_REG		0xBA
#define CAL_MC_REG		0xBC
#define CAL_MD_REG		0xBE

// Standard atmospheric pressure
#define atm_pressure 101325 

// Buffers to write/read values to I2C device
u8 SendBuffer [2];
u8 RecvBuffer [3];
u8 Calib_Data [22];

// Calibration data variables
short AC1 = 0;
short  AC2 = 0;
short  AC3 = 0;
unsigned short  AC4 = 0;
unsigned short  AC5 = 0;
unsigned short  AC6 = 0;
short  B1 = 0;
short  B2 = 0;
short  MB = 0;
short  MC = 0;
short  MD = 0;

// Intermediates variables used to calculate temperature and pressure
long X1 = 0;
long X2 = 0;
long X3 = 0;
long B3 = 0;
unsigned long B4 = 0;
long B5 = 0;
long B6 = 0;
unsigned long B7 = 0;

// Oversampling ratio of the pressure measurement
short oss = 0;

// Function declaration
int is_sensor_detected();
void read_calibration_data();
u16 read_raw_temp();
u16 read_raw_press();
float calculate_real_temp(u16 UT_raw_temp);
float calculate_real_press(u16 UT_raw_temp, u16 UP_raw_press);
float calculate_altitude(int press);
void uart_print_number_with_fractional_part(float n);

int main()
{
	init_platform();
    xil_printf("Beginning I2C sensor application\n\r");

	// Check if the communication is working
	if(is_sensor_detected() == 0) return 0;

	// Read calibration data
	read_calibration_data();

	u16 raw_temp_UT = 0;
	u16 raw_press_UP = 0;

	while (1){
		// Read raw temperature value
		raw_temp_UT = read_raw_temp();

		// Read raw pressure value
		raw_press_UP = read_raw_press();

		// Calculate real temperature value
		float temperature = calculate_real_temp(raw_temp_UT);

		// Calculate real pressure value
		int pressure = calculate_real_press(raw_temp_UT, raw_press_UP);

		// Calculate altitude
		int altitude = calculate_altitude(pressure);

		// Print temperature to UART
		xil_printf("Temperature ... ");
		uart_print_number_with_fractional_part(temperature);
		xil_printf(" °C\n\r");

		// Print pressure to UART
		xil_printf("Pressure ...... %d hPa\n\r", (pressure/100));

		// Print altitude to UART
		xil_printf("Altitude ...... %d m\n\r", altitude);

		xil_printf("\n\r");
		usleep(1000000);
	}

    cleanup_platform();
    return 0;
}

/** @brief Checks if the I2C sensor is detected
 *
 *  @return 1 if the sensor is detected, 0 if not
 */
int is_sensor_detected()
{	
	SendBuffer[0] = ID_REG_ADDR;
	XIic_Send(AXI_IIC_BASE_ADDR,IIC_SLAVE_ADDR,(u8 *)&SendBuffer, 1,XIIC_REPEATED_START);
	XIic_Recv(AXI_IIC_BASE_ADDR,IIC_SLAVE_ADDR,(u8 *)&RecvBuffer, 1,XIIC_STOP);

	if(RecvBuffer[0]==0x55){
		xil_printf("Sensor Detected\n\r");
		return 1;
	}
	else{
		xil_printf("Sensor NOT Detected\n\r");
		return 0;
	}
}

/** @brief Reads the calibration data from the calibration registers
 *
 *  @return void
 */
void read_calibration_data()
{
	SendBuffer[0] = CALIB_REG_START_ADDR;
	XIic_Send(AXI_IIC_BASE_ADDR,IIC_SLAVE_ADDR,(u8 *)&SendBuffer, 1,XIIC_REPEATED_START);
	XIic_Recv(AXI_IIC_BASE_ADDR,IIC_SLAVE_ADDR,(u8 *)&Calib_Data, 22,XIIC_STOP);
	usleep(1000000);
	AC1 = ((Calib_Data[0] << 8) | Calib_Data[1]);
	AC2 = ((Calib_Data[2] << 8) | Calib_Data[3]);
	AC3 = ((Calib_Data[4] << 8) | Calib_Data[5]);
	AC4 = ((Calib_Data[6] << 8) | Calib_Data[7]);
	AC5 = ((Calib_Data[8] << 8) | Calib_Data[9]);
	AC6 = ((Calib_Data[10] << 8) | Calib_Data[11]);
	B1 = ((Calib_Data[12] << 8) | Calib_Data[13]);
	B2 = ((Calib_Data[14] << 8) | Calib_Data[15]);
	MB = ((Calib_Data[16] << 8) | Calib_Data[17]);
	MC = ((Calib_Data[18] << 8) | Calib_Data[19]);
	MD = ((Calib_Data[20] << 8) | Calib_Data[21]);
}

/** @brief Reads raw temperature from the sensor
 *
 *  @return Raw temperature UT
 */
u16 read_raw_temp()
{
		// Writes to the device are simple accesses which consist of 
		// the register address and the data followed by an I2C stop.
		// Here we write 0x2E value in the control register (0xF4), 
		// so the sensor will be configured to send the temperature value 
		// (and not pressure) when we access the value registers.
		SendBuffer[0] = CONTROL_REG_ADDR;
		SendBuffer[1] = 0x2E;
		XIic_Send(AXI_IIC_BASE_ADDR, IIC_SLAVE_ADDR,(u8 *)&SendBuffer, 2,XIIC_STOP); 

		usleep(4500); // wait 4.5 ms as mentioned in the BMP180 sensor datasheet

		// Reads require an initial write to select the register we wish to read, 
		// followed by an I2C restart before performing the read operation.
		// Here we want to read in the out_lsb and out_msb registers (address 0xF6 and 0xF7 respectively)
		SendBuffer[0] = 0xF6;
		XIic_Send(AXI_IIC_BASE_ADDR, IIC_SLAVE_ADDR, (u8 *)&SendBuffer, 1, XIIC_REPEATED_START); 
		XIic_Recv(AXI_IIC_BASE_ADDR, IIC_SLAVE_ADDR, (u8 *)&RecvBuffer, 2, XIIC_STOP);
		
		// UT value, MSB in 0xF6 register, LSB in 0xF7 register
		return (RecvBuffer[0] << 8) + ( RecvBuffer[1] );
}

/** @brief Reads raw pressure from the sensor
 *
 *  @return Raw pressure UP
 */
u16 read_raw_press()
{	
	SendBuffer[0] = CONTROL_REG_ADDR;
	SendBuffer[1] = 0x34 + (oss<<6); // 0x34 if oss == 0, 0x74 if oss == 1, 0xB4 if oss == 2, 0xF4 if oss == 3, 
	XIic_Send(AXI_IIC_BASE_ADDR, IIC_SLAVE_ADDR,(u8 *)&SendBuffer, 2,XIIC_STOP); 

	// wait as mentioned in the BMP180 sensor datasheet
	switch (oss)
	{
		case (0):
			usleep(4500); 
			break;
		case (1):
			usleep(7500); 
			break;
		case (2):
			usleep(13500);
			break;
		case (3):
			usleep(25500);
			break;
	}

	SendBuffer[0] = 0xF6;
	XIic_Send(AXI_IIC_BASE_ADDR, IIC_SLAVE_ADDR, (u8 *)&SendBuffer, 1, XIIC_REPEATED_START); 
	XIic_Recv(AXI_IIC_BASE_ADDR, IIC_SLAVE_ADDR, (u8 *)&RecvBuffer, 3, XIIC_STOP);

	// UP value, MSB in 0xF6 register, LSB in 0xF7 register, optionally XLSB in 0xF8 register
	return (((RecvBuffer[0]<<16) + (RecvBuffer[1]<<8) + RecvBuffer[2]) >> (8-oss));
}

/** @brief Calculates real temperature from the raw temperature value obtained from the BMP180 sensor
 *
 *  @param UT_raw_temp The raw temperature obtained from the BMP180 sensor
 *  @return Real temperature value
 */
float calculate_real_temp(u16 UT_raw_temp)
{	
	X1 = ((UT_raw_temp-AC6) * (AC5/(pow(2,15))));
	X2 = ((MC*(pow(2,11))) / (X1+MD));
	B5 = X1+X2;
	long temp_x10 = (B5+8)/(pow(2,4));
	return temp_x10/10.0;
}

/** @brief Calculates real pressure from the raw pressure value obtained from the BMP180 sensor
 *
 *  @param UT_raw_temp The raw pressure obtained from the BMP180 sensor
 *  @return Real pressure value
 */
float calculate_real_press(u16 UT_raw_temp, u16 UP_raw_press)
{
	float press = 0;
	X1 = ((UT_raw_temp-AC6) * (AC5/(pow(2,15))));
	X2 = ((MC*(pow(2,11))) / (X1+MD));
	B5 = X1+X2;
	B6 = B5-4000;
	X1 = (B2 * (B6*B6/(pow(2,12))))/(pow(2,11));
	X2 = AC2*B6/(pow(2,11));
	X3 = X1+X2;
	B3 = (((AC1*4+X3)<<oss)+2)/4;
	X1 = AC3*B6/pow(2,13);
	X2 = (B1 * (B6*B6/(pow(2,12))))/(pow(2,16));
	X3 = ((X1+X2)+2)/pow(2,2);
	B4 = AC4*(unsigned long)(X3+32768)/(pow(2,15));
	B7 = ((unsigned long)UP_raw_press-B3)*(50000>>oss);
	if (B7<0x80000000) 
	{
		press = ((float)B7*2)/B4;
	} else 
	{
		press = ((float)B7/B4)*2;
	}
	X1 = (press/(pow(2,8)))*(press/(pow(2,8)));
	X1 = (X1*3038)/(pow(2,16));
	X2 = (-7357*press)/(pow(2,16));
	press = press + (X1+X2+3791)/(pow(2,4));

	return press;
}

/** @brief Calculates altitude from the pressure
 *
 *  @param UT_raw_temp The real pressure value
 *  @return Altitude
 */
float calculate_altitude(int press)
{	
	float alt = 44330*(1-(pow(((float)press/(float)atm_pressure), 1/5.255))); 
	return alt;
}

/** @brief Prints a number with one digit after the decimal to uart
 *
 *	xil_printf doesn't support %f. It only supports %d,l,x,c,s.
 *  To print to UART a number with digits after the decimal point
 *  we need to use xil_printf with two integers,
 *  one for the integer part and one for the fractional part
 *
 *  @param n number to print
 *  @return void
 */
void uart_print_number_with_fractional_part(float n)
{
	int number_integer_part, number_fractional_part;
	number_integer_part = n;
	number_fractional_part = (n - number_integer_part)*10;
	xil_printf("%d,%d", number_integer_part, number_fractional_part);
}
