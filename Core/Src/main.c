/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdarg.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CH1 0x5000                       // Using CH6 as CH1 
#define CH2 0x1000                       // Channel 2
#define CH3 0x2000                       // Channel 3
#define CH4 0x3000                       // Channel 4
#define CH5 0x4000                       // Channel 5
#define WRM 0x8000                       // Write Register Mode(won't change channels' output)
#define WTM 0x9000                       // Write Through Mode(change channels' output)

#define _DEBUG_BUF_SIZE_ 100		
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern int Temp_Max_threshold;
extern int Temp_recover_threshold;

int Temp_compensation = 0; // DAC Mode Toggle -> 1

float T0 = 25.0;       
float Vgs_T0 = 1.5;

float Vgs_Driver_T0 = 2.3;
float Vgs_Main_T0 = 2.3;
float Vgs_Peak_T0 = 1.5;

float K_t = 0.05;

float Driver_Kt = 0.05;
float Main_Kt = 0.05;
float Peak_Kt = 0.05;

float Temp, last_T = 25.0;


//float Vgs_T = 1.0;
float Vgs_Driver = 1.0;
float Vgs_Main = 1.0;
float Vgs_Peak = 1.0;

char rx_buffer[100]; 
uint8_t rx_data;  
uint16_t rx_index = 0; 

/**********FOR TEST**********/
float CH1_V = 2.0;
float CH2_V = 2.0;
float CH3_V = 1.0;
/**********FOR TEST**********/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#define FLASH_LAST_4KB_ADDR   ((uint32_t*)0x0800F000)
#define FLASH_PAGE_START_ADDR 0x0800F000 
#define DRIVER_ID 0x01
#define MAIN_ID 0x02
#define PEAK_ID 0x03

#define MAX_ENTRIES 10
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct 
{
	float temp_low;
	float temp_high;
	float coeff;
} TempCompEntry;

TempCompEntry Vgs_driver_table[MAX_ENTRIES];
TempCompEntry Vgs_main_table[MAX_ENTRIES];
TempCompEntry Vgs_peak_table[MAX_ENTRIES];

size_t driver_count = 0;
size_t main_count = 0;
size_t peak_count = 0;

void read_Tempcomptable(uint32_t flash_address, size_t max_entries)
{
	int i = 0;
	for(i = 0 ; i < max_entries ; ++i){
		uint8_t param_type = *(uint8_t *)flash_address;
		
		if(param_type == 0xff){
			break;
		}
		
		flash_address += 4;
		
		int32_t temp_low = *(int32_t *)flash_address;
		flash_address += 4;
		
		int32_t temp_high = *(int32_t *)flash_address;
		flash_address += 4;
		
		float coeff = *(float *)flash_address;
		flash_address += 4;
		
		TempCompEntry entry;
		entry.temp_low = temp_low;
		entry.temp_high = temp_high;
		entry.coeff = coeff;
		
		if(param_type == DRIVER_ID && driver_count < MAX_ENTRIES){
			Vgs_driver_table[driver_count++] = entry;
		} else if (param_type == MAIN_ID && main_count < MAX_ENTRIES) {
			Vgs_main_table[main_count++] = entry;
		} else if(param_type == PEAK_ID && peak_count < MAX_ENTRIES) {
			Vgs_peak_table[peak_count++] = entry;
		}
		
	}
}

void erase_flash_page(uint32_t page_address)
{
    HAL_FLASH_Unlock();  
	
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError;


    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = page_address;
    EraseInitStruct.NbPages = 1; 

    HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);

    HAL_FLASH_Lock();  
}

void write_to_flash(uint32_t address, uint32_t data)
{
    HAL_FLASH_Unlock();  

    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data);

    HAL_FLASH_Lock();  
}


/***************************************************************
  *  @brief     Provides debug information and prints it via the serial port using UART.
  *  @param     Multiple parameters   
  *  @note      Use this function where you want to observe the variable
  *  @Sample usage:     Debug_print(".....\r\n", ...); 
 **************************************************************/
void Debug_print(const char *format, ...) {
	char buf[_DEBUG_BUF_SIZE_];
	va_list args;
	va_start(args, format);
	vsnprintf(buf, _DEBUG_BUF_SIZE_, format, args);
	va_end(args);

	HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
}


/* DSA Config Interface*/
/***************************************************************
  *  @brief     Adjustment of DSA according to received data from I2C Master
  *  @param     gear range (0~63)   
  *  @note      6 GPIO pins avaliable
  *  @Sample usage:
 **************************************************************/
int32_t DSA_Adjust_Command(float target_gain)
{
    if (target_gain < 0.0f || target_gain > 31.5f) {
        return -1; 
    }
		
    uint16_t gpio_pins[] = {GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_11}; 
    float gain_values[] = {0.5f, 1.0f, 2.0f, 4.0f, 8.0f, 16.0f}; 

    for (int i = 5; i >= 0; i--) {
        if (target_gain >= gain_values[i]) {
            HAL_GPIO_WritePin(GPIOA, gpio_pins[i], GPIO_PIN_SET);  
            target_gain -= gain_values[i]; 
        } else {
            HAL_GPIO_WritePin(GPIOA, gpio_pins[i], GPIO_PIN_RESET);
        }
    }

    return 0;
}

float DSA_Status(void)
{
    uint16_t gpio_pins[] = {GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_11};
    float gain_values[] = {0.5f, 1.0f, 2.0f, 4.0f, 8.0f, 16.0f}; 

    float current_gain = 0.0f;  
    GPIO_PinState pin_state; 

    for (int i = 0; i < 6; i++) {
        pin_state = HAL_GPIO_ReadPin(GPIOA, gpio_pins[i]);  

        if (pin_state == GPIO_PIN_SET) {
            current_gain += gain_values[i];
        }
    }

    return current_gain;
}
	 
/***************************************************************
  *  @brief     
  *  @param     cmd Can only be either 0 or 1    
  *  @note      VC1-HIGH VC2-LOW: DPD | cmd-0: DPD cmd-1: another
  *  @Sample usage:
 **************************************************************/
int32_t Tor_Switch(uint32_t cmd)
{
  uint16_t data;
  if(cmd == 0) {
		 data = WTM; // Using WTM mode update channels
    HAL_SPI_Transmit(&hspi2, (uint8_t *)&data, sizeof(data), HAL_MAX_DELAY);

    data = CH4 | 0xe8d; // Voltage 3V 
    HAL_SPI_Transmit(&hspi2, (uint8_t *)&data, sizeof(data), HAL_MAX_DELAY);
		
		data = WTM;

    data = CH5 | 0x0; // Voltage 0V
    HAL_SPI_Transmit(&hspi2, (uint8_t *)&data, sizeof(data), HAL_MAX_DELAY);

  } else if (cmd == 1) {
		
		   data = WTM; // Using WTM mode update channels
    HAL_SPI_Transmit(&hspi2, (uint8_t *)&data, sizeof(data), HAL_MAX_DELAY);
		
    data = CH4 | 0x0; // Voltage 0V
    HAL_SPI_Transmit(&hspi2, (uint8_t *)&data, sizeof(data), HAL_MAX_DELAY);

		data = WTM;
		
    data = CH5 | 0xe8d; // Voltage 3V
    HAL_SPI_Transmit(&hspi2, (uint8_t *)&data, sizeof(data), HAL_MAX_DELAY);

 
  } else {
    return -1;
  }
  return 0;
}

void channle1_test(uint16_t data)
{
	HAL_SPI_Transmit(&hspi2, (uint8_t *)&data, sizeof(data), HAL_MAX_DELAY);
}	



void Parse_Command(char *command) {
    char param[10];
    float value;  uint16_t data; // Test delete later
    
    if (sscanf(command, "SET %s %f", param, &value) == 2) {
			if(Temp_compensation){
			      if (strcmp(param, "T0") == 0) {
            T0 = value;
						Debug_print("\r\nT0 set to: %.2f\r\n", T0);
        } else if (strcmp(param, "VGS_T0") == 0) {
            Vgs_T0 = value;
						Debug_print("\r\nVgs_T0 set to: %.2f\r\n", Vgs_T0);
        } else if (strcmp(param, "K_T") == 0) {
            K_t = value;
						Debug_print("\r\nK_t set to: %.2f\r\n", K_t);
        } else if(strcmp(param, "DRIVE_T0") == 0) {
						Vgs_Driver_T0 = value;
						Debug_print("\r\nVgs_Driver_T0 set to %.2f\r\n", Vgs_Driver_T0);
				} else if(strcmp(param, "MAIN_T0") == 0) {
						Vgs_Main_T0 = value;
						Debug_print("\r\nVgs_Main_T0 set to %.2f\r\n", Vgs_Main_T0);
				}	else if(strcmp(param, "PEAK_T0") == 0) {
						Vgs_Peak_T0 = value;
						Debug_print("\r\nVgs_Peak_T0 set to %.2f\r\n", Vgs_Peak_T0);
				}	else if(strcmp(param, "DRIVE_K") == 0) {
						Driver_Kt = value;
						Debug_print("\r\nDriver_Kt set to %.2f\r\n", Driver_Kt);
				}	else if(strcmp(param, "MAIN_K") == 0) {
						Main_Kt = value;
						Debug_print("\r\nMain_Kt set to %.2f\r\n", Main_Kt);
				}	else if(strcmp(param, "PEAK_K") == 0) {
						Peak_Kt = value;
						Debug_print("\r\nPeak_Kt set to %.2f\r\n", Peak_Kt);
				} else {
						Debug_print("\r\nTemperature compensation Mode try -- HELP\r\n");
				}
			} else {
				 if(strcmp(param, "CH1") == 0) {
						Debug_print("\r\nCH1 Driver set to %.4f\r\n", value);
						CH1_V = value;
						value += 0.015;
						uint16_t Driver_t = (uint16_t)(value / 3.3 * 4096);
						Debug_print("Calculated DAC value for CH1: %x\r\n", Driver_t);
						data = CH1 | Driver_t;
						channle1_test(data);
				} else if(strcmp(param, "CH2") == 0) {
						Debug_print("\r\nCH2 Main set to %.4f\r\n", value);
						CH2_V = value;
						value += 0.014;
						uint16_t Main_t = (uint16_t)(value / 3.3 * 4096);
						Debug_print("Calculated DAC value for CH1: %x\r\n", Main_t);
						data = CH2 | Main_t;
						channle1_test(data);
				} else if(strcmp(param, "CH3") == 0) {
						Debug_print("\r\nCH3 Peak set to %.4f\r\n", value);
						CH3_V = value;
						value += 0.008;
						uint16_t Peak_t = (uint16_t)(value / 3.3 * 4096);
						Debug_print("Calculated DAC value for CH1: %x\r\n", Peak_t);
						data = CH3 | Peak_t;
						channle1_test(data);
				}	else {
						Debug_print("\r\nAdjust DAC Channel Mode try -- HELP\r\n");
				}
			}
    } else if (strcmp(command, "CHECK") == 0) {
			if(Temp_compensation){
				Debug_print("\r\n--------------------------------------------------------------------\r\n");
				Debug_print("Temperature Compensation Mode\r\n");
				Debug_print("Temp = %.2f	T0 = %.2f\r\n", Temp, T0);
				Debug_print("CH1 Driver: Vgs_Driver: %.2f Driver_T0: %.2f  Driver_Kt: %.2f\r\n", Vgs_Driver, Vgs_Driver_T0, Driver_Kt);
				Debug_print("CH2 Main:     Vgs_Main: %.2f   Main_T0: %.2f    Main_Kt: %.2f\r\n", Vgs_Main, Vgs_Main_T0, Main_Kt);
				Debug_print("ch3 Peak:     Vgs_Peak: %.2f   Peak_T0: %.2f    Peak_Kt: %.2f\r\n", Vgs_Peak, Vgs_Peak_T0, Peak_Kt);
				Debug_print("DSA : %.1f\r\n", DSA_Status());
				Debug_print("--------------------------------------------------------------------\r\n");
			} else {
				Debug_print("\r\n---------------------------------------------\r\n");
				Debug_print("Adjust DAC Mode\r\n");
				Debug_print("Temp = %.2f	T0 = %.2f\r\n", Temp, T0);
				Debug_print("CH1 = %.2f\r\n", CH1_V);
				Debug_print("CH2 = %.2f\r\n", CH2_V);
				Debug_print("CH3 = %.2f\r\n", CH3_V);
				Debug_print("---------------------------------------------\r\n");
			}

    }	else if(strcmp(command, "ADC") == 0) {
				if (HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY) == HAL_OK) {
						uint32_t adcValue = HAL_ADC_GetValue(&hadc);
						adcValue = adcValue * 3.3 / 4096;
						Debug_print("\r\nADC Value: %lu\r\n", adcValue);
				} else {
						Debug_print("\r\nFailed to read ADC value\r\n");
				}
		} else if(sscanf(command, "TOR %f", &value) == 1) {
				Tor_Switch(value);
				if (value == 1.0) {
            Debug_print("\r\nDirection: Forwards\r\n");   // Forwards
        } else {
						Debug_print("\r\nDirection: Backwards\r\n");  // Backwards
        }
		} else if(sscanf(command, "GAIN %f", &value) == 1) {
				DSA_Adjust_Command(value);
				Debug_print("\r\nDSA set to %f\r\n", value);
		}	else if(strcmp(command, "TEMP_COMP") == 0) {
				Temp_compensation = !Temp_compensation;
				if(Temp_compensation) {
					Debug_print("\r\nTemperature compensation enabled\r\n");
				} else {
					Debug_print("\r\nTemperature compensation disabled\r\n");
				}
		}	else if(strcmp(command, "HELP") == 0) {
				Debug_print("\r\n*********************Command List*********************\r\n");
				Debug_print("TEMP_COMP     toggle temperature compensation mode\r\n");
				Debug_print("CHECK         check current status\r\n");
				Debug_print("TOR <0/1>     toggle forwards/backwards\r\n");
				Debug_print("GAIN <param>  Set DSA range from 0~31.5\r\n");
				if(Temp_compensation){
					Debug_print("\r\nEnable Temperature Compensation Mode:\r\n");
					Debug_print("	   Vgs_t = -K_t * (T - T0) + Vgs_t0\r\n");
					Debug_print("Set Command:\r\n");
					Debug_print("SET T0 <param>         set T0 Temperature\r\n");
					Debug_print("SET DRIVE_T0 <param>   set driver(CH1) voltage\r\n");
					Debug_print("SET MAIN_T0 <param>    set main(CH2) voltage\r\n");
					Debug_print("SET PEAK_T0 <param>    set peak(CH3) voltage\r\n");
					Debug_print("\r\nSET DRIVE_K <param>    set driver K_t\r\n");
					Debug_print("SET MAIN_K <param>    set main K_t\r\n");
					Debug_print("SET PEAK_K <param>    set peak K_t\r\n");
				} else {
					Debug_print("\r\nDisable Temperature Compensation Mode Command:\r\n");
					Debug_print("SET CH1 <param>     set CH1 voltage\r\n");
					Debug_print("SET CH2 <param>     set CH2 voltage\r\n");
					Debug_print("SET CH3 <param>     set CH3 voltage\r\n");
				}
				Debug_print("*************************************************\r\n");
		}
		
		else {
        Debug_print("\r\nInvalid command format try --HELP\r\n");
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (rx_data == '\r' || rx_data == '\n')
        {
            rx_buffer[rx_index] = '\0';  
            Parse_Command((char*)rx_buffer);  
            rx_index = 0; 
        }
        else
        {
					  // Echo Terminal characters
//						HAL_UART_Transmit(&huart1, &rx_data, 1, HAL_MAX_DELAY);
					
            rx_buffer[rx_index++] = rx_data; 

            if (rx_index >= sizeof(rx_buffer))
            {
                rx_index = 0;
            }
        }

        HAL_UART_Receive_IT(&huart1, &rx_data, 1); // Receive 1 Byte once
    }
}

//int32_t Vgs_Adjust() 
//{
////    T = Read_Temperature(0x49);
//    if (Temp == 0.0f) {
//        Debug_print("Temperature T is zero, skipping adjustment.\r\n");
//        return 0; // Early exit if T is zero
//    }
//		
//    if (fabs(Temp - last_T) >= 2.0)
//    {
//			Vgs_Driver = -0.0012 * (Temp - T0) + Vgs_Driver_T0;
//			if(Vgs_Driver > 3.0) {
//				Debug_print("Warning: Vgs_T exceed 3V, current value: %.2f, Set to 3\r\n", Vgs_Driver);
//				Vgs_Driver = 3.0;
//			}
//			
//			Vgs_Main = -0.0012 * (Temp - T0) + Vgs_Main_T0;
//			if(Vgs_Main > 3.0) {
//				Debug_print("Warning: Vgs_T exceed 3V, current value: %.2f, Set to 3\r\n", Vgs_Main);
//				Vgs_Main = 3.0;
//			}			
//			
//			Vgs_Peak = -0.0012 * (Temp - T0) + Vgs_Peak_T0;
//			if(Vgs_Peak > 3.0) {
//				Debug_print("Warning: Vgs_T exceed 3V, current value: %.2f, Set to 3\r\n", Vgs_Peak);
//				Vgs_Peak = 3.0;
//			}					
//			
//			uint16_t volt;
//			
//			uint16_t Driver = Vgs_Driver / 3.3 * 4096;
//			uint16_t Main = Vgs_Main / 3.3 * 4096;
//			uint16_t Peak = Vgs_Peak / 3.3 * 4096;
//			Debug_print("\r\nVgs_adjust: Vgs_Driver = %.4f, T = %.2f, Drive = %x\r\n", Vgs_Driver, Temp, Driver);
//			Debug_print("Vgs_adjust: Vgs_Main = %.4f, T = %.2f, Main = %x\r\n", Vgs_Main, Temp, Main);
//			Debug_print("Vgs_adjust: Vgs_Peak = %.4f, T = %.2f, Peak = %x\r\n", Vgs_Peak, Temp, Peak);
//			
//			volt = CH1 | Driver;
//			Debug_print("volt = %x \r\n", volt);
////			HAL_SPI_Transmit(&hspi2, (uint8_t *)&volt, sizeof(volt), HAL_MAX_DELAY);
////			HAL_StatusTypeDef status;
////			status = HAL_SPI_Transmit(&hspi2, (uint8_t *)&volt, sizeof(volt), HAL_MAX_DELAY);
////			if (status != HAL_OK) {
////					Debug_print("CH1 SPI transmission failed with error: %d\r\n", status);
////			} else {
////					Debug_print("CH1 transmission completed %d\r\n", status);
////			}
//			channle1_test(CH1 | Driver);

//			
//			volt = CH2 | Main;
//			Debug_print("volt = %x \r\n", volt);
////			HAL_SPI_Transmit(&hspi2, (uint8_t *)&volt, sizeof(volt), HAL_MAX_DELAY);
////			status = HAL_SPI_Transmit(&hspi2, (uint8_t *)&volt, sizeof(volt), HAL_MAX_DELAY);
////			if (status != HAL_OK) {
////					Debug_print("CH2 SPI transmission failed with error: %d\r\n", status);
////			} else {
////					Debug_print("CH2 transmission completed %d\r\n", status);
////			}
//			channle1_test(CH2 | Main);
//			
//			volt = CH3 | Peak;
//			Debug_print("volt = %x \r\n", volt);
////			HAL_SPI_Transmit(&hspi2, (uint8_t *)&volt, sizeof(volt), HAL_MAX_DELAY);
////			status = HAL_SPI_Transmit(&hspi2, (uint8_t *)&volt, sizeof(volt), HAL_MAX_DELAY);
////			if (status != HAL_OK) {
////					Debug_print("CH3 SPI transmission failed with error: %d\r\n", status);
////			} else {
////					Debug_print("CH3 transmission completed %d\r\n", status);
////			}
//			channle1_test(CH3 | Peak);
//			
//			volt = WTM;
////			HAL_SPI_Transmit(&hspi2, (uint8_t *)&volt, sizeof(volt), HAL_MAX_DELAY);
////			status = HAL_SPI_Transmit(&hspi2, (uint8_t *)&volt, sizeof(volt), HAL_MAX_DELAY);
////			if (status != HAL_OK) {
////					Debug_print("WTM SPI transmission failed with error: %d\r\n", status);
////			}else {
////					Debug_print("WTM transmission completed %d\r\n", status);
////			}
//			channle1_test(WTM);
//			
//			last_T = Temp;
//    }
//    else
//    {
//        // Reserved 
//    }
//		return 0;
//}

//int32_t Vgs_Adjust() 
//{
//    if (Temp == 0.0f) {
//        Debug_print("Temperature T is zero, skipping adjustment.\r\n");
//        return 0; 
//    }

//    Vgs_Driver = -0.0012 * (Temp - T0) + Vgs_Driver_T0;
//    if (Vgs_Driver > 3.0) {
//        Debug_print("Warning: Vgs_Driver exceed 3V, current value: %.2f, Set to 3\r\n", Vgs_Driver);
//        Vgs_Driver = 3.0;
//    }

//    Vgs_Main = -0.0012 * (Temp - T0) + Vgs_Main_T0;
//    if (Vgs_Main > 3.0) {
//        Debug_print("Warning: Vgs_Main exceed 3V, current value: %.2f, Set to 3\r\n", Vgs_Main);
//        Vgs_Main = 3.0;
//    }

//    Vgs_Peak = -0.0012 * (Temp - T0) + Vgs_Peak_T0;
//    if (Vgs_Peak > 3.0) {
//        Debug_print("Warning: Vgs_Peak exceed 3V, current value: %.2f, Set to 3\r\n", Vgs_Peak);
//        Vgs_Peak = 3.0;
//    }

//    uint16_t Driver = (uint16_t)(Vgs_Driver / 3.3 * 4096);
//    uint16_t Main = (uint16_t)(Vgs_Main / 3.3 * 4096);
//    uint16_t Peak = (uint16_t)(Vgs_Peak / 3.3 * 4096);

//    Debug_print("\r\nVgs_adjust: Vgs_Driver = %.4f, T = %.2f, Drive = %x\r\n", Vgs_Driver, Temp, Driver);
//    Debug_print("Vgs_adjust: Vgs_Main = %.4f, T = %.2f, Main = %x\r\n", Vgs_Main, Temp, Main);
//    Debug_print("Vgs_adjust: Vgs_Peak = %.4f, T = %.2f, Peak = %x\r\n", Vgs_Peak, Temp, Peak);

//    channle1_test(CH1 | Driver); 
//    channle1_test(CH2 | Main);    
//    channle1_test(CH3 | Peak);    
//    channle1_test(WTM);          

//    return 0;
//}


void Close_PA(void) 
{
	uint16_t data;
	
	data = WTM;
	HAL_SPI_Transmit(&hspi2, (uint8_t *)&data, sizeof(data), HAL_MAX_DELAY);
	
	data = CH1;
	HAL_SPI_Transmit(&hspi2, (uint8_t *)&data, sizeof(data), HAL_MAX_DELAY);
	
	data = CH2;
	HAL_SPI_Transmit(&hspi2, (uint8_t *)&data, sizeof(data), HAL_MAX_DELAY);
	
	data = CH3;
	HAL_SPI_Transmit(&hspi2, (uint8_t *)&data, sizeof(data), HAL_MAX_DELAY);
	
	data = WRM;
	HAL_SPI_Transmit(&hspi2, (uint8_t *)&data, sizeof(data), HAL_MAX_DELAY);
	
	return ;
}

int32_t Open_PA(void)
{
	uint16_t data;

	data = WTM;
	HAL_SPI_Transmit(&hspi2, (uint8_t *)&data, sizeof(data), HAL_MAX_DELAY);
	
	data = CH1 | 0x9F0;
	HAL_SPI_Transmit(&hspi2, (uint8_t *)&data, sizeof(data), HAL_MAX_DELAY);
	
	data = CH2 | 0x9F0;
	HAL_SPI_Transmit(&hspi2, (uint8_t *)&data, sizeof(data), HAL_MAX_DELAY);
	
	// 1.8G initial 1V
	data = CH3 | 0x4FE;
	HAL_SPI_Transmit(&hspi2, (uint8_t *)&data, sizeof(data), HAL_MAX_DELAY);
	
	// 2.1G initial 1.3V
//	data = CH3 | 0x657;
//	HAL_SPI_Transmit(&hspi2, (uint8_t *)&data, sizeof(data), HAL_MAX_DELAY);
	return 0;
}

void read_flash_and_print(uint32_t address, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++) {
        char flash_char = *(char*)(address + i); 
        Debug_print("%c", flash_char); 
    }
    Debug_print("\r\n");
}

void print_Tempcomptable(const TempCompEntry *entry, size_t count, const char *table_name){
	size_t i = 0;
	for(i = 0 ; i < count ; ++i) {
		Debug_print("%s Entry %zu: Temp[%.2f, %.2f], Coeff = %.6f\r\n", table_name, i, entry[i].temp_low, entry[i].temp_high, entry[i].coeff);
	}
}

float get_coeff_from_table(TempCompEntry *table, size_t count, float Temp) {
    for (size_t i = 0; i < count; ++i) {
        if (Temp >= table[i].temp_low && Temp <= table[i].temp_high) {
            return table[i].coeff;
        }
    }
    return -0.0012; 
}

void Adjust_Vgs_Driver(float Temp)
{
//    Vgs_Driver = -0.0012 * (Temp - T0) + Vgs_Driver_T0;
		float coeff = get_coeff_from_table(Vgs_driver_table, driver_count, Temp);
		Vgs_Driver = coeff * (Temp - T0) + Vgs_Driver_T0;
    if (Vgs_Driver > 3.0) {
        Debug_print("Warning: Vgs_Driver exceed 3V, current value: %.2f, Set to 3\r\n", Vgs_Driver);
        Vgs_Driver = 3.0;
    }

    uint16_t Driver = (uint16_t)(Vgs_Driver / 3.3 * 4096);
    Debug_print("\r\nAdjust_Vgs_Driver: Vgs_Driver = %.4f, T = %.2f\r\n", Vgs_Driver, Temp);
	  uint16_t data = CH1 | Driver;
    channle1_test(data);
}

void Adjust_Vgs_Main(float Temp)
{
//    Vgs_Main = -0.0012 * (Temp - T0) + Vgs_Main_T0;
		float coeff = get_coeff_from_table(Vgs_main_table, main_count, Temp);
		Vgs_Main = coeff * (Temp - T0) + Vgs_Main_T0;
    if (Vgs_Main > 3.0) {
        Debug_print("Warning: Vgs_Main exceed 3V, current value: %.2f, Set to 3\r\n", Vgs_Main);
        Vgs_Main = 3.0;
    }

    uint16_t Main = (uint16_t)(Vgs_Main / 3.3 * 4096);
    Debug_print("\r\nAdjust_Vgs_Main: Vgs_Main = %.4f, T = %.2f\r\n", Vgs_Main, Temp);
		uint16_t data = CH2 | Main;
    channle1_test(data);
}

void Adjust_Vgs_Peak(float Temp)
{
//    Vgs_Peak = -0.0012 * (Temp - T0) + Vgs_Peak_T0;
		float coeff = get_coeff_from_table(Vgs_peak_table, peak_count, Temp);
		Vgs_Peak = coeff * (Temp - T0) + Vgs_Peak_T0;
    if (Vgs_Peak > 3.0) {
        Debug_print("Warning: Vgs_Peak exceed 3V, current value: %.2f, Set to 3\r\n", Vgs_Peak);
        Vgs_Peak = 3.0;
    }

    uint16_t Peak = (uint16_t)(Vgs_Peak / 3.3 * 4096);
    Debug_print("\r\nAdjust_Vgs_Peak: Vgs_Peak = %.4f, T = %.2f\r\n", Vgs_Peak, Temp);
		uint16_t data = CH3 | Peak;
    channle1_test(data);
}

int32_t Vgs_Adjust()
{
    if (Temp == 0.0f) {
        Debug_print("Temperature T is zero, skipping adjustment.\r\n");
        return 0; 
    }

    Adjust_Vgs_Driver(Temp);
    Adjust_Vgs_Main(Temp);
    Adjust_Vgs_Peak(Temp);

    channle1_test(WTM);

    return 0;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	int PA_ON = 0;
	int temp_exceed = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_ADC_Start(&hadc);
	
	if (HAL_I2C_EnableListen_IT(&hi2c2) != HAL_OK)
	{
			Error_Handler();
	}
	
	HAL_UART_Receive_IT(&huart1, (uint8_t*)rx_buffer, 1);
	
	Open_PA();
//	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_SET)
//	{
//		Open_PA();
//		PA_ON = 1;
//	}
//	else
//	{
//		Close_PA();
//		PA_ON = 0;
//	}
	
	 DSA_Adjust_Command(0);
	
  // Tor_Switch(1);
	uint16_t data = 0;
	data = WTM; // Using WTM mode update channels
    HAL_SPI_Transmit(&hspi2, (uint8_t *)&data, sizeof(data), HAL_MAX_DELAY);
		
    data = CH4 | 0x0; // Voltage 0V
    HAL_SPI_Transmit(&hspi2, (uint8_t *)&data, sizeof(data), HAL_MAX_DELAY);

		data = WTM;
		
    data = CH5 | 0xe8d; // Voltage 3V
    HAL_SPI_Transmit(&hspi2, (uint8_t *)&data, sizeof(data), HAL_MAX_DELAY);
		
//		erase_flash_page(FLASH_PAGE_START_ADDR);
		
//		uint32_t read_data;
//		write_to_flash(0x0800F000, 0x12345678);
//		read_data = *((uint32_t*)0x0800F000);
//		write_to_flash(0x0800F010, 0x48656C6C);
//    write_to_flash(0x0800F014, 0x6F20466C);
//    write_to_flash(0x0800F018, 0x61736821);
//    write_to_flash(0x0800F000, 0x12345678);  // ?? 0x12345678 ? 0x0800F000
//// ????????? Flash
//		write_to_flash(0x0800F010, 0x6C6C6548);  // ?? "Hell"
//		write_to_flash(0x0800F014, 0x6C46206F);  // ?? "o Fl"
//		write_to_flash(0x0800F018, 0x21687361);  // ?? "ash!"

//		uint32_t read_data = *((uint32_t*)0x0800F000);
		read_Tempcomptable(0x800F000, MAX_ENTRIES);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		Temp = Read_Temperature(0x49);
		Debug_print("Temp = %.2f\r\n", Temp);
		HAL_Delay(1000);
//		Debug_print("Test 2:%x\r\n", read_data);
//		read_flash_and_print(0x0800F010, 12);
//		print_Tempcomptable(Vgs_driver_table, driver_count, "Driver");
//		print_Tempcomptable(Vgs_main_table, main_count, "Main");
//		print_Tempcomptable(Vgs_peak_table, peak_count, "Peak");
//		Debug_print("\r\n");
//		HAL_Delay(5000);
//    if (Temp > Temp_Max_threshold) 
//    {
//      if (!temp_exceed) 
//      {
//        Close_PA();
//        temp_exceed = 1; 
//      }
//    }
//    else if (Temp < Temp_recover_threshold) 
//    {
//      if (temp_exceed) 
//      {
//        Open_PA();
//        temp_exceed = 0; 
//      }
//    }		
		if(Temp_compensation){
			if(fabs(Temp - last_T) >= 0.5) {
					channle1_test(WTM);
					Adjust_Vgs_Driver(Temp);
					Adjust_Vgs_Main(Temp);
					Adjust_Vgs_Peak(Temp);
					last_T = Temp;	
			}
			HAL_Delay(1000);
		}
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
