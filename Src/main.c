
/**
******************************************************************************
* File Name          : main.c
* Description        : GLADRITE
* VERSION 1.0
******************************************************************************

******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "gpio.h"
#include "stm32f0xx_it.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

#define CLK1_HIGH (GPIOA->BSRR = GPIO_PIN_1)
#define CLK1_LOW (GPIOA->BRR = GPIO_PIN_1)

#define LE1_HIGH (GPIOA->BSRR = GPIO_PIN_0)
#define LE1_LOW (GPIOA->BRR = GPIO_PIN_0)

#define SDI1_HIGH (GPIOA->BSRR = GPIO_PIN_2)
#define SDI1_LOW (GPIOA->BRR = GPIO_PIN_2)

#define OE1_HIGH (GPIOA->BSRR = GPIO_PIN_3)
#define OE1_LOW (GPIOA->BRR = GPIO_PIN_3)


#define BACKLIGHT_OFF (GPIOA->BSRR = GPIO_PIN_3)
#define BACKLIGHT_ON (GPIOA->BRR = GPIO_PIN_3)

#define OUT1_HIGH (GPIOA->BSRR = GPIO_PIN_11)
#define OUT1_LOW (GPIOA->BRR = GPIO_PIN_11)

#define OUT2_HIGH (GPIOA->BSRR = GPIO_PIN_7)
#define OUT2_LOW (GPIOA->BRR = GPIO_PIN_7)

#define OUT3_HIGH (GPIOB->BSRR = GPIO_PIN_0)
#define OUT3_LOW (GPIOB->BRR = GPIO_PIN_0)


#define SW3_Pin GPIO_PIN_12
#define SW3_GPIO_Port GPIOA
#define SW1_Pin GPIO_PIN_3
#define SW1_GPIO_Port GPIOB
#define SW2_Pin GPIO_PIN_4
#define SW2_GPIO_Port GPIOB

#define POWERSUPPLY_12V 0
#define POWERSUPPLY_24V 1

#define POWERON 1
#define POWEROFF 0

// 12V Sysytem Specs for Power monitor indicator

#define SYS12V_13_1V   1350
#define SYS12V_12_5V   1280

// 24V Sysytem Specs for Power monitor indicator

#define SYS24V_27_2V   2933
#define SYS24V_25_0V   2691

#define SWITCHACTIVENO (1-1)    // To accomodate switch count satrt from 0
#define BACKLIGHTTIMERMS 60000 // 1 minutes
#define POWERBUTTONTIMEOUTMS 3000 // 0.3 minutes
#define POWERINDICATORSAMPLEMS 2000 // 0.2 minutes

#define LBM_AMBER  0x0050;
#define LBM_RED  0x0010;
#define LBM_GREEN  0x0040;

#define FALSE 0
#define TRUE 1

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint16_t setLedStatus,last_led_config,v12_v24,d1,s;
uint8_t i;

volatile uint8_t SW2_Count = 0;
volatile uint8_t SW2_Activated = 0;
volatile uint32_t backlight_timer = 0;
volatile uint32_t powerOnButton_timer = 0;
volatile uint32_t powerIndicatorStatus = LBM_GREEN;
volatile uint8_t buttonWasActive = FALSE;



uint16_t val;
uint32_t high_count, low_count;
uint8_t ACTIVE,BUTTONS_ACTIVE;
uint32_t battery_count;
__IO uint16_t  ADC1ConvertedValue = 0, ADC1ConvertedVoltage = 0;

uint8_t powerStatus = POWEROFF;

ADC_HandleTypeDef hadc;

uint16_t adcSample;

//---------------------------------------------------------------------------
uint32_t STATUS_BLUE = 0x2000;
uint32_t STATUS_RED = 0x4000;
uint32_t STATUS_GREEN = 0x8000;

uint32_t SWITCH1_BLUE = 0x0100;
uint32_t SWITCH1_RED = 0x0020;
uint32_t SWITCH1_GREEN = 0x0080;

uint32_t SWITCH2_BLUE = 0x0400;
uint32_t SWITCH2_RED = 0x0200;
uint32_t SWITCH2_GREEN = 0x0800;

uint32_t SWITCH3_BLUE = 0x0001;
uint32_t SWITCH3_RED = 0x0003;
uint32_t SWITCH3_GREEN = 0x0002;
//---------------------------------------------------------------------------


static __IO uint32_t TimingDelay;

void Delay(__IO uint32_t nTime);
void TimingDelay_Decrement(void);
void Backlight_Decrement(void);
void PowerOnButton_Decrement(void);

static void MX_ADC_Init(void);
void BATTERY_TEST(void);
void Error_Handler(void);

void systemTestFunc (void);


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* ADC init function */
void MX_ADC_Init(void)
{
  
  ADC_ChannelConfTypeDef sConfig;
  
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler =  ADC_CLOCK_ASYNC;
  hadc.Init.Resolution = ADC_RESOLUTION12b;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = OVR_DATA_PRESERVED;
  HAL_ADC_Init(&hadc);
  
  /**Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
    LE1_LOW;
  }
  /* USER CODE END Error_Handler */ 
}

//-------------------RGB SETUP----------------------------------------------------------------------------
void RGB1_SETUP(void)
{
  LE1_LOW;
  OE1_HIGH;
  SDI1_HIGH;
  CLK1_LOW;
  
  for(i = 0; i < 16; i++)
  {
    if(setLedStatus & 0x01)
    {
      SDI1_HIGH;
    }
    else
    {
      SDI1_LOW;
    }
    
    CLK1_HIGH;
    CLK1_LOW;
    
    if (i ==15)
    {       
      LE1_HIGH;
      LE1_LOW;
    }
    
    setLedStatus = setLedStatus >> 1;                
  }
  
  OE1_LOW;
}
//-------------------RGB SETUP----------------------------------------------------------------------------

int main(void)
{
  
  /* USER CODE BEGIN 1 */
  
  /* USER CODE END 1 */
  
  /* MCU Configuration----------------------------------------------------------*/
  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  
  /* Configure the system clock */
  SystemClock_Config();
  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1 , 1);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 1 , 2);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
  
  //----------------------OFF STATE---------------------------------------------
  setLedStatus = POWEROFF;
  RGB1_SETUP();
  Delay(2000);
  OUT1_LOW;
  OUT2_LOW;
  OUT3_LOW;
  battery_count = 0;
  ACTIVE =0;
  v12_v24 =2;
  SW2_Count = 0;
  //----------------------OFF STATE---------------------------------------------
  
  
  
  HAL_ADC_Start(&hadc);  
  
  if (HAL_ADC_PollForConversion(&hadc, 100) == HAL_OK)
  {
    val = HAL_ADC_GetValue(&hadc);
    //   ADC1ConvertedVoltage = (val *3300)/0xFFF;
    
    HAL_ADC_Stop(&hadc);
  }
  
  if (val > 1800)
  {   
    v12_v24 = POWERSUPPLY_24V;             // 24V system
  }
  if (val < 1600)
  {   
    v12_v24 = POWERSUPPLY_12V;             // 12V system
  }  
  
  while (1)
  {
    if ((backlight_timer == 0) && (powerStatus == POWERON))
    {
      powerStatus = POWEROFF;
      setLedStatus = POWEROFF;
      RGB1_SETUP();
      powerOnButton_timer = 0;
      backlight_timer = 0;
      SW2_Count = 0;  
      buttonWasActive = FALSE;
      OUT1_LOW;
      OUT2_LOW;
      OUT3_LOW;
      
    }
    
    else if ((powerOnButton_timer == 0) && (powerStatus == POWEROFF))
    {
      SW2_Count = 1;
    }
    
    else if (powerStatus == POWERON)      //----------DOUBLE BUTTONS:-----SW1 & SW2---------------------------
    {
      if ((HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_RESET)
          && (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == GPIO_PIN_RESET) )
      {
        setLedStatus = SWITCH1_BLUE + SWITCH2_GREEN + SWITCH3_BLUE + powerIndicatorStatus + STATUS_GREEN;
        //setLedStatus=0x0000;
        RGB1_SETUP();
        Delay(1);
        buttonWasActive = TRUE;
        backlight_timer = BACKLIGHTTIMERMS;
        //SW2_Count = 0;
        OUT1_LOW;
        OUT2_LOW;
        OUT3_LOW;
      }
        //-----------DOUBLE BUTTONS:----SW1 & SW2--------------------------------------------------------------
              
       //----------------------DOUBLE BUTTONS:----sw1 & sw3--------------------------------------------------------
      else if (((HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == GPIO_PIN_RESET)
          && (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == GPIO_PIN_RESET) ) )
      {
        setLedStatus = SWITCH1_BLUE + SWITCH2_GREEN + SWITCH3_BLUE + powerIndicatorStatus + STATUS_GREEN;
        RGB1_SETUP();
        Delay(1);
        buttonWasActive = TRUE;
        backlight_timer = BACKLIGHTTIMERMS;
        //SW2_Count = 0;
        OUT1_LOW;
        OUT2_LOW;
        OUT3_LOW;
      }        
      //----------------------DOUBLE BUTTONS:----sw1 & sw3--------------------------------------------------------
       
     // --DOUBLE BUTTONS:-
      else if (((HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == GPIO_PIN_RESET)
          && (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_RESET) ) )
      {
        setLedStatus = SWITCH1_BLUE + SWITCH2_GREEN + SWITCH3_BLUE + powerIndicatorStatus + STATUS_GREEN;
        RGB1_SETUP();
        Delay(1);
        buttonWasActive = TRUE;
        backlight_timer = BACKLIGHTTIMERMS;
        //SW2_Count = 0;
        OUT1_LOW;
        OUT2_LOW;
        OUT3_LOW;
        // --DOUBLE BUTTONS:-
      } 
      
        //------------SW1---------------------------
      
      else if(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == GPIO_PIN_RESET)
      {
        setLedStatus = SWITCH1_GREEN + SWITCH2_GREEN + SWITCH3_BLUE + powerIndicatorStatus + STATUS_GREEN;
        RGB1_SETUP();
        Delay(1);
        buttonWasActive = TRUE;
        backlight_timer = BACKLIGHTTIMERMS;
        //SW2_Count = 0;
        OUT1_HIGH;
        OUT3_HIGH;
        
        }
      //------------SW1---------------------------
      
      
      //----------------SW3---------------------
      else if (HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == GPIO_PIN_RESET)
      {
        setLedStatus = SWITCH1_BLUE + SWITCH2_GREEN + SWITCH3_GREEN + powerIndicatorStatus + STATUS_GREEN;
        RGB1_SETUP();
        Delay(1);
        buttonWasActive = TRUE;
        backlight_timer = BACKLIGHTTIMERMS;
        //SW2_Count = 0;
        OUT2_HIGH;
        OUT3_HIGH;
        //OUT3_HIGH;
        //----------------SW3---------------------
        
      }
      else
      {
        // Used to control the backlight time out after each button is pressed
        if (buttonWasActive == TRUE)
        {
          setLedStatus = SWITCH1_BLUE + SWITCH2_GREEN + SWITCH3_BLUE + powerIndicatorStatus + STATUS_GREEN;
          RGB1_SETUP();
          buttonWasActive = FALSE;
          backlight_timer = BACKLIGHTTIMERMS;
          //SW2_Count = 0;
          OUT1_LOW;
          OUT2_LOW;
          OUT3_LOW;
        }
      }    
    }
    else
    {
      // do nothing
    }
  }  
  
}

void batteryLEDLevelUpdate (uint16_t batteryValue)
{
  uint16_t aboveGreenVal = 0;
  uint16_t belowAmberVal = 0;
  
  if (v12_v24 == POWERSUPPLY_24V)
  {
    aboveGreenVal = SYS24V_27_2V;
    belowAmberVal = SYS24V_25_0V;
  }
  else
  {
    aboveGreenVal = SYS12V_13_1V;
    belowAmberVal = SYS12V_12_5V;
  }
  //-------------------------------Power status indicator-----------------------------------------------------------
  if ((batteryValue > aboveGreenVal) && (powerIndicatorStatus != 0x0040))
  { 
    // setLedStatus = 0x4309; sw1-sw3-sw2-st-lbm
    powerIndicatorStatus = LBM_GREEN;
    setLedStatus = SWITCH1_BLUE + SWITCH2_GREEN + SWITCH3_BLUE + powerIndicatorStatus + STATUS_GREEN;
    RGB1_SETUP();
  }
  else if (((batteryValue < aboveGreenVal) && (batteryValue > belowAmberVal))  && (powerIndicatorStatus != 0x0050))  // 2520SW2_Count == 2
  {
    powerIndicatorStatus = LBM_AMBER;
    setLedStatus = SWITCH1_BLUE + SWITCH2_GREEN + SWITCH3_BLUE + powerIndicatorStatus + STATUS_GREEN;
    RGB1_SETUP();
  }
  else if ((batteryValue < belowAmberVal) && (powerIndicatorStatus != 0x0010)) 
  {
    powerIndicatorStatus = LBM_RED;
    setLedStatus = SWITCH1_BLUE + SWITCH2_GREEN + SWITCH3_BLUE + powerIndicatorStatus + STATUS_GREEN;
    RGB1_SETUP();
  } 
}
//----------------------------------------------------------------------------------------------------------------------
// System tick callback running every 1ms used check Power level indicator
// every POWERINDICATORSAMPLEMS by calling batteryLEDLevelUpdate function as defined
void HAL_SYSTICK_Callback(void)
{
  static uint32_t powerIndicatorCntTimer = POWERINDICATORSAMPLEMS;
  
  powerIndicatorCntTimer--;
  
  if (powerIndicatorCntTimer == 0) // timer expires
  {
    // Start power level check only if device is ON
    if (powerStatus == POWERON)
    {
      HAL_ADC_Start(&hadc);  
      
      if (HAL_ADC_PollForConversion(&hadc, 100) == HAL_OK)
      {
        val = HAL_ADC_GetValue(&hadc);
        // ADC1ConvertedVoltage = (val *3300)/0xFFF;
      }
      HAL_ADC_Stop(&hadc);
      
      batteryLEDLevelUpdate(val);   
    }
    // Check finished, Start timer ready for next period
    powerIndicatorCntTimer = POWERINDICATORSAMPLEMS;
  }
}
//------------------------------------------------------------------------------------------------------------
// Callback to recieve interrupt from SW1, SW2 and SW3 upon button press
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // Check for which button pressed
  if (GPIO_Pin == SW2_Pin)
  {    
    // SW2 power control button process
    if (SW2_Count > SWITCHACTIVENO)//if (SW2_Count >= SWITCHACTIVENO)//--------check number of times SW2 is pressed
    {
      if (powerStatus == POWEROFF)
      {
        // update battery indicator LED value
        batteryLEDLevelUpdate(val);
        // Turn on device
        setLedStatus = SWITCH1_BLUE + SWITCH2_GREEN + SWITCH3_BLUE + powerIndicatorStatus + STATUS_GREEN;
        RGB1_SETUP();
        backlight_timer = BACKLIGHTTIMERMS;
        powerOnButton_timer = 0;
        powerStatus = POWERON;
      } 
      else 
      {
        // turn off device
        setLedStatus = POWEROFF;
        RGB1_SETUP();
        powerOnButton_timer = 0;
        backlight_timer = 0;
        buttonWasActive = FALSE;
        OUT1_LOW;
        OUT2_LOW;
        OUT3_LOW;
        powerStatus = POWEROFF;
      }
      //SW2_Count = 0;//-------------------set sw2_count to zero to restart count
    }
    else
    {
      // Increment switch and reset backlight_timer and powerOnButton_timer
      // which controls the time window before SW2_Count reset to 0 in main()
      backlight_timer = BACKLIGHTTIMERMS;
      powerOnButton_timer = POWERBUTTONTIMEOUTMS;
     // SW2_Count += 1;  
    }     
  }   
  
  
  //-------------------------------------SW1
  
  // Check if SW1 pressed and reset backlight_timer to keep device alive
  // Also reset SW2_Count to capture a new SW2 button press sequence upon 
  // pressing either SW1 or SW3 button
  if (GPIO_Pin == SW1_Pin)
  {  
    backlight_timer = BACKLIGHTTIMERMS; 
    //SW2_Count = 0;
  }
  
  //-------------------------------------SW3
  if (GPIO_Pin == SW3_Pin)
  {  
    backlight_timer = BACKLIGHTTIMERMS;
    //SW2_Count = 0;
  }  
  // // Allow time for debouncing 
  // Delay(10);
 
}
// --------------------------------------------------------------------------------------------------------------------------SW2 SWITCH TEST();


/** System Clock Configuration
*/
void SystemClock_Config(void)
{
  
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
  
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;
  
  while(TimingDelay != 0);
}

/**
* @brief  Decrements the TimingDelay variable.
* @param  None
* @retval None
*/
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}

void Backlight_Decrement(void)
{
  if (backlight_timer != 0x00)
  {
    backlight_timer--;
  }
}

void PowerOnButton_Decrement(void)
{
  if (powerOnButton_timer != 0x00)
  {
    powerOnButton_timer--;
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
* @brief Reports the name of the source file and the source line number
* where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
  
}

#endif

/**
* @}
*/ 

/**
* @}
*/ 
