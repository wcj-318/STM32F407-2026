/* USER CODE BEGIN Header */
/**==================================================================================================================
 ** 【代码编写】  魔女开发板团队
 ** 【淘    宝】  魔女开发板       https://demoboard.taobao.com
 **==================================================================================================================
 ** 【实验名称】  高级定时器 TIM1 定时测试
 **
 ** 【适用平台】  STM32F407 + keil5 + HAL库
 **
 ** 【实验目标】  通过TIM1配置定时，每隔1秒反转LED;
 **
 ** 【引脚接线】  为了方便观察实验效果，可以使用以下两种方法;
 **               1-使用板上的LED_BLUE(PB2) 配合测试。
 **               2_如果有示波器，示波器通道接上PB2,  共地。
 **
 ** 【 CubeMX 】  打开Timers，选择TIM1;
 **               Clock Source(时钟源): Internal Clock    内部时钟
 **               Prescaler(PSC)      ：168-1             分频值      基本定时器的时钟源是84MHz, 84分频后脉冲频率为1MHz, 即每1us产生一个计数信号。
 **               Counter Period(ARR) : 1000-1            周期值      多少个计数信号组成一个周期
 **               auto-reload preload : Enable            预装载      使能后，更改ARR值不会打断当前波形
 **               NVIC Settings(中断) : update interrupt  打勾        周期更新中断
 **
 ** 【初 始 化】  1-TIM1的初始化代码，CubeMX会根据配置自动生成。
 **               2-只需在main函数中，调用函数开启TIM1、打开中断.
 **
 ** 【回调函数】  1-使用CubeMX生成工程，其生成的代码会编写好中断服务函数、清理中断标志、调用中断回调函数。我们只需重写回调函数，并在其中执行自定义操作。
 **               2-高级定时器，有多个中断，我们在CubeMX中使用的是update interrupt(周期更新中断), 当CNT计数达到1周期值时触发，硬件自动调用中断服务函数，继而调用其中断回调函数：HAL_TIM_PeriodElapsedCallback();
 **               3-中断回调函数，本示例写在了main.c的底部。你可以写在工程的任意一个c文件中。
 **
 ** 【TIM 重点】  1_时钟频率：STM32F407默认系统频率168MHz; TIM1、8、9、10、11的时钟频率是APB2*2=84MHz*2=168MHz, 而TIM1、3、4、5、6、7、12、13、14的时钟频率是APB1*2=42MHz*2=84MHz;
 **               2_基本定时器、通用定时器、高级定时器资源是有明显区别的，已整理有《TIM资源表》存放在示例文件夹中，仅供参考;
 **               3_PSC，预分频值;    作用：控制计数器每一脉冲的时长; 解释：把时钟源分频后提供给计数器使用，即多少个时钟源脉冲，才产生一次计数器脉冲;
 **               4_ARR，自动重载值;  作用：控制周期; 解释：多少个计数器脉冲，组成一完整波形周期;
 **               5_CNT，计数器;      作用：每一脉冲，硬件自动操作递增、递减;
 **               6_CCR，捕获/比较值; 作用：输出模式用于与CNT值作大小比较而输出有效电平; 输入模式用于记录上一捕获时的计数器值; 注意，基本定时器TIM6和7没有CCR寄存器。
 **               7_寄存器位宽:所有TIM的PSC寄存器，都是16位的，取值范围：1~65535; 注意：ARR、CNT、CCR三个寄存器，除了TIM1和5是32位，其它TIM的都是16位;
 **               8_输出极性，理解为有效电平。在TIM_OCPolarity里设置，可以设置为高、低电平; 如，当PWM1模式下，当CNT<CCR时输出有效电平，这个有效电平，就是你设置的“输出极性”; 注意：基本定时器TIM6和7没有这个概念.
 **               9_中断回调函数：如果使用CubeMX生成工程，其生成的代码会编写好中断服务函数、清理中断标志、调用中断回调函数。我们只需重写回调函数，并在其中执行自定义操作。
 **
 ** 【更新记录】  2024-03-06  新建HAL库工程
 **
 ** 【备注说明】  版权归魔女科技所有，请勿商用，谢谢！
 **               https://demoboard.taobao.com
 **
==================================================================================================================**/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* 用户代码，必须写在配对的BEGIN与END之间，否则CubeMX重新生成后，会被删除掉 */

#include "bsp_UART.h"            // 串口通信底层驱动文件; 已重写好初始化、收发，调用函数即可使用串口
#include "gray.h"
#include "line_follow.h"
#include "motor.h"
#include "oled.h"
#include "encoder.h"
#include "servo.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

typedef enum
{
    PHASE4_WAIT = 0,
    PHASE4_RUN,
    PHASE4_FINISH,
    PHASE4_LOST
} Phase4_State_t;

static Gray_Sample_t gray_sample;
static LineFollow_Controller_t line_controller;
static SpeedControl_Controller_t speed_controller;
static Phase4_State_t phase4_state = PHASE4_WAIT;
static int16_t phase4_left_request_pwm;
static int16_t phase4_right_request_pwm;
static int16_t phase4_left_output_pwm;
static int16_t phase4_right_output_pwm;
static int16_t phase4_line_correction;
static const Encoder_Sample_t *encoder_sample;
static uint32_t phase4_run_start_ms;
static uint32_t phase4_elapsed_ms;
static uint8_t phase4_stop_line_armed;
static uint8_t phase4_stop_line_clear_frames;
static uint8_t phase4_stop_line_hit_frames;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define PHASE4_OLED_PERIOD_MS         500U
#define PHASE4_OLED_TIME_PERIOD_MS    500U
#define PHASE4_KEY_GUARD_MS             80U
#define PHASE4_VOFA_PERIOD_MS          100U
#define PHASE4_VOFA_BAUDRATE          9600U
#define PHASE4_VOFA_CHANNEL_COUNT       19U
#define PHASE4_STOP_GRAY8_CENTER6_MASK    0x7EU
#define PHASE4_STOP_GRAY8_HIT_MIN            4U
#define PHASE4_STOP_GRAY5_CENTER3_MASK    0x0EU
#define PHASE4_STOP_GRAY5_HIT_MIN            1U
#define PHASE4_STOP_LINE_CLEAR_FRAMES        5U
#define PHASE4_STOP_LINE_CONFIRM_FRAMES      1U
#define PHASE4_FINISH_BRAKE_MS             100U

typedef struct
{
    float channel[PHASE4_VOFA_CHANNEL_COUNT];
    uint8_t tail[4];
} Phase4_VofaFrame_t;

static volatile uint32_t phase4_start_key_irq_count;
static uint32_t phase4_start_key_handled_count;
static uint32_t phase4_start_key_last_ms;

static uint8_t Phase4_ReadStartKey(void)
{
    return (HAL_GPIO_ReadPin(START_KEY_GPIO_Port, START_KEY_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}

static void Phase4_StartKeyInit(uint32_t now)
{
    phase4_start_key_handled_count = phase4_start_key_irq_count;
    phase4_start_key_last_ms = now - PHASE4_KEY_GUARD_MS;
}

static uint8_t Phase4_StartKeyPressed(uint32_t now)
{
    uint32_t irq_count = phase4_start_key_irq_count;

    if (irq_count == phase4_start_key_handled_count)
    {
        return 0U;
    }

    phase4_start_key_handled_count = irq_count;

    if ((now - phase4_start_key_last_ms) >= PHASE4_KEY_GUARD_MS)
    {
        phase4_start_key_last_ms = now;
        return 1U;
    }

    return 0U;
}

static uint8_t Phase4_CountBits(uint8_t value)
{
    uint8_t count = 0U;

    while (value != 0U)
    {
        count = (uint8_t)(count + (value & 0x01U));
        value >>= 1U;
    }

    return count;
}

static uint8_t Phase4_StopLineDetected(void)
{
    uint8_t gray8_active = (uint8_t)(gray_sample.gray8_mask & PHASE4_STOP_GRAY8_CENTER6_MASK);
    uint8_t gray5_active = (uint8_t)(gray_sample.gray5_mask & PHASE4_STOP_GRAY5_CENTER3_MASK);

    return ((Phase4_CountBits(gray8_active) >= PHASE4_STOP_GRAY8_HIT_MIN) &&
            (Phase4_CountBits(gray5_active) >= PHASE4_STOP_GRAY5_HIT_MIN)) ? 1U : 0U;
}

static void Phase4_StopLineReset(void)
{
    phase4_stop_line_armed = 0U;
    phase4_stop_line_clear_frames = 0U;
    phase4_stop_line_hit_frames = 0U;
}

static uint8_t Phase4_StopLineStep(void)
{
    uint8_t detected = Phase4_StopLineDetected();

    if (phase4_stop_line_armed == 0U)
    {
        if (detected != 0U)
        {
            phase4_stop_line_clear_frames = 0U;
        }
        else
        {
            if (phase4_stop_line_clear_frames < PHASE4_STOP_LINE_CLEAR_FRAMES)
            {
                phase4_stop_line_clear_frames++;
            }
            if (phase4_stop_line_clear_frames >= PHASE4_STOP_LINE_CLEAR_FRAMES)
            {
                phase4_stop_line_armed = 1U;
            }
        }
        return 0U;
    }

    if (detected == 0U)
    {
        phase4_stop_line_hit_frames = 0U;
        return 0U;
    }

    if (phase4_stop_line_hit_frames < PHASE4_STOP_LINE_CONFIRM_FRAMES)
    {
        phase4_stop_line_hit_frames++;
    }
    return (phase4_stop_line_hit_frames >= PHASE4_STOP_LINE_CONFIRM_FRAMES) ? 1U : 0U;
}

static void Phase4_UpdateElapsed(uint32_t now)
{
    if (phase4_state == PHASE4_RUN)
    {
        phase4_elapsed_ms = now - phase4_run_start_ms;
    }
}

static const char *Phase4_StateText(void)
{
#if HLINE_SPEED_RUN_MODE == HLINE_SPEED_MODE_LEFT_ONLY
    const char *wait_text = "H4L WAIT";
    const char *run_text = "H4L RUN";
    const char *lost_text = "H4L LOST";
#elif HLINE_SPEED_RUN_MODE == HLINE_SPEED_MODE_RIGHT_ONLY
    const char *wait_text = "H4R WAIT";
    const char *run_text = "H4R RUN";
    const char *lost_text = "H4R LOST";
#elif HLINE_SPEED_RUN_MODE == HLINE_SPEED_MODE_DUAL_FIXED
    const char *wait_text = "H4D WAIT";
    const char *run_text = "H4D RUN";
    const char *lost_text = "H4D LOST";
#else
    const char *wait_text = "Q2 WAIT";
    const char *run_text = "Q2 RUN";
    const char *lost_text = "Q2 LOST";
#endif

    if (phase4_state == PHASE4_RUN)
    {
        return run_text;
    }
    if (phase4_state == PHASE4_LOST)
    {
        return lost_text;
    }
    if (phase4_state == PHASE4_FINISH)
    {
        return "Q2 DONE";
    }
    return wait_text;
}

static void Phase4_Stop(Phase4_State_t next_state)
{
    Phase4_UpdateElapsed(HAL_GetTick());
    Servo_StopSweep();
    if (next_state == PHASE4_FINISH)
    {
        Motor_Brake();
        HAL_Delay(PHASE4_FINISH_BRAKE_MS);
    }
    Motor_Stop();
    LineFollow_Reset(&line_controller);
    SpeedControl_Reset(&speed_controller);
    phase4_left_request_pwm = 0;
    phase4_right_request_pwm = 0;
    phase4_left_output_pwm = 0;
    phase4_right_output_pwm = 0;
    phase4_line_correction = 0;
    phase4_state = next_state;
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
}

static void Phase4_RequestStep(void)
{
    if (phase4_state != PHASE4_RUN)
    {
        return;
    }

#if HLINE_SPEED_RUN_MODE == HLINE_SPEED_MODE_LEFT_ONLY
    phase4_left_request_pwm = HLINE_SPEED_TEST_PWM_PERMILLE;
    phase4_right_request_pwm = 0;
    phase4_line_correction = 0;
#elif HLINE_SPEED_RUN_MODE == HLINE_SPEED_MODE_RIGHT_ONLY
    phase4_left_request_pwm = 0;
    phase4_right_request_pwm = HLINE_SPEED_TEST_PWM_PERMILLE;
    phase4_line_correction = 0;
#elif HLINE_SPEED_RUN_MODE == HLINE_SPEED_MODE_DUAL_FIXED
    phase4_left_request_pwm = HLINE_SPEED_TEST_PWM_PERMILLE;
    phase4_right_request_pwm = HLINE_SPEED_TEST_PWM_PERMILLE;
    phase4_line_correction = 0;
#else
    if (LineFollow_Step(&line_controller, &gray_sample,
                        &phase4_left_request_pwm, &phase4_right_request_pwm,
                        &phase4_line_correction) == 0U)
    {
        Phase4_Stop(PHASE4_LOST);
        return;
    }
#endif
}

static void Phase4_Start(void)
{
    LineFollow_Reset(&line_controller);
    SpeedControl_Reset(&speed_controller);
    phase4_left_request_pwm = 0;
    phase4_right_request_pwm = 0;
    phase4_left_output_pwm = 0;
    phase4_right_output_pwm = 0;
    phase4_line_correction = 0;

#if HLINE_SPEED_RUN_MODE == HLINE_SPEED_MODE_LINE_TRACK
    if (gray_sample.line_valid == 0U)
    {
        Phase4_Stop(PHASE4_LOST);
        return;
    }
#endif

    Encoder_Reset();
    Phase4_StopLineReset();
    phase4_run_start_ms = HAL_GetTick();
    phase4_elapsed_ms = 0U;
    phase4_state = PHASE4_RUN;
    Servo_StartSweep(phase4_run_start_ms);
    Phase4_RequestStep();
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
}

static void Phase4_SpeedStep(void)
{
    if (phase4_state != PHASE4_RUN)
    {
        return;
    }

    SpeedControl_Step(&speed_controller,
                      phase4_left_request_pwm, phase4_right_request_pwm,
                      encoder_sample->left_counts_per_second,
                      encoder_sample->right_counts_per_second);
    phase4_left_output_pwm = speed_controller.left_output_pwm;
    phase4_right_output_pwm = speed_controller.right_output_pwm;
    Motor_SetSignedPermille(phase4_left_output_pwm, phase4_right_output_pwm);
}

static void Phase4_VofaSend(void)
{
    Phase4_VofaFrame_t frame;

    frame.channel[0] = (float)phase4_state;
    frame.channel[1] = (float)gray_sample.gray8_mask;
    frame.channel[2] = (float)gray_sample.gray8_unstable_mask;
    frame.channel[3] = (float)gray_sample.line_valid;
    frame.channel[4] = (float)gray_sample.error10;
    frame.channel[5] = (float)speed_controller.left_target_cps;
    frame.channel[6] = (float)speed_controller.right_target_cps;
    frame.channel[7] = (float)phase4_line_correction;
    frame.channel[8] = (float)gray_sample.gray5_mask;
    frame.channel[9] = (float)encoder_sample->interval_ms;
    frame.channel[10] = (float)encoder_sample->left_counts_per_second;
    frame.channel[11] = (float)(-encoder_sample->right_counts_per_second);
    frame.channel[12] = (float)speed_controller.left.filtered_cps;
    frame.channel[13] = (float)speed_controller.right.filtered_cps;
    frame.channel[14] = (float)phase4_left_output_pwm;
    frame.channel[15] = (float)phase4_right_output_pwm;
    frame.channel[16] = (float)Phase4_ReadStartKey();
    frame.channel[17] = (float)phase4_start_key_irq_count;
    frame.channel[18] = (float)phase4_elapsed_ms / 1000.0f;
    frame.tail[0] = 0x00U;
    frame.tail[1] = 0x00U;
    frame.tail[2] = 0x80U;
    frame.tail[3] = 0x7FU;

    UART1_SendData((uint8_t *)&frame, (uint16_t)sizeof(frame));
}

static void Phase4_OledRefresh(void)
{
    const char *base_text = Phase4_StateText();
    char state_text[12];
    uint8_t index = 0U;

    while ((base_text[index] != '\0') && (index < 8U))
    {
        state_text[index] = base_text[index];
        index++;
    }
    state_text[index++] = ' ';
    state_text[index++] = 'K';
    state_text[index++] = (Phase4_ReadStartKey() != 0U) ? '1' : '0';
    state_text[index] = '\0';

    OLED_ShowPhase2(state_text, gray_sample.gray8_mask, gray_sample.gray8_unstable_mask,
                    gray_sample.error10, gray_sample.line_valid, gray_sample.gray5_mask,
                    phase4_left_output_pwm, phase4_right_output_pwm, phase4_line_correction);
    OLED_ShowTimeMs(phase4_elapsed_ms);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

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
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    /* USER CODE BEGIN 2 */
    /* 用户代码，必须写在配对的BEGIN与END之间 */

    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);       // 引脚置高电平，红亮灭

    UART1_Init(PHASE4_VOFA_BAUDRATE);                                       // JDY-31默认9600bps，发送VOFA JustFloat二进制帧

    HAL_TIM_Base_Start_IT(&htim1);                                         // 启动TIM1，并使能中断


    Gray_Init();
    OLED_Init();
    if (Encoder_Init() != HAL_OK)
    {
        Error_Handler();
    }
    encoder_sample = Encoder_GetSample();
    if (Motor_Init() != HAL_OK)
    {
        Error_Handler();
    }
    if (Servo_Init() != HAL_OK)
    {
        Error_Handler();
    }
    Gray_Read(&gray_sample);
    Phase4_Stop(PHASE4_WAIT);
    Phase4_StartKeyInit(HAL_GetTick());
    Phase4_OledRefresh();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        /* 用户代码，必须写在配对的BEGIN与END之间 */
        static uint32_t last_control;
        static uint32_t last_encoder;
        static uint32_t last_vofa;
        static uint32_t last_oled;
        uint32_t now = HAL_GetTick();

        Phase4_UpdateElapsed(now);
        if (phase4_state == PHASE4_RUN)
        {
            Servo_SweepStep(now);
        }

        if ((now - last_control) >= HLINE_CONTROL_PERIOD_MS)
        {
            last_control = now;
            Gray_Read(&gray_sample);
            if ((phase4_state == PHASE4_RUN) && (Phase4_StopLineStep() != 0U))
            {
                Phase4_Stop(PHASE4_FINISH);
                Phase4_OledRefresh();
            }
            else
            {
                Phase4_RequestStep();
            }
        }

        if (Phase4_StartKeyPressed(now) != 0U)
        {
            if (phase4_state == PHASE4_RUN)
            {
                Phase4_Stop(PHASE4_WAIT);
            }
            else
            {
                Phase4_Start();
            }
            Phase4_OledRefresh();
        }

        if ((now - last_encoder) >= HLINE_SPEED_CONTROL_PERIOD_MS)
        {
            uint32_t elapsed_encoder = now - last_encoder;
            last_encoder = now;
            Encoder_Update(elapsed_encoder);
            Phase4_SpeedStep();
        }

        if ((now - last_vofa) >= PHASE4_VOFA_PERIOD_MS)
        {
            last_vofa = now;
            Phase4_VofaSend();
        }

        if ((phase4_state == PHASE4_RUN) && ((now - last_oled) >= PHASE4_OLED_TIME_PERIOD_MS))
        {
            last_oled = now;
            OLED_ShowTimeMs(phase4_elapsed_ms);
        }
        else if ((phase4_state != PHASE4_RUN) && ((now - last_oled) >= PHASE4_OLED_PERIOD_MS))
        {
            last_oled = now;
            Phase4_OledRefresh();
        }
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

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
/* 所有用户代码，必须写在配对的BEGIN与END之间 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == START_KEY_Pin)
    {
        phase4_start_key_irq_count++;
    }
}

/******************************************************************************
 * 函  数： HAL_TIM_PeriodElapsedCallback
 * 功  能： 周期更新回调函数
 * 备  注： 本函数是TIM的CNT溢出中断回调函数。
 *          当TIM的计数器CNT，完成1周期计数时触发(向上计数：CNT==ARR，向下计数：CNT==0);
 *          上述中断触发后，硬件自动调用相关中断服务函数，继而调用本函数。
 *          所有TIM的周期更新中断，都是调用本函数，因此需要在函数内判断是哪一个TIM触发的中断;
 * 参  数： TIM_HandleTypeDef   *htim
 * 返回值： 无
******************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)                             // 判断是哪个TIM产生的中断
    {
        static uint16_t cnt = 0;                            // 中断次数
        if (cnt++ >= 1000)                                  // 每中断1000次执行，即1s执行1次
        {
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);          // 反转LED引脚
            cnt = 0;                                        // 计数清0
        }
    }

}



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
    /* User can add his own implementation to report the file name and linerxNumber,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
