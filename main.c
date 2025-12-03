  /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) EE2028 Teaching Team
  ******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"			//	accelerometer
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.h"			//	magnetometer
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"				//	gyroscope
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"			//	temperature sensor
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.h"			//	pressure sensor
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.h"			//	humidity sensor
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.h"					//	LED
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_buzzer.h" 			//	buzzer

#include "stdio.h"
#include "string.h"
#include <math.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"

//------------------------------------------------------ DECLARATIONS ----------------------------------------------------------------//
#define TEMP_MIN 20
#define TEMP_MAX 39

#define HUM_MIN 50
#define HUM_MAX 80

#define PRES_MIN 990
#define PRES_MAX 1010

#define MAG_KINDA_NEAR 	3000
#define MAG_NEAR 		6000
#define MAG_VERY_NEAR 	8000

#define ABNORMAL_TEMP   0x01
#define ABNORMAL_HUM    0x02
#define ABNORMAL_PRES   0x04

#define WARN_INTERVAL 			3000
#define DOUBLE_PRESS_INTERVAL	500
#define FALLEN_TIMEOUT 5000

//------------------------------------------------------------- FLAGS ------------------------------------------------------------------------//

volatile uint8_t button_pressed = 0; 		//	button flag -> volatile because can be triggered any time within the loops
volatile uint8_t current_game = 1; 			//	game flag -> volatile because can be changed at any time within the loops
volatile _Bool tilt_detected = 0; 			//	tilt flag -> raised when there is a tilt detected
volatile _Bool game_over = 0; 				//	game over flag -> volatile because game over can occur any time within the loop
volatile _Bool uart_busy = 0; 				//	checks for whether UART is busy
_Bool theme_played = 0; 					//	theme played flag
_Bool end_played = 0; 						//	end played flag

//--------------------------------------------------- INITIALISE FUNCTIONS --------------------------------------------------------------------//

void UART1_Init(void); 						//	declaration of UART initialization

UART_HandleTypeDef huart1; 					//	UART_HandleTypeDef is a structure (struct) defined by the STM32 HAL library
											//	to hold all configuration and runtime information for a specific UART;
											//	creating an instance (a variable) of that structure, named huart1, that will represent UART1 in your code

static void MX_GPIO_Init (void);
static void MX_I2C1_Init(void);
I2C_HandleTypeDef hi2c1;
static void TILT_EXTI_Init(void);

//----------------------------------------------------- CALLBACK FUNCTIONS ----------------------------------------------------------------------//

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart); //	Callback when transmission is complete
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

//------------------------------------------------------- GAME FUNCTIONS ------------------------------------------------------------------------//
void Game1_Run(void);
void Game2_Run(void);

//------------------------------------------------------ HELPER FUNCTIONS -----------------------------------------------------------------------//

uint8_t get_abnormal_sensors(float t, float h, float p); 								// get abnormal sensors
void send_environment_warnings(uint8_t abnormal, float temp, float hum, float pres); 	// generate and send non-blocking environment warning messages
void LED_blink(uint32_t delay_ms); 														// blink LED at a certain frequency
float get_magnetometer_value(void); 													// get magnetometer reading
uint32_t get_blink_delay(float magnet_value); 											// get appropriate blink delay
void send_game_message_IT(const char *msg); 											// sends non-blocking game messages

_Bool due(uint32_t now, uint32_t deadline)												// handle time intervals
{
    return (int32_t)(now - deadline) >= 0;	//	return True when difference between now and deadline is >= 0, i.e. current time had passed the deadline
    										//	cast to signed integer as wrap-safe check
}

//------------------------------------------------------- MELODY FUNCTIONS ------------------------------------------------------------------------//
void start_melody(void);
void update_melody(void);
static _Bool is_melody_playing(void);

void start_emelody(void);
void update_emelody(void);
static _Bool is_emelody_playing(void);

void start_glmelody(void);
void update_glmelody(void);

//-------------------------------------------------------- OLED FUNCTIONS -------------------------------------------------------------------------//
void DrawStartupScreen(void);
void DrawPlayerFallenScreen(void);
void DrawPlayerUprightScreen(void);
void DrawGameOverScreen(void);

//------------------------------------------------------- MAIN FUNCTION ---------------------------------------------------------------------------//

int main(void)
{
	HAL_Init();
	MX_GPIO_Init();
	UART1_Init();
	MX_I2C1_Init();

// EXTI10_15 only one interrupt handler -> button and tilt interrupt have same priority
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);	// highest priority
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

// Peripheral initializations using BSP functions
	BSP_ACCELERO_Init();
	BSP_ACCELERO_InitTilt();
	TILT_EXTI_Init();
	BSP_GYRO_Init();
	BSP_MAGNETO_Init();
	BSP_TSENSOR_Init();
	BSP_PSENSOR_Init();
	BSP_HSENSOR_Init();

	BSP_LED_Init(LED2);
	BSP_Buzzer_Init();

	ssd1306_Init();

//------------------------------------------ START-UP THEME SONG AND INSTRUCTION TEXT --------------------------------------------------------------//

// theme song and startup screen

	   if (!theme_played) {
		   DrawStartupScreen();
	       start_melody();
	       while (is_melody_playing()) {
	           update_melody();
	           BSP_Buzzer_Update(); 		// stops each note after duration
	       }
	       theme_played = 1;
	   }

	    ssd1306_Fill(Black);
	    ssd1306_UpdateScreen();

	   uint32_t now = HAL_GetTick();
	   while (HAL_GetTick() - now < 2000) {
	   }

// start-up messages
	   char setup_msg1[] =  "Welcome to the SOTONG GAMES! Double press to switch games\r\n";
	   HAL_UART_Transmit(&huart1, (uint8_t*)setup_msg1, strlen(setup_msg1), 0xFFFF);
	   //uint32_t now = HAL_GetTick();
	   while (HAL_GetTick() - now < 2000) {
	   }

	   char setup_msg2[] =  "Let's start with Game 1: RED LIGHT, GREEN LIGHT\r\n";
	   HAL_UART_Transmit(&huart1, (uint8_t*)setup_msg2, strlen(setup_msg2), 0xFFFF);
	   now = HAL_GetTick();
	   while (HAL_GetTick() - now < 2000) {
	   }

	   // game over variables
	   char received_char;
	   char buffer[8];						// buffer to store user response
	   uint8_t idx;

	   while (1)
	   {
//-------------------------------------------------------- GAME OVER LOGIC ----------------------------------------------------------------------------//
		   if (game_over == 1) {

               uint32_t wait1 = HAL_GetTick();
               while (uart_busy && (HAL_GetTick() - wait1 < 500)) {
                   // idle until UART done, up to 500ms max -> prevents game over restart message from being lost
               }

        	   if (!end_played) {
        		   DrawGameOverScreen();
        		   ssd1306_UpdateScreen();
        	       start_emelody();
        	       while (is_emelody_playing()) {
        	           update_emelody();
        	           BSP_Buzzer_Update(); 		// stops each note after duration
        	       }
        	       end_played = 1;
        	   }

		       char restart_msg[] = "Do you want to play again?\r\n Input 'Y1' to continue with Game 1, 'Y2' to continue with Game 2, 'N' to end.\r\n";
		       HAL_UART_Transmit(&huart1, (uint8_t *)restart_msg, strlen(restart_msg), 0xFFFF);

		       memset(buffer, 0, sizeof(buffer)); 				//initialize memory
		       idx = 0;
		       uint32_t start_time = HAL_GetTick();
		       uint32_t timeout = 60000; 						// 1 minute time out
		       uint8_t restart_chosen = 0;

		       while (!restart_chosen) {						// restart_flag

				   // receive characters one by one (short timeout to allow timeout checking)
				   if (HAL_UART_Receive(&huart1, (uint8_t *)&received_char, 1, 100) == HAL_OK) {

					   HAL_UART_Transmit(&huart1, (uint8_t *)&received_char, 1, 0xFFFF);   				// echo typed char

					   if (received_char == '\r' || received_char == '\n') {							// check for enter key: handle both CR and LF

						   buffer[idx] = '\0';  														// terminate string
						   while (idx > 0 && (buffer[idx - 1] == '\r' || buffer[idx - 1] == '\n')) { 	// strip trailing CR/LF if any
							   buffer[--idx] = '\0';

						   }
						   // print newline for neatness
						   HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, 0xFFFF);

						   // process full input -> after enter (CR/LF) is pressed
						   if (strcmp(buffer, "Y1") == 0) {

							   current_game = 1;
							   game_over = 0;
							   restart_chosen = 1;
							   end_played = 0;
							   char msg1[] = "\r\nRestarting Game 1...\r\n";
							   HAL_UART_Transmit(&huart1, (uint8_t *)msg1, strlen(msg1), 0xFFFF);

						   } else if (strcmp(buffer, "Y2") == 0) {

							   current_game = 2;
							   game_over = 0;
							   restart_chosen = 1;
							   end_played = 0;
							   char msg2[] = "\r\nRestarting Game 2...\r\n";
							   HAL_UART_Transmit(&huart1, (uint8_t *)msg2, strlen(msg2), 0xFFFF);

						   } else if (strcmp(buffer, "N") == 0) {

							   char bye_msg[] = "\r\nGoodbye!\r\n";
							   HAL_UART_Transmit(&huart1, (uint8_t *)bye_msg, strlen(bye_msg), 0xFFFF);

							   while (1); 																			// stop program

						   } else {

							   char invalid_msg[] = "\r\nInvalid input! Please enter 'Y1', 'Y2', or 'N'.\r\n";
							   HAL_UART_Transmit(&huart1, (uint8_t *)invalid_msg, strlen(invalid_msg), 0xFFFF);		// reprompt user
							   HAL_UART_Transmit(&huart1, (uint8_t *)restart_msg, strlen(restart_msg), 0xFFFF);
							   memset(buffer, 0, sizeof(buffer));   												// clear buffer for next attempt
							   idx = 0;

						   }
					   } else {
						   if (idx < sizeof(buffer) - 1) { 															// regular character input
							   buffer[idx++] = received_char;
						   }
					   }
					   start_time = HAL_GetTick();  																// reset 1 minute timeout on any key press
				   }

				   // check 1 minute timeout
				   if (HAL_GetTick() - start_time >= timeout) {

					   const char timeout_msg[] = "\r\nNo input received within 1 minute. Exiting game.\r\n";
					   HAL_UART_Transmit(&huart1, (uint8_t *)timeout_msg, strlen(timeout_msg), HAL_MAX_DELAY);
					   while (1); 																					// end program

				   }
			   }
		   }

//-------------------------------------------------------- GAME SWITCHING LOGIC ----------------------------------------------------------------------------//
		   if (current_game == 1){

			   Game1_Run();
			   continue; //once exit the game because of the switch, will restart loop to change the game

		   } else{

			   Game2_Run();
			   continue; //once exit the game because of the switch, will restart loop to change the game

		   }
	   }
	}


//---------------------------------------------------------------- GAME1 ------------------------------------------------------------------------------------//

void Game1_Run(void) {

    uint32_t now = HAL_GetTick();

    typedef enum { PHASE_GREEN, PHASE_RED } phase_t;

    phase_t phase = PHASE_RED;
    uint32_t phase_timer = 0;
    uint32_t env_timer = 0;
    uint32_t led_timer = 0;
    uint32_t motion_timer = 0;

    float a_ref[3] = {0};
    static _Bool a_ref_valid = 0;			//	flag for capturing a_ref
    uint32_t baseline_time = 0;

    static _Bool gl_played = 0; 			//	red light green light theme played flag

	const float ACCEL_THRESHOLD = 0.8f; 	// accel threshold
    const float GYRO_THRESHOLD = 0.2f; 		// gyro threshold

//-------------------------------------------------------- GAME START MESSAGE ----------------------------------------------------------------------------//
    char start_msg_1[] = "Entering Red Light, Green Light as Player\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)start_msg_1, strlen(start_msg_1), 0xFFFF);

    while (HAL_GetTick() - now < 2000) {					// give players time to read the message
    }

    while (!game_over) {

        uint32_t now = HAL_GetTick();

//--------------------------------------------------------- GAME CHANGE CHECK -----------------------------------------------------------------------------//

        if (button_pressed) {
            static uint32_t last_press_time = 0;
            static uint8_t press_count = 0;
            button_pressed = 0;

            if (now - last_press_time < DOUBLE_PRESS_INTERVAL) {

                press_count++;

            } else {

                press_count = 1;

            }

            last_press_time = now;
            if (press_count == 2) {

                press_count = 0;
                current_game = 2; 											// switch to Game 2 and exit Game1_Run early
                gl_played = 1;
                BSP_Buzzer_StopTone();
                send_game_message_IT("Switching to Game 2\r\n");
                uint32_t wait_start = HAL_GetTick();

                while (uart_busy && (HAL_GetTick() - wait_start < 500)) {

                    	// idle until UART done, up to 500ms max -> prevents game 2 start up message from being lost

                }

                return; // return to main
            }
        }

//------------------------------------------------------- PHASE SWITCHING (Red/Green Light) ----------------------------------------------------------------------------//

        if (due(now, phase_timer)) {
            phase = (phase == PHASE_GREEN) ? PHASE_RED : PHASE_GREEN;
            phase_timer = now + 10000;

            if (phase == PHASE_GREEN) {
            	gl_played = 0;
                env_timer = now + 2000;
            }

            else {
                led_timer = now + 500;
                motion_timer = now + 2000;
                a_ref_valid = 0;					//	reset flag
                baseline_time = now + 800;			//	allow some time for values to settle before capturing baseline
            }

            const char *msg = (phase == PHASE_GREEN) ? "Green Light!\r\n" : "Red Light!\r\n";
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 0xFFFF);
        }

//-------------------------------------------------------------------- GREEN LIGHT ----------------------------------------------------------------------------------------//

        if (phase == PHASE_GREEN) {

//------------------------------------------------- RLGL THEME SONG -----------------------------------------------------------//

        	// Start melody non-blocking (only once when green light phase starts)
        	if (!gl_played) {
        	    start_glmelody();
        	    gl_played = 1;
        	}

        	// Update melody during the green light phase
        	update_glmelody();

//------------------------------------------------- LED ON -----------------------------------------------------------//

            BSP_LED_On(LED2); 							// Always ON during Green Light

//------------------------------------------------- Environment Monitoring -----------------------------------------------------------//
            if (due(now, env_timer)) {
                float T = BSP_TSENSOR_ReadTemp();
                float P = BSP_PSENSOR_ReadPressure();
                float H = BSP_HSENSOR_ReadHumidity();

                char TPH_print[128];

                sprintf(TPH_print, "Temperature: %0.2f\tPressure: %0.2f\tHumidity: %0.2f\r\n", T, P, H);
                HAL_UART_Transmit(&huart1, (uint8_t*)TPH_print, strlen(TPH_print), 0xFFFF);
                env_timer += 2000;  				// next sensor reading after 2 seconds
            }
        }

//---------------------------------------------------------------------- RED LIGHT ------------------------------------------------------------------------------------------//
        else {

            int16_t a_i16[3] = {0};
            BSP_ACCELERO_AccGetXYZ(a_i16);

            float a[3] = {
                (float)a_i16[0] * (9.8f / 1000.0f),
                (float)a_i16[1] * (9.8f / 1000.0f),
                (float)a_i16[2] * (9.8f / 1000.0f)
            };


            float g_f[3] = {0};
            BSP_GYRO_GetXYZ(g_f);

			const float gyro_sensitivity = 70000;
			const float gx_offset = 490;
			const float gy_offset = -1400;
			const float gz_offset = 420;

			float g[3] = {
				(float)(g_f[0] - gx_offset) / gyro_sensitivity,
				(float)(g_f[1] - gy_offset) / gyro_sensitivity,
				(float)(g_f[2] - gz_offset) / gyro_sensitivity
			};

//------------------------------------------------- Motion Detection -----------------------------------------------------------//
			// capture baseline once at the start of Red Light after a short delay
			if (!a_ref_valid && due(now, baseline_time)) {
				a_ref[0] = a[0]; a_ref[1] = a[1]; a_ref[2] = a[2];
				a_ref_valid = 1;
			}

			// compute delta from baseline (vector difference)
			if (a_ref_valid) {
				float dx = a[0] - a_ref[0];
			    float dy = a[1] - a_ref[1];
			    float dz = a[2] - a_ref[2];
			    float delta_a = sqrtf(dx*dx + dy*dy + dz*dz);

			    if ((a_ref_valid) && (delta_a > ACCEL_THRESHOLD)) {
			        send_game_message_IT("Game Over! Motion detected.\r\n");
	                game_over = 1;
	                return;
			    }
			}

            if (fabs(g[0]) > GYRO_THRESHOLD || fabs(g[1]) > GYRO_THRESHOLD || fabs(g[2]) > GYRO_THRESHOLD) {
                send_game_message_IT("Game Over! Rotation detected.\r\n");
                game_over = 1;
                return;
            }

//------------------------------------------------- LED Toggling -----------------------------------------------------------//

            if (due(now, led_timer)) {
                BSP_LED_Toggle(LED2);
                led_timer += 500;
            }

//------------------------------------------------- Motion Monitoring -----------------------------------------------------------//

            if (due(now, motion_timer)) {
				char AG_print[160];
				sprintf(AG_print,
					"Accel X: %0.2f\tAccel Y: %0.2f\tAccel Z: %0.2f\r\n"
					"Gyro X: %0.2f\tGyro Y: %0.2f\tGyro Z: %0.2f\r\n",
					a[0], a[1], a[2], g[0], g[1], g[2]);
				HAL_UART_Transmit(&huart1, (uint8_t*)AG_print, strlen(AG_print), 0xFFFF);

				motion_timer += 2000;
            }
        }
    }
}





//---------------------------------------------------------------- GAME2 ------------------------------------------------------------------------------------//


void Game2_Run(void) {
//-------------------------------------------------- VARIABLE INITIALISATION ------------------------------------------------------------------//
	uint32_t env_timer = 0;
	uint32_t trigger_time = 0;
	uint32_t last_warn_time = 0;
	uint32_t buzzer_start_time = 0;
	uint8_t abnormal;
	float temp_data;
	float humidity_data;
	float pressure_data;
	_Bool triggered = 0;
	_Bool warned = 0;
	_Bool buzzer_on = 0;
	uint32_t now = HAL_GetTick();

	_Bool tilt_active = 0;
	uint32_t tilt_start_time = 0;

//---------------------------------------------------- GAME START MESSAGE ---------------------------------------------------------------------//
	send_game_message_IT("Entering CATCH AND RUN as Player\r\nPress the blue button to escape when enforcers are near!\r\n");
	while (HAL_GetTick() - now < 2000) { // gives players time to read the message
	    }
	while (!game_over) {
		now = HAL_GetTick();
		if (tilt_detected && !tilt_active) {
		    tilt_detected = 0;
		    tilt_active = 1;
		    tilt_start_time = now;

		    send_game_message_IT("Player has fallen over!\r\n");
		    DrawPlayerFallenScreen();
		}

		// If tilt state is active
		if (tilt_active) {

		    // --- CASE 1: Player has righted the MCU ---
		    if (!tilt_detected) {
		        // If player upright but has not pressed button yet
		        if (!button_pressed && (now - tilt_start_time >= FALLEN_TIMEOUT)) {
		            // Timeout → player didn't press button in time
		            send_game_message_IT("Player stood up but froze. Game over!\r\n");
		            ssd1306_Fill(Black);
		            ssd1306_UpdateScreen();
		            DrawGameOverScreen();
		            ssd1306_UpdateScreen();
		            game_over = 1;
		            return;
		        }

		        // If player upright AND presses button → recover
		        if (button_pressed) {
		            button_pressed = 0;
		            tilt_active = 0;
		            send_game_message_IT("Player upright. Resuming game.\r\n");
		            DrawPlayerUprightScreen();

		            uint32_t wait_start = HAL_GetTick();
		            while (HAL_GetTick() - wait_start < 500) {
		                // short delay to show screen
		            }

		            ssd1306_Fill(Black);
		            ssd1306_UpdateScreen();
		        }
		    }

		    // --- CASE 2: Remained tilted for too long ---
		    else if (tilt_detected && (now - tilt_start_time >= FALLEN_TIMEOUT)) {
		        send_game_message_IT("Player stayed down too long. Game over!\r\n");
		        ssd1306_Fill(Black);
		        ssd1306_UpdateScreen();
		        DrawGameOverScreen();
		        ssd1306_UpdateScreen();
		        game_over = 1;
		        tilt_detected = 0;
		        return;
		    }

		    // Skip rest of loop while tilt logic is active
		    continue;
		}

//---------------------------------------------------------------- GAME CHANGE CHECK ------------------------------------------------------------------------------------//
		if (button_pressed) {
			static uint32_t last_press_time = 0;
			static uint8_t press_count = 0;
			button_pressed = 0;
			if (now - last_press_time < DOUBLE_PRESS_INTERVAL) {
				press_count++;
			} else {
				press_count = 1;
			}
			last_press_time = now;
			if (press_count == 2) {
				press_count = 0;
				if (buzzer_on) {
					BSP_Buzzer_StopTone();
					buzzer_on = 0;
				}
				current_game = 1; 			// switch to Game 1 ; exit Game2_Run early

				send_game_message_IT("Switching back to Game 1\r\n");
				uint32_t wait_start = HAL_GetTick();
				while (uart_busy && (HAL_GetTick() - wait_start < 500)) {
				// idle until UART done, up to 500ms max -> prevents game 1 start up message from being lost
				}
				return; //go back to main
			}
		}

		// ENVIRONMENT MONITORING  ------------------------------------------------------------------
		if (now - env_timer >= 1000) {
			env_timer = now;
			temp_data = BSP_TSENSOR_ReadTemp();
			humidity_data = BSP_HSENSOR_ReadHumidity();
			pressure_data = BSP_PSENSOR_ReadPressure();
			abnormal = get_abnormal_sensors(temp_data, humidity_data, pressure_data);
		}
		// only sends out warnings every WARN_INTERVAL -> prevents spam that blocks game messages
		if (abnormal && now - last_warn_time >= WARN_INTERVAL) {
			send_environment_warnings(abnormal, temp_data, humidity_data, pressure_data);
			last_warn_time = now;
		}

		// PROXIMITY DETECTION  ------------------------------------------------------------------
		float magnet_value = get_magnetometer_value();
		uint32_t blink_delay = get_blink_delay(magnet_value);
		static uint32_t near_since = 0;

		// If enforcer is near
		if (magnet_value > MAG_KINDA_NEAR) {
		// Step 1: Send warning once when first detected
			if (!warned) {
				send_game_message_IT("Enforcer nearby! Be careful.\r\n");
				BSP_Buzzer_StartTone(1000);
				buzzer_on = 1;
				buzzer_start_time = now;
				warned = 1;
				near_since = now;   // start proximity timer
			}
			// Step 2: If enforcer has stayed nearby for >1s, start trigger
			if (!triggered && warned && (now - near_since >= 1000)) {
				triggered = 1;
				trigger_time = now;
		    }
		} else if (warned || triggered) {
		    // Enforcer moved away — reset everything
		    warned = 0;
		    triggered = 0;
		    near_since = 0;
		    BSP_LED_Off(LED2);
		    continue;
		}
		// turn off buzzer after 200 ms
		if (buzzer_on && (HAL_GetTick() - buzzer_start_time >= 200)) {
		    BSP_Buzzer_StopTone();
		    buzzer_on = 0;
		}

		// Blink LED if enforcer is near
		if (blink_delay > 0) {
		    LED_blink(blink_delay);
		}

		// If trigger active → escape or game over
		if (triggered) {
		    if (button_pressed) {
		        button_pressed = 0;
		        send_game_message_IT("Player escaped, good job!\r\n");
		        triggered = 0;
		        if (buzzer_on) {
		            BSP_Buzzer_StopTone();
		            buzzer_on = 0;
		        }
		    } else if (now - trigger_time >= 2500) { //trigger start time only 500ms after enforcer first detected
		        send_game_message_IT("Game over!\r\n");
		        game_over = 1;
		        return;
		    }
		}

	}
}

//---------------------------------------------------------------- INITIALISED FUNCTIONS ------------------------------------------------------------------------------------//
void UART1_Init(void)
{
	/* Pin configuration for UART. BSP_COM_Init() can do
	this automatically */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	/* Configuring UART1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling =
	UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit =
	UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		while(1);
	}
	 // Enable NVIC for USART1
	HAL_NVIC_SetPriority(USART1_IRQn, 2, 0); //SET PRIORITY
	HAL_NVIC_EnableIRQ(USART1_IRQn);
}

static void MX_GPIO_Init(void)
{
    // Enable clocks for GPIOC and GPIOB
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

static void MX_I2C1_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    __HAL_RCC_I2C1_CLK_ENABLE();
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00C0216C;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
    HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0);
}

// MCU hardware/peripheral init -> makes the STM32 respond to the tilt interrupt signal by accelerometer
static void TILT_EXTI_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // enable GPIOD clock (since INT1 is on PD11)
    __HAL_RCC_GPIOD_CLK_ENABLE();

    // configure PD11 (INT1 from LSM6DSL) as input with external interrupt
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

//---------------------------------------------------------------- CALLBACK FUNCTIONS ------------------------------------------------------------------------------------//

// Callback when transmission is complete
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        uart_busy = 0; // UART ready for next message
    }
}

void USART1_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart1);
}

// Callback for external interrupts
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == BUTTON_EXTI13_Pin)
    {
        button_pressed = 1;
    }
    else if (GPIO_Pin == GPIO_PIN_11) // PD11 = INT1 from LSM6DSL
    {
        tilt_detected = 1;
    }
}

//---------------------------------------------------------------- HELPER FUNCTIONS ------------------------------------------------------------------------------------//

// get abnormal sensors
uint8_t get_abnormal_sensors(float t, float h, float p) {
    uint8_t result = 0;
    if (t < TEMP_MIN || t > TEMP_MAX) result |= ABNORMAL_TEMP;
    if (h < HUM_MIN  || h > HUM_MAX)  result |= ABNORMAL_HUM;
    if (p < PRES_MIN || p > PRES_MAX) result |= ABNORMAL_PRES;
    return result;
}

// generate and send non-blocking environment warning messages
void send_environment_warnings(uint8_t abnormal, float temp, float hum, float pres) {
    static char msg[256];  // guarantees the buffer persists in memory for the entire program runtime, instead of being destroyed when the function returns
    msg[0] = '\0';         // clear previous content
    char buf[64];
    // temperature
    if (abnormal & ABNORMAL_TEMP) {
        if (temp > TEMP_MAX) sprintf(buf, "Too hot to handle! T:%.1fC. ", temp);
        else if (temp < TEMP_MIN) sprintf(buf, "Ice age detected! T:%.1fC. ", temp);
        strcat(msg, buf); //strcat - concatenate (append) one string to the end of another.
    }
    // humidity
    if (abnormal & ABNORMAL_HUM) {
        if (hum > HUM_MAX) sprintf(buf, "Feels like a sauna! H:%.1f%%. ", hum);
        else if (hum < HUM_MIN) sprintf(buf, "Too dry! H:%.1f%%. ", hum);
        strcat(msg, buf);
    }
    // pressure
    if (abnormal & ABNORMAL_PRES) {
        if (pres > PRES_MAX) sprintf(buf, "High pressure! P:%.1fatm. ", pres);
        else if (pres < PRES_MIN) sprintf(buf, "Low pressure zone! P:%.1fatm. ", pres);
        strcat(msg, buf);
    }
    strcat(msg, "\r\n"); // add new line
    // non-blocking transmit
    if (!uart_busy) {
        uart_busy = 1;
        HAL_UART_Transmit_IT(&huart1, (uint8_t*)msg, strlen(msg));
    }
}

// blink LED at a certain frequency
void LED_blink(uint32_t delay_ms) {
	static uint32_t last_toggle_time = 0; // remembers last toggle tick
	if (HAL_GetTick() - last_toggle_time >= delay_ms) {
		BSP_LED_Toggle(LED2);   // toggle LED
		last_toggle_time = HAL_GetTick();  // reset timer
	}
}


// get magnetometer reading
float get_magnetometer_value(void) {
    int16_t mag_data_i16[3] = {0};
    float mag_data[3];

    // read the magnetometer
    BSP_MAGNETO_GetXYZ(mag_data_i16);

    // convert to float
    mag_data[0] = (float)mag_data_i16[0];
    mag_data[1] = (float)mag_data_i16[1];
    mag_data[2] = (float)mag_data_i16[2];

    // compute "magnitude" as sum of absolute values
    float magnet_value = fabsf(mag_data[0]) + fabsf(mag_data[1]) + fabsf(mag_data[2]);

    return magnet_value;
}

// get appropriate blink delay
uint32_t get_blink_delay(float magnet_value) {
    if (magnet_value > MAG_VERY_NEAR) {
        return 100;   // very fast blink
    } else if (magnet_value > MAG_NEAR) {
        return 400;   // medium blink
    } else if (magnet_value > MAG_KINDA_NEAR) {
        return 800;   //slow blink
    } else {
    	return 0;
    }
}

// send game messages
void send_game_message_IT(const char *msg) {
    if (!uart_busy) {
        uart_busy = 1;
        HAL_UART_Transmit_IT(&huart1, (uint8_t*)msg, strlen(msg));
    }
}

//---------------------------------------------------------------- MELODY FUNCTIONS ------------------------------------------------------------------------------------//
// structure to define a single note: frequency + duration
typedef struct {
    uint32_t freq;      // frequency in Hz, 0 = rest
    uint32_t duration;  // duration in milliseconds
} Note_t;				// name -> Note_t

// simplified Squid Game main theme (adjust frequencies/durations as needed)

static const Note_t SquidGameMelody[] = {
    { 490, 200 },  	// B4
	{ 0,   50 },	// rest
    { 490, 200 },	// B4
	{ 0,   50 },  	// rest
    { 490, 400 },  	// B4
	{ 0,   50 },  	// rest

    { 490, 200 },  	// B4
	{ 0,   50 },	// rest
    { 490, 200 },	// B4
	{ 0,   50 },  	// rest
    { 490, 400 },  	// B4
	{ 0,   50 },  	// rest

	{ 622, 300 },  	// Eflat5
	{ 0,   50 },  	// rest
	{ 490, 300 },  	// B4
	{ 0,   50 },  	// rest
	{ 490, 300 },  	// B4
	{ 0,   50 },  	// rest
	{ 442, 300 },  	// A4
	{ 0,   50 },  	// rest
	{ 400, 300 },  	// G4
	{ 0,   50 },  	// rest
	{ 442, 300 },  	// A4
	{ 0,   50 },  	// rest
	{ 490, 300 },  	// B4
	{ 0,   50 },  	// rest

    { 0,   0 }     	// end marker
};

static const Note_t SquidGameEndMelody[] = {
    { 490, 200 },  	// B4
	{ 0,   50 },	// rest
    { 490, 200 },	// B4
	{ 0,   50 },  	// rest
    { 490, 400 },  	// B4
	{ 0,   50 },  	// rest

    { 490, 200 },  	// B4
	{ 0,   50 },	// rest
    { 490, 200 },	// B4
	{ 0,   50 },  	// rest
    { 490, 400 },  	// B4
	{ 0,   50 },  	// rest

	{ 490, 300 },  	// B4
	{ 0,   20 },  	// rest
	{ 442, 300 },  	// A4
	{ 0,   20 },  	// rest
	{ 392, 300 },  	// G4
	{ 0,   20 },  	// rest
	{ 442, 300 },  	// A4
	{ 0,   20 },  	// rest
	{ 392, 300 },  	// G4
	{ 0,   20 },  	// rest
	{ 327, 300 },  	// E4
	{ 0,   20 },  	// rest
	{ 327, 300 },  	// E4

    { 0,   0 }     	// end marker
};

static const Note_t GL_Melody[] = {
    { 466, 300 },  	// Bflat4
	{ 0,   50 },	// rest
	{ 618, 300 },  	// Eflat5
	{ 0,   50 },	// rest

	{ 618, 600 },  	// Eflat5
	{ 0,   50 },	// rest
	{ 618, 600 },  	// Eflat5
	{ 0,   50 },	// rest
	{ 550, 600 },  	// Dflat5
	{ 0,   50 },	// rest

	{ 618, 300 },  	// Eflat5
	{ 0,   50 },	// rest
	{ 618, 300 },  	// Eflat5
	{ 0,   50 },	// rest

	{ 466, 300 },  	// Bflat4
	{ 0,   50 },	// rest
	{ 466, 300 },  	// Bflat4
	{ 0,   50 },	// rest
	{ 550, 200 },  	// Dflat5

	{ 0,   420 },	// rest

    { 0,   0 }     	// end marker
};

//---------------------------------------------------------------- START MELODY ------------------------------------------------------------------------------------//
// playback state variables
static uint8_t melody_index = 0;
static uint32_t note_start_time = 0;
static _Bool melody_playing = 0;

// start the melody from the first note
void start_melody(void) {
    melody_index = 0;
    melody_playing = 1;
    note_start_time = HAL_GetTick();

    if (SquidGameMelody[melody_index].freq > 0) {
        BSP_Buzzer_StartTone(SquidGameMelody[melody_index].freq);
    }
}

// update the melody (call this periodically in main loop)
void update_melody(void) {
    if (!melody_playing) return;
    uint32_t now = HAL_GetTick();
    uint32_t note_duration = SquidGameMelody[melody_index].duration;
    if (now - note_start_time >= note_duration) {
        melody_index++;
        // check for end of melody, stop at end
        if (SquidGameMelody[melody_index].freq == 0 &&
            SquidGameMelody[melody_index].duration == 0) {
            melody_playing = 0;
            BSP_Buzzer_StopTone();
        } else {
            // play next note or rest
            if (SquidGameMelody[melody_index].freq > 0) {
                BSP_Buzzer_StartTone(SquidGameMelody[melody_index].freq);
            } else {
                BSP_Buzzer_StopTone(); // rest
            }
            note_start_time = now;
        }
    }
}

// returns true if the melody is still playing
static _Bool is_melody_playing(void) {
    return melody_playing;
}

//---------------------------------------------------------------- END MELODY ------------------------------------------------------------------------------------//
// playback state variables
static uint8_t emelody_index = 0;
static uint32_t enote_start_time = 0;
static _Bool emelody_playing = 0;

// start the melody from the first note
void start_emelody(void) {
    emelody_index = 0;
    emelody_playing = 1;
    enote_start_time = HAL_GetTick();

    if (SquidGameEndMelody[emelody_index].freq > 0) {
        BSP_Buzzer_StartTone(SquidGameEndMelody[emelody_index].freq);
    }
}

// update the melody (call this periodically in main loop)
void update_emelody(void) {
    if (!emelody_playing) return;
    uint32_t now = HAL_GetTick();
    uint32_t enote_duration = SquidGameEndMelody[emelody_index].duration;
    if (now - enote_start_time >= enote_duration) {
        emelody_index++;
        // check for end of melody, stop at end
        if (SquidGameEndMelody[emelody_index].freq == 0 &&
            SquidGameEndMelody[emelody_index].duration == 0) {
            emelody_playing = 0;
            BSP_Buzzer_StopTone();
        } else {
            // play next note or rest
            if (SquidGameEndMelody[emelody_index].freq > 0) {
                BSP_Buzzer_StartTone(SquidGameEndMelody[emelody_index].freq);
            } else {
                BSP_Buzzer_StopTone(); // rest
            }
            enote_start_time = now;
        }
    }
}

// returns true if the melody is still playing
static _Bool is_emelody_playing(void) {
    return emelody_playing;
}

//---------------------------------------------------------------- GL MELODY ------------------------------------------------------------------------------------//
// playback state variables
static uint8_t glmelody_index = 0;
static uint32_t glnote_start_time = 0;
static _Bool glmelody_playing = 0;

// start the melody from the first note
void start_glmelody(void) {
    glmelody_index = 0;
    glmelody_playing = 1;
    glnote_start_time = HAL_GetTick();

    if (GL_Melody[glmelody_index].freq > 0) {
        BSP_Buzzer_StartTone(GL_Melody[glmelody_index].freq);
    }
}

// update the melody (call this periodically in main loop)
void update_glmelody(void) {
    if (!glmelody_playing) return;
    uint32_t now = HAL_GetTick();
    uint32_t glnote_duration = GL_Melody[glmelody_index].duration;
    if (now - glnote_start_time >= glnote_duration) {
        glmelody_index++;
        // check for end of melody, stop at end
        if (GL_Melody[glmelody_index].freq == 0 &&
        		GL_Melody[glmelody_index].duration == 0) {
            glmelody_playing = 0;
            BSP_Buzzer_StopTone();
        } else {
            // play next note or rest
            if (GL_Melody[glmelody_index].freq > 0) {
                BSP_Buzzer_StartTone(GL_Melody[glmelody_index].freq);
            } else {
                BSP_Buzzer_StopTone(); // rest
            }
            glnote_start_time = now;
        }
    }
}


//---------------------------------------------------------------- OLED FUNCTIONS ------------------------------------------------------------------------------------//

void DrawStartupScreen(void) {
    // Clear screen
    ssd1306_Fill(Black);
    // --- TEXT: centered "SOTONG GAME" ---
    ssd1306_SetCursor(30, 5);
    ssd1306_WriteString("SOTONG GAME", Font_7x10, White);
    // --- SYMBOLS (O, □, △) LEFT SIDE ---
    ssd1306_DrawCircle(10, 35, 5, White);                  // Circle
    ssd1306_DrawRectangle(20, 30, 30, 40, White);          // Square
    SSD1306_VERTEX tri_left[] = {
        {40, 30}, {45, 40}, {35, 40}, {40, 30}
    };
    ssd1306_Polyline(tri_left, 4, White);
    // --- SYMBOLS (O, □, △) RIGHT SIDE ---
    ssd1306_DrawCircle(118, 35, 5, White);                 // Circle
    ssd1306_DrawRectangle(98, 30, 108, 40, White);         // Square
    SSD1306_VERTEX tri_right[] = {
        {88, 30}, {93, 40}, {83, 40}, {88, 30}
    };
    ssd1306_Polyline(tri_right, 4, White);
    // --- CENTER SQUID ---
    // Head (triangle)
    SSD1306_VERTEX head[] = {
        {57, 45}, {67, 28}, {77, 45}, {57, 45}
    };
    ssd1306_Polyline(head, 4, White);
    // Eyes
    ssd1306_FillCircle(61, 36, 2, White);
    ssd1306_FillCircle(73, 36, 2, White);
    // Body
    ssd1306_DrawRectangle(62, 45, 72, 58, White);
    // Tentacles
    ssd1306_Line(62, 58, 59, 63, White);
    ssd1306_Line(65, 58, 63, 63, White);
    ssd1306_Line(69, 58, 71, 63, White);
    ssd1306_Line(72, 58, 76, 63, White);
    // Update screen
    ssd1306_UpdateScreen();
}

void DrawPlayerFallenScreen(void) {
    ssd1306_Fill(Black);
    // --- TEXT: split into two lines, centered ---
    // Line 1: "Player has fallen"
    char *line1 = "Player has fallen";
    uint8_t x1 = (128 - (strlen(line1) * 7)) / 2;  // 7 pixels per char
    ssd1306_SetCursor(x1, 0);
    ssd1306_WriteString(line1, Font_7x10, White);
    // Line 2: "over!"
    char *line2 = "over!";
    uint8_t x2 = (128 - (strlen(line2) * 7)) / 2;
    ssd1306_SetCursor(x2, 12);  // Next line (10 pixels font height + 2 px spacing)
    ssd1306_WriteString(line2, Font_7x10, White);
    // --- HORIZONTAL SQUID ---
    // Body rectangle (lying down)
    ssd1306_DrawRectangle(50, 35, 85, 45, White);
    // Head (semi-circle lying sideways)
    ssd1306_DrawCircle(45, 40, 5, White);
    // Eyes (shocked ":0")
    ssd1306_FillCircle(42, 38, 2, White);  // left eye
    ssd1306_FillCircle(42, 42, 2, White);  // right eye
    ssd1306_SetCursor(52, 38);             // mouth ":0"
    ssd1306_WriteString(":0", Font_7x10, White);
    // Tentacles (lying down)
    ssd1306_Line(85, 35, 95, 30, White);
    ssd1306_Line(85, 38, 95, 38, White);
    ssd1306_Line(85, 41, 95, 46, White);
    ssd1306_Line(85, 44, 95, 50, White);
    // Update screen
    ssd1306_UpdateScreen();
}

void DrawPlayerUprightScreen(void) {
    ssd1306_Fill(Black);
    // --- TEXT: "Player has gotten up!" centered ---
    char *line1 = "Player has";
    char *line2 = "gotten up!";
    uint8_t x1 = (128 - (strlen(line1) * 7)) / 2; // 7 px per char for Font_7x10
    uint8_t x2 = (128 - (strlen(line2) * 7)) / 2;
    ssd1306_SetCursor(x1, 0);
    ssd1306_WriteString(line1, Font_7x10, White);
    ssd1306_SetCursor(x2, 12);  // Next line
    ssd1306_WriteString(line2, Font_7x10, White);
    // --- VERTICAL SQUID (shifted manually) ---
    // Body rectangle (standing)
    ssd1306_DrawRectangle(62, 40, 72, 60, White);
    // Head (triangle pointing upwards)
    SSD1306_VERTEX head[] = {
        {57, 40}, {67, 30}, {77, 40}, {57, 40}
    };
    ssd1306_Polyline(head, 4, White);
    // Eyes
    ssd1306_FillCircle(61, 35, 2, White);
    ssd1306_FillCircle(73, 35, 2, White);
    // Mouth (smile)
    SSD1306_VERTEX smile[] = {
        {62, 42}, {64, 44}, {66, 44}, {68, 42}
    };
    ssd1306_Polyline(smile, 4, White);
    // Tentacles (hanging down)
    ssd1306_Line(62, 60, 59, 65, White);
    ssd1306_Line(65, 60, 63, 65, White);
    ssd1306_Line(69, 60, 71, 65, White);
    ssd1306_Line(72, 60, 76, 65, White);
    // Update screen
    ssd1306_UpdateScreen();
}

void DrawGameOverScreen(void) {
    ssd1306_Fill(Black);
    // --- TEXT: "GAME OVER" centered at top ---
    char *line = "GAME OVER";
    uint8_t x = (128 - (strlen(line) * 7)) / 2;  // 7 pixels per char for Font_7x10
    ssd1306_SetCursor(x, 0);
    ssd1306_WriteString(line, Font_7x10, White);

    // --- DEAD SQUID (lying horizontally) ---
    // Body rectangle
    ssd1306_DrawRectangle(50, 35, 85, 45, White);
    // Head (circle)
    ssd1306_DrawCircle(45, 40, 5, White);

    // Eyes as Xs (better aligned inside the circle)
    // Left eye
    ssd1306_Line(42, 38, 48, 42, White);  // /
    ssd1306_Line(42, 42, 48, 38, White);  // \

    // Right eye
    ssd1306_Line(43, 36, 47, 40, White);  // /
    ssd1306_Line(43, 40, 47, 36, White);  // \

    // Tentacles (lying down)
    ssd1306_Line(85, 35, 95, 30, White);
    ssd1306_Line(85, 38, 95, 38, White);
    ssd1306_Line(85, 41, 95, 46, White);
    ssd1306_Line(85, 44, 95, 50, White);

    // Update screen
    ssd1306_UpdateScreen();
}
