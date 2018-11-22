
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Copyright (c) 2018 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdlib.h"
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "math.h"
#include "stm32l475e_iot01_qspi.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;

DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Filter_HandleTypeDef hdfsdm1_filter1;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
float M_PI  = 3.1415926;
float e = 2.718281828459045235360;
int systick_flag = 0;
int tim3_flag = 0;
int SAMPLE_SIZE = 1600; //generating samples for only 0.1s
int SAMPLE_FREQ = 16000;
float f1 = 261.63, f2 = 392.00, f = 440; // C4 and G4
float s1 = 0, s2 = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_DAC1_Init(void);
void StartDefaultTask(void const * argument);
void eig_mat_f32(float32_t *pSrcA, float32_t *pDstA , float32_t *pDstB);
void cov(float32_t *x_array, float32_t *cov_array);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

int fputc(int ch, FILE *f) {
    while (HAL_OK != HAL_UART_Transmit(&huart1, (uint8_t *) &ch, 1, 30000));
    return ch;
}
int fgetc(FILE *f) {
    uint8_t ch = 0;
    while (HAL_OK != HAL_UART_Receive(&huart1, (uint8_t *)&ch, 1, 30000));
    return ch;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
float test;
float test1;


float32_t W_array[4];
float32_t WT_array[4];
float32_t w_array[2] = {0.3, 0.6}; //Initialized to "random value"
float32_t wLast_array[2];
float32_t wT_array[2];
float32_t a_array[4] = {1,2,
												2,1};



//W = 2x2 weight matrix, used to find A, initialized with 3 6 / 6 3
int k = 0;
float conv = 1e-7; //convergence criteria
int maxIt = 10; //Max number of iterations
float w_magn = 0; // w magnitude


float32_t dotProd;



int main(void)
{
    /* USER CODE BEGIN 1 */
    
    arm_matrix_instance_f32 A;
    arm_mat_init_f32(&A,2,2,a_array);

    float32_t adwt_array[4];
    arm_matrix_instance_f32 Adwt;
    arm_mat_init_f32(&Adwt,2,2,a_array);

    float32_t s_array[3200];
    arm_matrix_instance_f32 s_m;
    arm_mat_init_f32(&s_m,2,1600,s_array);

    float32_t x_array[3200];
    arm_matrix_instance_f32 x_m;
    arm_mat_init_f32(&x_m,2,1600,x_array);

    float32_t xT_array[3200];
    arm_matrix_instance_f32 xT_m;
    arm_mat_init_f32(&xT_m,2,1600,xT_array);

    float32_t cov_array[4];
    arm_matrix_instance_f32 cov_m;
    arm_mat_init_f32(&cov_m,2,2,cov_array);

    float32_t eigenVal_array[4];
    arm_matrix_instance_f32 eigenVal_m;
    arm_mat_init_f32(&eigenVal_m,2,2,eigenVal_array);

    float32_t eigenVect_array[4];
    arm_matrix_instance_f32 eigenVect_m;
    arm_mat_init_f32(&eigenVect_m,2,2,eigenVect_array);

    float32_t eigenVectT_array[4];
    arm_matrix_instance_f32 eigenVectT_m;
    arm_mat_init_f32(&eigenVectT_m,2,2,eigenVectT_array);

    float32_t whitesig_array[4];
    arm_matrix_instance_f32 whitesig_m;
    arm_mat_init_f32(&whitesig_m,2,2,whitesig_array);

    float32_t whitening_array[4];
    arm_matrix_instance_f32 whitening_m;
    arm_mat_init_f32(&whitening_m,2,2,whitening_array);

    float32_t dewhitening_array[3200];
    arm_matrix_instance_f32 dewhitening_m;
    arm_mat_init_f32(&dewhitening_m,2,2,dewhitening_array);

    //numOfIC = 2, B = zeros(numOfIC) --> B = 2x2 matrix
    float32_t B_array[4];
    arm_matrix_instance_f32 B_m;
    arm_mat_init_f32(&B_m,2,2,B_array);

    float32_t BT_array[4];
    arm_matrix_instance_f32 BT_m;
    arm_mat_init_f32(&BT_m,2,2,BT_array);

    float32_t temp_m_array[4];
    arm_matrix_instance_f32 temp_m;
    arm_mat_init_f32(&temp_m,2,2,temp_m_array);

    float32_t temp_v_array[2];
    arm_matrix_instance_f32 temp_v;
    arm_mat_init_f32(&temp_m,2,1,temp_v_array);

    float32_t temp_v_1600_array[1600];
    arm_matrix_instance_f32 temp_v_1600;
    arm_mat_init_f32(&temp_v_1600,1600,1,temp_v_1600_array);

    arm_matrix_instance_f32 w;
    arm_mat_init_f32(&w,2,1,w_array);

    arm_matrix_instance_f32 wT;
    arm_mat_init_f32(&wT,2,1,wT_array);

    arm_matrix_instance_f32 W;
    arm_mat_init_f32(&W,2,2,W_array);

    arm_matrix_instance_f32 WT;
    arm_mat_init_f32(&WT,2,2,WT_array);

    float32_t A_array[4];
    arm_matrix_instance_f32 A_m;
    arm_mat_init_f32(&A_m,2,2,A_array);



    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/

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
    MX_USART1_UART_Init();
    MX_DFSDM1_Init();
    MX_DAC1_Init();
    BSP_QSPI_Init();
    /* USER CODE BEGIN 2 */
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
    // BSP_QSPI_Erase_Chip();
    /* USER CODE END 2 */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* Create the thread(s) */
    /* definition and creation of defaultTask */
    //osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
    //defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */


    /* Start scheduler */
    //osKernelStart();

    /* We should never get here as control is now taken by the scheduler */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */



    // TODO: Once you have verified that your sine wave generation
    // is correct, you must think about how to store the wave.
    //    int size = SAMPLE_SIZE * 32 / 8;
    //    test = arm_sin_f32(M_PI/2);
    //    test1 = arm_sin_f32(5 * M_PI/2);
    //    for (int t = 0; t < 25; t ++ ){
    //        BSP_QSPI_Erase_Sector(t);
    //    }

    //BSP_QSPI_Erase_Chip();

    for(int t = 0; t < SAMPLE_SIZE; t++){
        float x = 2 * M_PI * t / SAMPLE_FREQ;

        //Generates the signal and stores in in SRAM
        s_array[t] = arm_sin_f32((float) x*f1);
        s_array[SAMPLE_SIZE + t] = arm_sin_f32((float) x*f2);

    }

    //Mixes the signals
    arm_mat_mult_f32(&A, &s_m, &x_m);

    /*****computing cov and eig (pcamat)*****/
    //Finds the covariance matrix
    cov(&x_array[0], &cov_array[0]);

    //Computes the eigenvalues and eigenvectors of the covariance matrix
    eig_mat_f32(&cov_array[0], &eigenVal_array[0] , &eigenVect_array[0]);

    //in matlab, D = eigenvalues  and E = eigenvectors


    /*****calculating whitening & dewhitening matricesa (whitenv)*****/
    for(int i = 0; i< 4; i++){
        whitening_array[i] = sqrt(eigenVal_array[i]);                 // sqrt(D) ** saving sqrt(D) temporarily into whitening_m
    }
    arm_mat_mult_f32(&eigenVect_m, &whitening_m, &dewhitening_m);     // dewhitening_m = E * sqrt(D)
    arm_mat_inverse_f32    (&whitening_m, &whitening_m);              // inv(sqrt(D))
    arm_mat_trans_f32(&eigenVect_m,&eigenVectT_m);                    // E'
    arm_mat_mult_f32(&whitening_m, &eigenVectT_m, &whitening_m);    	// whitening_m = inv(sqrt(D)) * E'

    // Project to the eigenvectors of the covariance matrix.
    // Whiten the samples and reduce dimension simultaneously
    arm_mat_mult_f32(&whitening_m, &x_m, &whitesig_m);                // whitesig = whiteningMatrix * mixedsig


    /****FastICA***/
    //B = zeros(numOfIC), numOfIC = 2, B ---> 2x2 matrix of zeros
    B_array[0] = 0;
    B_array[1] = 0;
    B_array[2] = 0;
    B_array[3] = 0;

    for(int i = 0; i < 2; i++){
        // w_array initialised to be {0.3, 0.6};

        // Take a random initial vector of length 1 and orthogonalise it
        // with respect to the other vectors
        arm_mat_trans_f32(&B_m,&BT_m);                                 	//B'
        arm_mat_mult_f32(&B_m, &BT_m, &temp_m);                         //B * B'
        arm_mat_mult_f32(&temp_m, &w, &temp_v);                         //B * B' * w
        arm_mat_sub_f32(&w,&temp_v,&w);                                 //w = w - B * B' * w
        w_magn = sqrt(pow(w_array[0],2) + pow(w_array[1],2));           //norm(w) = sqrt(w[1]^2 + w[2]^2)
        arm_mat_scale_f32(&w, 1/w_magn, &w);                            //w = w/norm(w)

        
        wLast_array[0] = 0;																							//wLast = zeros(size(w))
        wLast_array[1] = 0;


        while(k < maxIt){

            // Project the vector into the space orthogonal to the space
            // spanned by the earlier found basis vectors.
            arm_mat_trans_f32(&B_m,&BT_m);                              //B'
						arm_mat_mult_f32(&B_m, &BT_m, &temp_m);                     //B * B'
						arm_mat_mult_f32(&temp_m, &w, &temp_v);                     //B * B' * w
						arm_mat_sub_f32(&w,&temp_v,&w);                             //w = w - B * B' * w
						w_magn = sqrt(pow(w_array[0],2) + pow(w_array[1],2));       //norm(w) = sqrt(w[1]^2 + w[2]^2)
						arm_mat_scale_f32(&w, 1/w_magn, &w);                        //w = w/norm(w)

            float diff[2], sum[2];
            arm_sub_f32(&w_array[0], &wLast_array[1],&diff[0],2);       //diff = w - wLast
            arm_add_f32(&w_array[0], &wLast_array[1],&sum[1],2);        //sum  = w + wLast
            if(sqrt(pow(diff[0],2) + pow(diff[1],2)) < conv ||
               sqrt(pow(sum[0],2) + pow(sum[1],2)) < conv){

                // Saving the vector
                // B(:,i) = w
                B_array[i] = w_array[0];
                B_array[i + 2] = w_array[1];

                // Calculate the dewhitened vector
                // A(:,i) = dewhiteningM * w
                arm_mat_mult_f32(&dewhitening_m, &w, &temp_v);
                adwt_array[i] = temp_v_array[0];
                adwt_array[i + 2] = temp_v_array[1];

                // Calculate ICA filter
                // W(round,:) = w' * whiteningMatrix
                arm_mat_trans_f32(&w, &wT);
                arm_mat_mult_f32(&wT, &whitening_m, &temp_v);
                W_array[i] =  temp_v_array[0];
                W_array[i + 1] =  temp_v_array[1];

                break;
            }

            //WLast = w;
            wLast_array[0] = w_array[0];
            wLast_array[1] = w_array[1];

            // pow3
            // w = (x * ((x' * w) . ^3)) / SAMPLE_SIZE - (3*w)
            arm_mat_trans_f32(&x_m, &xT_m);
            arm_mat_mult_f32(&xT_m, &w, &temp_v_1600);
            for(int j = 0; j < SAMPLE_SIZE; j++){
                temp_v_1600_array[j] = pow(temp_v_1600_array[j],3);
            }
            arm_mat_mult_f32(&x_m, &temp_v_1600, &temp_v);
            arm_mat_scale_f32(&temp_v, 1/SAMPLE_SIZE, &temp_v); // (X * ((X' * w).^3)) / numSamples

            arm_mat_scale_f32(&w, 3, &w); // save 3*w directly into w

            arm_mat_sub_f32(&temp_v, &w, &w);

            // w = w / norm(w)
            w_magn = sqrt(pow(w_array[0],2) + pow(w_array[1],2));
            arm_mat_scale_f32(&w, 1/w_magn, &w);
            k++;
        }

        /*TODO:  Will have a problem here b/c w_array would always be [0.5,0.6], need to find a way to generate
         a random w_array for next iteration without overwriting the current w_array
				*
				* reb: i think it is okay?
				*/
				
        // reset random w
        w_array[0] = 0.5;
        w_array[1] = 0.6;
    }

    arm_mat_trans_f32(&W, &WT);

    // s = WT * x;
    for (int t = 0; t < SAMPLE_SIZE; t ++ ){
        arm_mat_mult_f32(&WT, &x_m, &s_m);
        // BSP_QSPI_Write((uint8_t *)&s_array[0], t*0x100, 8);
        //  1 sample from 2 channels in one page
    }

    //Reads the stored samples
    int i = 0;
    while (1)
    {
        if (systick_flag){
            //            float x = 2 * M_PI * i / SAMPLE_FREQ;
            //            s1 = arm_sin_f32((float) x * f1) * 512 + 512;
            //            s2 = arm_sin_f32((float) x * f2) * 512 + 512;
            //            HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_1,DAC_ALIGN_12B_R, (uint32_t)s1);
            //            HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_2,DAC_ALIGN_12B_R, (uint32_t)s2);
            systick_flag = 0;
            // BSP_QSPI_Read((uint8_t *)&s_array[0], i*0x100, 8);
            HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_1,DAC_ALIGN_12B_R, (uint32_t)(s_array[i]*512 + 512));
            HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_2,DAC_ALIGN_12B_R, (uint32_t)(s_array[i+SAMPLE_SIZE]*512 + 512));
            if(i == SAMPLE_SIZE){
                i = 0;
            }else{
                i++;
            }
        }
    }


}

//Computes the covariance of the data matrix and outputs the covariance matrix
void cov(float32_t *x_array, float32_t *cov_array){
    float32_t meanX;
    float32_t meanY;
    float32_t covariance;
    //Finds the mean of both columns
    for(int t = 0; t < SAMPLE_SIZE; t++){
        meanX += x_array[t];
        meanY += x_array[SAMPLE_SIZE + t];
    }

    meanX = meanX / SAMPLE_SIZE;
    meanY = meanY / SAMPLE_SIZE;

    //Finds the covariance of 1-1
    covariance = 0;
    for(int t = 0; t < SAMPLE_SIZE; t++){
        covariance += (x_array[t] - meanX) * (x_array[t] - meanX);
    }
    cov_array[0] = covariance / SAMPLE_SIZE;

    //Finds the covariance of 1-2 (and 2-1)
    covariance = 0;
    for(int t = 0; t < SAMPLE_SIZE; t++){
        covariance += (x_array[t] - meanX) * (x_array[t + SAMPLE_SIZE] - meanY);
    }
    cov_array[1] = covariance / SAMPLE_SIZE;
    cov_array[2] = cov_array[1];

    //Finds the covariance of 2-2
    covariance = 0;
    for(int t = 0; t < SAMPLE_SIZE; t++){
        covariance += (x_array[t + SAMPLE_SIZE] - meanY) * (x_array[t + SAMPLE_SIZE] - meanY);
    }
    cov_array[3] = covariance / SAMPLE_SIZE;
}

//Takes a 2x2 matrix as input and computes the eigenvalues (stored in A) and eigenvectors (stored in B)
void eig_mat_f32(float32_t *pSrcA, float32_t *pDstA , float32_t *pDstB){
    float32_t a = 1;
    float32_t b = - (pSrcA[0] + pSrcA[3]);
    float32_t det = pSrcA[0] * pSrcA[3] - pSrcA[1] * pSrcA[2];
    float32_t c =  det;
    float32_t eig1 = 0;
    float32_t eig2 = 0;

    if((pow(b, 2) - 4 * a * c) >= 0){
        eig1 = (-b + sqrt(pow(b, 2) - 4 * a * c))/(2*a);
        eig2 = (-b - sqrt(pow(b, 2) - 4 * a * c))/(2*a);
    }

    pDstA[0] = eig1;
    pDstA[1] = 0;
    pDstA[2] = 0;
    pDstA[3] = eig2;

    pDstB[0] = -(pSrcA[1]/(pSrcA[0] - eig1));
    pDstB[1] = -(pSrcA[3]/(pSrcA[2] - eig2));
    pDstB[2] = 1;
    pDstB[3] = 1;

    //return 0;
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 40;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Initializes the CPU, AHB and APB busses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_SAI1
    |RCC_PERIPHCLK_DFSDM1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
    PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
    PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
    PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
    PeriphClkInit.PLLSAI1.PLLSAI1N = 32;
    PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
    PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
    PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
    PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure the main internal regulator output voltage
     */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure the Systick interrupt time
     */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/16000);

    /**Configure the Systick
     */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* DAC1 init function */
static void MX_DAC1_Init(void)
{

    DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization
     */
    hdac1.Instance = DAC1;
    if (HAL_DAC_Init(&hdac1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**DAC channel OUT1 config
     */
    sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
    sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
    sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**DAC channel OUT2 config
     */
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/* DFSDM1 init function */
static void MX_DFSDM1_Init(void)
{

    hdfsdm1_filter0.Instance = DFSDM1_Filter0;
    hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
    hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
    hdfsdm1_filter0.Init.RegularParam.DmaMode = DISABLE;
    hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC4_ORDER;
    hdfsdm1_filter0.Init.FilterParam.Oversampling = 128;
    hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
    if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    hdfsdm1_filter1.Instance = DFSDM1_Filter1;
    hdfsdm1_filter1.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
    hdfsdm1_filter1.Init.RegularParam.FastMode = ENABLE;
    hdfsdm1_filter1.Init.RegularParam.DmaMode = DISABLE;
    hdfsdm1_filter1.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC4_ORDER;
    hdfsdm1_filter1.Init.FilterParam.Oversampling = 128;
    hdfsdm1_filter1.Init.FilterParam.IntOversampling = 1;
    if (HAL_DFSDM_FilterInit(&hdfsdm1_filter1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    hdfsdm1_channel1.Instance = DFSDM1_Channel1;
    hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
    hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_AUDIO;
    hdfsdm1_channel1.Init.OutputClock.Divider = 32;
    hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
    hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
    hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
    hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_FALLING;
    hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
    hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
    hdfsdm1_channel1.Init.Awd.Oversampling = 1;
    hdfsdm1_channel1.Init.Offset = -1152;
    hdfsdm1_channel1.Init.RightBitShift = 0x0D;
    if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    hdfsdm1_channel2.Instance = DFSDM1_Channel2;
    hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
    hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_AUDIO;
    hdfsdm1_channel2.Init.OutputClock.Divider = 32;
    hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
    hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
    hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
    hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
    hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
    hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
    hdfsdm1_channel2.Init.Awd.Oversampling = 1;
    hdfsdm1_channel2.Init.Offset = -1152;
    hdfsdm1_channel2.Init.RightBitShift = 0x0D;
    if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_1, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter1, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/** Pinout Configuration
 */
static void MX_GPIO_Init(void)
{

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/**
 * moving average
 */

void sma_c(float *sign, float *signFilt, int N, int D){

    int n;
    int i;
    float sum = 0;
    for(n=0; n<=N; n++){
        sum = 0;
        // D/2 before, (D/2)- after
        if(D%2==0){
            for(i=(n-D/2); i<=(n+((D/2)-1)); i++){
                if (i>=0&&i<=N){
                    sum += sign[i];
                }
            }
        }

        if(D%2!=0){
            for(i=(n-D/2); i<=(n+((D/2))); i++){
                if (i>=0&&i<=N){
                    sum += sign[i];
                }
            }
        }
        signFilt[n]=sum/D;
    }
}


/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

    /* USER CODE BEGIN 5 */
    /* Infinite loop */
    for(;;)
    {
        osDelay(1);
    }
    /* USER CODE END 5 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM17 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM17) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
