#include "freertos/FreeRTOS.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "stdio.h"
#include "stdlib.h"
#include "ctype.h"
#include "string.h"
#include "driver/ledc.h"
#include "freertos/semphr.h"

#define BUF_SIZE                (1024)
#define SERVOBASE_1             4
#define SERVOBASEARM_2          5
#define SERVOTOPARM_3           18
#define GRIPPER                 19

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE

#define LEDC_CHANNEL_0          LEDC_CHANNEL_0
#define LEDC_CHANNEL_1          LEDC_CHANNEL_1
#define LEDC_CHANNEL_2          LEDC_CHANNEL_2
#define LEDC_CHANNEL_3          LEDC_CHANNEL_3
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (50) // Frequency in Hertz. 
#define LEDC_NUM                (4)





//input tasks
void task_1_baseservo(void *args);
void task_2_basearmservo(void *args);
void task_3_toparmservo(void *args);
void task_4_instructionque(void *args);
void task_5_gripper(int pwm_position);

void func_instr_organiser(char instr_data_cpy[BUF_SIZE]);


static xTaskHandle taskhandle_1_baseservo;
static xTaskHandle taskhandle_2_basearmservo;
static xTaskHandle taskhandle_3_toparmservo;
static xTaskHandle taskhandle_4_instructionque;
static xTaskHandle taskhandle_5_gripper;

SemaphoreHandle_t mutex_baseServoinstr = NULL;
SemaphoreHandle_t mutex_baseArmServoinstr = NULL;
SemaphoreHandle_t mutex_toparmServoinstr = NULL;


static char instr_data[128];
float baseServo_instr = 0;
static float baseArmServo_instr = 0;
static float topArmServo_instr  = 0;


void app_main() {
    //setting up outputs

    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 2*1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
  


    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel[LEDC_NUM] = {
        {
            .speed_mode     = LEDC_MODE,
            .channel        = LEDC_CHANNEL_0,
            .timer_sel      = LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = SERVOBASE_1,
            .duty           = 0, // Set duty to 0%
            .hpoint         = 0
 
        },

        {
            .speed_mode     = LEDC_MODE,
            .channel        = LEDC_CHANNEL_1,
            .timer_sel      = LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = SERVOBASEARM_2,
            .duty           = 0, // Set duty to 0%
            .hpoint         = 0

        },
        {
            .speed_mode     = LEDC_MODE,
            .channel        = LEDC_CHANNEL_2,
            .timer_sel      = LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = SERVOTOPARM_3,
            .duty           = 0, // Set duty to 0%
            .hpoint         = 0

        },
        {
            .speed_mode     = LEDC_MODE,
            .channel        = LEDC_CHANNEL_3,
            .timer_sel      = LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = GRIPPER,
            .duty           = 0, // Set duty to 0%
            .hpoint         = 0

        },

      
    };
    for (int ch = 0; ch < LEDC_NUM; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }
    mutex_baseServoinstr = xSemaphoreCreateMutex();
    mutex_baseArmServoinstr = xSemaphoreCreateMutex();
    mutex_toparmServoinstr = xSemaphoreCreateMutex();

        // Install UART driver using an event queue here
    printf("\nstarting  the main loop \n");
    printf("\n");
    printf("\n");
    printf("\n");
    printf("\n");
    if (mutex_baseServoinstr != NULL && mutex_baseArmServoinstr != NULL && mutex_toparmServoinstr != NULL){
        printf("All the mutex was created successfully.\n");
    }

    
    xTaskCreate(task_1_baseservo,"function for base servo",2024,NULL,3,&taskhandle_1_baseservo);
    xTaskCreate(task_2_basearmservo,"function for base arm servo",2024,NULL,2,&taskhandle_2_basearmservo);
    xTaskCreate(task_3_toparmservo, "function for top arm servo", 2024, NULL, 3, &taskhandle_3_toparmservo);
    xTaskCreate(task_4_instructionque, "function for receiving user instruction", 2024,NULL ,2, &taskhandle_4_instructionque);
    
}

void task_1_baseservo(void *args){
    printf("hi i am servo base\n");
    vTaskDelay(1000 / portTICK_RATE_MS);
    while (1)
    {
        if(baseServo_instr != 0){
            if (xSemaphoreTake(mutex_baseServoinstr, 10) == pdTRUE ){
                // printf("semaphore for base servo received\n");
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, (8191 * baseServo_instr) / 100);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);
                vTaskDelay(1000 / portTICK_RATE_MS);
                printf("base servo  set to  : %.2f\n",baseServo_instr);
                // printf("released mutex from task 1\n");
                baseServo_instr = 0;
                vTaskDelay(500 / portTICK_PERIOD_MS);
            }
            xSemaphoreGive(mutex_baseServoinstr);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void task_2_basearmservo(void *args){
    printf("hi i am servo for base arm\n");   
    vTaskDelay(1000 / portTICK_RATE_MS);
    while (1)
    {
        if(baseArmServo_instr != 0){
            if (xSemaphoreTake(mutex_baseArmServoinstr, 10) == pdTRUE){
                // printf("semaphore for base arm servo received\n");
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, (8191 * baseArmServo_instr) / 100);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);
                printf("basearm are set to : %.2f\n",baseArmServo_instr);
                // printf("released mutex from task 2\n");
                baseArmServo_instr = 0;
                vTaskDelay(500 / portTICK_PERIOD_MS);
            }
            xSemaphoreGive(mutex_baseArmServoinstr);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void task_3_toparmservo(void *args){
    printf("hi i am servo for top arm\n");
    vTaskDelay(1000 / portTICK_RATE_MS);
    while (1)
    {
        if(topArmServo_instr != 0){
            if (xSemaphoreTake(mutex_toparmServoinstr, 10) == pdTRUE){
                // printf("semaphore for top arm received\n");
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, (8191 * topArmServo_instr) / 100);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_2);
                printf("top arm is set to : %.2f\n",topArmServo_instr);
                
                
                // printf("released mutex from task 3\n");
                topArmServo_instr = 0;
                vTaskDelay(500 / portTICK_PERIOD_MS);
            }
            xSemaphoreGive(mutex_toparmServoinstr);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void task_5_gripper(int pwm_position){

    // position 7 for closed grip
    // position 3 for open grip
    vTaskDelay(100 / portTICK_RATE_MS);
    if(pwm_position == 7){
        printf("grip is opening\n");
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_3, (8191 * 7) / 100);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_3);
        vTaskDelay(500 / portTICK_PERIOD_MS);
            
    }
    if(pwm_position == 5){
        printf("grip is closing\n");
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_3, (8191 * 3) / 100);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_3);
        vTaskDelay(500 / portTICK_PERIOD_MS);
            
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);

}


void task_4_instructionque(void *args){
    printf("hi i am instruction receiver \n");
    
    int dataPositionTracker = 0;
    int length = 0;
    printf("dataset size is set to %d\n", sizeof(instr_data));
    memset(instr_data, 0, sizeof instr_data);
    vTaskDelay(50 / portTICK_RATE_MS);
    while (1)
    {
        if (xSemaphoreTake(mutex_baseServoinstr, (TickType_t)15) == pdTRUE && xSemaphoreTake(mutex_baseArmServoinstr, (TickType_t)15) == pdTRUE && xSemaphoreTake(mutex_toparmServoinstr, 15) == pdTRUE)
        {
            memset(instr_data, 0, sizeof instr_data);
            // xTaskCreate(task_5_gripper, "task for opening and closng gripper", 2024, 3, 2, &taskhandle_5_gripper);
            task_5_gripper(5); // top arm should close gripper
            baseServo_instr = 0;
            printf("\n");
            printf("instruction receiver has taken all the mutex\n");
            while (1)
            {
                
                ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_0, (size_t *)&length));
                // length = uart_read_bytes(UART_NUM_0, &data, length , 100);
                if (length > 0 )
                {
                    if (dataPositionTracker < sizeof(instr_data))
                    {
                        // printf("\nadd is %p\n", &instr_data[dataPositionTracker]);

                        uart_read_bytes(UART_NUM_0, &instr_data[dataPositionTracker], length, 10 / portTICK_RATE_MS);
                        if (instr_data[dataPositionTracker] != '\n')
                        {
                            uart_write_bytes(UART_NUM_0, &instr_data[dataPositionTracker], sizeof(instr_data[dataPositionTracker]));
                            // printf("instr %c send to position %d\n", instr_data[dataPositionTracker],dataPositionTracker);
                            dataPositionTracker = dataPositionTracker + 1;
                        }
                       
                        if (instr_data[dataPositionTracker] == '\n' && dataPositionTracker != 0)
                        {
                            task_5_gripper(7); // open gtipper
                            // instr_data[dataPositionTracker] = NULL;
                
                            printf("instruction:       %s\n", (char *)instr_data);
                            func_instr_organiser(instr_data);
                            // printf("length     :       %d\n",  dataPositionTracker);
                            printf("-----------reseased all mutex---------\n");
                            xSemaphoreGive(mutex_baseServoinstr);
                            xSemaphoreGive(mutex_baseArmServoinstr);
                            xSemaphoreGive(mutex_toparmServoinstr);
                            vTaskDelay(50 / portTICK_RATE_MS);
                            uart_flush(UART_NUM_0);
                            break;
                        }
                        length = 0;
                    }
                    
                }
                
                
                vTaskDelay(50 / portTICK_RATE_MS);
            }
        }
    }

}




void func_instr_organiser(char instr_data_cpy[BUF_SIZE]){
    
  
    printf("hi i am instruc orginaser function\n");
    printf("instruction cpy :       ");
    vTaskDelay(50 / portTICK_RATE_MS);
    for (int k = 0; k < BUF_SIZE; k++)
    {
                               
        printf("%c",instr_data_cpy[k]);
        if(instr_data_cpy[k] == '\n'){
            // printf("found end of line breaking\n");
            break;
        }
    }
    for (int i = 0; i < BUF_SIZE; i++)
    {
        if (instr_data_cpy[i] == 'x' || instr_data_cpy[i] == 'X'){
            for (int x = i; x < BUF_SIZE; x++){
                if(instr_data_cpy[x] == '\n'){
                    // printf("found end of line breaking\n");
                    break;
                }
                if (isdigit(instr_data_cpy[x])){
                    // printf("%c", instr_data_cpy[x]);
                    if (baseServo_instr == 0){
                        baseServo_instr = atof(&instr_data_cpy[x]);
                    }
                }
                if (instr_data_cpy[x] == 'y' || instr_data_cpy[x] == 'Y'){
                    break;
                }
                i = x;
            }
            // printf("\nx = %.2f\n",baseServo_instr);
            
        }
        if (instr_data_cpy[i] == 'y' || instr_data_cpy[i] == 'Y'){
            for (int y = i; y < BUF_SIZE; y++){
                if(instr_data_cpy[y] == '\n'){
                    // printf("found end of line breaking\n");
                    break;
                }
                if (isdigit(instr_data_cpy[y])){
                    // printf("%c", instr_data_cpy[y]);
                    if (baseArmServo_instr == 0){
                        baseArmServo_instr = atof(&instr_data_cpy[y]);
                    }
                }
                
                if (instr_data_cpy[y] == 'z' || instr_data_cpy[y] == 'Z'){
                    break;
                }
                i = y;
            } 
            // printf("y = %.2f\n",baseArmServo_instr);
        }
        if (instr_data_cpy[i] == 'z' || instr_data_cpy[i] == 'Z'){
            for (int z = i; z < BUF_SIZE; z++){
                if(instr_data_cpy[z] == '\n'){
                    // printf("found end of line breaking\n");
                    break;
                }
                if (isdigit(instr_data_cpy[z])){
                    // printf("%c", instr_data_cpy[z]);
                    if (topArmServo_instr == 0){
                        topArmServo_instr = atof(&instr_data_cpy[z]);
                    }
                }
                i = z;
            } 
            // printf("z = %.2f\n",topArmServo_instr);
        }
        if(instr_data_cpy[i] == '\n'){
            // printf("found end of line breaking\n");
            break;
        }
    }
    memset(instr_data_cpy, 0, 128);
}

