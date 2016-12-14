// STM32F030K6T6
/**
 * unsigned char  uint8_t
 * unsigned short uint16_t
 * unsigned int   uint32_t
 */
#include "stm32f0xx.h"
#include "arm_math.h"
#include "stdlib.h"
#include "device_settings.h"

#define CLOCK_SPEED 16000000
#define USART_BAUD_RATE 115200
#define TIMER3_PERIOD_TICKS (unsigned int)(CLOCK_SPEED * 15 / USART_BAUD_RATE)
#define TIMER3_SEC_PER_PERIOD ((float)TIMER3_PERIOD_TICKS / CLOCK_SPEED)
#define TIMER3_MS_PER_PERIOD ((float)TIMER3_PERIOD_TICKS * 1000 / CLOCK_SPEED)
#define TIMER14_PERIOD 24
#define TIMER14_PRESCALER 0xFFFF
#define TIMER14_TACTS_PER_SECOND (CLOCK_SPEED / TIMER14_PERIOD / TIMER14_PRESCALER)

#define USART1_TX_DMA_CHANNEL DMA1_Channel2
#define USART1_TDR_ADDRESS (unsigned int)(&(USART1->TDR))
#define NETWORK_STATUS_LED_PIN GPIO_Pin_1
#define NETWORK_STATUS_LED_PORT GPIOA
#define SERVER_AVAILABILITI_LED_PIN GPIO_Pin_2
#define SERVER_AVAILABILITI_LED_PORT GPIOA
#define ESP8266_CONTROL_PIN GPIO_Pin_15
#define ESP8266_CONTROL_PORT GPIOA
#define PROJECTOR_RELAY_PIN GPIO_Pin_7
#define PROJECTOR_RELAY_PORT GPIOA

// General flags
#define USART_DATA_RECEIVED_FLAG 1
#define SERVER_IS_AVAILABLE_FLAG 2
#define SUCCESSUFULLY_CONNECTED_TO_NETWORK_FLAG 4
#define SEND_DEBUG_INFO_FLAG 8
#define TURN_PROJECTOR_ON 16

#define GET_VISIBLE_NETWORK_LIST_TASK 1
#define DISABLE_ECHO_TASK 2
#define CONNECT_TO_NETWORK_TASK 4
//
#define GET_AP_CONNECTION_STATUS_AND_CONNECT_TASK 16
#define GET_OWN_IP_ADDRESS_TASK 32
#define SET_OWN_IP_ADDRESS_TASK 64
#define CONNECT_TO_SERVER_TASK 128
#define SET_BYTES_TO_SEND_IN_REQUEST_TASK 256
#define GET_REQUEST_SENT_AND_RESPONSE_RECEIVED_TASK 512
#define POST_REQUEST_SENT_TASK 1024
#define GET_CURRENT_DEFAULT_WIFI_MODE_TASK 2048
#define SET_DEFAULT_STATION_WIFI_MODE_TASK 4096
#define CLOSE_CONNECTION_TASK 8192
#define GET_CONNECTION_STATUS_TASK 16384
#define GET_SERVER_AVAILABILITY_REQUEST_TASK 32768
#define GET_SERVER_AVAILABILITY_TASK 65536
#define ESTABLISH_LONG_POLLING_CONNECTION_TASK 131072
#define ESTABLISH_LONG_POLLING_CONNECTION_REQUEST_TASK 262144

#define USART_DATA_RECEIVED_BUFFER_SIZE 1000
#define PIPED_REQUEST_COMMANDS_TO_SEND_SIZE 3
#define PIPED_REQUEST_CIPSTART_COMMAND_INDEX 0
#define PIPED_REQUEST_CIPSEND_COMMAND_INDEX 1
#define PIPED_REQUEST_INDEX 2

#define PIPED_TASKS_TO_SEND_SIZE 30
#define PIPED_TASKS_HISTORY_SIZE 10
#define DEFAULT_ACCESS_POINT_GAIN_SIZE 4

#define TIMER3_10MS (unsigned short)(10 / TIMER3_MS_PER_PERIOD)
#define TIMER3_100MS (unsigned short)(100 / TIMER3_MS_PER_PERIOD)
#define TIMER14_100MS 1
#define TIMER14_200MS 2
#define TIMER14_500MS 5
#define TIMER14_1S 10
#define TIMER14_2S 20
#define TIMER14_3S 30
#define TIMER14_5S 50
#define TIMER14_10S 102
#define TIMER14_30S 305
#define TIMER14_60S 610
#define TIMER14_10MIN 6103

typedef enum {
   EXECUTE_FUNCTION_IMMEDIATELY,
   DO_NOT_EXECUTE_FUNCTION_IMMEDIATELY
} ImmediatelyFunctionExecution;

#define RECEIVED_USART_DATA_FOR_DEBUG_INFO_MAX_LENGTH 100

unsigned int piped_tasks_to_send_g[PIPED_TASKS_TO_SEND_SIZE];
unsigned int piped_tasks_history_g[PIPED_TASKS_HISTORY_SIZE];
char *piped_request_commands_to_send_g[PIPED_REQUEST_COMMANDS_TO_SEND_SIZE]; // AT+CIPSTART="TCP","address",port; AT+CIPSEND=bytes_to_send; a request
unsigned int sent_task_g;
unsigned int general_flags_g;

char USART_OK[] __attribute__ ((section(".text.const"))) = "OK";
char USART_ERROR[] __attribute__ ((section(".text.const"))) = "ERROR";
char ESP8226_REQUEST_DISABLE_ECHO[] __attribute__ ((section(".text.const"))) = "ATE0\r\n";
char ESP8226_RESPONSE_BUSY[] __attribute__ ((section(".text.const"))) = "busy";
char ESP8226_REQUEST_GET_VISIBLE_NETWORK_LIST[] __attribute__ ((section(".text.const"))) = "AT+CWLAP\r\n";
char ESP8226_RESPONSE_VISIBLE_NETWORK_LIST_PREFIX[] __attribute__ ((section(".text.const"))) = "+CWLAP:";
char ESP8226_REQUEST_GET_AP_CONNECTION_STATUS[] __attribute__ ((section(".text.const"))) = "AT+CWJAP?\r\n";
char ESP8226_RESPONSE_NOT_CONNECTED_STATUS[] __attribute__ ((section(".text.const"))) = "No AP";
char ESP8226_REQUEST_CONNECT_TO_NETWORK_AND_SAVE[] __attribute__ ((section(".text.const"))) = "AT+CWJAP_DEF=\"<1>\",\"<2>\"\r\n";
char ESP8226_REQUEST_GET_VERSION_ID[] __attribute__ ((section(".text.const"))) = "AT+GMR\r\n";
char ESP8226_RESPONSE_CONNECTED[] __attribute__ ((section(".text.const"))) = "CONNECT";
char ESP8226_CONNECTION_CLOSED[] __attribute__ ((section(".text.const"))) = "CLOSED\r\n\r\nOK";
char ESP8226_REQUEST_CONNECT_TO_SERVER[] __attribute__ ((section(".text.const"))) = "AT+CIPSTART=\"TCP\",\"<1>\",<2>\r\n";
char ESP8226_REQUEST_DISCONNECT_FROM_SERVER[] __attribute__ ((section(".text.const"))) = "AT+CIPCLOSE\r\n";
char ESP8226_REQUEST_SERVER_PING[] __attribute__ ((section(".text.const"))) = "AT+PING=\"<1>\"\r\n";
char ESP8226_REQUEST_START_SENDING[] __attribute__ ((section(".text.const"))) = "AT+CIPSEND=<1>\r\n";
char ESP8226_RESPONSE_START_SENDING_READY[] __attribute__ ((section(".text.const"))) = ">";
char ESP8226_RESPONSE_SENDING[] __attribute__ ((section(".text.const"))) = "busy s...";
char ESP8226_RESPONSE_SUCCSESSFULLY_SENT[] __attribute__ ((section(".text.const"))) = "\r\nSEND OK\r\n";
char ESP8226_RESPONSE_ALREADY_CONNECTED[] __attribute__ ((section(".text.const"))) = "ALREADY CONNECTED";
char ESP8226_RESPONSE_PREFIX[] __attribute__ ((section(".text.const"))) = "+IPD";
char ESP8226_REQUEST_GET_CURRENT_DEFAULT_WIFI_MODE[] __attribute__ ((section(".text.const"))) = "AT+CWMODE_DEF?\r\n";
char ESP8226_RESPONSE_WIFI_MODE_PREFIX[] __attribute__ ((section(".text.const"))) = "+CWMODE_DEF:";
char ESP8226_RESPONSE_WIFI_STATION_MODE[] __attribute__ ((section(".text.const"))) = "1";
char ESP8226_REQUEST_SET_DEFAULT_STATION_WIFI_MODE[] __attribute__ ((section(".text.const"))) = "AT+CWMODE_DEF=1\r\n";
char ESP8226_REQUEST_GET_OWN_IP_ADDRESS[] __attribute__ ((section(".text.const"))) = "AT+CIPSTA_DEF?\r\n";
char ESP8226_RESPONSE_CURRENT_OWN_IP_ADDRESS_PREFIX[] __attribute__ ((section(".text.const"))) = "+CIPSTA_DEF:ip:";
char ESP8226_REQUEST_SET_OWN_IP_ADDRESS[] __attribute__ ((section(".text.const"))) = "AT+CIPSTA_DEF=\"<1>\"\r\n";
char ESP8226_REQUEST_SEND_STATUS_INFO_AND_ESTABLISH_LONG_POLLING_REQUEST[] __attribute__ ((section(".text.const"))) =
      "POST /server/esp8266/projectorDeferred HTTP/1.1\r\nContent-Length: <1>\r\nHost: <2>\r\nUser-Agent: ESP8266\r\nContent-Type: application/json\r\nAccept: application/json\r\nConnection: keep-alive\r\n\r\n<3>\r\n";
char DEBUG_STATUS_JSON[] __attribute__ ((section(".text.const"))) =
      "{\"gain\":\"<1>\",\"debugInfoIncluded\":<2>,\"errors\":\"<3>\",\"usartOverrunErrors\":\"<4>\",\"usartIdleLineDetections\":\"<5>\",\"usartNoiseDetection\":\"<6>\",\"usartFramingErrors\":\"<7>\",\"lastErrorTask\":\"<8>\",\"usartData\":\"<9>\",\"timeStamp\":\"<10>\"}";
char TIMESTAMP_JSON_ELEMENT[] __attribute__ ((section(".text.const"))) = "timeStamp";
char TURN_ON_TRUE_JSON_ELEMENT[] __attribute__ ((section(".text.const"))) = "\"turnOn\":true";
char STATUS_JSON[] __attribute__ ((section(".text.const"))) = "{\"gain\":\"<1>\",\"debugInfoIncluded\":<2>,\"timeStamp\":\"<3>\"}";
char ESP8226_RESPONSE_OK_STATUS_CODE[] __attribute__ ((section(".text.const"))) = "\"statusCode\":\"OK\"";
char ESP8226_RESPONSE_HTTP_STATUS_200_OK[] __attribute__ ((section(".text.const"))) = "200 OK";
char ESP8226_RESPONSE_HTTP_STATUS_400_BAD_REQUEST[] __attribute__ ((section(".text.const"))) = "HTTP/1.1 400 Bad Request";
char SERVER_STATUS_INCLUDE_DEBUG_INFO[] __attribute__ ((section(".text.const"))) = "\"includeDebugInfo\":true";
char JSON_OBJECT_PREFIX[] __attribute__ ((section(".text.const"))) = "{";
char RESPONSE_CLOSED_BY_TOMCAT[] __attribute__ ((section(".text.const"))) = "\r\n+IPD,5:0\r\n\r\nCLOSED\r\n";
char RESPONSE_CLOSED_BY_TOMCAT_PREFIX[] __attribute__ ((section(".text.const"))) = "\r\n+IPD,5:0";
char RESPONSE_CLOSED_BY_TOMCAT_SUFFIX[] __attribute__ ((section(".text.const"))) = "CLOSED\r\n";
char RESPONSE_SERVICE_UNAVAILABLE[] __attribute__ ((section(".text.const"))) = "503 Service Unavailable";

char *usart_data_to_be_transmitted_buffer_g = NULL;
char *received_usart_error_data_g = NULL;
char usart_data_received_buffer_g[USART_DATA_RECEIVED_BUFFER_SIZE];
char default_access_point_gain_g[DEFAULT_ACCESS_POINT_GAIN_SIZE] = {' ', ' ', ' ', ' '};
volatile unsigned short usart_received_bytes_g;
volatile unsigned int final_task_for_request_resending_g;

void (*send_usart_data_function_g)() = NULL;
void (*on_response_g)() = NULL;
volatile unsigned int send_usart_data_time_counter_g;
volatile unsigned short send_usart_data_timout_sec_g = 0xFFFF;
volatile unsigned char send_usart_data_errors_counter_g;
volatile unsigned short send_usart_data_errors_unresetable_counter_g;
volatile unsigned int last_error_task_g;
volatile unsigned short network_searching_status_led_counter_g;
volatile unsigned char esp8266_disabled_counter_g;
volatile unsigned char esp8266_disabled_timer_g = TIMER14_5S;
unsigned short checking_connection_status_and_server_availability_timer_g;
volatile unsigned short visible_network_list_timer_g = TIMER14_10MIN;;
volatile unsigned char resets_occured_g;
volatile unsigned int response_timestamp_ms_g;
volatile unsigned int response_timestamp_counter_g;

volatile unsigned short usart_overrun_errors_counter_g;
volatile unsigned short usart_idle_line_detection_counter_g;
volatile unsigned short usart_noise_detection_counter_g;
volatile unsigned short usart_framing_errors_counter_g;

void IWDG_Config();
void Clock_Config();
void Pins_Config();
void TIMER3_Confing();
void TIMER14_Confing();
unsigned char handle_disable_echo_task(unsigned int current_piped_task_to_send, unsigned int *sent_flag);
unsigned char handle_get_connection_status_and_connect_task(unsigned int current_piped_task_to_send, unsigned int *sent_flag);
unsigned char handle_get_connection_status_task(unsigned int current_piped_task_to_send, unsigned int *sent_flag);
unsigned char handle_connect_to_network_task(unsigned int current_piped_task_to_send, unsigned int *sent_flag);
unsigned char handle_connect_to_server_task(unsigned int current_piped_task_to_send, unsigned int *sent_flag);
unsigned char handle_set_bytes_to_send_in_request_task(unsigned int current_piped_task_to_send, unsigned int *sent_flag);
unsigned char handle_get_request_sent_and_response_received_task(unsigned int current_piped_task_to_send, unsigned int *sent_flag);
unsigned char handle_get_current_default_wifi_mode_task(unsigned int current_piped_task_to_send, unsigned int *sent_flag);
unsigned char handle_set_default_station_wifi_mode_task(unsigned int current_piped_task_to_send, unsigned int *sent_flag);
unsigned char handle_get_own_ip_address_task(unsigned int current_piped_task_to_send, unsigned int *sent_flag);
unsigned char handle_set_own_ip_address_task(unsigned int current_piped_task_to_send, unsigned int *sent_flag);
unsigned char handle_close_connection_task(unsigned int current_piped_task_to_send, unsigned int *sent_flag);
unsigned char handle_get_visible_network_list_task(unsigned int current_piped_task_to_send, unsigned int *sent_task);
unsigned char handle_establish_long_polling_connection_task(unsigned int current_piped_task_to_send);
unsigned char handle_establish_long_polling_connection_request_task(unsigned int current_piped_task_to_send, unsigned int *sent_task);
void establish_long_polling_connection(unsigned int request_task);
void reset_device_state();
void set_flag(unsigned int *flags, unsigned int flag_value);
void reset_flag(unsigned int *flags, unsigned int flag_value);
unsigned char read_flag(unsigned int *flags, unsigned int flag_value);
void DMA_Config();
void USART_Config();
void disable_echo();
void get_network_list();
void connect_to_network();
void get_ap_connection_status();
void schedule_function_resending(void (*function_to_execute)(), unsigned short timeout, ImmediatelyFunctionExecution execute);
void send_usard_data(char string[]);
unsigned char is_usart_response_contains_elements(char *data_to_be_contained[], unsigned char elements_count);
unsigned char is_usart_response_contains_element(char string_to_be_contained[]);
unsigned char contains_string(char being_compared_string[], char string_to_be_contained[]);
void clear_usart_data_received_buffer();
unsigned short get_received_data_length();
unsigned char is_received_data_length_equal(unsigned short length);
void *set_string_parameters(char string[], char *parameters[]);
unsigned short get_string_length(char string[]);
unsigned char is_string_starts_with(char long_string[], char short_string[]);
unsigned int get_current_piped_task_to_send();
void delete_current_piped_task();
void add_piped_task_to_send_into_tail(unsigned int task);
void add_piped_task_to_send_into_head(unsigned int task);
void delete_piped_task(unsigned int task);
void on_successfully_receive_general_actions();
void prepare_http_request_without_parameters(char request_template[], unsigned int request_task);
void prepare_http_request(char address[], char port[], char request[], void (*on_response)(), unsigned int request_task);
void resend_usart_get_request_using_global_final_task();
void *num_to_string(unsigned int number);
unsigned int divide_by_10(unsigned int dividend);
unsigned char get_first_digit(unsigned int long_digit);
void *array_to_string(char array[], unsigned char array_length);
char *get_gson_element_value(char *json_string, char *json_element_to_find);
void connect_to_server();
void resend_usart_get_request(unsigned int final_task);
void set_bytes_amount_to_send();
void send_request(unsigned int sent_task_to_set);
void get_current_default_wifi_mode();
void set_default_wifi_mode();
void enable_esp8266();
void disable_esp8266();
unsigned char is_esp8266_enabled(unsigned char include_timer);
void clear_piped_request_commands_to_send();
void delete_all_piped_tasks();
unsigned char is_piped_task_to_send_scheduled(unsigned int task);
unsigned char is_piped_tasks_scheduler_full();
unsigned char is_piped_tasks_scheduler_empty();
void schedule_global_function_resending_and_send_request(unsigned int task, unsigned short timeout);
char *generate_request(char *request_template);
void *add_debug_info(char *gain, char *debug_info_included, char *response_timestamp);
unsigned int calculate_response_timestamp();
void get_own_ip_address();
void set_own_ip_address();
void close_connection();
void add_error();
void check_connection_status_and_server_availability();
void check_visible_network_list();
void add_piped_task_into_history(unsigned int task);
unsigned int get_last_piped_task_in_history();
void *get_received_usart_error_data();
void save_default_access_point_gain();

void SysTick_Handler() {
}

void DMA1_Channel2_3_IRQHandler() {
   DMA_ClearITPendingBit(DMA1_IT_TC2);
}

void TIM14_IRQHandler() {
   TIM_ClearITPendingBit(TIM14, TIM_IT_Update);

   if (visible_network_list_timer_g) {
      visible_network_list_timer_g--;
   }
   if (checking_connection_status_and_server_availability_timer_g) {
      checking_connection_status_and_server_availability_timer_g--;
   }
   if (!is_esp8266_enabled(0)) {
      esp8266_disabled_counter_g++;
   }
   if (esp8266_disabled_timer_g) {
      esp8266_disabled_timer_g--;
   }
}

void TIM3_IRQHandler() {
   TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

   // Some error eventually occurs when only the first symbol exists
   if (usart_received_bytes_g > 1) {
      set_flag(&general_flags_g, USART_DATA_RECEIVED_FLAG);
   }
   usart_received_bytes_g = 0;
   if (send_usart_data_function_g != NULL) {
      send_usart_data_time_counter_g++;
   }
   network_searching_status_led_counter_g++;
}

void EXTI0_1_IRQHandler() {

}

void EXTI4_15_IRQHandler() {
}

void USART1_IRQHandler() {
   if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET) {
      TIM_SetCounter(TIM3, 0);
      usart_data_received_buffer_g[usart_received_bytes_g] = USART_ReceiveData(USART1);
      usart_received_bytes_g++;

      if (usart_received_bytes_g >= USART_DATA_RECEIVED_BUFFER_SIZE) {
         usart_received_bytes_g = 0;
      }
   }

   if (USART_GetFlagStatus(USART1, USART_FLAG_ORE)) {
      USART_ClearITPendingBit(USART1, USART_IT_ORE);
      USART_ClearFlag(USART1, USART_FLAG_ORE);
      usart_overrun_errors_counter_g++;
   } else if (USART_GetFlagStatus(USART1, USART_FLAG_IDLE)) {
      USART_ClearITPendingBit(USART1, USART_IT_ORE);
      USART_ClearFlag(USART1, USART_FLAG_IDLE);
      usart_idle_line_detection_counter_g++;
   } else if (USART_GetFlagStatus(USART1, USART_FLAG_NE)) {
      USART_ClearITPendingBit(USART1, USART_IT_ORE);
      USART_ClearFlag(USART1, USART_FLAG_NE);
      usart_noise_detection_counter_g++;
   } else if (USART_GetFlagStatus(USART1, USART_FLAG_FE)) {
      USART_ClearITPendingBit(USART1, USART_IT_ORE);
      USART_ClearFlag(USART1, USART_FLAG_FE);
      usart_framing_errors_counter_g++;
   }
}

int main() {
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_DBGMCU, ENABLE);
   IWDG_Config();
   Clock_Config();
   Pins_Config();
   disable_esp8266();
   DMA_Config();
   USART_Config();
   TIMER3_Confing();
   TIMER14_Confing();

   add_piped_task_to_send_into_tail(DISABLE_ECHO_TASK);
   add_piped_task_to_send_into_tail(GET_CURRENT_DEFAULT_WIFI_MODE_TASK);
   add_piped_task_to_send_into_tail(GET_OWN_IP_ADDRESS_TASK);
   add_piped_task_to_send_into_tail(GET_AP_CONNECTION_STATUS_AND_CONNECT_TASK);
   add_piped_task_to_send_into_tail(GET_VISIBLE_NETWORK_LIST_TASK);
   add_piped_task_to_send_into_tail(ESTABLISH_LONG_POLLING_CONNECTION_TASK);

   while (1) {
      if (is_esp8266_enabled(1)) {
         // Seconds
         unsigned short send_usart_data_passed_time_sec = (unsigned short) (TIMER3_SEC_PER_PERIOD * send_usart_data_time_counter_g);
         unsigned int sent_task = 0;

         if (read_flag(&general_flags_g, USART_DATA_RECEIVED_FLAG)) {
            reset_flag(&general_flags_g, USART_DATA_RECEIVED_FLAG);

            if (usart_data_to_be_transmitted_buffer_g != NULL) {
               free(usart_data_to_be_transmitted_buffer_g);
               usart_data_to_be_transmitted_buffer_g = NULL;
            }

            /*if (is_string_starts_with(usart_data_received_buffer_g, RESPONSE_CLOSED_BY_TOMCAT_PREFIX)
                  || is_string_starts_with(usart_data_received_buffer_g, RESPONSE_CLOSED_BY_TOMCAT_SUFFIX)) {
               is_string_starts_with(usart_data_received_buffer_g, RESPONSE_CLOSED_BY_TOMCAT_PREFIX);
            } else {
               sent_task = sent_task_g;
            }*/
            sent_task = sent_task_g;
         } else if (send_usart_data_function_g != NULL && send_usart_data_passed_time_sec >= send_usart_data_timout_sec_g) {
            if (usart_data_to_be_transmitted_buffer_g != NULL) {
               free(usart_data_to_be_transmitted_buffer_g);
               usart_data_to_be_transmitted_buffer_g = NULL;
            }

            send_usart_data_function_g();
         }

         unsigned int current_piped_task_to_send = get_current_piped_task_to_send();
         if (send_usart_data_function_g != NULL) {
            current_piped_task_to_send = 0;
         }

         if (current_piped_task_to_send || sent_task) {
            unsigned char not_handled = 1;
            not_handled = handle_disable_echo_task(current_piped_task_to_send, &sent_task);

            if (not_handled) {
               not_handled = handle_get_connection_status_and_connect_task(current_piped_task_to_send, &sent_task);
            }
            if (not_handled) {
               not_handled = handle_get_connection_status_task(current_piped_task_to_send, &sent_task);
            }
            if (not_handled) {
               not_handled = handle_connect_to_network_task(current_piped_task_to_send, &sent_task);
            }
            if (not_handled) {
               not_handled = handle_connect_to_server_task(current_piped_task_to_send, &sent_task);
            }
            if (not_handled) {
               not_handled = handle_set_bytes_to_send_in_request_task(current_piped_task_to_send, &sent_task);
            }
            if (not_handled) {
               not_handled = handle_get_request_sent_and_response_received_task(current_piped_task_to_send, &sent_task);
            }
            if (not_handled) {
               not_handled = handle_get_current_default_wifi_mode_task(current_piped_task_to_send, &sent_task);
            }
            if (not_handled) {
               not_handled = handle_set_default_station_wifi_mode_task(current_piped_task_to_send, &sent_task);
            }
            if (not_handled) {
               not_handled = handle_get_own_ip_address_task(current_piped_task_to_send, &sent_task);
            }
            if (not_handled) {
               not_handled = handle_set_own_ip_address_task(current_piped_task_to_send, &sent_task);
            }
            if (not_handled) {
               not_handled = handle_close_connection_task(current_piped_task_to_send, &sent_task);
            }
            if (not_handled) {
               not_handled = handle_get_visible_network_list_task(current_piped_task_to_send, &sent_task);
            }
            if (not_handled) {
               not_handled = handle_establish_long_polling_connection_task(current_piped_task_to_send);
            }
            if (not_handled) {
               not_handled = handle_establish_long_polling_connection_request_task(current_piped_task_to_send, &sent_task);
            }
         }

         check_visible_network_list();

         // LED blinking
         if (!read_flag(&general_flags_g, SUCCESSUFULLY_CONNECTED_TO_NETWORK_FLAG) && network_searching_status_led_counter_g >= TIMER3_100MS) {
            network_searching_status_led_counter_g = 0;

            if (GPIO_ReadOutputDataBit(NETWORK_STATUS_LED_PORT, NETWORK_STATUS_LED_PIN)) {
               GPIO_WriteBit(NETWORK_STATUS_LED_PORT, NETWORK_STATUS_LED_PIN, Bit_RESET);
            } else {
               GPIO_WriteBit(NETWORK_STATUS_LED_PORT, NETWORK_STATUS_LED_PIN, Bit_SET);
            }
         }

         if (send_usart_data_errors_counter_g >= 10 || is_piped_tasks_scheduler_full()) {
            reset_device_state();
         }
         if (resets_occured_g >= 5) {
            NVIC_SystemReset();
         }

         if (read_flag(&general_flags_g, SUCCESSUFULLY_CONNECTED_TO_NETWORK_FLAG)) {
            GPIO_WriteBit(NETWORK_STATUS_LED_PORT, NETWORK_STATUS_LED_PIN, Bit_SET);
         } else {
            GPIO_WriteBit(NETWORK_STATUS_LED_PORT, NETWORK_STATUS_LED_PIN, Bit_RESET);
         }
         if (read_flag(&general_flags_g, SERVER_IS_AVAILABLE_FLAG)) {
            GPIO_WriteBit(SERVER_AVAILABILITI_LED_PORT, SERVER_AVAILABILITI_LED_PIN, Bit_SET);
         } else {
            GPIO_WriteBit(SERVER_AVAILABILITI_LED_PORT, SERVER_AVAILABILITI_LED_PIN, Bit_RESET);
         }
         if (read_flag(&general_flags_g, TURN_PROJECTOR_ON)) {
            GPIO_WriteBit(PROJECTOR_RELAY_PORT, PROJECTOR_RELAY_PIN, Bit_SET);
         } else {
            GPIO_WriteBit(PROJECTOR_RELAY_PORT, PROJECTOR_RELAY_PIN, Bit_RESET);
         }
      } else if (esp8266_disabled_counter_g >= TIMER14_1S) {
         esp8266_disabled_counter_g = 0;
         enable_esp8266();
      }

      IWDG_ReloadCounter();
   }
}

unsigned char handle_disable_echo_task(unsigned int current_piped_task_to_send, unsigned int *sent_flag) {
   unsigned char not_handled = 1;

   if (current_piped_task_to_send == DISABLE_ECHO_TASK) {
      not_handled = 0;
      schedule_function_resending(disable_echo, 2, EXECUTE_FUNCTION_IMMEDIATELY);
   } else if (read_flag(sent_flag, DISABLE_ECHO_TASK)) {
      not_handled = 0;
      reset_flag(&sent_task_g, DISABLE_ECHO_TASK);

      if (is_usart_response_contains_element(USART_OK)) {
         on_successfully_receive_general_actions();
      } else {
         add_error();
      }
   }
   return not_handled;
}

unsigned char handle_get_connection_status_and_connect_task(unsigned int current_piped_task_to_send, unsigned int *sent_flag) {
   unsigned char not_handled = 1;

   if (current_piped_task_to_send == GET_AP_CONNECTION_STATUS_AND_CONNECT_TASK) {
      not_handled = 0;
      schedule_function_resending(get_ap_connection_status, 2, EXECUTE_FUNCTION_IMMEDIATELY);
   } else if (read_flag(sent_flag, GET_AP_CONNECTION_STATUS_AND_CONNECT_TASK)) {
      not_handled = 0;
      reset_flag(&sent_task_g, GET_AP_CONNECTION_STATUS_AND_CONNECT_TASK);

      if (is_usart_response_contains_element(DEFAULT_ACCESS_POINT_NAME) || is_usart_response_contains_element(ESP8226_RESPONSE_NOT_CONNECTED_STATUS)) {
         on_successfully_receive_general_actions();

         if (is_usart_response_contains_element(DEFAULT_ACCESS_POINT_NAME)) {
            // Has already been connected
            set_flag(&general_flags_g, SUCCESSUFULLY_CONNECTED_TO_NETWORK_FLAG);
         } else if (is_usart_response_contains_element(ESP8226_RESPONSE_NOT_CONNECTED_STATUS)) {
            reset_flag(&general_flags_g, SUCCESSUFULLY_CONNECTED_TO_NETWORK_FLAG);
            // Connect
            add_piped_task_to_send_into_head(CONNECT_TO_NETWORK_TASK);
         }
      } else {
         reset_flag(&general_flags_g, SUCCESSUFULLY_CONNECTED_TO_NETWORK_FLAG);
         add_error();
      }
   }
   return not_handled;
}

unsigned char handle_get_connection_status_task(unsigned int current_piped_task_to_send, unsigned int *sent_flag) {
   unsigned char not_handled = 1;

   if (current_piped_task_to_send == GET_CONNECTION_STATUS_TASK) {
      not_handled = 0;
      schedule_function_resending(get_ap_connection_status, 2, EXECUTE_FUNCTION_IMMEDIATELY);
   } else if (read_flag(sent_flag, GET_CONNECTION_STATUS_TASK)) {
      not_handled = 0;
      reset_flag(&sent_task_g, GET_CONNECTION_STATUS_TASK);

      if (is_usart_response_contains_element(DEFAULT_ACCESS_POINT_NAME) || is_usart_response_contains_element(ESP8226_RESPONSE_NOT_CONNECTED_STATUS)) {
         on_successfully_receive_general_actions();

         if (is_usart_response_contains_element(DEFAULT_ACCESS_POINT_NAME)) {
            // Has already been connected
            set_flag(&general_flags_g, SUCCESSUFULLY_CONNECTED_TO_NETWORK_FLAG);
         } else if (is_usart_response_contains_element(ESP8226_RESPONSE_NOT_CONNECTED_STATUS)) {
            reset_flag(&general_flags_g, SUCCESSUFULLY_CONNECTED_TO_NETWORK_FLAG);
         }
      } else {
         reset_flag(&general_flags_g, SUCCESSUFULLY_CONNECTED_TO_NETWORK_FLAG);
         add_error();
      }
   }
   return not_handled;
}

unsigned char handle_connect_to_network_task(unsigned int current_piped_task_to_send, unsigned int *sent_flag) {
   unsigned char not_handled = 1;

   if (current_piped_task_to_send == CONNECT_TO_NETWORK_TASK) {
      not_handled = 0;
      schedule_function_resending(connect_to_network, 10, EXECUTE_FUNCTION_IMMEDIATELY);
   } else if (read_flag(sent_flag, CONNECT_TO_NETWORK_TASK)) {
      not_handled = 0;
      reset_flag(&sent_task_g, CONNECT_TO_NETWORK_TASK);

      if (is_usart_response_contains_element(USART_OK)) {
         on_successfully_receive_general_actions();

         set_flag(&general_flags_g, SUCCESSUFULLY_CONNECTED_TO_NETWORK_FLAG);
      } else {
         add_error();
      }
   }
   return not_handled;
}

unsigned char handle_connect_to_server_task(unsigned int current_piped_task_to_send, unsigned int *sent_flag) {
   unsigned char not_handled = 1;

   if (current_piped_task_to_send == CONNECT_TO_SERVER_TASK) {
      not_handled = 0;
      schedule_function_resending(connect_to_server, 10, EXECUTE_FUNCTION_IMMEDIATELY);
   } else if (read_flag(sent_flag, CONNECT_TO_SERVER_TASK)) {
      not_handled = 0;
      reset_flag(&sent_task_g, CONNECT_TO_SERVER_TASK);

      char *data_to_be_contained[] = {ESP8226_RESPONSE_CONNECTED, USART_OK};
      if (is_usart_response_contains_elements(data_to_be_contained, 2) || is_usart_response_contains_element(ESP8226_RESPONSE_ALREADY_CONNECTED)) {
         on_successfully_receive_general_actions();
      } else {
         add_error();
      }
   }
   return not_handled;
}

unsigned char handle_set_bytes_to_send_in_request_task(unsigned int current_piped_task_to_send, unsigned int *sent_flag) {
   unsigned char not_handled = 1;

   if (current_piped_task_to_send == SET_BYTES_TO_SEND_IN_REQUEST_TASK) {
      not_handled = 0;
      schedule_function_resending(set_bytes_amount_to_send, 2, EXECUTE_FUNCTION_IMMEDIATELY);
   } else if (read_flag(sent_flag, SET_BYTES_TO_SEND_IN_REQUEST_TASK)) {
      not_handled = 0;
      reset_flag(&sent_task_g, SET_BYTES_TO_SEND_IN_REQUEST_TASK);

      if (is_usart_response_contains_element(ESP8226_RESPONSE_START_SENDING_READY)) {
         on_successfully_receive_general_actions();
      } else {
         //resend_usart_get_request(GET_REQUEST_SENT_AND_RESPONSE_RECEIVED_FLAG);
         add_error();
      }
   }
   return not_handled;
}

unsigned char handle_get_request_sent_and_response_received_task(unsigned int current_piped_task_to_send, unsigned int *sent_flag) {
   unsigned char not_handled = 1;

   if (current_piped_task_to_send == GET_REQUEST_SENT_AND_RESPONSE_RECEIVED_TASK) {
      not_handled = 0;
      schedule_global_function_resending_and_send_request(GET_REQUEST_SENT_AND_RESPONSE_RECEIVED_TASK, 10);
   } else if (read_flag(sent_flag, GET_REQUEST_SENT_AND_RESPONSE_RECEIVED_TASK)) {
      not_handled = 0;
      reset_flag(&sent_task_g, GET_REQUEST_SENT_AND_RESPONSE_RECEIVED_TASK);

      char *data_to_be_contained[] = {ESP8226_RESPONSE_SUCCSESSFULLY_SENT, ESP8226_RESPONSE_PREFIX};
      if (is_usart_response_contains_elements(data_to_be_contained, 2)) {
         on_successfully_receive_general_actions();
      } else {
         //resend_usart_get_request(GET_REQUEST_SENT_AND_RESPONSE_RECEIVED_FLAG);
         add_error();
      }
   }
   return not_handled;
}

unsigned char handle_get_current_default_wifi_mode_task(unsigned int current_piped_task_to_send, unsigned int *sent_flag) {
   unsigned char not_handled = 1;

   if (current_piped_task_to_send == GET_CURRENT_DEFAULT_WIFI_MODE_TASK) {
      not_handled = 0;
      schedule_function_resending(get_current_default_wifi_mode, 2, EXECUTE_FUNCTION_IMMEDIATELY);
   } else if (read_flag(sent_flag, GET_CURRENT_DEFAULT_WIFI_MODE_TASK)) {
      not_handled = 0;
      reset_flag(&sent_task_g, GET_CURRENT_DEFAULT_WIFI_MODE_TASK);

      if (is_usart_response_contains_element(ESP8226_RESPONSE_WIFI_MODE_PREFIX)) {
         on_successfully_receive_general_actions();

         if (!is_usart_response_contains_element(ESP8226_RESPONSE_WIFI_STATION_MODE)) {
            add_piped_task_to_send_into_head(SET_DEFAULT_STATION_WIFI_MODE_TASK);
         }
      } else {
         add_error();
      }
   }
   return not_handled;
}

unsigned char handle_set_default_station_wifi_mode_task(unsigned int current_piped_task_to_send, unsigned int *sent_flag) {
   unsigned char not_handled = 1;

   if (current_piped_task_to_send == SET_DEFAULT_STATION_WIFI_MODE_TASK) {
      not_handled = 0;
      schedule_function_resending(set_default_wifi_mode, 2, EXECUTE_FUNCTION_IMMEDIATELY);
   } else if (read_flag(sent_flag, SET_DEFAULT_STATION_WIFI_MODE_TASK)) {
      not_handled = 0;
      reset_flag(&sent_task_g, SET_DEFAULT_STATION_WIFI_MODE_TASK);

      if (is_usart_response_contains_element(USART_OK)) {
         on_successfully_receive_general_actions();
      } else {
         add_error();
      }
   }
   return not_handled;
}

unsigned char handle_get_own_ip_address_task(unsigned int current_piped_task_to_send, unsigned int *sent_flag) {
   unsigned char not_handled = 1;

   if (current_piped_task_to_send == GET_OWN_IP_ADDRESS_TASK) {
      not_handled = 0;
      schedule_function_resending(get_own_ip_address, 5, EXECUTE_FUNCTION_IMMEDIATELY);
   } else if (read_flag(sent_flag, GET_OWN_IP_ADDRESS_TASK)) {
      not_handled = 0;
      reset_flag(&sent_task_g, GET_OWN_IP_ADDRESS_TASK);

      if (is_usart_response_contains_element(ESP8226_OWN_IP_ADDRESS) || is_usart_response_contains_element(ESP8226_RESPONSE_CURRENT_OWN_IP_ADDRESS_PREFIX)) {
         on_successfully_receive_general_actions();

         unsigned char some_another_ip = !is_usart_response_contains_element(ESP8226_OWN_IP_ADDRESS);
         if (some_another_ip) {
            add_piped_task_to_send_into_head(SET_OWN_IP_ADDRESS_TASK);
         }
      } else {
         add_error();
      }
   }
   return not_handled;
}

unsigned char handle_set_own_ip_address_task(unsigned int current_piped_task_to_send, unsigned int *sent_flag) {
   unsigned char not_handled = 1;

   if (current_piped_task_to_send == SET_OWN_IP_ADDRESS_TASK) {
      not_handled = 0;
      schedule_function_resending(set_own_ip_address, 2, EXECUTE_FUNCTION_IMMEDIATELY);
   } else if (read_flag(sent_flag, SET_OWN_IP_ADDRESS_TASK)) {
      not_handled = 0;
      reset_flag(&sent_task_g, SET_OWN_IP_ADDRESS_TASK);

      if (is_usart_response_contains_element(USART_OK)) {
         on_successfully_receive_general_actions();
      } else {
         add_error();
      }
   }
   return not_handled;
}

unsigned char handle_close_connection_task(unsigned int current_piped_task_to_send, unsigned int *sent_flag) {
   unsigned char not_handled = 1;

   if (current_piped_task_to_send == CLOSE_CONNECTION_TASK) {
      not_handled = 0;
      schedule_function_resending(close_connection, 20, EXECUTE_FUNCTION_IMMEDIATELY);
   } else if (read_flag(sent_flag, CLOSE_CONNECTION_TASK)) {
      not_handled = 0;
      reset_flag(&sent_task_g, CLOSE_CONNECTION_TASK);
      on_successfully_receive_general_actions();
   }
   return not_handled;
}

unsigned char handle_get_visible_network_list_task(unsigned int current_piped_task_to_send, unsigned int *sent_task) {
   unsigned char not_handled = 1;

   if (current_piped_task_to_send == GET_VISIBLE_NETWORK_LIST_TASK) {
      not_handled = 0;
      schedule_function_resending(get_network_list, 20, EXECUTE_FUNCTION_IMMEDIATELY);
   } else if (read_flag(sent_task, GET_VISIBLE_NETWORK_LIST_TASK)) {
      not_handled = 0;
      reset_flag(&sent_task_g, GET_VISIBLE_NETWORK_LIST_TASK);

      if (is_usart_response_contains_element(ESP8226_RESPONSE_VISIBLE_NETWORK_LIST_PREFIX)) {
         on_successfully_receive_general_actions();
         save_default_access_point_gain();
      } else {
         add_error();
      }
   }
   return not_handled;
}

unsigned char handle_establish_long_polling_connection_task(unsigned int current_piped_task_to_send) {
   unsigned char not_handled = 1;

   if (current_piped_task_to_send == ESTABLISH_LONG_POLLING_CONNECTION_TASK) {
      not_handled = 0;
      // Request 1 part. Preparation
      delete_piped_task(current_piped_task_to_send);
      establish_long_polling_connection(ESTABLISH_LONG_POLLING_CONNECTION_REQUEST_TASK);
   }
   return not_handled;
}

unsigned char handle_establish_long_polling_connection_request_task(unsigned int current_piped_task_to_send, unsigned int *sent_task) {
   unsigned char not_handled = 1;

   if (current_piped_task_to_send == ESTABLISH_LONG_POLLING_CONNECTION_REQUEST_TASK) {
      not_handled = 0;
      // Part 2
      schedule_global_function_resending_and_send_request(ESTABLISH_LONG_POLLING_CONNECTION_REQUEST_TASK, 330); // 5.5 minutes
   } else if (read_flag(sent_task, ESTABLISH_LONG_POLLING_CONNECTION_REQUEST_TASK)) {
      not_handled = 0;

      if ((is_usart_response_contains_element(ESP8226_RESPONSE_HTTP_STATUS_200_OK) || is_usart_response_contains_element(ESP8226_RESPONSE_SUCCSESSFULLY_SENT)) &&
            !is_usart_response_contains_element(ESP8226_RESPONSE_OK_STATUS_CODE) && !is_usart_response_contains_element(RESPONSE_SERVICE_UNAVAILABLE)) {
         // Sometimes only "SEND OK" is received. Another data will be received later
         clear_usart_data_received_buffer();
      } else {
         reset_flag(&sent_task_g, ESTABLISH_LONG_POLLING_CONNECTION_REQUEST_TASK);

         if (is_usart_response_contains_element(ESP8226_RESPONSE_OK_STATUS_CODE)) {
            on_successfully_receive_general_actions();

            if (is_usart_response_contains_element(SERVER_STATUS_INCLUDE_DEBUG_INFO)) {
               set_flag(&general_flags_g, SEND_DEBUG_INFO_FLAG);
            } else {
               reset_flag(&general_flags_g, SEND_DEBUG_INFO_FLAG);
            }
            if (is_usart_response_contains_element(TURN_ON_TRUE_JSON_ELEMENT)) {
               set_flag(&general_flags_g, TURN_PROJECTOR_ON);
            } else {
               reset_flag(&general_flags_g, TURN_PROJECTOR_ON);
            }

            /*char *timestamp = get_gson_element_value(usart_data_received_buffer_g, TIMESTAMP_JSON_ELEMENT);
            if (timestamp != NULL) {
               response_timestamp_ms_g = atoi(timestamp);
               free(timestamp);
            }*/

            set_flag(&general_flags_g, SERVER_IS_AVAILABLE_FLAG);
            add_piped_task_to_send_into_tail(ESTABLISH_LONG_POLLING_CONNECTION_TASK);
         } else {
            send_usart_data_timout_sec_g = 15; // Reset long timeout of long polling request

            reset_flag(&general_flags_g, SERVER_IS_AVAILABLE_FLAG);
            add_error();
         }
      }
   }
   return not_handled;
}

void reset_device_state() {
   resets_occured_g++;
   delete_all_piped_tasks();
   clear_piped_request_commands_to_send();
   clear_usart_data_received_buffer();
   on_response_g = NULL;
   send_usart_data_function_g = NULL;

   if (received_usart_error_data_g != NULL) {
      free(received_usart_error_data_g);
      received_usart_error_data_g = NULL;
   }

   general_flags_g = 0;
   sent_task_g = 0;
   send_usart_data_errors_counter_g = 0;

   add_piped_task_to_send_into_tail(GET_AP_CONNECTION_STATUS_AND_CONNECT_TASK);
   add_piped_task_to_send_into_tail(ESTABLISH_LONG_POLLING_CONNECTION_TASK);
}

void check_connection_status_and_server_availability() {
   if (checking_connection_status_and_server_availability_timer_g == 0 && is_piped_tasks_scheduler_empty()) {
      checking_connection_status_and_server_availability_timer_g = TIMER14_30S;
      add_piped_task_to_send_into_tail(GET_CONNECTION_STATUS_TASK);
      add_piped_task_to_send_into_tail(GET_SERVER_AVAILABILITY_TASK);
   }
}

void check_visible_network_list() {
   if (!visible_network_list_timer_g && is_piped_tasks_scheduler_empty()) {
      visible_network_list_timer_g = TIMER14_10MIN;
      add_piped_task_to_send_into_tail(GET_VISIBLE_NETWORK_LIST_TASK);
   }
}

/**
 * @param timeout timeout in seconds
 */
void schedule_global_function_resending_and_send_request(unsigned int task, unsigned short timeout) {
   final_task_for_request_resending_g = task;
   schedule_function_resending(resend_usart_get_request_using_global_final_task, timeout, DO_NOT_EXECUTE_FUNCTION_IMMEDIATELY);

   send_request(task);
}

void add_error() {
   send_usart_data_errors_counter_g++;
   send_usart_data_errors_unresetable_counter_g++;
   last_error_task_g = get_current_piped_task_to_send();

   if (received_usart_error_data_g != NULL) {
      free(received_usart_error_data_g);
      received_usart_error_data_g = NULL;
   }
   received_usart_error_data_g = get_received_usart_error_data();
}

void establish_long_polling_connection(unsigned int request_task) {
   char *request = generate_request(ESP8226_REQUEST_SEND_STATUS_INFO_AND_ESTABLISH_LONG_POLLING_REQUEST);

   prepare_http_request(ESP8226_SERVER_IP_ADDRESS, ESP8226_SERVER_PORT, request, NULL, request_task);
}

char *generate_request(char *request_template) {
   char *gain = array_to_string(default_access_point_gain_g, DEFAULT_ACCESS_POINT_GAIN_SIZE);
   //char *response_timestamp = num_to_string(calculate_response_timestamp());
   char *debug_info_included = read_flag(&general_flags_g, SEND_DEBUG_INFO_FLAG) ? "true" : "false";
   char *status_json;

   if (read_flag(&general_flags_g, SEND_DEBUG_INFO_FLAG)) {
      status_json = add_debug_info(gain, debug_info_included, "-1");
   } else {
      char *parameters_for_status[] = {gain, debug_info_included, "-1", NULL};
      status_json = set_string_parameters(STATUS_JSON, parameters_for_status);
   }
   free(gain);

   if (received_usart_error_data_g != NULL) {
      free(received_usart_error_data_g);
      received_usart_error_data_g = NULL;
   }

   unsigned short status_string_length = get_string_length(status_json);
   char *status_string_length_string = num_to_string(status_string_length);
   char *parameters_for_request[] = {status_string_length_string, ESP8226_SERVER_IP_ADDRESS, status_json, NULL};
   char *request = set_string_parameters(request_template, parameters_for_request);

   free(status_json);
   free(status_string_length_string);
   return request;
}

void *add_debug_info(char *gain, char *debug_info_included, char *response_timestamp) {
   char *errors_amount_string = num_to_string(send_usart_data_errors_unresetable_counter_g);
   char *usart_overrun_errors_counter_string = num_to_string(usart_overrun_errors_counter_g);
   char *usart_idle_line_detection_counter_string = num_to_string(usart_idle_line_detection_counter_g);
   char *usart_noise_detection_counter_string = num_to_string(usart_noise_detection_counter_g);
   char *usart_framing_errors_counter_string = num_to_string(usart_framing_errors_counter_g);
   char *last_error_task_string = num_to_string(last_error_task_g);
   char *received_usart_error_data = last_error_task_g && received_usart_error_data_g != NULL ? received_usart_error_data_g : "";
   char *parameters_for_status[] = {gain, debug_info_included, errors_amount_string, usart_overrun_errors_counter_string,
         usart_idle_line_detection_counter_string, usart_noise_detection_counter_string, usart_framing_errors_counter_string,
         last_error_task_string, received_usart_error_data, response_timestamp, NULL};
   char *status_json = set_string_parameters(DEBUG_STATUS_JSON, parameters_for_status);

   free(errors_amount_string);
   free(usart_overrun_errors_counter_string);
   free(usart_idle_line_detection_counter_string);
   free(usart_noise_detection_counter_string);
   free(usart_framing_errors_counter_string);
   free(last_error_task_string);

   last_error_task_g = 0;
   return status_json;
}

void get_own_ip_address() {
   send_usard_data(ESP8226_REQUEST_GET_OWN_IP_ADDRESS);
   set_flag(&sent_task_g, GET_OWN_IP_ADDRESS_TASK);
}

void set_own_ip_address() {
   char *parameters[] = {ESP8226_OWN_IP_ADDRESS, NULL};
   usart_data_to_be_transmitted_buffer_g = set_string_parameters(ESP8226_REQUEST_SET_OWN_IP_ADDRESS, parameters);
   send_usard_data(usart_data_to_be_transmitted_buffer_g);
   set_flag(&sent_task_g, SET_OWN_IP_ADDRESS_TASK);
}

void close_connection() {
   send_usard_data(ESP8226_REQUEST_DISCONNECT_FROM_SERVER);
   set_flag(&sent_task_g, CLOSE_CONNECTION_TASK);
}

void get_current_default_wifi_mode() {
   send_usard_data(ESP8226_REQUEST_GET_CURRENT_DEFAULT_WIFI_MODE);
   set_flag(&sent_task_g, GET_CURRENT_DEFAULT_WIFI_MODE_TASK);
}

void set_default_wifi_mode() {
   send_usard_data(ESP8226_REQUEST_SET_DEFAULT_STATION_WIFI_MODE);
   set_flag(&sent_task_g, SET_DEFAULT_STATION_WIFI_MODE_TASK);
}

void prepare_http_request_without_parameters(char request_template[], unsigned int request_task) {
   char *parameters_for_request[] = {ESP8226_SERVER_IP_ADDRESS, NULL};
   char *request = set_string_parameters(request_template, parameters_for_request);

   prepare_http_request(ESP8226_SERVER_IP_ADDRESS, ESP8226_SERVER_PORT, request, NULL, request_task);
}

/**
 * "request" shall be allocated with "malloc" function. Later it will be removed with "free" function
 */
void prepare_http_request(char address[], char port[], char request[], void (*execute_on_response)(), unsigned int request_task) {
   clear_piped_request_commands_to_send();
   send_usart_data_function_g = NULL;

   char *parameters[] = {address, port, NULL};
   piped_request_commands_to_send_g[PIPED_REQUEST_CIPSTART_COMMAND_INDEX] = set_string_parameters(ESP8226_REQUEST_CONNECT_TO_SERVER, parameters);

   unsigned short request_length = get_string_length(request);
   char *request_length_string = num_to_string(request_length);
   char *start_sending_parameters[] = {request_length_string, NULL};
   piped_request_commands_to_send_g[PIPED_REQUEST_CIPSEND_COMMAND_INDEX] = set_string_parameters(ESP8226_REQUEST_START_SENDING, start_sending_parameters);
   free(request_length_string);

   piped_request_commands_to_send_g[PIPED_REQUEST_INDEX] = request;

   on_response_g = execute_on_response;

   add_piped_task_to_send_into_tail(CONNECT_TO_SERVER_TASK);
   add_piped_task_to_send_into_tail(SET_BYTES_TO_SEND_IN_REQUEST_TASK);
   add_piped_task_to_send_into_tail(request_task);
}

void resend_usart_get_request_using_global_final_task() {
   resend_usart_get_request(final_task_for_request_resending_g);
}

void resend_usart_get_request(unsigned int final_task) {
   send_usart_data_function_g = NULL;
   delete_piped_task(SET_BYTES_TO_SEND_IN_REQUEST_TASK);
   //delete_piped_task(CLOSE_CONNECTION_FLAG);
   delete_piped_task(final_task);

   //add_piped_task_to_send_into_tail(CLOSE_CONNECTION_FLAG);
   add_piped_task_to_send_into_tail(CONNECT_TO_SERVER_TASK);
   add_piped_task_to_send_into_tail(SET_BYTES_TO_SEND_IN_REQUEST_TASK);
   add_piped_task_to_send_into_tail(final_task);
}

void clear_piped_request_commands_to_send() {
   for (unsigned char i = 0; i < PIPED_REQUEST_COMMANDS_TO_SEND_SIZE; i++) {
      char *command = piped_request_commands_to_send_g[i];
      if (command != NULL) {
         free(command);
         piped_request_commands_to_send_g[i] = NULL;
      }
   }
}

void connect_to_server() {
   if (piped_request_commands_to_send_g[PIPED_REQUEST_CIPSTART_COMMAND_INDEX] == NULL) {
      return;
   }

   send_usard_data(piped_request_commands_to_send_g[PIPED_REQUEST_CIPSTART_COMMAND_INDEX]);
   set_flag(&sent_task_g, CONNECT_TO_SERVER_TASK);
}

void set_bytes_amount_to_send() {
   if (piped_request_commands_to_send_g[PIPED_REQUEST_CIPSEND_COMMAND_INDEX] == NULL) {
      return;
   }

   send_usard_data(piped_request_commands_to_send_g[PIPED_REQUEST_CIPSEND_COMMAND_INDEX]);
   set_flag(&sent_task_g, SET_BYTES_TO_SEND_IN_REQUEST_TASK);
}

void send_request(unsigned int sent_task_to_set) {
   if (piped_request_commands_to_send_g[PIPED_REQUEST_INDEX] == NULL) {
      return;
   }

   send_usard_data(piped_request_commands_to_send_g[PIPED_REQUEST_INDEX]);
   set_flag(&sent_task_g, sent_task_to_set);
}

void on_successfully_receive_general_actions() {
   send_usart_data_function_g = NULL;
   send_usart_data_errors_counter_g = 0;
   delete_current_piped_task();
}

// +CWLAP:("Asus",-74,...)
void save_default_access_point_gain() {
   if (!is_usart_response_contains_element(DEFAULT_ACCESS_POINT_NAME)) {
      return;
   }

   unsigned char first_comma_is_found = 0;
   char *access_point_starting_position = strstr(usart_data_received_buffer_g, DEFAULT_ACCESS_POINT_NAME);

   if (access_point_starting_position == NULL) {
      for (unsigned char i = 0; i < DEFAULT_ACCESS_POINT_GAIN_SIZE; i++) {
         default_access_point_gain_g[i] = ' ';
      }
   }

   while (*access_point_starting_position != '\0') {
      if (first_comma_is_found && *access_point_starting_position == ',') {
         access_point_starting_position--;
         break;
      }

      if (*access_point_starting_position == ',') {
         first_comma_is_found = 1;
      }
      access_point_starting_position++;
   }

   for (unsigned char i = DEFAULT_ACCESS_POINT_GAIN_SIZE - 1; i != 0xFF; i--) {
      if (*access_point_starting_position == ',') {
         for (unsigned char i2 = i; i2 != 0xFF; i2--) {
            default_access_point_gain_g[i2] = ' ';
         }
         break;
      }

      default_access_point_gain_g[i] = *access_point_starting_position;
      access_point_starting_position--;
   }
}

unsigned int get_current_piped_task_to_send() {
   return piped_tasks_to_send_g[0];
}

void delete_current_piped_task() {
   for (unsigned char i = 0; piped_tasks_to_send_g[i] != 0; i++) {
      unsigned int next_task = piped_tasks_to_send_g[i + 1];
      piped_tasks_to_send_g[i] = next_task;
   }
}

void add_piped_task_to_send_into_tail(unsigned int task) {
   for (unsigned char i = 0; i < PIPED_TASKS_TO_SEND_SIZE; i++) {
      if (piped_tasks_to_send_g[i] == 0) {
         piped_tasks_to_send_g[i] = task;
         break;
      }
   }
}

void add_piped_task_to_send_into_head(unsigned int task) {
   for (unsigned char i = PIPED_TASKS_TO_SEND_SIZE - 1; i != 0; i--) {
      piped_tasks_to_send_g[i] = piped_tasks_to_send_g[i - 1];
   }
   piped_tasks_to_send_g[0] = task;
}

void delete_piped_task(unsigned int task) {
   unsigned char task_is_found = 0;
   for (unsigned char i = 0; i < PIPED_TASKS_TO_SEND_SIZE; i++) {
      if (piped_tasks_to_send_g[i] == task || task_is_found) {
         piped_tasks_to_send_g[i] = piped_tasks_to_send_g[i + 1];
         task_is_found = 1;
      }

      if (piped_tasks_to_send_g[i] == 0) {
         break;
      }
   }
}

void delete_all_piped_tasks() {
   for (unsigned char i = 0; i < PIPED_TASKS_TO_SEND_SIZE; i++) {
     piped_tasks_to_send_g[i] = 0;
   }
}

unsigned char is_piped_task_to_send_scheduled(unsigned int task) {
   for (unsigned char i = 0; i < PIPED_TASKS_TO_SEND_SIZE; i++) {
     if (piped_tasks_to_send_g[i] == task) {
        return 1;
     }
   }
   return 0;
}

unsigned char is_piped_tasks_scheduler_full() {
   return piped_tasks_to_send_g[PIPED_TASKS_TO_SEND_SIZE - 2] != 0 ? 1 : 0;
}

unsigned char is_piped_tasks_scheduler_empty() {
   return piped_tasks_to_send_g[0] == 0 ? 1 : 0;
}

void add_piped_task_into_history(unsigned int task) {
   if (task == 0) {
      return;
   }

   unsigned char i;
   unsigned int last_task = piped_tasks_history_g[PIPED_TASKS_HISTORY_SIZE - 1];
   for (i = 0; i < PIPED_TASKS_HISTORY_SIZE - 1; i++) {
      if (piped_tasks_history_g[i] == 0) {
         break;
      }

      if (last_task) {
         // The last task already exists
         unsigned int next_task = piped_tasks_history_g[i + 1];
         piped_tasks_history_g[i] = next_task;
      }
   }
   piped_tasks_history_g[i] = task;
}

unsigned int get_last_piped_task_in_history() {
   for (unsigned char i = PIPED_TASKS_HISTORY_SIZE - 1; i != 0xFF; i--) {
      if (piped_tasks_history_g[i] != 0) {
         return piped_tasks_history_g[i];
      }
   }
   return piped_tasks_history_g[0];
}

void *get_received_usart_error_data() {
   unsigned char received_data_length = 0;

   while (received_data_length < RECEIVED_USART_DATA_FOR_DEBUG_INFO_MAX_LENGTH &&
         usart_data_received_buffer_g[received_data_length] != '\0') {
      received_data_length++;
   }

   // 1 is for '\0' as the last character
   char *result_string = malloc(received_data_length + 1);

   for (unsigned char i = 0; i < received_data_length; i++) {
      char received_char = usart_data_received_buffer_g[i];

      if (received_char <= '\"') {
         if (received_char == '\r') {
            received_char = 'r';
         } else if (received_char == '\n') {
            received_char = 'n';
         } else if (received_char == '\"') {
            received_char = '\'';
         } else {
            received_char += 65; // Starts from 'A'
         }
      }
      *(result_string + i) = received_char;
   }
   *(result_string + received_data_length) = '\0';
   return result_string;
}

unsigned char is_usart_response_contains_element(char string_to_be_contained[]) {
   if (contains_string(usart_data_received_buffer_g, string_to_be_contained)) {
      return 1;
   } else {
      return 0;
   }
}

//char *data_to_be_contained[] = {ESP8226_REQUEST_DISABLE_ECHO, USART_OK};
unsigned char is_usart_response_contains_elements(char *data_to_be_contained[], unsigned char elements_count) {
   for (unsigned char elements_index = 0; elements_index < elements_count; elements_index++) {
      if (!contains_string(usart_data_received_buffer_g, data_to_be_contained[elements_index])) {
         return 0;
      }
   }
   return 1;
}

unsigned char contains_string(char being_compared_string[], char string_to_be_contained[]) {
   unsigned char found = 0;

   if (*being_compared_string == '\0' || *string_to_be_contained == '\0') {
      return found;
   }

   for (; *being_compared_string != '\0'; being_compared_string++) {
      unsigned char all_chars_are_equal = 1;

      for (char *char_address = string_to_be_contained; *char_address != '\0';
            char_address++, being_compared_string++) {
         if (*being_compared_string == '\0') {
            return found;
         }

         all_chars_are_equal = *being_compared_string == *char_address ? 1 : 0;

         if (!all_chars_are_equal) {
            break;
         }
      }

      if (all_chars_are_equal) {
         found = 1;
         break;
      }
   }
   return found;
}

void disable_echo() {
   send_usard_data(ESP8226_REQUEST_DISABLE_ECHO);
   set_flag(&sent_task_g, DISABLE_ECHO_TASK);
}

void get_network_list() {
   send_usard_data(ESP8226_REQUEST_GET_VISIBLE_NETWORK_LIST);
   set_flag(&sent_task_g, GET_VISIBLE_NETWORK_LIST_TASK);
}

void get_ap_connection_status() {
   send_usard_data(ESP8226_REQUEST_GET_AP_CONNECTION_STATUS);
   set_flag(&sent_task_g, GET_AP_CONNECTION_STATUS_AND_CONNECT_TASK);
}

void connect_to_network() {
   char *parameters[] = {DEFAULT_ACCESS_POINT_NAME, DEFAULT_ACCESS_POINT_PASSWORD, NULL};
   usart_data_to_be_transmitted_buffer_g = set_string_parameters(ESP8226_REQUEST_CONNECT_TO_NETWORK_AND_SAVE, parameters);
   send_usard_data(usart_data_to_be_transmitted_buffer_g);
   set_flag(&sent_task_g, CONNECT_TO_NETWORK_TASK);
}

/**
 * @param timeout timeout in seconds
 */
void schedule_function_resending(void (*function_to_execute)(), unsigned short timeout, ImmediatelyFunctionExecution execute) {
   send_usart_data_timout_sec_g = timeout;
   send_usart_data_function_g = function_to_execute;

   if (execute == EXECUTE_FUNCTION_IMMEDIATELY) {
      function_to_execute();
   }
}

void IWDG_Config() {
   DBGMCU_APB1PeriphConfig(DBGMCU_IWDG_STOP, ENABLE);

   IWDG_Enable();
   IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
   IWDG_SetPrescaler(IWDG_Prescaler_256);
   IWDG_SetReload(156); // 1 second
   while (IWDG_GetFlagStatus(IWDG_FLAG_PVU) == SET);
   while (IWDG_GetFlagStatus(IWDG_FLAG_RVU) == SET);
}

void Clock_Config() {
   RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
   RCC_PLLCmd(DISABLE);
   while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == SET);
   RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_4); // 8MHz / 2 * 4
   RCC_PCLKConfig(RCC_HCLK_Div1);
   RCC_PLLCmd(ENABLE);
   while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
   RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
}

void Pins_Config() {
   // Connect BOOT0 directly to ground, RESET to VDD with a resistor

   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);

   GPIO_InitTypeDef gpioInitType;
   gpioInitType.GPIO_Pin = GPIO_Pin_All & ~(GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_9 | GPIO_Pin_10); // PA13, PA14 - Debugger pins
   gpioInitType.GPIO_Mode = GPIO_Mode_IN;
   gpioInitType.GPIO_Speed = GPIO_Speed_Level_1; // 2 MHz
   gpioInitType.GPIO_PuPd = GPIO_PuPd_UP;
   GPIO_Init(GPIOA, &gpioInitType);

   // For USART1
   gpioInitType.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
   gpioInitType.GPIO_PuPd = GPIO_PuPd_NOPULL;
   gpioInitType.GPIO_Mode = GPIO_Mode_AF;
   gpioInitType.GPIO_OType = GPIO_OType_OD;
   GPIO_Init(GPIOA, &gpioInitType);
   GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
   GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);

   gpioInitType.GPIO_Pin = GPIO_Pin_All;
   gpioInitType.GPIO_PuPd = GPIO_PuPd_UP;
   gpioInitType.GPIO_Mode = GPIO_Mode_IN;
   GPIO_Init(GPIOB, &gpioInitType);

   // PA1 LED
   gpioInitType.GPIO_Pin = NETWORK_STATUS_LED_PIN;
   gpioInitType.GPIO_Mode = GPIO_Mode_OUT;
   gpioInitType.GPIO_Speed = GPIO_Speed_Level_1;
   gpioInitType.GPIO_PuPd = GPIO_PuPd_DOWN;
   gpioInitType.GPIO_OType = GPIO_OType_PP;
   GPIO_Init(NETWORK_STATUS_LED_PORT, &gpioInitType);

   // PA2 LED
   gpioInitType.GPIO_Pin = SERVER_AVAILABILITI_LED_PIN;
   GPIO_Init(SERVER_AVAILABILITI_LED_PORT, &gpioInitType);

   // ESP8266 enable/disable
   gpioInitType.GPIO_Pin = ESP8266_CONTROL_PIN;
   GPIO_Init(ESP8266_CONTROL_PORT, &gpioInitType);

   gpioInitType.GPIO_Pin = PROJECTOR_RELAY_PIN;
   GPIO_Init(PROJECTOR_RELAY_PORT, &gpioInitType);
}

/**
 * USART frame time Tfr = (1 / USART_BAUD_RATE) * 10bits
 * Timer time to be sure the frame is ended Tt = Tfr + 0.5 * Tfr
 * Frequency = 16Mhz, USART_BAUD_RATE = 115200. Tt = 0.13ms
 */
void TIMER3_Confing() {
   DBGMCU_APB1PeriphConfig(DBGMCU_TIM3_STOP, ENABLE);
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

   TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
   TIM_TimeBaseStructure.TIM_Period = TIMER3_PERIOD_TICKS;
   TIM_TimeBaseStructure.TIM_Prescaler = 0;
   TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

   NVIC_EnableIRQ(TIM3_IRQn);
   TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

   TIM_Cmd(TIM3, ENABLE);
}

/**
 * 0.0983s with 16MHz clock
 */
void TIMER14_Confing() {
   DBGMCU_APB1PeriphConfig(DBGMCU_TIM14_STOP, ENABLE);
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

   TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
   TIM_TimeBaseStructure.TIM_Period = TIMER14_PERIOD;
   TIM_TimeBaseStructure.TIM_Prescaler = TIMER14_PRESCALER;
   TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);

   TIM_ClearITPendingBit(TIM14, TIM_IT_Update);
   NVIC_EnableIRQ(TIM14_IRQn);
   TIM_ITConfig(TIM14, TIM_IT_Update, ENABLE);

   TIM_Cmd(TIM14, ENABLE);
}

void DMA_Config() {
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);

   DMA_InitTypeDef dmaInitType;
   dmaInitType.DMA_PeripheralBaseAddr = USART1_TDR_ADDRESS;
   //dmaInitType.DMA_MemoryBaseAddr = (uint32_t)(&usartDataToBeTransmitted);
   dmaInitType.DMA_DIR = DMA_DIR_PeripheralDST; // Specifies if the peripheral is the source or destination
   dmaInitType.DMA_BufferSize = 0;
   dmaInitType.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
   dmaInitType.DMA_MemoryInc = DMA_MemoryInc_Enable; // DMA_MemoryInc_Enable if DMA_InitTypeDef.DMA_BufferSize > 1
   dmaInitType.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
   dmaInitType.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
   dmaInitType.DMA_Mode = DMA_Mode_Normal;
   dmaInitType.DMA_Priority = DMA_Priority_High;
   dmaInitType.DMA_M2M = DMA_M2M_Disable;
   DMA_Init(USART1_TX_DMA_CHANNEL, &dmaInitType);

   DMA_ITConfig(USART1_TX_DMA_CHANNEL, DMA_IT_TC, ENABLE);

   NVIC_SetPriority(USART1_IRQn, 10);
   NVIC_EnableIRQ(USART1_IRQn);

   DMA_Cmd(USART1_TX_DMA_CHANNEL, ENABLE);
}

void USART_Config() {
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

   USART_OverSampling8Cmd(USART1, DISABLE);

   USART_InitTypeDef USART_InitStructure;
   USART_InitStructure.USART_BaudRate = USART_BAUD_RATE;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
   USART_Init(USART1, &USART_InitStructure);

   USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
   USART_ITConfig(USART1, USART_IT_ERR, ENABLE);

   NVIC_SetPriority(DMA1_Channel2_3_IRQn, 11);
   NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

   USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

   USART_Cmd(USART1, ENABLE);
}

void set_flag(unsigned int *flags, unsigned int flag_value) {
   *flags |= flag_value;
}

void reset_flag(unsigned int *flags, unsigned int flag_value) {
   *flags &= ~(*flags & flag_value);
}

unsigned char read_flag(unsigned int *flags, unsigned int flag_value) {
   return (*flags & flag_value) > 0 ? 1 : 0;
}

void send_usard_data(char *string) {
   send_usart_data_time_counter_g = 0;
   clear_usart_data_received_buffer();
   DMA_Cmd(USART1_TX_DMA_CHANNEL, DISABLE);
   unsigned short bytes_to_send = get_string_length(string);

   if (bytes_to_send == 0) {
      return;
   }

   DMA_SetCurrDataCounter(USART1_TX_DMA_CHANNEL, bytes_to_send);
   USART1_TX_DMA_CHANNEL->CMAR = (unsigned int) string;
   USART_ClearFlag(USART1, USART_FLAG_TC);
   DMA_Cmd(USART1_TX_DMA_CHANNEL, ENABLE);
}

/**
 * Do not forget to call free() function on returned pointer when it's no longer needed
 *
 * *parameters - array of pointers to strings. The last parameter has to be NULL
 */
void *set_string_parameters(char string[], char *parameters[]) {
   unsigned char open_brace_found = 0;
   unsigned char parameters_amount = 0;
   unsigned short result_string_length = 0;

   for (; parameters[parameters_amount] != NULL; parameters_amount++) {
   }

   // Calculate the length without symbols to be replaced ('<x>')
   for (char *string_pointer = string; *string_pointer != '\0'; string_pointer++) {
      if (*string_pointer == '<') {
         if (open_brace_found) {
            return NULL;
         }
         open_brace_found = 1;
         continue;
      }
      if (*string_pointer == '>') {
         if (!open_brace_found) {
            return NULL;
         }
         open_brace_found = 0;
         continue;
      }
      if (open_brace_found) {
         continue;
      }

      result_string_length++;
   }

   if (open_brace_found) {
      return NULL;
   }

   for (unsigned char i = 0; parameters[i] != NULL; i++) {
      result_string_length += get_string_length(parameters[i]);
   }
   // 1 is for the last \0 character
   result_string_length++;

   char *allocated_result = malloc(result_string_length); // (string_length + 1) * sizeof(char)

   if (allocated_result == NULL) {
      return NULL;
   }

   unsigned short result_string_index = 0, input_string_index = 0;
   for (; result_string_index < result_string_length - 1; result_string_index++) {
      char input_string_char = string[input_string_index];

      if (input_string_char == '<') {
         input_string_index++;
         input_string_char = string[input_string_index];

         if (input_string_char < '1' || input_string_char > '9') {
            return NULL;
         }

         unsigned char parameter_numeric_value = input_string_char - '0';
         if (parameter_numeric_value > parameters_amount) {
            return NULL;
         }

         input_string_index++;
         input_string_char = string[input_string_index];

         if (input_string_char >= '0' && input_string_char <= '9') {
            // (parameter_numeric_value << 3) + (parameter_numeric_value << 1) == parameter_numeric_value * 10
            parameter_numeric_value = (parameter_numeric_value << 3) + (parameter_numeric_value << 1) + input_string_char - '0';
         }
         input_string_index++;

         // Parameters are starting with 1
         char *parameter = parameters[parameter_numeric_value - 1];

         for (; *parameter != '\0'; parameter++, result_string_index++) {
            *(allocated_result + result_string_index) = *parameter;
         }
         result_string_index--;
      } else {
         *(allocated_result + result_string_index) = string[input_string_index];
         input_string_index++;
      }
   }
   *(allocated_result + result_string_length - 1) = '\0';
   return allocated_result;
}

unsigned short get_string_length(char string[]) {
   unsigned short length = 0;

   for (char *string_pointer = string; *string_pointer != '\0'; string_pointer++, length++) {
   }
   return length;
}

unsigned char is_string_starts_with(char long_string[], char short_string[]) {
   unsigned char starts_with = 1;

   for (unsigned short i = 0; short_string[i] != '\0'; i++) {
      if (long_string[i] == '\0' || long_string[i] != short_string[i]) {
         starts_with = 0;
         break;
      }
   }
   return starts_with;
}

/**
 * Do not forget to call free() function on returned pointer when it's no longer needed
 */
void *num_to_string(unsigned int number) {
   char *result_string_pointer = NULL;

   if (number == 0) {
      result_string_pointer = malloc(2);
      result_string_pointer[0] = '0';
      result_string_pointer[1] = '\0';
      return result_string_pointer;
   }

   unsigned char string_size = 1;
   unsigned int divider = 1;
   unsigned int divider_tmp = 10;

   for (unsigned char i = 0; divider_tmp <= number; i++) {
      divider = divider_tmp;
      divider_tmp *= 10;
      string_size++;
   }

   unsigned int remaining = number;
   unsigned char string_length = 0;

   while (string_size > 0) {
      unsigned char result_character = get_first_digit(remaining);
      //unsigned char result_character = (unsigned char) (remaining / divider);

      if (result_string_pointer == NULL && result_character) {
         result_string_pointer = malloc(string_size + 1);
         string_length = string_size;
      }
      if (result_string_pointer != NULL) {
         unsigned char index = string_length - string_size;
         *(result_string_pointer + index) = result_character + '0';
      }

      remaining -= result_character * divider;
      divider = divide_by_10(divider);
      //divider /= 10;
      string_size--;
   }
   result_string_pointer[string_length] = '\0';
   return result_string_pointer;
}

/**
 * Only 10, 100, 1000, e.t.c can be divided by 10
 */
unsigned int divide_by_10(unsigned int dividend) {
   if (dividend < 10) {
      return 0;
   }

   unsigned int subtrahend = 0;
   unsigned int subtrahend_tmp = 9;

   while (subtrahend_tmp < dividend) {
      subtrahend = subtrahend_tmp;
      subtrahend_tmp *= 10;
   }
   return dividend - subtrahend;
}

unsigned char get_first_digit(unsigned int long_digit) {
   if (long_digit < 10) {
      return (unsigned char) long_digit;
   }

   // Find the most subtrahend
   unsigned int subtrahend = 0;
   unsigned int subtrahend_tmp = 10;

   while (subtrahend_tmp <= long_digit) {
      subtrahend = subtrahend_tmp;
      subtrahend_tmp *= 10;
   }

   unsigned char result = 1;
   unsigned int remaining_result = long_digit - subtrahend;

   while (remaining_result >= subtrahend) {
      result++;
      remaining_result -= subtrahend;
   }
   return result;
}

/**
 * Do not forget to call free() function on returned pointer when it's no longer needed
 */
void *array_to_string(char array[], unsigned char array_length) {
   char *result = malloc(array_length + 1);

   for (unsigned char i = 0; i < array_length; i++) {
      result[i] = array[i];
   }
   result[array_length] = '\0';
   return result;
}

char *get_gson_element_value(char *json_string, char *json_element_to_find) {
   char *json_element_to_find_in_string = strstr(json_string, json_element_to_find);
   unsigned int json_element_to_find_length = (unsigned int) strnlen(json_element_to_find, 50);

   if (json_element_to_find_in_string != NULL) {
      json_element_to_find_in_string += json_element_to_find_length;
      json_element_to_find_in_string++;
   } else {
      return NULL;
   }

   if (*json_element_to_find_in_string != ':') {
      return NULL;
   }

   json_element_to_find_in_string++;
   if (*json_element_to_find_in_string == '\"') {
      json_element_to_find_in_string++;
   }

   char *json_element_to_find_value = json_element_to_find_in_string;
   unsigned int returning_value_length = 0;

   while (!(*json_element_to_find_value == '\0' || *json_element_to_find_value == '\"' || *json_element_to_find_value == '}')) {
      returning_value_length++;
      json_element_to_find_value++;
   }
   json_element_to_find_value = json_element_to_find_value - returning_value_length;

   char *returning_value = malloc(returning_value_length + 1);

   for (unsigned int i = 0; i < returning_value_length; i++) {
      *(returning_value + i) = *(json_element_to_find_value + i);
   }
   *(returning_value + returning_value_length) = '\0';
   return returning_value;
}

void clear_usart_data_received_buffer() {
   for (unsigned short i = 0; i < USART_DATA_RECEIVED_BUFFER_SIZE; i++) {
      if (usart_data_received_buffer_g[i] == '\0') {
         break;
      }

      usart_data_received_buffer_g[i] = '\0';
   }
}

unsigned short get_received_data_length() {
   for (unsigned short i = 0; i < USART_DATA_RECEIVED_BUFFER_SIZE; i++) {
      if (usart_data_received_buffer_g[i] == '\0') {
         return i;
      }
   }
   return 0;
}

unsigned char is_received_data_length_equal(unsigned short length) {
   for (unsigned short i = 0; i < USART_DATA_RECEIVED_BUFFER_SIZE; i++) {
      if (i == length && usart_data_received_buffer_g[i] == '\0') {
         return 1;
      } else if ((i < length && usart_data_received_buffer_g[i] == '\0') ||
            i > length) {
         return 0;
      }
   }
   return 0;
}

void enable_esp8266() {
   GPIO_WriteBit(ESP8266_CONTROL_PORT, ESP8266_CONTROL_PIN, Bit_SET);
   esp8266_disabled_timer_g = TIMER14_5S;
}

void disable_esp8266() {
   GPIO_WriteBit(ESP8266_CONTROL_PORT, ESP8266_CONTROL_PIN, Bit_RESET);
   GPIO_WriteBit(NETWORK_STATUS_LED_PORT, NETWORK_STATUS_LED_PIN, Bit_RESET);
   GPIO_WriteBit(SERVER_AVAILABILITI_LED_PORT, SERVER_AVAILABILITI_LED_PIN, Bit_RESET);
}

unsigned char is_esp8266_enabled(unsigned char include_timer) {
   return include_timer ? (GPIO_ReadOutputDataBit(ESP8266_CONTROL_PORT, ESP8266_CONTROL_PIN) && esp8266_disabled_timer_g == 0) :
         GPIO_ReadOutputDataBit(ESP8266_CONTROL_PORT, ESP8266_CONTROL_PIN);
}
