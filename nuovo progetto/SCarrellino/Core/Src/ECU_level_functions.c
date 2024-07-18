#include "stdio.h"
#include "can.h" 
#include "adc.h"
#include "I2C_LCD.h"
#include "stdbool.h"
#include "fsm.h"
#include "scarrellino_fsm.h"
#include "string.h"
#include "main.h"
#include "tim.h"
#include "ECU_level_functions.h"
#include "usart.h"


extern  char ntc_temp_buffer[20];
extern volatile uint8_t ntc_temp;
extern volatile bool ADC_conv_flag;

char state_buff[30];
char state[15];
char buff[24];
extern FSM_HandleTypeDef hfsm;





//value of the timer
extern volatile uint16_t counter;

//value of the timer with sign
extern volatile int16_t count;

//real value of the position of the rotary encoder
extern volatile int16_t position; 
uint8_t old_position;

extern uint8_t number_of_positions;


/** @brief display pages manager function */
void display_routine(){


if(ADC_conv_flag == 1){

    encoder_position_adjustment();

    switch (position)
    {
    case 0:
        display_routine_0();
        break;

    case 1:
        display_routine_1();
        break;

    case 2:
        display_routine_2();
        break;

    default:
        display_routine_0();
        break;
    }
    


}
}


/** @brief display page: temperature, state of charge */
void display_routine_0(){

    double extern charge_temp, SOC;
    



    //temperature extern
    I2C_LCD_Home(I2C_LCD);
    sprintf(ntc_temp_buffer,"Ext Temp    : %d C ", ntc_temp);
    I2C_LCD_WriteString(I2C_LCD, (char*) &ntc_temp_buffer);
    ADC_conv_flag = 0;




    //State
    I2C_LCD_ACapo(I2C_LCD);
    switch (hfsm.current_state)
    {
    case 0:
        strcpy(state, "IDLE");
        break;

    case 1:
        strcpy(state, "CHARGE");
        break;
    case 2:
        strcpy(state, "STOP CHARGE");
    case 3:
        strcpy(state, "DONE");
    }


    sprintf(state_buff, "State : %s   ", state);
    I2C_LCD_WriteString(I2C_LCD,(char *) &state_buff);


    // state of charge
    I2C_LCD_ACapo(I2C_LCD);
    sprintf(ntc_temp_buffer,"Batt Charge : %.1lf%% ", (SOC*0.04 + 50));
    I2C_LCD_WriteString(I2C_LCD, (char*) &ntc_temp_buffer);


    //charging temperature
    if (hfsm.current_state == 1){
        I2C_LCD_ACapo(I2C_LCD);
        sprintf(ntc_temp_buffer,"Charge Temp : %.0lf C ", charge_temp);
        I2C_LCD_WriteString(I2C_LCD, (char*) &ntc_temp_buffer);
     
    }

    else{
        I2C_LCD_ACapo(I2C_LCD);
        sprintf(ntc_temp_buffer, "                   ");
        I2C_LCD_WriteString(I2C_LCD, (char*) &ntc_temp_buffer);
        }


/*
    I2C_LCD_ACapo(I2C_LCD);
    sprintf(state_buff, "position = %d  ", position);
    I2C_LCD_WriteString(I2C_LCD,(char *) &state_buff);
*/

    //set the last page displayed
    old_position = 0;

}



double extern v_max_id_rx;
double extern v_min_id_rx;
double extern v_max_rx;
double extern v_min_rx;
double extern v_mean_rx;



/** @brief display page 1: min e max */
void display_routine_1(){

    I2C_LCD_Home(I2C_LCD);

    sprintf(buff, "CELLS INFO");
    I2C_LCD_WriteString(I2C_LCD,(char *) &buff);
    I2C_LCD_ACapo(I2C_LCD);


    sprintf(buff, "v max  =%.2lf ID=%.0lf",v_max_rx, v_max_id_rx);
    I2C_LCD_WriteString(I2C_LCD,(char *) &buff);
    I2C_LCD_ACapo(I2C_LCD);


    sprintf(buff, "v min  =%.2lf ID=%.0lf",v_min_rx, v_min_id_rx );
    I2C_LCD_WriteString(I2C_LCD,(char *) &buff);
    I2C_LCD_ACapo(I2C_LCD);

    sprintf(buff,"v mean =%.2lf", v_mean_rx );
    I2C_LCD_WriteString(I2C_LCD,(char *) &buff);
    I2C_LCD_ACapo(I2C_LCD);



/*
    sprintf(state_buff, "position = %d  ", position);
    I2C_LCD_WriteString(I2C_LCD,(char *) &state_buff);
*/

    //set the last page displayed
    old_position = 1;

}


double extern I_out_rx, V_out_rx, mains_i_rx, mains_v_rx;


/** @brief display page 2  */
void display_routine_2(){

    I2C_LCD_Home(I2C_LCD);

    sprintf(buff, "CHARGE");
    I2C_LCD_WriteString(I2C_LCD,(char *) &buff);
    I2C_LCD_ACapo(I2C_LCD);

    sprintf(buff, "V = %.2lf I = %.2lf", V_out_rx, I_out_rx);
    I2C_LCD_WriteString(I2C_LCD,(char *) &buff);
    I2C_LCD_ACapo(I2C_LCD);

    sprintf(buff, "MAINS");
    I2C_LCD_WriteString(I2C_LCD,(char *) &buff);
    I2C_LCD_ACapo(I2C_LCD);

    sprintf(buff, "V = %.2lf I = %.2lf", mains_v_rx, mains_i_rx);
    I2C_LCD_WriteString(I2C_LCD,(char *) &buff);
    I2C_LCD_ACapo(I2C_LCD);

 /*   
    sprintf(state_buff, "position = %d  ", position);
    I2C_LCD_WriteString(I2C_LCD,(char *) &state_buff);
*/

    //set the last page displayed
    old_position = 2;

}






extern uint8_t volatile error_code;

/** @brief display errors  */
void error_display(){
     char error_buffer[40] = "";


//aggiungi errore sconosciuto
  #define display_error                \
          do{                         \
                        I2C_LCD_Clear(I2C_LCD); \
                        I2C_LCD_Home(I2C_LCD);  \
                        if(error_code != 30) sprintf(error_buffer, "error code: %d ", error_code); \
                        else sprintf(error_buffer, "error code: unknown "); \
                        I2C_LCD_WriteString(I2C_LCD, (char *)&error_buffer);  \
                        \
                        I2C_LCD_ACapo(I2C_LCD);                    \
                        sprintf(error_buffer, "CHARGE STOPPED "); \
                        I2C_LCD_WriteString(I2C_LCD, (char *)&error_buffer);  \
          } while (0)
          


  switch (error_code)
  {
  case 0:
    strcpy(error_buffer, "init fsm error");
    break;
  
  case 1:
    strcpy(error_buffer, "start fsm error");
    break;
    
  case 2:
    strcpy(error_buffer, "CAN start error");
        break;

  case 3:
    strcpy(error_buffer, "CAN IT activation error");
        break;

  case 4:
    strcpy(error_buffer, "CAN generic error");
    break;

  case 5:
    strcpy(error_buffer, "CAN Rx error");
      break;

  
    case 6:
    strcpy(error_buffer, "CAN 2 start error");
      break;

  
  case 7:
      strcpy(error_buffer, "watchdog error");
      break;

  case 8:
      strcpy(error_buffer, "can send error");
      break;

  }

  display_error;
  HAL_UART_Transmit(&LOG_UART, (uint8_t *) &error_buffer, strlen(error_buffer),10);
}




/** @brief adjust the encoder position if it goes out of the display pages */
void encoder_position_adjustment(){

if (position > number_of_positions) {
        __HAL_TIM_SET_COUNTER(&htim3, number_of_positions * 4 );
        position = number_of_positions;
        }

    else if (position < 0) {
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        position = 0;
    }

    if (position != old_position) I2C_LCD_Clear(I2C_LCD);
}


//#ifdef __CAN_H__


//funzione che trasmette dati via can
/*per usarla bisogna dargli il puntatore all'header del modulo can che volgiamo usare,
l'array col messaggio da inviare, l'ID che gli vogliamo assegnare e la variabile in cui verrà salvata la mailbox che verrà usata 
*/
/*
void txheader(CAN_TxHeaderTypeDef *TxHeader, uint8_t Txdata, uint8_t id){

    
    TxHeader->DLC = sizeof(Txdata);
    TxHeader->StdId = id;
    TxHeader->IDE = CAN_ID_STD;
    TxHeader->RTR = CAN_RTR_DATA;

}

void CAN_ECU_Tx(CAN_HandleTypeDef *hcan, uint8_t Txdata, uint8_t id, uint32_t TxMailbox ){


CAN_TxHeaderTypeDef TxHeader;

txheader(&TxHeader, Txdata, id);
    
    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, (uint8_t *)Txdata, &TxMailbox) != HAL_OK){
       
        Error_Handler();
    }}


#endif*/



uint32_t MilElapsed(bool reset){
   uint32_t static start;
 

   if(reset == 1) start = HAL_GetTick();
      uint32_t  mil = HAL_GetTick() - start;

   return mil;

}


void buzzer_routine(){

    extern bool buzzer_charge_on;
    extern bool buzzer_stop_charge_on;
    
    uint8_t static flag = 0;
    
    if(buzzer_charge_on == 1){

        if(flag == 0){

            MilElapsed(1);
            buzzer_800hz;

            #ifndef silence
            buzzer_on;
            #endif

            flag = 1;
        }

        if ((MilElapsed(0) >= 600) & (flag == 1)){
            buzzer_1khz;
            flag = 2;
            }

        if((MilElapsed(0) >= 1200) & (flag == 2)){
            buzzer_12khz;
            flag = 3;
        }

        if((MilElapsed(0) >= 1800) & (flag == 3)){
            buzzer_off;
            buzzer_charge_on = 0;
            flag = 0;
        }    
        
    }

    if(buzzer_stop_charge_on == 1){

        if(flag == 0){
            MilElapsed(1);
            buzzer_12khz;

            #ifndef silence
            buzzer_on;
            #endif
            
            flag = 1;
        }
        if ((MilElapsed(0) >= 600) & (flag == 1)){
            buzzer_1khz;
            flag = 2;
            }
        if((MilElapsed(0) >= 1200) & (flag == 2)){
            buzzer_800hz;
            flag = 3;
        }
        if((MilElapsed(0) >= 1800) & (flag == 3)){
            buzzer_off;
            buzzer_stop_charge_on = 0;
            flag = 0;
        }    
    }
}





void IMD_AMS_error_handler(){

    extern double imd_err_is_active, ams_err_is_active;                                                  
                                                                          
     if(imd_err_is_active == 1){  
       IMD_err_on;                                                        
       WarnLedOn;                                                         
       }                                                                  
                                                                          
     else{                                                                
       IMD_err_off;                                                       
       WarnLedOff;                                                        
     }                                                                    
                                                                          
                                                                          
     if(ams_err_is_active == 1)                                           
     {                                                                    
       AMS_err_on;                                                        
       WarnLedOn;                                                         
   }                                                                      
                                                                          
     else{                                                                
       AMS_err_off;                                                      
       WarnLedOff;                                                        
     }                                                                    
     }



void TSAC_FAN_Handler(){
    
    double extern charge_temp;

    if(charge_temp <= 29) TSAC_fan_off;

    if((charge_temp >= 31) & (charge_temp <= 39)) {
        TSAC_fan_half;
        }

    if(charge_temp >= 41) {
        TSAC_fan_max;
        } 
}