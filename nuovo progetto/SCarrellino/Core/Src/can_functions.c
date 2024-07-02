/*

CAN high level functions

*/

#include "main.h"
#include "can.h"
#include "mcb.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "hvcb.h"
#include "can_functions.h"
#include "interrupt.h"
#include "nlg5_database_can.h"

//wait for the CAN Tx to be ready
HAL_StatusTypeDef can_wait(CAN_HandleTypeDef *hcan, uint8_t timeout) {
    uint32_t tick = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) {
        if(HAL_GetTick() - tick > timeout) return HAL_TIMEOUT;
    }
    return HAL_OK;
    }


// send messages on the CAN bus
HAL_StatusTypeDef can_send(CAN_HandleTypeDef *hcan, uint8_t *buffer, CAN_TxHeaderTypeDef *header, uint32_t mailbox) {
    if(can_wait(hcan, 1) != HAL_OK) return HAL_TIMEOUT;

    volatile HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(hcan, header, buffer, &mailbox);

    return status;
}

/*void can_send_msg(uint32_t id) {
    uint8_t buffer[8] = {0};
    extern CAN_TxHeaderTypeDef TxHeader;

    union {
        struct mcb_sens_front_1_t front_1;
        struct mcb_sens_front_2_t front_2;
    } msgs;

    TxHeader.StdId = id;

    switch (id)
    {
    case MCB_SENS_FRONT_1_FRAME_ID:
        //msgs.front_1.brake_straingauge_voltage_m_v = ;

        TxHeader.DLC = mcb_sens_front_1_pack(buffer, &msgs.front_1, MCB_SENS_FRONT_1_LENGTH);
        break;
    
    default:
        break;
    }
    can_send(&hcan1, buffer, &TxHeader);
}

*/





CAN_TxHeaderTypeDef   TxHeader;

uint8_t               buffer_tx[8];



uint8_t extern  pre_ams_imd_error, post_ams_latch_error, post_ams_imd_error, sdc_closed_pre_tlb_batt_error, ams_error_latch_error,\
                imd_error_latch_error, sdc_prch_rly_error;


extern volatile bool                     can_rx_flag;

extern uint8_t                           RxData;

struct mcb_tlb_battery_shut_status_t     tlb_shut_rx;
extern CAN_RxHeaderTypeDef               RxHeader;
struct hvcb_hvb_rx_v_cell_t              v_cell_rx;
struct nlg5_database_can_nlg5_act_i_t    brusa_rx_voltage;
struct hvcb_hvb_rx_t_cell_t              t_cell;
struct hvcb_hvb_rx_soc_t                 SOC_struct_rx;




#define                                  tlb_battery_tsal_id 1u



double extern v_max_id_rx, v_min_id_rx, v_max_rx, v_min_rx, v_mean_rx;


double mains_v_rx, mains_i_rx, I_out_rx, V_out_rx;


double charge_temp;

double SOC;




extern can_message can_buffer[can_message_rx_number];
extern can_message can_buffer_brusa;



char buffer[400] = {0};

volatile extern uint8_t       can_id;


/*

 from TLB BATTERY we receive signals about imd error, ams error, sdc open state, sdc precharge relay state 

 from the PODIUM HV BMS info about the voltages of the hv battery pack cells 

 form the BRUSA CHARGER info about charging and mains voltage and current 

 */

void can_rx_routine(void){


    if (can_rx_flag == 1){
        
        HAL_UART_Transmit(&huart2, (uint8_t *)"messaggio ricevuto: \n\r", strlen("messaggio ricevuto: \n\r"),100);


        if((can_buffer[0].data_present == 0) & (can_buffer[1].data_present == 0) & (can_buffer_brusa.data_present == 0) & (can_buffer[2].data_present == 0) ) can_rx_flag = 0;



        //TLB BATTERY
        if(can_buffer[1].data_present == 1){
        
        can_buffer[1].data_present = 0;

        mcb_tlb_battery_shut_status_init(&tlb_shut_rx);
        mcb_tlb_battery_shut_status_unpack(&tlb_shut_rx,(uint8_t *) &can_buffer[1].data, MCB_TLB_BATTERY_SHUT_STATUS_LENGTH);

        

        pre_ams_imd_error = mcb_tlb_battery_shut_status_is_shut_closed_pre_ams_imd_latch_decode(tlb_shut_rx.is_shut_closed_pre_ams_imd_latch);
        post_ams_latch_error = mcb_tlb_battery_shut_status_is_shut_closed_post_ams_latch_decode(tlb_shut_rx.is_shut_closed_post_ams_latch);
        post_ams_imd_error = mcb_tlb_battery_shut_status_is_shut_closed_post_imd_latch_decode(tlb_shut_rx.is_shut_closed_post_imd_latch);
        sdc_closed_pre_tlb_batt_error = mcb_tlb_battery_shut_status_is_shutdown_closed_pre_tlb_batt_final_decode(tlb_shut_rx.is_shutdown_closed_pre_tlb_batt_final);
        ams_error_latch_error = mcb_tlb_battery_shut_status_is_ams_error_latched_decode(tlb_shut_rx.is_ams_error_latched);
        imd_error_latch_error = mcb_tlb_battery_shut_status_is_imd_error_latched_decode(tlb_shut_rx.is_imd_error_latched);
        sdc_prch_rly_error = mcb_tlb_battery_shut_status_is_sd_prch_rly_closed_decode(tlb_shut_rx.is_sd_prch_rly_closed);

        

        sprintf(buffer, "pre_ams_imd = %d \n\rpost_ams_latch = %d \n\rpost_ams_imd = %d \n\rpre_tlb_batt = %d \n\rams_error_latch = %d \n\rimd_error_latch = %d \n\rsd_prch_rly = %d \n\r", pre_ams_imd_error, post_ams_latch_error,post_ams_imd_error, sdc_closed_pre_tlb_batt_error, ams_error_latch_error, imd_error_latch_error, sdc_prch_rly_error);
        HAL_UART_Transmit(&LOG_UART, (uint8_t*) &buffer, strlen(buffer), 200);

    
        }

        // PODIUM HV BMS voltage
        if (can_buffer[0].data_present == 1){

            can_buffer[0].data_present = 0;

            hvcb_hvb_rx_v_cell_init(&v_cell_rx);
            hvcb_hvb_rx_v_cell_unpack(&v_cell_rx,(uint8_t *) &can_buffer[0].data, HVCB_HVB_RX_V_CELL_LENGTH);

            v_max_id_rx = hvcb_hvb_rx_v_cell_hvb_idx_cell_u_max_decode(v_cell_rx.hvb_idx_cell_u_max);
            v_min_id_rx = hvcb_hvb_rx_v_cell_hvb_idx_cell_u_min_decode(v_cell_rx.hvb_idx_cell_u_min);
            v_max_rx    = hvcb_hvb_rx_v_cell_hvb_u_cell_max_decode (v_cell_rx.hvb_u_cell_max);
            v_min_rx     = hvcb_hvb_rx_v_cell_hvb_u_cell_min_decode(v_cell_rx.hvb_u_cell_min);
            v_mean_rx    = hvcb_hvb_rx_v_cell_hvb_u_cell_mean_decode(v_cell_rx.hvb_u_cell_mean);


            sprintf(buffer, "v max id = %.0f \n\rv min id = %.0f \n\rv max = %.2f \n\rv min = %.2f \n\rv mean = %.2f \n\r", v_max_id_rx, v_min_id_rx, v_max_rx, v_min_rx, v_mean_rx);
            HAL_UART_Transmit(&LOG_UART, (uint8_t*) &buffer, strlen(buffer), 200);

        }


        //BRUSA voltage
        if(can_buffer_brusa.data_present == 1){
            
            can_buffer_brusa.data_present = 0;

            memset(&brusa_rx_voltage, 0, sizeof(brusa_rx_voltage));
            nlg5_database_can_nlg5_act_i_unpack(&brusa_rx_voltage,(uint8_t *) &can_buffer_brusa.data, NLG5_DATABASE_CAN_NLG5_ACT_I_LENGTH);

            mains_i_rx = nlg5_database_can_nlg5_act_i_nlg5_mc_act_decode(brusa_rx_voltage.nlg5_mc_act);
            mains_v_rx = nlg5_database_can_nlg5_act_i_nlg5_mv_act_decode(brusa_rx_voltage.nlg5_mv_act);
            V_out_rx   = nlg5_database_can_nlg5_act_i_nlg5_ov_act_decode(brusa_rx_voltage.nlg5_ov_act);
            I_out_rx   = nlg5_database_can_nlg5_act_i_nlg5_oc_act_decode(brusa_rx_voltage.nlg5_oc_act);

            sprintf(buffer, "V = %.2lf\n\rI = %.2lf", V_out_rx, I_out_rx);
            HAL_UART_Transmit(&LOG_UART, (uint8_t*) &buffer, strlen(buffer), 200);

            
        }

        // PODIUM HV BMS temp
        if (can_buffer[2].data_present == 1){

            can_buffer[2].data_present = 0;

            hvcb_hvb_rx_t_cell_init(&t_cell);
            hvcb_hvb_rx_t_cell_unpack(&t_cell, (uint8_t *) &can_buffer[2].data, HVCB_HVB_RX_T_CELL_LENGTH);

            charge_temp = hvcb_hvb_rx_t_cell_hvb_t_cell_max_decode(t_cell.hvb_t_cell_max);

            sprintf(buffer, "charging temp = %.2lf", charge_temp);
            HAL_UART_Transmit(&LOG_UART, (uint8_t*) &buffer, strlen(buffer), 200);

        }

        if(can_buffer[3].data_present == 1){

            can_buffer[3].data_present = 0;

            hvcb_hvb_rx_soc_init(&SOC_struct_rx);
            hvcb_hvb_rx_soc_unpack(&SOC_struct_rx, (uint8_t* ) &can_buffer[3].data, HVCB_HVB_RX_SOC_LENGTH);

            SOC = hvcb_hvb_rx_soc_hvb_r_so_c_hvb_u_cell_min_decode(SOC_struct_rx.hvb_r_so_c_hvb_u_cell_min);

            sprintf(buffer, "SOC = %.2lf", SOC);
            HAL_UART_Transmit(&LOG_UART, (uint8_t*) &buffer, strlen(buffer), 200);


        }

    }
}




#ifdef TEST


/*
double tsal_green = 1;
double air_pos_closed = 1;
double air_neg_closed = 1;
double relay_precharge_closed = 1;
double dc_bus_over60 = 0;
double air_pos_intentional = 1;
double air_neg_intentional = 1;
double intentional_state_relay_precharge = 1;
double short2_gnd_air_pos = 0;
double short2_gnd_air_neg = 0;
double is_any_short2_gnd_present = 0;
double is_any_imp_present = 0;
double is_air_pos_imp_present = 0;

*/

struct mcb_tlb_battery_shut_status_t tlb_shut;
void can_tx_1(){


//struct mcb_tlb_battery_tsal_status_t tlb_tsal;
//struct mcb_sens_front_1_t can_front;


//mcb_tlb_battery_shut packing
mcb_tlb_battery_shut_status_init(&tlb_shut);

double pre_ams_imd = 0u;
double post_ams_latch = 0;
double post_ams_imd = 0;
double sdc_closed_pre_tlb_batt = 0;
double ams_error_latch = 0;
double imd_error_latch = 0;
double sdc_prch_rly = 0u; 

tlb_shut.is_shut_closed_pre_ams_imd_latch       = mcb_tlb_battery_shut_status_is_shut_closed_pre_ams_imd_latch_encode(pre_ams_imd);
tlb_shut.is_shut_closed_post_ams_latch          = mcb_tlb_battery_shut_status_is_shut_closed_post_ams_latch_encode(post_ams_latch);
tlb_shut.is_shut_closed_post_imd_latch          = mcb_tlb_battery_shut_status_is_shut_closed_post_imd_latch_encode(post_ams_imd);
tlb_shut.is_shutdown_closed_pre_tlb_batt_final  = mcb_tlb_battery_shut_status_is_shutdown_closed_pre_tlb_batt_final_encode(sdc_closed_pre_tlb_batt);
tlb_shut.is_ams_error_latched                   = mcb_tlb_battery_shut_status_is_ams_error_latched_encode(ams_error_latch);
tlb_shut.is_imd_error_latched                   = mcb_tlb_battery_shut_status_is_imd_error_latched_encode(imd_error_latch);
tlb_shut.is_sd_prch_rly_closed                  = mcb_tlb_battery_shut_status_is_sd_prch_rly_closed_encode(sdc_prch_rly);

mcb_tlb_battery_shut_status_pack((uint8_t *)&buffer_tx, &tlb_shut, sizeof(buffer_tx));



// impostazioni e Tx can
TxHeader.DLC = sizeof(buffer_tx);
TxHeader.IDE = CAN_ID_STD;
TxHeader.StdId = tlb_battery_shut_id;
TxHeader.ExtId = 0;
TxHeader.RTR = CAN_RTR_DATA;
TxHeader.TransmitGlobalTime = DISABLE;



if(can_send(&hcan1,(uint8_t *) &buffer_tx, &TxHeader, CAN_RX_FIFO0) != HAL_OK) Error_Handler();



}


void can_tx_2(){


struct hvcb_hvb_rx_v_cell_t          v_cell;

double v_max = 4.6;
double v_min = 3.2;
double v_mean = 3.4;
double v_max_id = 24u;
double v_min_id = 70u;

//hvcb_hvb_rx_v_cell packing

hvcb_hvb_rx_v_cell_init(&v_cell);

v_cell.hvb_idx_cell_u_max = hvcb_hvb_rx_v_cell_hvb_idx_cell_u_max_encode(v_max_id);
v_cell.hvb_idx_cell_u_min = hvcb_hvb_rx_v_cell_hvb_idx_cell_u_min_encode(v_min_id);
v_cell.hvb_u_cell_max     = hvcb_hvb_rx_v_cell_hvb_u_cell_max_encode (v_max);
v_cell.hvb_u_cell_min     = hvcb_hvb_rx_v_cell_hvb_u_cell_min_encode(v_min);
v_cell.hvb_u_cell_mean    = hvcb_hvb_rx_v_cell_hvb_u_cell_mean_encode(v_mean);

hvcb_hvb_rx_v_cell_pack((uint8_t *)&buffer_tx, &v_cell, sizeof(buffer_tx));


TxHeader.DLC = sizeof(buffer_tx);
TxHeader.StdId = v_cell_id;

if(can_send(&hcan1,(uint8_t *) &buffer_tx, &TxHeader, CAN_RX_FIFO0)!= HAL_OK) Error_Handler();

}




void can_tx_3(){

    struct nlg5_database_can_nlg5_act_i_t    brusa;

    double mains_v = 230.4, mains_i = 22.456;
    double I_out   = 10.5,  V_out   = 100.578;

    // brusa packing
    memset(&brusa, 0, sizeof(brusa));

    brusa.nlg5_mc_act = nlg5_database_can_nlg5_act_i_nlg5_mc_act_encode(mains_i);
    brusa.nlg5_mv_act = nlg5_database_can_nlg5_act_i_nlg5_mv_act_encode(mains_v);
    brusa.nlg5_oc_act = nlg5_database_can_nlg5_act_i_nlg5_oc_act_encode(I_out);
    brusa.nlg5_ov_act = nlg5_database_can_nlg5_act_i_nlg5_ov_act_encode(V_out);

    nlg5_database_can_nlg5_act_i_pack((uint8_t *)&buffer_tx, &brusa, sizeof(buffer_tx));

    TxHeader.DLC   = sizeof(buffer_tx);
    TxHeader.StdId = NLG5_DATABASE_CAN_NLG5_ACT_I_FRAME_ID;

    if (can_send(&hcan2,(uint8_t *) &buffer_tx, &TxHeader, CAN_RX_FIFO1) != HAL_OK) Error_Handler(); 
    
    
}



void can_tx_4(){

    struct  hvcb_hvb_rx_t_cell_t charge_temp_struct;

    double charge_temp = 41;

    hvcb_hvb_rx_t_cell_init(&charge_temp_struct);
    charge_temp_struct.hvb_t_cell_max = hvcb_hvb_rx_t_cell_hvb_t_cell_max_encode(charge_temp);

    hvcb_hvb_rx_t_cell_pack((uint8_t *) &buffer_tx, &charge_temp_struct, 8);

    TxHeader.DLC   = sizeof(buffer_tx);
    TxHeader.StdId = HVCB_HVB_RX_T_CELL_FRAME_ID;

    if (can_send(&hcan1,(uint8_t *) &buffer_tx, &TxHeader, CAN_RX_FIFO1) != HAL_OK) Error_Handler(); 

}


void can_tx_5(){

    struct hvcb_hvb_rx_soc_t SOC_struct;

    double SOC = 0;

    hvcb_hvb_rx_soc_init(&SOC_struct);
    SOC_struct.hvb_r_so_c_hvb_u_cell_min = hvcb_hvb_rx_soc_hvb_r_so_c_hvb_u_cell_min_encode(SOC);

    hvcb_hvb_rx_soc_pack((uint8_t *) &buffer_tx, &SOC_struct, HVCB_HVB_RX_SOC_LENGTH);

    TxHeader.DLC   = sizeof(buffer_tx);
    TxHeader.StdId = HVCB_HVB_RX_SOC_LENGTH;

    if (can_send(&hcan1,(uint8_t *) &buffer_tx, &TxHeader, CAN_RX_FIFO1) != HAL_OK) Error_Handler(); 

}






#endif