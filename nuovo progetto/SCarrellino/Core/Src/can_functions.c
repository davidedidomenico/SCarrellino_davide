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
#include "SW_Watchdog.h"

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



double extern   sdc_tsac_initial_in_is_active, 
                sdc_post_ams_imd_relay_is_active, 
                sdc_tsac_final_in_is_active, 
                sdc_prch_rly_is_closed;



double extern   air_neg_cmd_is_active                   ,        
                air_neg_is_closed                       ,
                air_neg_stg_mech_state_signal_is_active ,
                air_pos_cmd_is_active                   ,                 
                air_pos_is_closed                       ,
                air_pos_stg_mech_state_signal_is_active ,
                ams_err_is_active                       ,
                dcbus_is_over60_v                       ,
                dcbus_prech_rly_cmd_is_active           ,
                dcbus_prech_rly_is_closed               ,
                imd_err_is_active                       ,
                imp_ai_rs_signals_is_active             ,
                imp_any_is_active                       ,
                imp_hv_relays_signals_is_active         ,
                tsal_green_is_active                    ;   



extern volatile bool                     can_rx_flag;

extern uint8_t                           RxData;

struct mcb_tlb_bat_sd_csensing_status_t  tlb_scd_rx;
extern CAN_RxHeaderTypeDef               RxHeader;
struct hvcb_hvb_rx_v_cell_t              v_cell_rx;
struct nlg5_database_can_nlg5_act_i_t    brusa_rx_voltage;
struct hvcb_hvb_rx_t_cell_t              t_cell;
struct hvcb_hvb_rx_soc_t                 SOC_struct_rx;




double extern v_max_id_rx, 
              v_min_id_rx,
              v_max_rx,
              v_min_rx, 
              v_mean_rx;


double mains_v_rx,
       mains_i_rx, 
       I_out_rx, 
       V_out_rx;


double charge_temp;

double SOC = -50/0.04;


double vcu_clr_err,
       vcu_b_bal_req,
       vcu_b_all_vt_req;




extern can_message can_buffer[can_message_rx_number];
extern can_message can_buffer_brusa;



char buffer[1200] = {0};

volatile extern uint8_t       can_id;


/*

 from TLB BATTERY we receive signals about imd error, ams error, sdc open state, sdc precharge relay state 

 from the PODIUM HV BMS info about the voltages of the hv battery pack cells 

 form the BRUSA CHARGER info about charging and mains voltage and current 

 */
void can_rx_routine(void){

    if (can_rx_flag == 1){
        
        //HAL_UART_Transmit(&LOG_UART, (uint8_t *)"messaggio ricevuto: \n\r", strlen("messaggio ricevuto: \n\r"),100);


        if((can_buffer[0].data_present == 0) & (can_buffer[1].data_present == 0) & (can_buffer[5].data_present == 0) & \
           (can_buffer[2].data_present == 0) & (can_buffer[3].data_present == 0) & (can_buffer[4].data_present == 0) & (can_buffer_brusa.data_present == 0) ) can_rx_flag = 0;



        //TLB BATTERY
        if(can_buffer[1].data_present == 1){

        SW_Wachdog_Refresh("MCB_TLB_BAT_SD_CSENSING_STATUS_FRAME");
        
        can_buffer[1].data_present = 0;

        mcb_tlb_bat_sd_csensing_status_init(&tlb_scd_rx);
        mcb_tlb_bat_sd_csensing_status_unpack(&tlb_scd_rx,(uint8_t *) &can_buffer[1].data, MCB_TLB_BAT_SD_CSENSING_STATUS_LENGTH);

        

        sdc_tsac_initial_in_is_active = mcb_tlb_bat_sd_csensing_status_sdc_tsac_initial_in_is_active_decode(tlb_scd_rx.sdc_tsac_initial_in_is_active);
        sdc_post_ams_imd_relay_is_active = mcb_tlb_bat_sd_csensing_status_sdc_post_ams_imd_relay_is_active_decode(tlb_scd_rx.sdc_post_ams_imd_relay_is_active);
        sdc_tsac_final_in_is_active = mcb_tlb_bat_sd_csensing_status_sdc_tsac_final_in_is_active_decode(tlb_scd_rx.sdc_tsac_final_in_is_active);
        sdc_prch_rly_is_closed = mcb_tlb_bat_sd_csensing_status_sdc_prch_rly_is_closed_decode(tlb_scd_rx.sdc_prch_rly_is_closed);
        

        

        sprintf(buffer, "sdc_tsac_initial_in_is_active      = %.0lf\n\r\
sdc_post_ams_imd_relay_is_active      = %.0lf \n\r\
sdc_tsac_final_in_is_active           = %.0lf \n\r\
sdc_prch_rly_is_closed                = %.0lf \n\r"\
                                                                ,sdc_tsac_initial_in_is_active ,          \
                                                                 sdc_post_ams_imd_relay_is_active,             \
                                                                 sdc_tsac_final_in_is_active,        \
                                                                 sdc_prch_rly_is_closed);
       // HAL_UART_Transmit(&LOG_UART, (uint8_t*) &buffer, strlen(buffer), 200);

    
        }

        // PODIUM HV BMS voltage
        if (can_buffer[0].data_present == 1){

            SW_Wachdog_Refresh("HVCB_HVB_RX_V_CELL_FRAME");

            can_buffer[0].data_present = 0;

            hvcb_hvb_rx_v_cell_init(&v_cell_rx);
            hvcb_hvb_rx_v_cell_unpack(&v_cell_rx,(uint8_t *) &can_buffer[0].data, HVCB_HVB_RX_V_CELL_LENGTH);

            v_max_id_rx = hvcb_hvb_rx_v_cell_hvb_idx_cell_u_max_decode(v_cell_rx.hvb_idx_cell_u_max);
            v_min_id_rx = hvcb_hvb_rx_v_cell_hvb_idx_cell_u_min_decode(v_cell_rx.hvb_idx_cell_u_min);
            v_max_rx    = hvcb_hvb_rx_v_cell_hvb_u_cell_max_decode (v_cell_rx.hvb_u_cell_max);
            v_min_rx     = hvcb_hvb_rx_v_cell_hvb_u_cell_min_decode(v_cell_rx.hvb_u_cell_min);
            v_mean_rx    = hvcb_hvb_rx_v_cell_hvb_u_cell_mean_decode(v_cell_rx.hvb_u_cell_mean);


            sprintf(buffer, "v max id = %.0f \n\rv min id = %.0f \n\rv max = %.2f \n\rv min = %.2f \n\rv mean = %.2f \n\r", v_max_id_rx, v_min_id_rx, v_max_rx, v_min_rx, v_mean_rx);
            //HAL_UART_Transmit(&LOG_UART, (uint8_t*) &buffer, strlen(buffer), 200);

        }


        //BRUSA voltage
        if(can_buffer_brusa.data_present == 1){

            SW_Wachdog_Refresh("NLG5_DATABASE_CAN_NLG5_ACT_I_FRAME");

            
            can_buffer_brusa.data_present = 0;

            memset(&brusa_rx_voltage, 0, sizeof(brusa_rx_voltage));
            nlg5_database_can_nlg5_act_i_unpack(&brusa_rx_voltage,(uint8_t *) &can_buffer_brusa.data, NLG5_DATABASE_CAN_NLG5_ACT_I_LENGTH);

            mains_i_rx = nlg5_database_can_nlg5_act_i_nlg5_mc_act_decode(brusa_rx_voltage.nlg5_mc_act);
            mains_v_rx = nlg5_database_can_nlg5_act_i_nlg5_mv_act_decode(brusa_rx_voltage.nlg5_mv_act);
            V_out_rx   = nlg5_database_can_nlg5_act_i_nlg5_ov_act_decode(brusa_rx_voltage.nlg5_ov_act);
            I_out_rx   = nlg5_database_can_nlg5_act_i_nlg5_oc_act_decode(brusa_rx_voltage.nlg5_oc_act);

            sprintf(buffer, "V = %.2lf\n\rI = %.2lf\n\r", V_out_rx, I_out_rx);
           // HAL_UART_Transmit(&LOG_UART, (uint8_t*) &buffer, strlen(buffer), 200);

            
        }

        // PODIUM HV BMS temp
        if (can_buffer[2].data_present == 1){

           SW_Wachdog_Refresh("HVCB_HVB_RX_T_CELL_FRAME");
            can_buffer[2].data_present = 0;

            hvcb_hvb_rx_t_cell_init(&t_cell);
            hvcb_hvb_rx_t_cell_unpack(&t_cell, (uint8_t *) &can_buffer[2].data, HVCB_HVB_RX_T_CELL_LENGTH);

            charge_temp = hvcb_hvb_rx_t_cell_hvb_t_cell_max_decode(t_cell.hvb_t_cell_max);

            sprintf(buffer, "charging temp = %.2lf\n\r", charge_temp);
           // HAL_UART_Transmit(&LOG_UART, (uint8_t*) &buffer, strlen(buffer), 200);

        }

        //PODIUM HV BMS state of charge
        if(can_buffer[3].data_present == 1){

            SW_Wachdog_Refresh("HVCB_HVB_RX_SOC_FRAME");

            can_buffer[3].data_present = 0;

            hvcb_hvb_rx_soc_init(&SOC_struct_rx);
            hvcb_hvb_rx_soc_unpack(&SOC_struct_rx, (uint8_t* ) &can_buffer[3].data, HVCB_HVB_RX_SOC_LENGTH);

            SOC = hvcb_hvb_rx_soc_hvb_r_so_c_hvb_u_cell_min_decode(SOC_struct_rx.hvb_r_so_c_hvb_u_cell_min);

            sprintf(buffer, "SOC = %.2lf\n\r", (SOC*0.04 + 50));
          //  HAL_UART_Transmit(&LOG_UART, (uint8_t*) &buffer, strlen(buffer), 200);


        }


        if(can_buffer[4].data_present == 1){

            SW_Wachdog_Refresh("MCB_TLB_BAT_SIGNALS_STATUS_FRAME");


            can_buffer[4].data_present = 0;

            struct mcb_tlb_bat_signals_status_t static tlb_signals;

            mcb_tlb_bat_signals_status_init(&tlb_signals);
            mcb_tlb_bat_signals_status_unpack(&tlb_signals,(uint8_t* ) &can_buffer[4].data,MCB_TLB_BAT_SIGNALS_STATUS_LENGTH);

            air_neg_cmd_is_active                      = mcb_tlb_bat_signals_status_air_neg_cmd_is_active_decode(tlb_signals.air_neg_cmd_is_active); 
            air_neg_is_closed                          = mcb_tlb_bat_signals_status_air_neg_is_closed_decode(tlb_signals.air_neg_is_closed);
            air_neg_stg_mech_state_signal_is_active    = mcb_tlb_bat_signals_status_air_neg_stg_mech_state_signal_is_active_decode(tlb_signals.air_neg_stg_mech_state_signal_is_active);
            air_pos_cmd_is_active                      = mcb_tlb_bat_signals_status_air_pos_cmd_is_active_decode(tlb_signals.air_pos_cmd_is_active);
            air_pos_is_closed                          = mcb_tlb_bat_signals_status_air_pos_is_closed_decode(tlb_signals.air_pos_is_closed);
            air_pos_stg_mech_state_signal_is_active    = mcb_tlb_bat_signals_status_air_pos_stg_mech_state_signal_is_active_decode(tlb_signals.air_pos_stg_mech_state_signal_is_active);
            ams_err_is_active                          = mcb_tlb_bat_signals_status_ams_err_is_active_decode(tlb_signals.ams_err_is_active);
            dcbus_is_over60_v                          = mcb_tlb_bat_signals_status_dcbus_is_over60_v_decode(tlb_signals.dcbus_is_over60_v);
            dcbus_prech_rly_cmd_is_active              = mcb_tlb_bat_signals_status_dcbus_prech_rly_cmd_is_active_decode(tlb_signals.dcbus_prech_rly_cmd_is_active);
            dcbus_prech_rly_is_closed                  = mcb_tlb_bat_signals_status_dcbus_prech_rly_is_closed_decode(tlb_signals.dcbus_prech_rly_is_closed);
            imd_err_is_active                          = mcb_tlb_bat_signals_status_imd_err_is_active_decode(tlb_signals.imd_err_is_active);
            imp_ai_rs_signals_is_active                = mcb_tlb_bat_signals_status_imp_ai_rs_signals_is_active_decode(tlb_signals.imp_ai_rs_signals_is_active);
            imp_any_is_active                          = mcb_tlb_bat_signals_status_imp_any_is_active_decode(tlb_signals.imp_any_is_active);
            imp_hv_relays_signals_is_active            = mcb_tlb_bat_signals_status_imp_hv_relays_signals_is_active_decode(tlb_signals.imp_hv_relays_signals_is_active);
            tsal_green_is_active                       = mcb_tlb_bat_signals_status_tsal_green_is_active_decode(tlb_signals.tsal_green_is_active);


            sprintf(buffer,"air_neg_cmd_is_active                    =   %.0lf,\n\r\
air_neg_is_closed                        =   %.0lf,\n\r\
air_neg_stg_mech_state_signal_is_active  =   %.0lf,\n\r\
air_pos_cmd_is_active                    =   %.0lf,\n\r\
air_pos_is_closed                        =   %.0lf,\n\r\
air_pos_stg_mech_state_signal_is_active  =   %.0lf,\n\r\
ams_err_is_active                        =   %.0lf,\n\r\
dcbus_is_over60_v                        =   %.0lf,\n\r\
dcbus_prech_rly_cmd_is_active            =   %.0lf,\n\r\
dcbus_prech_rly_is_closed                =   %.0lf,\n\r\
imd_err_is_active                        =   %.0lf,\n\r\
imp_ai_rs_signals_is_active              =   %.0lf,\n\r\
imp_any_is_active                        =   %.0lf,\n\r\
imp_hv_relays_signals_is_active          =   %.0lf,\n\r\
tsal_green_is_active                     =   %.0lf\n\r ",                          air_neg_cmd_is_active                   ,\
                                                                                   air_neg_is_closed                       ,\
                                                                                   air_neg_stg_mech_state_signal_is_active ,\
                                                                                   air_pos_cmd_is_active                   ,\
                                                                                   air_pos_is_closed                       ,\
                                                                                   air_pos_stg_mech_state_signal_is_active ,\
                                                                                   ams_err_is_active                       ,\
                                                                                   dcbus_is_over60_v                       ,\
                                                                                   dcbus_prech_rly_cmd_is_active           ,\
                                                                                   dcbus_prech_rly_is_closed               ,\
                                                                                   imd_err_is_active                       ,\
                                                                                   imp_ai_rs_signals_is_active             ,\
                                                                                   imp_any_is_active                       ,\
                                                                                   imp_hv_relays_signals_is_active         ,\
                                                                                   tsal_green_is_active                    );

           // HAL_UART_Transmit(&LOG_UART, (uint8_t*) &buffer, strlen(buffer), 200);


        }


        if(can_buffer[5].data_present == 1){

            SW_Wachdog_Refresh("HVCB_HVB_TX_VCU_CMD_FRAME");

            can_buffer[5].data_present = 0;

            struct hvcb_hvb_tx_vcu_cmd_t static AIR_Cmd;

            hvcb_hvb_tx_vcu_cmd_init(&AIR_Cmd);

            hvcb_hvb_tx_vcu_cmd_unpack(&AIR_Cmd,(uint8_t* ) can_buffer[5].data, HVCB_HVB_TX_VCU_CMD_LENGTH);


            vcu_b_all_vt_req = hvcb_hvb_tx_vcu_cmd_vcu_b_all_vt_req_decode(AIR_Cmd.vcu_b_all_vt_req);
            vcu_b_bal_req    =  hvcb_hvb_tx_vcu_cmd_vcu_b_bal_req_decode(AIR_Cmd.vcu_b_bal_req);
            vcu_clr_err      =  hvcb_hvb_tx_vcu_cmd_vcu_clr_err_decode(AIR_Cmd.vcu_clr_err);
            


    }
}
}





#ifdef TEST


struct mcb_tlb_bat_sd_csensing_status_t tlb_shut;
void can_tx_1(){


//struct mcb_tlb_battery_tsal_status_t tlb_tsal;
//struct mcb_sens_front_1_t can_front;


//mcb_tlb_battery_shut packing
mcb_tlb_bat_sd_csensing_status_init(&tlb_shut);

double static sdc_tsac_initial_in_is_active = 1u;
double static sdc_post_ams_imd_relay_is_active = 1;
double static sdc_tsac_final_in_is_active = 1;
double static sdc_prch_rly_is_closed = 1;


tlb_shut.sdc_tsac_initial_in_is_active          = mcb_tlb_bat_sd_csensing_status_sdc_tsac_initial_in_is_active_encode(sdc_tsac_initial_in_is_active);
tlb_shut.sdc_post_ams_imd_relay_is_active       = mcb_tlb_bat_sd_csensing_status_sdc_post_ams_imd_relay_is_active_encode(sdc_post_ams_imd_relay_is_active);
tlb_shut.sdc_tsac_final_in_is_active            = mcb_tlb_bat_sd_csensing_status_sdc_tsac_final_in_is_active_encode(sdc_tsac_final_in_is_active);
tlb_shut.sdc_prch_rly_is_closed                = mcb_tlb_bat_sd_csensing_status_sdc_prch_rly_is_closed_encode(sdc_prch_rly_is_closed);


mcb_tlb_bat_sd_csensing_status_pack((uint8_t *)&buffer_tx, &tlb_shut, sizeof(buffer_tx));



// impostazioni e Tx can
TxHeader.DLC = sizeof(buffer_tx);
TxHeader.IDE = CAN_ID_STD;
TxHeader.StdId = MCB_TLB_BAT_SD_CSENSING_STATUS_FRAME_ID;
TxHeader.ExtId = 0;
TxHeader.RTR = CAN_RTR_DATA;
TxHeader.TransmitGlobalTime = DISABLE;



if(can_send(&hcan2,(uint8_t *) &buffer_tx, &TxHeader, CAN_TX_MAILBOX2) != HAL_OK) Error_Handler();



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
TxHeader.StdId = HVCB_HVB_RX_V_CELL_FRAME_ID;

if(can_send(&hcan1,(uint8_t *) &buffer_tx, &TxHeader, CAN_TX_MAILBOX1)!= HAL_OK) Error_Handler();

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

    if (can_send(&hcan2,(uint8_t *) &buffer_tx, &TxHeader, CAN_TX_MAILBOX0) != HAL_OK) Error_Handler(); 
    
    
}



void can_tx_4(){

    struct  hvcb_hvb_rx_t_cell_t charge_temp_struct;

    double charge_temp = 44;

    hvcb_hvb_rx_t_cell_init(&charge_temp_struct);
    charge_temp_struct.hvb_t_cell_max = hvcb_hvb_rx_t_cell_hvb_t_cell_max_encode(charge_temp);

    hvcb_hvb_rx_t_cell_pack((uint8_t *) &buffer_tx, &charge_temp_struct, 8);

    TxHeader.DLC   = sizeof(buffer_tx);
    TxHeader.StdId = HVCB_HVB_RX_T_CELL_FRAME_ID;

    if (can_send(&hcan1,(uint8_t *) &buffer_tx, &TxHeader, CAN_TX_MAILBOX2) != HAL_OK) Error_Handler(); 

}


void can_tx_5(){

    struct hvcb_hvb_rx_soc_t SOC_struct;

    double SOC = 0;

    hvcb_hvb_rx_soc_init(&SOC_struct);
    SOC_struct.hvb_r_so_c_hvb_u_cell_min = hvcb_hvb_rx_soc_hvb_r_so_c_hvb_u_cell_min_encode(SOC);

    hvcb_hvb_rx_soc_pack((uint8_t *) &buffer_tx, &SOC_struct, HVCB_HVB_RX_SOC_LENGTH);

    TxHeader.DLC   = sizeof(buffer_tx);
    TxHeader.StdId = HVCB_HVB_RX_SOC_FRAME_ID;

    if (can_send(&hcan1,(uint8_t *) &buffer_tx, &TxHeader, CAN_TX_MAILBOX1) != HAL_OK) Error_Handler(); 

}


void can_tx_6(){

struct mcb_tlb_bat_signals_status_t static tlb_signals;



mcb_tlb_bat_signals_status_init(&tlb_signals);

double static air_neg_cmd_is_active     = 0,\
air_neg_is_closed                       = 0,\
air_neg_stg_mech_state_signal_is_active = 0,\
air_pos_cmd_is_active                   = 0,\                 
air_pos_is_closed                       = 0,\
air_pos_stg_mech_state_signal_is_active = 0,\
ams_err_is_active                       = 0,\
dcbus_is_over60_v                       = 0,\
dcbus_prech_rly_cmd_is_active           = 0,\
dcbus_prech_rly_is_closed               = 0,\
imd_err_is_active                       = 0,\
imp_ai_rs_signals_is_active             = 0,\
imp_any_is_active                       = 0,\
imp_hv_relays_signals_is_active         = 0,\
tsal_green_is_active                    = 1;             

tlb_signals.air_neg_cmd_is_active                   = mcb_tlb_bat_signals_status_air_neg_cmd_is_active_encode(air_neg_cmd_is_active) ;
tlb_signals.air_neg_is_closed                       = mcb_tlb_bat_signals_status_air_neg_is_closed_encode(air_neg_is_closed);
tlb_signals.air_neg_stg_mech_state_signal_is_active = mcb_tlb_bat_signals_status_air_neg_stg_mech_state_signal_is_active_encode(air_neg_stg_mech_state_signal_is_active);
tlb_signals.air_pos_cmd_is_active                   = mcb_tlb_bat_signals_status_air_pos_cmd_is_active_encode(air_pos_cmd_is_active);
tlb_signals.air_pos_is_closed                       = mcb_tlb_bat_signals_status_air_pos_is_closed_encode(air_pos_is_closed);
tlb_signals.air_pos_stg_mech_state_signal_is_active = mcb_tlb_bat_signals_status_air_pos_stg_mech_state_signal_is_active_encode(air_pos_stg_mech_state_signal_is_active);
tlb_signals.ams_err_is_active                       = mcb_tlb_bat_signals_status_ams_err_is_active_encode(ams_err_is_active);
tlb_signals.dcbus_is_over60_v                       = mcb_tlb_bat_signals_status_dcbus_is_over60_v_encode(dcbus_is_over60_v);
tlb_signals.dcbus_prech_rly_cmd_is_active           = mcb_tlb_bat_signals_status_dcbus_prech_rly_cmd_is_active_encode(dcbus_prech_rly_cmd_is_active);
tlb_signals.dcbus_prech_rly_is_closed               = mcb_tlb_bat_signals_status_dcbus_prech_rly_is_closed_encode(dcbus_prech_rly_is_closed);
tlb_signals.imd_err_is_active                       = mcb_tlb_bat_signals_status_imd_err_is_active_encode(imd_err_is_active);
tlb_signals.imp_ai_rs_signals_is_active             = mcb_tlb_bat_signals_status_imp_ai_rs_signals_is_active_encode(imp_ai_rs_signals_is_active);
tlb_signals.imp_any_is_active                       = mcb_tlb_bat_signals_status_imp_any_is_active_encode(imp_any_is_active);
tlb_signals.imp_hv_relays_signals_is_active         = mcb_tlb_bat_signals_status_imp_hv_relays_signals_is_active_encode(imp_hv_relays_signals_is_active);
tlb_signals.tsal_green_is_active                    = mcb_tlb_bat_signals_status_tsal_green_is_active_encode(tsal_green_is_active);

mcb_tlb_bat_signals_status_pack((uint8_t *) &buffer_tx, &tlb_signals, MCB_TLB_BAT_SIGNALS_STATUS_LENGTH);

TxHeader.DLC   = sizeof(buffer_tx);
TxHeader.StdId = MCB_TLB_BAT_SIGNALS_STATUS_FRAME_ID;

if (can_send(&hcan2,(uint8_t *) &buffer_tx, &TxHeader, CAN_TX_MAILBOX0) != HAL_OK) Error_Handler(); 

}



void can_tx_7(){

struct hvcb_hvb_tx_vcu_cmd_t static AIR_Cmd;

    hvcb_hvb_tx_vcu_cmd_init(&AIR_Cmd);

    double static AIR_Cmd_On = 1,\
                  err        = 0,\
                  message_en = 1,\
                  balancing  = 0;

    AIR_Cmd.vcu_b_hvb_inv_req = hvcb_hvb_tx_vcu_cmd_vcu_b_hvb_inv_req_encode(AIR_Cmd_On);
    AIR_Cmd.vcu_clr_err       = hvcb_hvb_tx_vcu_cmd_vcu_clr_err_encode(err);
    AIR_Cmd.vcu_b_bal_req     = hvcb_hvb_tx_vcu_cmd_vcu_b_bal_req_encode(balancing);
    AIR_Cmd.vcu_b_all_vt_req  = hvcb_hvb_tx_vcu_cmd_vcu_b_all_vt_req_encode(message_en);

    hvcb_hvb_tx_vcu_cmd_pack((uint8_t *) &buffer_tx, &AIR_Cmd, HVCB_HVB_TX_VCU_CMD_LENGTH);

    TxHeader.DLC   = HVCB_HVB_TX_VCU_CMD_LENGTH;
    TxHeader.StdId = HVCB_HVB_TX_VCU_CMD_FRAME_ID;

    if (can_send(&hcan1,(uint8_t *) &buffer_tx, &TxHeader, CAN_TX_MAILBOX0) != HAL_OK) Error_Handler(); 


}

#endif



//commands the activation of the AIRs
void AIR_CAN_Cmd_On(){

    struct hvcb_hvb_tx_vcu_cmd_t static AIR_Cmd;

    hvcb_hvb_tx_vcu_cmd_init(&AIR_Cmd);

    double static AIR_Cmd_On = 1;

    AIR_Cmd.vcu_b_hvb_inv_req = hvcb_hvb_tx_vcu_cmd_vcu_b_hvb_inv_req_encode(AIR_Cmd_On);
    AIR_Cmd.vcu_clr_err       = hvcb_hvb_tx_vcu_cmd_vcu_clr_err_encode(0);
    AIR_Cmd.vcu_b_bal_req     = hvcb_hvb_tx_vcu_cmd_vcu_b_bal_req_encode(1);
    AIR_Cmd.vcu_b_all_vt_req  = hvcb_hvb_tx_vcu_cmd_vcu_b_all_vt_req_encode(1);

    hvcb_hvb_tx_vcu_cmd_pack((uint8_t *) &buffer_tx, &AIR_Cmd, HVCB_HVB_TX_VCU_CMD_LENGTH);

    TxHeader.DLC   = HVCB_HVB_TX_VCU_CMD_LENGTH;
    TxHeader.StdId = HVCB_HVB_TX_VCU_CMD_FRAME_ID;

    if (can_send(&hcan2,(uint8_t *) &buffer_tx, &TxHeader, CAN_TX_MAILBOX0) != HAL_OK) {
        uint8_t volatile extern error_code;
        error_code = can_send_error;
        Error_Handler();} 


}



//commands the disactivation of the AIRs
void AIR_CAN_Cmd_Off(){

    struct hvcb_hvb_tx_vcu_cmd_t static AIR_Cmd;

    hvcb_hvb_tx_vcu_cmd_init(&AIR_Cmd);

    double static AIR_Cmd_Off = 0;

    AIR_Cmd.vcu_b_hvb_inv_req = hvcb_hvb_tx_vcu_cmd_vcu_b_hvb_inv_req_encode(AIR_Cmd_Off);
    AIR_Cmd.vcu_clr_err       = hvcb_hvb_tx_vcu_cmd_vcu_clr_err_encode(0);
    AIR_Cmd.vcu_b_bal_req     = hvcb_hvb_tx_vcu_cmd_vcu_b_bal_req_encode(0);
    AIR_Cmd.vcu_b_all_vt_req  = hvcb_hvb_tx_vcu_cmd_vcu_b_all_vt_req_encode(1);

    hvcb_hvb_tx_vcu_cmd_pack((uint8_t *) &buffer_tx, &AIR_Cmd, HVCB_HVB_TX_VCU_CMD_LENGTH);

    TxHeader.DLC   = HVCB_HVB_TX_VCU_CMD_LENGTH;
    TxHeader.StdId = HVCB_HVB_TX_VCU_CMD_FRAME_ID;

    if (can_send(&hcan2,(uint8_t *) &buffer_tx, &TxHeader, CAN_TX_MAILBOX0) != HAL_OK){
        uint8_t volatile extern error_code;
        error_code = can_send_error;
        Error_Handler(); }


}



void can_WD_setting(){

    SW_Watchdog_Typedef HVCB_HVB_RX_V_CELL_FRAME;

    strcat(HVCB_HVB_RX_V_CELL_FRAME.name, "HVCB_HVB_RX_V_CELL_FRAME");
    HVCB_HVB_RX_V_CELL_FRAME.index         = 0;
    HVCB_HVB_RX_V_CELL_FRAME.cycle_time    = HVCB_HVB_RX_V_CELL_CYCLE_TIME_MS;
    HVCB_HVB_RX_V_CELL_FRAME.watchdog_time = HVCB_HVB_RX_V_CELL_CYCLE_TIME_MS * 5;

   //SW_Watchdog_Typedef HVCB_HVB_TX_VCU_CMD_FRAME;

   //strcat(HVCB_HVB_TX_VCU_CMD_FRAME.name, "HVCB_HVB_TX_VCU_CMD_FRAME");
   //HVCB_HVB_TX_VCU_CMD_FRAME.index         = 1;
   //HVCB_HVB_TX_VCU_CMD_FRAME.cycle_time    = HVCB_HVB_TX_VCU_CMD_CYCLE_TIME_MS;
   //HVCB_HVB_TX_VCU_CMD_FRAME.watchdog_time = HVCB_HVB_TX_VCU_CMD_CYCLE_TIME_MS * 5;

    SW_Watchdog_Typedef HVCB_HVB_RX_T_CELL_FRAME;

    strcat(HVCB_HVB_RX_T_CELL_FRAME.name, "HVCB_HVB_RX_T_CELL_FRAME");
    HVCB_HVB_RX_T_CELL_FRAME.index         = 2;
    HVCB_HVB_RX_T_CELL_FRAME.cycle_time    = HVCB_HVB_RX_T_CELL_CYCLE_TIME_MS;
    HVCB_HVB_RX_T_CELL_FRAME.watchdog_time = HVCB_HVB_RX_T_CELL_CYCLE_TIME_MS * 5;

    SW_Watchdog_Typedef HVCB_HVB_RX_SOC_FRAME;

    strcat(HVCB_HVB_RX_SOC_FRAME.name, "HVCB_HVB_RX_SOC_FRAME");
    HVCB_HVB_RX_SOC_FRAME.index         = 3;
    HVCB_HVB_RX_SOC_FRAME.cycle_time    = HVCB_HVB_RX_SOC_CYCLE_TIME_MS;
    HVCB_HVB_RX_SOC_FRAME.watchdog_time = HVCB_HVB_RX_SOC_CYCLE_TIME_MS * 5;

   // SW_Watchdog_Typedef NLG5_DATABASE_CAN_NLG5_ACT_I_FRAME;

   // strcat(NLG5_DATABASE_CAN_NLG5_ACT_I_FRAME.name, "NLG5_DATABASE_CAN_NLG5_ACT_I_FRAME");
  //  NLG5_DATABASE_CAN_NLG5_ACT_I_FRAME.index         = 4;
  //  NLG5_DATABASE_CAN_NLG5_ACT_I_FRAME.cycle_time    = 100;
  //  NLG5_DATABASE_CAN_NLG5_ACT_I_FRAME.watchdog_time = 100 * 5;

    SW_Watchdog_Typedef MCB_TLB_BAT_SD_CSENSING_STATUS_FRAME;

    strcat(MCB_TLB_BAT_SD_CSENSING_STATUS_FRAME.name, "MCB_TLB_BAT_SD_CSENSING_STATUS_FRAME");
    MCB_TLB_BAT_SD_CSENSING_STATUS_FRAME.index         = 5;
    MCB_TLB_BAT_SD_CSENSING_STATUS_FRAME.cycle_time    = MCB_TLB_BAT_SD_CSENSING_STATUS_CYCLE_TIME_MS;
    MCB_TLB_BAT_SD_CSENSING_STATUS_FRAME.watchdog_time = MCB_TLB_BAT_SD_CSENSING_STATUS_CYCLE_TIME_MS * 5; 

    SW_Watchdog_Typedef MCB_TLB_BAT_SIGNALS_STATUS_FRAME;

    strcat(MCB_TLB_BAT_SIGNALS_STATUS_FRAME.name, "MCB_TLB_BAT_SIGNALS_STATUS_FRAME");
    MCB_TLB_BAT_SIGNALS_STATUS_FRAME.index         = 6;
    MCB_TLB_BAT_SIGNALS_STATUS_FRAME.cycle_time    = MCB_TLB_BAT_SIGNALS_STATUS_CYCLE_TIME_MS;
    MCB_TLB_BAT_SIGNALS_STATUS_FRAME.watchdog_time = MCB_TLB_BAT_SIGNALS_STATUS_CYCLE_TIME_MS * 5; 


   SW_Watchdog_set(&HVCB_HVB_RX_V_CELL_FRAME);
   //SW_Watchdog_set(&HVCB_HVB_TX_VCU_CMD_FRAME);
   SW_Watchdog_set(&HVCB_HVB_RX_T_CELL_FRAME);
   SW_Watchdog_set(&HVCB_HVB_RX_SOC_FRAME);
  // SW_Watchdog_set(&NLG5_DATABASE_CAN_NLG5_ACT_I_FRAME);
   SW_Watchdog_set(&MCB_TLB_BAT_SD_CSENSING_STATUS_FRAME);
   SW_Watchdog_set(&MCB_TLB_BAT_SIGNALS_STATUS_FRAME);


}


void can_WD_start(){

    SW_Watchdog_start("HVCB_HVB_RX_V_CELL_FRAME");
  //  SW_Watchdog_start("HVCB_HVB_TX_VCU_CMD_FRAME");
    SW_Watchdog_start("HVCB_HVB_RX_T_CELL_FRAME");
    SW_Watchdog_start("HVCB_HVB_RX_SOC_FRAME");
  //  SW_Watchdog_start("NLG5_DATABASE_CAN_NLG5_ACT_I_FRAME");
    SW_Watchdog_start("MCB_TLB_BAT_SD_CSENSING_STATUS_FRAME");
    SW_Watchdog_start("MCB_TLB_BAT_SIGNALS_STATUS_FRAME");

}

