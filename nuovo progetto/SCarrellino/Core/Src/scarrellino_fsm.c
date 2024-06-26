/*
 * "THE BEER-WARE LICENSE" (Revision 69):
 * Squadra Corse firmware team wrote this file. As long as you retain this notice
 * you can do whatever you want with this stuff. If we meet some day, and you
 * think this stuff is worth it, you can buy us a beer in return.
 *
 * Authors
 * - Filippo Rossi <filippo.rossi.sc@gmail.com>
 * - Federico Carbone <federico.carbone.sc@gmail.com>
 */
#include "main.h"
#include "adc.h"
#include "scarrellino_fsm.h"
#include "I2C_LCD.h"
#include "stdio.h"
#include "string.h"
#include "stdbool.h"
#include "ECU_level_functions.h"
#include "can_functions.h"

#ifndef __weak
#define __weak __attribute__((weak))
#endif // __weak


//error signals
uint8_t  pre_ams_imd_error ;
uint8_t  post_ams_latch_error;
uint8_t  post_ams_imd_error ;
uint8_t  sdc_closed_pre_tlb_batt_error;
uint8_t  ams_error_latch_error ;
uint8_t  imd_error_latch_error ;
uint8_t  sdc_prch_rly_error ;


//cell info signals
double v_max_id_rx;
double v_min_id_rx;
double v_max_rx;
double v_min_rx;
double v_mean_rx;



         
FSM_callback_function run_callback_1(){

display_routine();



    return 0;
}

FSM_callback_function transition_callback_1(){
    return 0;
}

// Private wrapper function signatures
uint32_t _FSM_SCARRELLINO_FSM_IDLE_event_handle(uint8_t event);
uint32_t _FSM_SCARRELLINO_FSM_IDLE_do_work();

uint32_t _FSM_SCARRELLINO_FSM_CHARGE_event_handle(uint8_t event);
uint32_t _FSM_SCARRELLINO_FSM_CHARGE_do_work();

uint32_t _FSM_SCARRELLINO_FSM_STOP_CHARGE_event_handle(uint8_t event);
uint32_t _FSM_SCARRELLINO_FSM_STOP_CHARGE_do_work();

uint32_t _FSM_SCARRELLINO_FSM_DONE_event_handle(uint8_t event);
uint32_t _FSM_SCARRELLINO_FSM_DONE_do_work();


FSM_StateTypeDef state_table[_FSM_SCARRELLINO_FSM_STATE_COUNT] = {
    [FSM_SCARRELLINO_FSM_IDLE] = {
        .event_handler = _FSM_SCARRELLINO_FSM_IDLE_event_handle,
        .entry = FSM_SCARRELLINO_FSM_IDLE_entry,
        .do_work = _FSM_SCARRELLINO_FSM_IDLE_do_work,
        .exit = FSM_SCARRELLINO_FSM_IDLE_exit,
    },
    [FSM_SCARRELLINO_FSM_CHARGE] = {
        .event_handler = _FSM_SCARRELLINO_FSM_CHARGE_event_handle,
        .entry = FSM_SCARRELLINO_FSM_CHARGE_entry,
        .do_work = _FSM_SCARRELLINO_FSM_CHARGE_do_work,
        .exit = FSM_SCARRELLINO_FSM_CHARGE_exit,
    },
    [FSM_SCARRELLINO_FSM_STOP_CHARGE] = {
        .event_handler = _FSM_SCARRELLINO_FSM_STOP_CHARGE_event_handle,
        .entry = FSM_SCARRELLINO_FSM_STOP_CHARGE_entry,
        .do_work = _FSM_SCARRELLINO_FSM_STOP_CHARGE_do_work,
        .exit = FSM_SCARRELLINO_FSM_STOP_CHARGE_exit,
    },
    [FSM_SCARRELLINO_FSM_DONE] = {
        .event_handler = _FSM_SCARRELLINO_FSM_DONE_event_handle,
        .entry = FSM_SCARRELLINO_FSM_DONE_entry,
        .do_work = _FSM_SCARRELLINO_FSM_DONE_do_work,
        .exit = FSM_SCARRELLINO_FSM_DONE_exit,
    },
};

FSM_ConfigTypeDef config = {
    .state_length = _FSM_SCARRELLINO_FSM_STATE_COUNT,
    .state_table = state_table,
};

STMLIBS_StatusTypeDef FSM_SCARRELLINO_FSM_init(
    FSM_HandleTypeDef *handle,
    uint8_t event_count,
    FSM_callback_function run_callback,
    FSM_callback_function transition_callback
) {
    return FSM_init(handle, &config, event_count, run_callback, transition_callback);
}

// State control functions

/** @brief wrapper of FSM_SCARRELLINO_FSM_event_handle, with exit state checking */
uint32_t _FSM_SCARRELLINO_FSM_IDLE_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_SCARRELLINO_FSM_IDLE_event_handle(event);

    switch (next) {
    case FSM_SCARRELLINO_FSM_IDLE:
    case FSM_SCARRELLINO_FSM_CHARGE:
        return next;
    default:
        return _FSM_SCARRELLINO_FSM_DIE;
    }
}

/** @brief wrapper of FSM_SCARRELLINO_FSM_do_work, with exit state checking */
uint32_t _FSM_SCARRELLINO_FSM_IDLE_do_work() {
    
    can_rx_routine();

    uint32_t next;

    if (ChargeEN == 1){
        
        if( pre_ams_imd_error                == 1 &
            post_ams_latch_error             == 0 &
            sdc_closed_pre_tlb_batt_error    == 0 &
            post_ams_imd_error               == 0 &
            ams_error_latch_error            == 0 &
            imd_error_latch_error            == 0 &
            sdc_prch_rly_error               == 1 &
            ChargeEN                         == 1 )
           {

            next = FSM_SCARRELLINO_FSM_CHARGE;
           }
        
    }

    else{
        next = FSM_SCARRELLINO_FSM_IDLE;
        
    }

    switch (next) {
    case FSM_SCARRELLINO_FSM_IDLE:
    case FSM_SCARRELLINO_FSM_CHARGE:
        return next;
    default:
        return _FSM_SCARRELLINO_FSM_DIE;
    }
}

/** @brief wrapper of FSM_SCARRELLINO_FSM_event_handle, with exit state checking */
uint32_t _FSM_SCARRELLINO_FSM_CHARGE_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_SCARRELLINO_FSM_CHARGE_event_handle(event);

    switch (next) {
    case FSM_SCARRELLINO_FSM_CHARGE:
    case FSM_SCARRELLINO_FSM_STOP_CHARGE:
        return next;
    default:
        return _FSM_SCARRELLINO_FSM_DIE;
    }
}


void FSM_SCARRELLINO_FSM_CHARGE_entry() {
    ChargeBlueLedOn;
    ChargeENcmdON;
}


/** @brief wrapper of FSM_SCARRELLINO_FSM_do_work, with exit state checking */
uint32_t _FSM_SCARRELLINO_FSM_CHARGE_do_work() {
    uint32_t next;

    can_rx_routine();


        if( pre_ams_imd_error                == 1 &
            post_ams_latch_error             == 0 &
            sdc_closed_pre_tlb_batt_error    == 0 &
            post_ams_imd_error               == 0 &
            ams_error_latch_error            == 0 &
            imd_error_latch_error            == 0 &
            sdc_prch_rly_error               == 1 &
            ChargeEN                         == 1 )
           {

        next = FSM_SCARRELLINO_FSM_CHARGE;
           }

    else{
        next = FSM_SCARRELLINO_FSM_STOP_CHARGE;

    }

    switch (next) {
    case FSM_SCARRELLINO_FSM_CHARGE:
    case FSM_SCARRELLINO_FSM_STOP_CHARGE:
        return next;
    default:
        return _FSM_SCARRELLINO_FSM_DIE;
    }
}



/** @brief wrapper of FSM_SCARRELLINO_FSM_event_handle, with exit state checking */
uint32_t _FSM_SCARRELLINO_FSM_STOP_CHARGE_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_SCARRELLINO_FSM_STOP_CHARGE_event_handle(event);

    switch (next) {
    case FSM_SCARRELLINO_FSM_STOP_CHARGE: // Reentrance is always supported on event handlers
    case FSM_SCARRELLINO_FSM_DONE:
        return next;
    default:
        return _FSM_SCARRELLINO_FSM_DIE;
    }
}

/** @brief wrapper of FSM_SCARRELLINO_FSM_do_work, with exit state checking */
uint32_t _FSM_SCARRELLINO_FSM_STOP_CHARGE_do_work() {
    
    
    ChargeBlueLedOff;
    ChargeENcmdOFF;

    uint32_t next = FSM_SCARRELLINO_FSM_DONE;

    switch (next) {
    case FSM_SCARRELLINO_FSM_DONE:
        return next;
    default:
        return _FSM_SCARRELLINO_FSM_DIE;
    }
}

/** @brief wrapper of FSM_SCARRELLINO_FSM_event_handle, with exit state checking */
uint32_t _FSM_SCARRELLINO_FSM_DONE_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_SCARRELLINO_FSM_DONE_event_handle(event);

    switch (next) {
    case FSM_SCARRELLINO_FSM_DONE:
    case FSM_SCARRELLINO_FSM_IDLE:
        return next;
    default:
        return _FSM_SCARRELLINO_FSM_DIE;
    }
}

/** @brief wrapper of FSM_SCARRELLINO_FSM_do_work, with exit state checking */
uint32_t _FSM_SCARRELLINO_FSM_DONE_do_work() {
    uint32_t next;

    if (HAL_GPIO_ReadPin(CH_EN_BUTTON_GPIO_IN_GPIO_Port, CH_EN_BUTTON_GPIO_IN_Pin) == 1){
        next = FSM_SCARRELLINO_FSM_DONE;
        HAL_GPIO_WritePin(STAT2_LED_GPIO_OUT_GPIO_Port, STAT2_LED_GPIO_OUT_Pin, 1);
        
    }

    else{
        next = FSM_SCARRELLINO_FSM_IDLE;
                HAL_GPIO_WritePin(STAT2_LED_GPIO_OUT_GPIO_Port, STAT2_LED_GPIO_OUT_Pin, 0);

    }
    

    switch (next) {
    case FSM_SCARRELLINO_FSM_DONE:
    case FSM_SCARRELLINO_FSM_IDLE:
            return next;
    default:
        return _FSM_SCARRELLINO_FSM_DIE;
    }
}


// State functions

/** @attention this function is a stub and as such is declared as weak. */
__weak FSM_SCARRELLINO_FSM_StateTypeDef FSM_SCARRELLINO_FSM_IDLE_event_handle(uint8_t event) {
    return FSM_SCARRELLINO_FSM_IDLE;
}

/** @attention this function is a stub and as such is declared as weak. */
__weak void FSM_SCARRELLINO_FSM_IDLE_entry() {
    return;
}

/** @attention this function is a stub and as such is declared as weak. */
__weak FSM_SCARRELLINO_FSM_StateTypeDef FSM_SCARRELLINO_FSM_IDLE_do_work() {
    return FSM_SCARRELLINO_FSM_IDLE;
}

/** @attention this function is a stub and as such is declared as weak. */
__weak void FSM_SCARRELLINO_FSM_IDLE_exit() {
    return;
}

/** @attention this function is a stub and as such is declared as weak. */
__weak FSM_SCARRELLINO_FSM_StateTypeDef FSM_SCARRELLINO_FSM_CHARGE_event_handle(uint8_t event) {
    return FSM_SCARRELLINO_FSM_CHARGE;
}

/** @attention this function is a stub and as such is declared as weak. */
//__weak void FSM_SCARRELLINO_FSM_CHARGE_entry() {
  //  return;
//}

/** @attention this function is a stub and as such is declared as weak. */
__weak FSM_SCARRELLINO_FSM_StateTypeDef FSM_SCARRELLINO_FSM_CHARGE_do_work() {
    return FSM_SCARRELLINO_FSM_CHARGE;
}

/** @attention this function is a stub and as such is declared as weak. */
__weak void FSM_SCARRELLINO_FSM_CHARGE_exit() {
    return;
}

/** @attention this function is a stub and as such is declared as weak. */
__weak FSM_SCARRELLINO_FSM_StateTypeDef FSM_SCARRELLINO_FSM_STOP_CHARGE_event_handle(uint8_t event) {
    return FSM_SCARRELLINO_FSM_STOP_CHARGE;
}

/** @attention this function is a stub and as such is declared as weak. */
__weak void FSM_SCARRELLINO_FSM_STOP_CHARGE_entry() {
    return;
}

/** @attention this function is a stub and as such is declared as weak. */
__weak FSM_SCARRELLINO_FSM_StateTypeDef FSM_SCARRELLINO_FSM_STOP_CHARGE_do_work() {
    return FSM_SCARRELLINO_FSM_STOP_CHARGE;
}

/** @attention this function is a stub and as such is declared as weak. */
__weak void FSM_SCARRELLINO_FSM_STOP_CHARGE_exit() {
    return;
}

/** @attention this function is a stub and as such is declared as weak. */
__weak FSM_SCARRELLINO_FSM_StateTypeDef FSM_SCARRELLINO_FSM_DONE_event_handle(uint8_t event) {
    return FSM_SCARRELLINO_FSM_DONE;
}

/** @attention this function is a stub and as such is declared as weak. */
__weak void FSM_SCARRELLINO_FSM_DONE_entry() {
    return;
}

/** @attention this function is a stub and as such is declared as weak. */
__weak FSM_SCARRELLINO_FSM_StateTypeDef FSM_SCARRELLINO_FSM_DONE_do_work() {
    return FSM_SCARRELLINO_FSM_DONE;
}

/** @attention this function is a stub and as such is declared as weak. */
__weak void FSM_SCARRELLINO_FSM_DONE_exit() {
    return;
}

