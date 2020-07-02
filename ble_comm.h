#ifndef _BLE_COMM_H_
#define _BLE_COMM_H_  
    

void ble_comm_timers_init(void);
void ble_stack_init(void);
void gap_params_init(void);
void gatt_init(void);
void advertising_init(void);
void services_init(void);

void sensor_simulator_init(void);
void conn_params_init(void);
void peer_manager_init(void);

void ble_comm_timers_start();
void advertising_start(bool p_erase_bonds);

void buttons_leds_init(bool * p_erase_bonds);

#endif 