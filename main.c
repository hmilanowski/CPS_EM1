#include <stdint.h>
#include <string.h>
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "nrf_ble_lesc.h"
#include "nrf_pwr_mgmt.h"

#include "ble_comm.h"
#include "ble_imu.h"
#include "UM7_drv.h"

extern ble_imu_t m_imu;
extern volatile bool gb_um7_read_imu_complete_flag;
accel_values_t m_accel_values;
gyro_values_t m_gyro_values;

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_lesc_request_handler();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;

    // Initialize.
    ble_comm_timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();

    sensor_simulator_init();
    conn_params_init();
    peer_manager_init();

    um7_drv_uarte_init();
    um7_drv_timer_imu_acuisition_init();
    um7_init();

    // Start execution.
    ble_comm_timers_start();
    advertising_start(erase_bonds);
    um7_cyclic_acquisition_start();

    // Enter main loop.
    for (;;)
    {
      idle_state_handle();
      if(gb_um7_read_imu_complete_flag)
      {
        um7_read_accel_from_ram(&m_accel_values);
        um7_read_gyro_from_ram(&m_gyro_values);
        //send preapre and send ble characteriostic 
//        ble_imu_update(&m_imu, &m_accel_values);
        gb_um7_read_imu_complete_flag = false;
      }
    }
}


