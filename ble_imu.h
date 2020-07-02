#ifndef IMU_SERVICE_H_
#define IMU_SERVICE_H_

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "UM7_drv.h"


#define BLE_UUID_BASE_UUID  {0x5F, 0xE9, 0x18, 0xEE, 0x5F, 0x79, 0x23, 0x15, 0xDE, 0x8B, 0x72, 0x06, 0x00, 0x00, 0x00, 0x00} // 128-bit base UUID
#define BLE_UUID_IMU_SERVICE_UUID                  0xF00D
#define BLE_UUID_ACCEL_CHARACTERISTC_UUID          0xACCE
#define BLE_UUID_GYRO_CHARACTERISTIC_UUID          0xABBE


typedef struct
{
    uint16_t                    conn_handle;          /* Handle of the current connection.*/
    uint16_t                    service_handle;       /* Handle of ble service. */
    ble_gatts_char_handles_t    accel_char_handles;   /* Handles related to the new characteristic. */
}ble_imu_t;


/**@brief Function for handling BLE Stack events related to imu service and characteristic.
 *
 * @details Handles all events from the BLE stack of interest to imu service.
 *
 * @param[in]   p_imu      imu structure.
 * @param[in]   p_ble_evt  event received from the BLE stack.
 */
void ble_imu_on_ble_evt(ble_imu_t * p_imu, ble_evt_t * p_ble_evt);

/**@brief Function for initializing our new service.
 *
 * @param[in]   p_imu       pointer to ble imu structure.
 */
void ble_imu_service_init(ble_imu_t * p_imu);

/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_imu                    imu structure.
 * @param[in]   characteristic_value     new characteristic value.
 */
void ble_imu_update(ble_imu_t *p_imu, accel_values_t * accel_values);

#endif  /* _ IMU_SERVICE_H_ */
