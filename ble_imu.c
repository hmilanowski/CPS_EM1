#include <stdint.h>
#include <string.h>
#include "nrf_gpio.h"
#include "ble_imu.h"
#include "ble_srv_common.h"
#include "app_error.h"


void ble_imu_on_ble_evt(ble_imu_t * p_imu, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            p_imu->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            p_imu->conn_handle = BLE_CONN_HANDLE_INVALID;
        default:
            break;
    }
}

/**@brief Function for adding our new characterstic to the service 
 *
 * @param[in]   p_imu        imu structure.
 *
 */
static uint32_t ble_char_accel_add(ble_imu_t * p_imu)
{
    uint32_t   err_code = 0;
    
    ble_uuid_t          char_uuid;   
    BLE_UUID_BLE_ASSIGN(char_uuid, BLE_UUID_ACCEL_CHARACTERISTC_UUID);
    
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;
    
    ble_gatts_attr_md_t cccd_md;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
        
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.vloc = BLE_GATTS_VLOC_STACK;    
    
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    attr_char_value.max_len     = sizeof(accel_values_t);
    attr_char_value.init_len    = sizeof(accel_values_t);
    uint8_t value[6]            = {0};
    attr_char_value.p_value     = value;

    err_code = sd_ble_gatts_characteristic_add(p_imu->service_handle,
                                       &char_md,
                                       &attr_char_value,
                                       &p_imu->accel_char_handles);
    APP_ERROR_CHECK(err_code);   

    return NRF_SUCCESS;
}


/**@brief Function for initiating our new service.
 *
 * @param[in]   p_imu        service structure.
 *
 */
void ble_imu_service_init(ble_imu_t * p_imu)
{
  uint32_t   err_code;

  ble_uuid_t        service_uuid;
  ble_uuid128_t     base_uuid = {BLE_UUID_BASE_UUID};
  service_uuid.uuid = BLE_UUID_IMU_SERVICE_UUID;
  err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
  APP_ERROR_CHECK(err_code);    

  p_imu->conn_handle = BLE_CONN_HANDLE_INVALID;

  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_imu->service_handle);
    
  APP_ERROR_CHECK(err_code);
    
  ble_char_accel_add(p_imu);
}

void ble_imu_update(ble_imu_t *p_imu, accel_values_t * accel_values)
{
  // Send value if connected and notifying
  if (p_imu->conn_handle != BLE_CONN_HANDLE_INVALID)
  {
    uint16_t               len = sizeof(accel_values_t);
    ble_gatts_hvx_params_t hvx_params;
  
    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_imu->accel_char_handles.value_handle;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    hvx_params.p_len  = &len;
    hvx_params.p_data = (uint8_t*)accel_values;  

    sd_ble_gatts_hvx(p_imu->conn_handle, &hvx_params);
  } 

}