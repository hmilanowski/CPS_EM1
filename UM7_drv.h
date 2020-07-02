#ifndef _UM7_DRV_H_
#define _UM7_DRV_H_

#include "stdint.h"

/**@brief Structure to hold acceleromter values. 
 * Sequence of x, y, and z is important to correspond with 
 * the sequence of which x, y, and z data are read from the sensor.
 * All values are unsigned 16 bit integers
*/
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
    int32_t timestamp;
}accel_values_t;


/**@brief Structure to hold gyroscope values. 
 * Sequence of x, y, and z is important to correspond with 
 * the sequence of which x, y, and z data are read from the sensor.
 * Time of read the data by UM7 sensor.
 * All values are unsigned 16 bit integers, timestamp is unsigned 32 but integer.
*/
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
    int32_t timestamp;
}gyro_values_t;

/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_imu                    imu structure.
 * @param[in]   characteristic_value     new characteristic value.
 */
void um7_drv_uarte_init(void);
void um7_drv_timer_imu_acuisition_init(void);

void um7_init(void);

void um7_uarte_tx(const uint8_t * pui8_data, uint8_t ui8_length);
void um7_uarte_rx(uint8_t * p_data, uint8_t ui8_length);

void um7_cyclic_acquisition_start(void);
void um7_cyclic_acquisition_suspend(void);
void um7_read_imu(void);

void um7_read_accel_from_ram(accel_values_t * accel_values);
void um7_read_gyro_from_ram(gyro_values_t * gyro_values);


#endif // _UM7_H_
