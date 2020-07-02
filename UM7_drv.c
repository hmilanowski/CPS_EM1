#include "nrf_gpio.h"
#include "nrfx_uarte.h"
#include "nrfx_timer.h"

#include "nrf_delay.h"

#include "UM7_drv.h"
#include "UM7_RegisterMap.h"


/* NR52 -> IMU SENSOR PINS */
#define UARTE_TX_PIN NRF_GPIO_PIN_MAP(0, 26)
#define UARTE_RX_PIN NRF_GPIO_PIN_MAP(0, 25)

/* UARTE INSTANCE ID */
#define UARTE_INSTANCE_ID 0

/* The buffer length [UARTE_RX_LENGTH] defines samples no. fits in the buffer. Each sample may contain accelerometer or gyroscope data. */
#define UARTE_RX_BUF_LENGTH  6
#define UARTE_TX_BUF_LENGTH  6

/* The buffer width, UARTE_RX_BUF_WIDTH, UARTE_TX_BUF_WIDTH, defines how much and what kind of data to read out/transfer for every sample. */
#define UM7_RX_BYTE_SIZE   11
#define UM7_TX_BYTE_SIZE   7

#define UM7_CMD_REG_COM_LEN 11
#define UM7_DATA_REG_COM_LEN 7

/* Define a type with a two dimensioanal array, TWIM_RX_BUF_WIDTH wide and TWIM_RX_BUF_LENGTH long, holding a list of imu sensor data */
typedef struct
{
  uint8_t buffer[UM7_TX_BYTE_SIZE];
} tx_array_list_t;

/* Define a type with a two dimensioanal array, TWIM_RX_BUF_WIDTH wide and TWIM_RX_BUF_LENGTH long, holding a list of imu sensor data */
typedef struct
{
  uint8_t buffer[UM7_RX_BYTE_SIZE];
} rx_array_list_t;

/* Peripheral instance init - UART with DMA and cyclic timer */
static const nrfx_uarte_t m_uarte_instance = NRFX_UARTE_INSTANCE(UARTE_INSTANCE_ID);

static nrfx_timer_t IMU_ACQUISITION_TIMER = NRFX_TIMER_INSTANCE(4);

/* Pointer to pass next transmission command to DMA TX Buffer  */
static uint8_t * gui8_rx_buffer_pointer;
static uint8_t * gui8_tx_buffer_pointer;

/* UARTE transmission sequence counter */
static uint8_t gui8_tx_transfer_ctr = 0;
static uint8_t gui8_rx_transfer_ctr = 0;

/* Declare a simple TX buffer holding the first register in MPU we want to read from. */
tx_array_list_t g_tx_buffer[UARTE_TX_BUF_LENGTH] = {
                           {'s', 'n', 'p', 0x00, DREG_GYRO_RAW_XY, ('s'+'n'+'p'+DREG_GYRO_RAW_XY)>>8, ('s'+'n'+'p'+DREG_GYRO_RAW_XY)&0xFF},\
                           {'s', 'n', 'p', 0x00, DREG_GYRO_RAW_Z, ('s'+'n'+'p'+DREG_GYRO_RAW_Z)>>8, ('s'+'n'+'p'+DREG_GYRO_RAW_Z)&0xFF},\
                           {'s', 'n', 'p', 0x00, DREG_GYRO_RAW_TIME, ('s'+'n'+'p'+DREG_GYRO_RAW_TIME)>>8, ('s'+'n'+'p'+DREG_GYRO_RAW_TIME)&0xFF},\
                           {'s', 'n', 'p', 0x00, DREG_ACCEL_RAW_XY, ('s'+'n'+'p'+DREG_ACCEL_RAW_XY)>>8, ('s'+'n'+'p'+DREG_ACCEL_RAW_XY)&0xFF},\
                           {'s', 'n', 'p', 0x00, DREG_ACCEL_RAW_Z, ('s'+'n'+'p'+DREG_ACCEL_RAW_Z)>>8, ('s'+'n'+'p'+DREG_ACCEL_RAW_Z)&0xFF},\
                           {'s', 'n', 'p', 0x00, DREG_ACCEL_RAW_TIME, ('s'+'n'+'p'+DREG_ACCEL_RAW_TIME)>>8, ('s'+'n'+'p'+DREG_ACCEL_RAW_TIME)&0xFF}};

rx_array_list_t g_rx_buffer[UARTE_RX_BUF_LENGTH];

/* Flag to indicate to the applications main context that UARTE_RX_BUF_LENGTH number of samples have been transferred from MPU */
volatile bool gb_um7_read_imu_complete_flag = false;
volatile bool gb_xfer_uarte_error = false;
volatile bool gb_um7_read_imu_task_flag = false;

//volatile accel_values_t m_accel_values;
//volatile gyro_values_t m_gyro_values;


static void uarte_event_handler(const nrfx_uarte_event_t * p_event, void * p_context)
{
  nrfx_err_t error_status;

  switch(p_event->type)
  {
    case NRFX_UARTE_EVT_TX_DONE:
      if(gb_um7_read_imu_task_flag)
      {
        gui8_tx_transfer_ctr++;
        if(gui8_tx_transfer_ctr >= 6)
        {
          gui8_tx_transfer_ctr = 0;
        }
        gui8_tx_buffer_pointer = g_tx_buffer[gui8_tx_transfer_ctr].buffer;
        error_status = nrfx_uarte_rx(&m_uarte_instance, gui8_rx_buffer_pointer, UM7_RX_BYTE_SIZE);
      }
      break;
    case NRFX_UARTE_EVT_RX_DONE:
      if(gb_um7_read_imu_task_flag)
      {
        gui8_rx_transfer_ctr++;
        if(gui8_rx_transfer_ctr >= 5)
        {
          gui8_rx_transfer_ctr = 0;
          gb_um7_read_imu_complete_flag = true;
          gb_um7_read_imu_task_flag = false;
        }
        gui8_rx_buffer_pointer = g_rx_buffer[gui8_rx_transfer_ctr].buffer;
        error_status = nrfx_uarte_tx(&m_uarte_instance, gui8_tx_buffer_pointer, UM7_TX_BYTE_SIZE);
      }
      break;
    case NRFX_UARTE_EVT_ERROR:
      gb_xfer_uarte_error = true;
      gui8_rx_transfer_ctr = 0;
      gui8_tx_transfer_ctr = 0;
      break;
  }
}


static void imu_acquisition_timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
  gb_um7_read_imu_task_flag = true;
  um7_read_imu();
}


/**@brief Function for initiating cyclic timer with interrupt handler.
 */
void um7_drv_timer_imu_acuisition_init(void)
{
  ret_code_t err_code;
  uint32_t imu_acquisition_period_ms = 200;
  uint32_t time_ticks;

  nrfx_timer_config_t timer_cfg = {                                     \
    .frequency          = NRF_TIMER_FREQ_4MHz ,                         \
    .mode               = NRF_TIMER_MODE_TIMER ,                        \
    .bit_width          = NRF_TIMER_BIT_WIDTH_32,                       \
    .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,       \
    .p_context          = NULL                                          \
  };
  err_code = nrfx_timer_init(&IMU_ACQUISITION_TIMER, &timer_cfg, imu_acquisition_timer_event_handler);
  APP_ERROR_CHECK(err_code);

  time_ticks = nrfx_timer_ms_to_ticks(&IMU_ACQUISITION_TIMER, imu_acquisition_period_ms);
  nrfx_timer_extended_compare(&IMU_ACQUISITION_TIMER, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
}


/**@brief Function for initiating uart and DMA transaction.
 */
void um7_drv_uarte_init(void)
{
  ret_code_t err_code;
  const nrfx_uarte_config_t uarte_config = {
      .pseltxd            = UARTE_TX_PIN,                                \
      .pselrxd            = UARTE_RX_PIN,                                \
      .pselcts            = NRF_UARTE_PSEL_DISCONNECTED,                 \
      .pselrts            = NRF_UARTE_PSEL_DISCONNECTED,                 \
      .p_context          = NULL,                                        \
      .hwfc               = NRF_UARTE_HWFC_DISABLED,                     \
      .parity             = NRF_UARTE_PARITY_EXCLUDED,                   \
      .baudrate           = NRF_UARTE_BAUDRATE_115200,                   \
      .interrupt_priority = NRFX_UARTE_DEFAULT_CONFIG_IRQ_PRIORITY,      \
  };

  err_code = nrfx_uarte_init(&m_uarte_instance, &uarte_config, uarte_event_handler);

  APP_ERROR_CHECK(err_code);
}


/**@brief Function for writing command type payload to UM7 sensor through uart with DMA.
 *
 * @param[in]   pui8_data        tx buffer data pointer.
 *
 */
static void um7_write_cmd_reg(const uint8_t * pui8_data)
{
  ret_code_t err_code;

  while(nrfx_uarte_tx_in_progress(&m_uarte_instance));  //check if tx send sth
  err_code = nrfx_uarte_tx(&m_uarte_instance, pui8_data, UM7_CMD_REG_COM_LEN);

  APP_ERROR_CHECK(err_code);
}


/**@brief Function for writing data type payload to UM7 sensor through uart with DMA.
 *
 * @param[in]   pui8_data        tx buffer data pointer.
 *
 */
static void um7_write_data_reg(const uint8_t * pui8_data)
{
  ret_code_t err_code;

  while(nrfx_uarte_tx_in_progress(&m_uarte_instance));  //check if tx send sth
  err_code = nrfx_uarte_tx(&m_uarte_instance, pui8_data, UM7_DATA_REG_COM_LEN);

  APP_ERROR_CHECK(err_code);
}

/**@brief Function for reading data type payload to UM7 sensor through uart with DMA.
 *
 * @param[in]   pui8_data        rx buffer data pointer.
 *
 */
static void um7_read_data_reg(uint8_t * pui8_data)
{
  nrfx_uarte_rx(&m_uarte_instance, pui8_data, UM7_DATA_REG_COM_LEN);
}

/**@brief Function for setting up UM7 module.
 *
 * @details Initializaing module and turning out sequential sent of register data.
 *          Module now sends data only on request from microcontroller.
 */
void um7_init(void)
{
  uint8_t data_reg_tx[] = {'s', 'n', 'p', 0x00, 0xAA, 0x01, 0xFB};
  uint8_t cmd_reg_tx[] = {'s', 'n', 'p', 0x80, 0, 0, 0, 0, 0, 0, 0};
  uint16_t checksum = 0;

  um7_write_data_reg(data_reg_tx);
  nrf_delay_ms(3);
  
  for (uint8_t reg_it = 0x01; reg_it <= 0x07; reg_it++)
  {
    cmd_reg_tx[4] = reg_it;
    checksum = 0;
    for(uint8_t it = 0; it < 9; it++)
    {
      checksum += cmd_reg_tx[it];
    }
    cmd_reg_tx[9] = checksum >> 8;
    cmd_reg_tx[10] = checksum & 0xFF;
    
    um7_write_cmd_reg(cmd_reg_tx);
    nrf_delay_ms(3);
  }
}


/**@brief Function for reading required set of data from module registers - accelerometer and gyrometer.
 *
 * @details Start the first transfer to command payload to DMA and trigger start of uart communication
 *          withUM7 module. The rest of transaction is implemented in interrupt handler.
 */
void um7_read_imu(void)
{
  ret_code_t err_code;
  gb_um7_read_imu_task_flag = true;
  gui8_rx_transfer_ctr = 0;
  gui8_tx_transfer_ctr = 0;

  gui8_tx_buffer_pointer = g_tx_buffer[gui8_tx_transfer_ctr].buffer;
  gui8_rx_buffer_pointer = g_rx_buffer[gui8_rx_transfer_ctr].buffer;

  while(nrfx_uarte_tx_in_progress(&m_uarte_instance));  //check if tx send sth
  err_code = nrfx_uarte_tx(&m_uarte_instance, gui8_tx_buffer_pointer, UM7_TX_BYTE_SIZE);
}


/**@brief Function for reading accelerometer data from specific RAM addres.
 *
 * @details Read data and covert it to real value.
 */
void um7_read_accel_from_ram(accel_values_t * accel_values)
{
  accel_values->x = g_rx_buffer[0].buffer[7]|g_rx_buffer[0].buffer[8];
  accel_values->y = g_rx_buffer[0].buffer[5]|g_rx_buffer[0].buffer[6];
  accel_values->z = g_rx_buffer[1].buffer[7]|g_rx_buffer[1].buffer[8];
  accel_values->timestamp = g_rx_buffer[2].buffer[5]|g_rx_buffer[2].buffer[6]|g_rx_buffer[2].buffer[7]|g_rx_buffer[2].buffer[8];
}


/**@brief Function for reading gyrometer data from specific RAM addres.
 *
 * @details Read data and covert it to real value.
 */
void um7_read_gyro_from_ram(gyro_values_t * gyro_values)
{
  gyro_values->x = g_rx_buffer[3].buffer[7]|g_rx_buffer[0].buffer[8];
  gyro_values->y = g_rx_buffer[3].buffer[5]|g_rx_buffer[0].buffer[6];
  gyro_values->z = g_rx_buffer[4].buffer[7]|g_rx_buffer[0].buffer[8];
  gyro_values->timestamp = g_rx_buffer[5].buffer[5]|g_rx_buffer[5].buffer[6]|g_rx_buffer[5].buffer[7]|g_rx_buffer[5].buffer[8];
}


/**@brief Function for starting sequential read from UM7 module.
 *
 * @details Start timer responsible for sequentiall read of data from UM7 sensor.
 */
void um7_cyclic_acquisition_start()
{
  nrfx_timer_enable(&IMU_ACQUISITION_TIMER);
}


/**@brief Function for suspend sequential read from UM7 module.
 *
 * @details Suspend timer responsible for sequentiall read of data from UM7 sensor.
 */
void um7_cyclic_acquisition_suspend()
{
  nrfx_timer_disable(&IMU_ACQUISITION_TIMER);
}

