/*
 * DAC Voltage Sweep with INA219 Current/Voltage Monitoring
 * Commands: "start" - begin voltage sweep from 3.3V to 0V
 * Simplified UART reception using polling
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>

LOG_MODULE_REGISTER(dac_sweep, LOG_LEVEL_INF);

/* DAC Configuration */
#define DAC_RESOLUTION_BITS 12
#define DAC_MAX_VALUE ((1 << DAC_RESOLUTION_BITS) - 1)  /* 4095 */
#define DAC_STEP_SIZE 2                                /* DAC steps per decrement */
#define STEP_DELAY_MS 1                                 /* 1ms delay between steps for max speed */

/* INA219 I2C Configuration */
#define INA219_I2C_ADDR 0x40        /* Default INA219 address */
#define INA219_REG_CONFIG 0x00
#define INA219_REG_SHUNT_VOLTAGE 0x01
#define INA219_REG_BUS_VOLTAGE 0x02
#define INA219_REG_POWER 0x03
#define INA219_REG_CURRENT 0x04
#define INA219_REG_CALIBRATION 0x05

/* UART Buffer - Simplified */
#define UART_BUF_SIZE 16
static char uart_rx_buf[UART_BUF_SIZE];
static int uart_rx_pos = 0;

/* Device handles */
static const struct device *dac_dev;
static const struct device *i2c_dev;
static const struct device *uart_dev;

/* INA219 Functions */
static int ina219_write_reg(uint8_t reg, uint16_t value)
{
    uint8_t data[3] = {reg, (value >> 8) & 0xFF, value & 0xFF};
    return i2c_write(i2c_dev, data, 3, INA219_I2C_ADDR);
}

static int ina219_read_reg(uint8_t reg, uint16_t *value)
{
    uint8_t data[2];
    int ret;
    
    ret = i2c_write_read(i2c_dev, INA219_I2C_ADDR, &reg, 1, data, 2);
    if (ret == 0) {
        *value = (data[0] << 8) | data[1];
    }
    return ret;
}

static int ina219_init(void)
{
    uint16_t config;
    int ret;
    
    /* Configure INA219 for maximum speed */
    /* BRNG=1 (32V), PG=3 (320mV), BADC=0 (9-bit), SADC=0 (9-bit), MODE=7 (continuous) */
    config = 0x3807;  /* 32V range, ±320mV shunt, 9-bit (fast), continuous */
    ret = ina219_write_reg(INA219_REG_CONFIG, config);
    if (ret != 0) {
        LOG_ERR("Failed to configure INA219");
        return ret;
    }
    
    /* Set calibration for 0.1 ohm shunt resistor */
    /* Calibration = 0.04096 / (Current_LSB * R_shunt) */
    /* Assuming 0.1 ohm shunt, max 3.2A, Current_LSB = 0.1mA */
    uint16_t cal_value = 4096;  /* Adjust based on your shunt resistor */
    ret = ina219_write_reg(INA219_REG_CALIBRATION, cal_value);
    if (ret != 0) {
        LOG_ERR("Failed to set INA219 calibration");
        return ret;
    }
    
    LOG_INF("INA219 initialized successfully");
    return 0;
}

static int ina219_read_measurements(float *bus_voltage, float *current)
{
    uint16_t bus_raw, current_raw;
    int ret;
    
    /* Read bus voltage register */
    ret = ina219_read_reg(INA219_REG_BUS_VOLTAGE, &bus_raw);
    if (ret != 0) {
        return ret;
    }
    
    /* Read current register */
    ret = ina219_read_reg(INA219_REG_CURRENT, &current_raw);
    if (ret != 0) {
        return ret;
    }
    
    /* Convert to actual values */
    /* Bus voltage: LSB = 4mV, shift right 3 bits */
    *bus_voltage = ((bus_raw >> 3) * 4) / 1000.0f;  /* Convert mV to V */
    
    /* Current: LSB = 0.1mA (depends on calibration) */
    *current = (int16_t)current_raw * 0.1f;  /* Convert to mA, handle signed */
    
    return 0;
}

/* Simplified UART input check using polling */
static bool check_start_command(void)
{
    uint8_t c;
    
    /* Poll for incoming character */
    if (uart_poll_in(uart_dev, &c) == 0) {
        /* Character received */
        if (c == '\r' || c == '\n') {
            /* End of command */
            uart_rx_buf[uart_rx_pos] = '\0';
            
            /* Check for "start" command */
            if (strncmp(uart_rx_buf, "start", 5) == 0) {
                uart_rx_pos = 0;  /* Reset buffer */
                return true;
            }
            
            uart_rx_pos = 0;  /* Reset buffer for next command */
        } else if (uart_rx_pos < UART_BUF_SIZE - 1) {
            /* Add character to buffer */
            uart_rx_buf[uart_rx_pos++] = c;
        } else {
            /* Buffer full, reset */
            uart_rx_pos = 0;
        }
    }
    
    return false;
}

/* DAC and measurement sweep function */
static void voltage_sweep(void)
{
    struct dac_channel_cfg dac_cfg;
    int16_t dac_value = DAC_MAX_VALUE;  /* Start at maximum (3.3V) */
    float bus_voltage, current;
    int ret;
    
    /* Configure DAC */
    dac_cfg.channel_id = 0;
    dac_cfg.resolution = DAC_RESOLUTION_BITS;
    dac_cfg.buffered = false;
    dac_cfg.internal = false;
    
    ret = dac_channel_setup(dac_dev, &dac_cfg);
    if (ret != 0) {
        LOG_ERR("Failed to setup DAC channel: %d", ret);
        return;
    }
    
    printf("=== Voltage Sweep Started ===\n");
    printf("DAC_Value, DAC_Voltage(V), Bus_Voltage(V), Current(mA)\n");
    
    while (dac_value > 0) {
        /* Set DAC output */
        ret = dac_write_value(dac_dev, 0, dac_value);
        if (ret != 0) {
            LOG_ERR("Failed to write DAC value: %d", ret);
            break;
        }
        
        /* Small delay for DAC to settle */
        k_sleep(K_MSEC(10));
        
        /* Read INA219 measurements */
        ret = ina219_read_measurements(&bus_voltage, &current);
        if (ret == 0) {
            /* Calculate expected DAC voltage */
            float dac_voltage = (dac_value * 3.3f) / DAC_MAX_VALUE;
            
            /* Print measurements */
            printf("%d, %.3f, %.3f, %.1f\n", 
                   dac_value, dac_voltage, bus_voltage, current);
        } else {
            LOG_ERR("Failed to read INA219: %d", ret);
        }
        
        /* Decrement DAC value */
        dac_value -= DAC_STEP_SIZE;
        if (dac_value < 0) dac_value = 0;  /* Ensure we hit exactly 0 */
        
        /* Wait for next step */
        k_sleep(K_MSEC(STEP_DELAY_MS));
    }
    
    printf("=== Voltage Sweep Completed ===\n");
}

int main(void)
{
    int ret;
    
    /* Initialize devices */
    dac_dev = DEVICE_DT_GET(DT_NODELABEL(mcp4725));
    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));  /* Adjust for your I2C bus */
    uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    
    /* Check device readiness */
    if (!device_is_ready(dac_dev)) {
        LOG_ERR("DAC device not ready");
        return -1;
    }
    
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return -1;
    }
    
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART device not ready");
        return -1;
    }
    
    LOG_INF("All devices ready");
    
    /* Initialize INA219 */
    ret = ina219_init();
    if (ret != 0) {
        LOG_ERR("INA219 initialization failed");
        return -1;
    }
    
    printf("\n=== DAC Voltage Sweep with INA219 Monitor ===\n");
    printf("Type 'start' and press Enter to begin voltage sweep\n");
    printf("Sweep: 3.3V -> 0V, Step delay: %dms\n", STEP_DELAY_MS);
    printf("Ready for commands...\n");
    
    while (1) {
        /* Check for start command using simple polling */
        if (check_start_command()) {
            printf("Starting voltage sweep...\n");
            voltage_sweep();
            printf("Ready for next command...\n");
        }
        
        k_sleep(K_MSEC(10));  /* Small delay to prevent busy waiting */
    }
    
    return 0;
}