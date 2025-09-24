#include "esc_hw4.h"

#include <math.h>
#include <stdio.h>

#include "auto_offset.h"
#include "cell_count.h"
#include "pico/stdlib.h"
#include "uart.h"
#include "uart_pio.h"

#define TIMEOUT_US 2000
#define PACKET_LENGHT 19
#define SIGNATURE_LENGHT 13
#define SIGNATURE_READ_COUNT 20
#define INIT_DELAY 15000
#define CELL_COUNT_DELAY 15000
#define CURRENT_OFFSET_DELAY 15000
#define NTC_BETA 3950.0
#define NTC_R1 10000.0
#define NTC_R_REF 47000.0
#define DIFFAMP_SHUNT (0.25 / 1000)
#define V_REF 3.3
#define ADC_RES 4096.0

static void process(esc_hw4_parameters_t *parameter, int *current_raw_offset, uint *current_raw, float *voltage_scale, float *current_scale);
float get_voltage(uint16_t voltage_raw, float v_scale);
float get_temperature(uint16_t temperature_raw);
float get_current(uint current_raw, int offset, float i_scale);

void esc_hw4_task(void *parameters) {
    esc_hw4_parameters_t parameter = *(esc_hw4_parameters_t *)parameters;
    *parameter.rpm = 0;
    *parameter.voltage = 0;
    *parameter.current = 0;
    *parameter.temperature_fet = 0;
    *parameter.temperature_bec = 0;
    *parameter.cell_voltage = 0;
    *parameter.consumption = 0;
    *parameter.cell_count = 1;
    xTaskNotifyGive(context.receiver_task_handle);
#ifdef SIM_SENSORS
    *parameter.rpm = 12345.67;
    *parameter.consumption = 123.4;
    *parameter.voltage = 12.34;
    *parameter.current = 5.678;
    *parameter.temperature_fet = 12.34;
    *parameter.temperature_bec = 23.45;
    *parameter.cell_voltage = 3.75;
#endif

    if (parameter.init_delay) vTaskDelay(INIT_DELAY / portTICK_PERIOD_MS);

    TaskHandle_t task_handle;
    uint cell_count_delay = CELL_COUNT_DELAY;
    cell_count_parameters_t cell_count_parameters = {cell_count_delay, parameter.voltage, parameter.cell_count};
    xTaskCreate(cell_count_task, "cell_count_task", STACK_CELL_COUNT, (void *)&cell_count_parameters, 1, &task_handle);
    xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

    int current_raw_offset = -1;
    uint current_raw = 0;
	float voltage_scale = 0.0;
	float current_scale = 0.0;
	if ((int)parameter.divisor) {
        voltage_scale = V_REF / ADC_RES * parameter.divisor;
    }
    if ((int)parameter.current_multiplier) {
        current_scale = V_REF / ADC_RES * parameter.current_multiplier;
	}
    if (parameter.current_is_manual_offset) {
        current_raw_offset = parameter.current_offset * ADC_RES / V_REF / parameter.current_multiplier;
        if (current_raw_offset > ADC_RES) current_raw_offset = ADC_RES;
    } else {
        uint current_delay = CURRENT_OFFSET_DELAY;
        auto_offset_int_parameters_t current_offset_parameters = {current_delay, &current_raw, &current_raw_offset};
        xTaskCreate(auto_offset_int_task, "esc_hw4_current_offset_task", STACK_AUTO_OFFSET, &current_offset_parameters,
                    1, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
    }

    uart1_begin(19200, UART1_TX_GPIO, UART_ESC_RX, TIMEOUT_US, 8, 1, UART_PARITY_NONE, false, false);

    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process(&parameter, &current_raw_offset, &current_raw, &voltage_scale, &current_scale);
    }
}

static void process(esc_hw4_parameters_t *parameter, int *current_raw_offset, uint *current_raw, float *voltage_scale, float *current_scale) {
    uint16_t pwm, throttle;
    static uint32_t timestamp = 0;
	static int signature_read_count = 0;
    uint8_t lenght = uart1_available();
    if (lenght == SIGNATURE_LENGHT && signature_read_count < SIGNATURE_READ_COUNT) {
        uint8_t data[SIGNATURE_LENGHT];
        uart1_read_bytes(data, SIGNATURE_LENGHT);
		if (data[1] == 0x9B && data[4] == 0x01 && data[12] == 0xB9) {
			if (!(int)parameter->divisor && !(int)*voltage_scale) {
                if (data[5] && data[6])
                    *voltage_scale = (float)data[5] / (float)data[6] / 10;
            }
            if (!(int)parameter->current_multiplier && !(int)*current_scale) {
                if (data[7] && data[8]) {
                    *current_scale = (float)data[7] / (float)data[8];
                    *current_raw_offset = (float)data[9] / *current_scale;
                }
            }
            debug("\nEsc HW4 signature (%u) < SignatureReadCount: %i VScale: %f CRawOffset: %i IScale: %f\n",
				uxTaskGetStackHighWaterMark(NULL), signature_read_count, *voltage_scale, *current_raw_offset, *current_scale);
            debug_buffer(data, SIGNATURE_LENGHT, "0x%X ");
			signature_read_count++;
        } else {
            debug("\nEsc HW4 signature packet error (%u): ", uxTaskGetStackHighWaterMark(NULL));
            debug_buffer(data, SIGNATURE_LENGHT, "0x%X ");
		}
	}
    if (lenght == PACKET_LENGHT || lenght == PACKET_LENGHT + 1) {
        uint8_t data[PACKET_LENGHT];
        uart1_read_bytes(data, PACKET_LENGHT);
        throttle = (uint16_t)data[4] << 8 | data[5];  // 0-1024
        pwm = (uint16_t)data[6] << 8 | data[7];       // 0-1024
        float rpm = (uint32_t)data[8] << 16 | (uint16_t)data[9] << 8 | data[10];
        // try to filter invalid data frames
        if (throttle < 1024 && pwm < 1024 && rpm < 200000 && data[11] <= 0xF && data[13] <= 0xF && data[15] <= 0xF &&
            data[17] <= 0xF) {
            *current_raw = (uint16_t)data[13] << 8 | data[14];
            float voltage = get_voltage((uint16_t)data[11] << 8 | data[12], *voltage_scale);
            float current = 0;
            if (throttle > parameter->current_thresold / 100.0 * 1024 && *current_raw_offset != -1) {
                current = get_current(*current_raw, *current_raw_offset, *current_scale);
                if (current > parameter->current_max) current = parameter->current_max;
            }
            float temperature_fet = get_temperature((uint16_t)data[15] << 8 | data[16]);
            float temperature_bec = get_temperature((uint16_t)data[17] << 8 | data[18]);
            rpm *= parameter->rpm_multiplier;
            if (parameter->pwm_out) xTaskNotifyGive(context.pwm_out_task_handle);
            *parameter->rpm = get_average(parameter->alpha_rpm, *parameter->rpm, rpm);
            if (*current_raw_offset != -1)
                *parameter->consumption += get_consumption(*parameter->current, parameter->current_max, &timestamp);
            *parameter->voltage = get_average(parameter->alpha_voltage, *parameter->voltage, voltage);
            *parameter->current = get_average(parameter->alpha_current, *parameter->current, current);
            *parameter->temperature_fet =
                get_average(parameter->alpha_temperature, *parameter->temperature_fet, temperature_fet);
            *parameter->temperature_bec =
                get_average(parameter->alpha_temperature, *parameter->temperature_bec, temperature_bec);
            *parameter->cell_voltage = *parameter->voltage / *parameter->cell_count;
            uint32_t packet = (uint32_t)data[1] << 16 | (uint16_t)data[2] << 8 | data[3];
            debug(
                "\nEsc HW4 (%u) < Packet: %i Rpm: %.0f Volt: %0.2f Curr: %.2f TempFet: %.0f TempBec: %.0f Cons: %.0f "
                "CellV: %.2f CRaw: %i CRawOffset: %i CurrMult: %.2f",
                uxTaskGetStackHighWaterMark(NULL), packet, *parameter->rpm, *parameter->voltage, *parameter->current,
                *parameter->temperature_fet, *parameter->temperature_bec, *parameter->consumption,
                *parameter->cell_voltage, *current_raw, *current_raw_offset, parameter->current_multiplier);
        } else {
            debug("\nEsc HW4 packet error (%u): ", uxTaskGetStackHighWaterMark(NULL));
            debug_buffer(data, PACKET_LENGHT, "0x%X ");
        }
    }
}

float get_voltage(uint16_t voltage_raw, float v_scale) {
    return voltage_raw * v_scale;
}

float get_temperature(uint16_t temperature_raw) {
    float voltage = temperature_raw * V_REF / ADC_RES;
    float ntcR_Rref = (voltage * NTC_R1 / (V_REF - voltage)) / NTC_R_REF;
    if (ntcR_Rref < 0.001) return 0;
    float temperature = 1 / (log(ntcR_Rref) / NTC_BETA + 1 / 298.15) - 273.15;
    if (temperature < 0) return 0;
    return temperature;
}

float get_current(uint current_raw, int offset, float i_scale) {
    // float current = (*parameter->current_raw - *parameter->current_offset) * V_REF / (parameter->ampgain *
    // DIFFAMP_SHUNT * ADC_RES);
    if ((int)current_raw - offset < 0) return 0;
    return ((int)current_raw - offset) * i_scale;
}
