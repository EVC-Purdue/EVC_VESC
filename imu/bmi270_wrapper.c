/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "bmi270_wrapper.h"
#include "utils_math.h"

#include <stdio.h>
#include <string.h>

/*! Macros to select the sensors                   */
#define ACCEL          UINT8_C(0x00)
#define GYRO           UINT8_C(0x01)

// Threads
static THD_FUNCTION(bmi_thread, arg);

// Private functions
static bool reset_init_bmi(BMI270_STATE *s);

void bmi270_wrapper_init(BMI270_STATE *s, stkalign_t *work_area, size_t work_area_size) {
	s->read_callback = 0;

	if (s->sensor.intf == BMI2_SPI_INTF) {
		s->rate_hz = MIN(s->rate_hz, 5000);
	} else {
		s->rate_hz = MIN(s->rate_hz, 1000);
	}

	if (reset_init_bmi(s)) {
		s->should_stop = false;
		chThdCreateStatic(work_area, work_area_size, NORMALPRIO, bmi_thread, s);
	}
}

void bmi270_wrapper_set_read_callback(BMI270_STATE *s, void(*func)(float *accel, float *gyro, float *mag)) {
	s->read_callback = func;
}

void bmi270_wrapper_stop(BMI270_STATE *s) {
	s->should_stop = true;
	while(s->is_running) {
		chThdSleep(1);
	}
}

static void bmi2_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
	chThdSleepMicroseconds(period);
}

// Initialize BMI270 Device
static bool reset_init_bmi(BMI270_STATE *s) {
    int8_t result;
	struct bmi2_dev *dev = &(s->sensor);

	s->sensor.delay_us = (bmi2_delay_fptr_t)bmi2_delay_us;

    // Initialize the device
    result = bmi270_init(dev);
    if (result != BMI2_OK) {
        return false;
    }

    // Configure accelerometer and gyroscope
    struct bmi2_sens_config config[2];
    config[ACCEL].type = BMI2_ACCEL;
    config[GYRO].type = BMI2_GYRO;

    result = bmi2_get_sensor_config(config, 2, dev);
    if (result != BMI2_OK) {
        return false;
    }

    config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_16G;
    config[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;

	if(s->rate_hz <= 25){
		config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_25HZ;
		config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_25HZ;
	}else if(s->rate_hz <= 50){
		config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_50HZ;
		config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_50HZ;
	}else if(s->rate_hz <= 100){
		config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
		config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
	}else if(s->rate_hz <= 200){
		config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_200HZ;
		config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;
	}else if(s->rate_hz <= 400){
		config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_400HZ;
		config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_400HZ;
	}else if(s->rate_hz <= 800){
		config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_800HZ;
		config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_800HZ;
	}else if(s->rate_hz <= 1600){
		config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_1600HZ;
		config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_1600HZ;
	}else{
		config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_1600HZ;
		config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_3200HZ;
	}

	if(s->filter == IMU_FILTER_LOW){
		config[ACCEL].cfg.acc.bwp = BMI2_ACC_CIC_AVG8;
		config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
		config[GYRO].cfg.gyr.bwp = BMI2_GYR_OSR4_MODE;
		config[GYRO].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
		config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
	}else if(s->filter == IMU_FILTER_MEDIUM){
		config[ACCEL].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
		config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
		config[ACCEL].cfg.acc.odr = fmin(config[ACCEL].cfg.acc.odr + 1, BMI2_ACC_ODR_1600HZ);
		config[GYRO].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
		config[GYRO].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
		config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
		config[GYRO].cfg.gyr.odr = fmin(config[GYRO].cfg.gyr.odr + 1, BMI2_GYR_ODR_3200HZ);

	}else if(s->filter == IMU_FILTER_HIGH){
		config[ACCEL].cfg.acc.bwp = BMI2_ACC_OSR4_AVG1;
		config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
		config[ACCEL].cfg.acc.odr = fmin(config[ACCEL].cfg.acc.odr + 2, BMI2_ACC_ODR_1600HZ);
		config[GYRO].cfg.gyr.bwp = BMI2_GYR_CIC_MODE;
		config[GYRO].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
		config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
		config[GYRO].cfg.gyr.odr = fmin(config[GYRO].cfg.gyr.odr + 2, BMI2_GYR_ODR_3200HZ);
	}

    result = bmi2_set_sensor_config(config, 2, dev);

	if (result == BMI2_OK) {
		uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };
		result = bmi2_sensor_enable(sensor_list, 2, dev);
	}

	return result == BMI2_OK;
}

static THD_FUNCTION(bmi_thread, arg) {
	BMI270_STATE *s = (BMI270_STATE*)arg;
	struct bmi2_dev *dev = &(s->sensor);

    struct bmi2_sens_data sensor_data[2];

	chRegSetThreadName("BMI270 Sampling");

	s->is_running = true;

	systime_t iteration_timer = chVTGetSystemTimeX();
	const systime_t desired_interval = US2ST(1000000 / s->rate_hz);

	for(;;) {
        int8_t result = bmi2_get_sensor_data(&(sensor_data[ACCEL]), dev);
        if (result != BMI2_OK) {
			chThdSleepMilliseconds(5);
			continue;
        }
        result = bmi2_get_sensor_data(&(sensor_data[GYRO]), dev);
        if (result != BMI2_OK) {
			chThdSleepMilliseconds(5);
			continue;
        }

		float tmp_accel[3], tmp_gyro[3], tmp_mag[3];

		tmp_accel[0] = (float)sensor_data[ACCEL].acc.x * 16.0 / 32768.0;
		tmp_accel[1] = (float)sensor_data[ACCEL].acc.y * 16.0 / 32768.0;
		tmp_accel[2] = (float)sensor_data[ACCEL].acc.z * 16.0 / 32768.0;

		tmp_gyro[0] = (float)sensor_data[GYRO].gyr.x * 2000.0 / 32768.0;
		tmp_gyro[1] = (float)sensor_data[GYRO].gyr.y * 2000.0 / 32768.0;
		tmp_gyro[2] = (float)sensor_data[GYRO].gyr.z * 2000.0 / 32768.0;

		memset(tmp_mag, 0, sizeof(tmp_mag));

		if (s->read_callback) {
			s->read_callback(tmp_accel, tmp_gyro, tmp_mag);
		}

		if (s->should_stop) {
			s->is_running = false;
			return;
		}

		// Delay between loops
		iteration_timer += desired_interval;
		systime_t current_time = chVTGetSystemTimeX();
		systime_t remainin_sleep_time = iteration_timer - current_time;
		if (remainin_sleep_time > 0 && remainin_sleep_time < desired_interval) {
			// Sleep the remaining time.
			chThdSleep(remainin_sleep_time);
		}
		else {
			// Read was too slow or CPU was too buzy, reset the schedule.
			iteration_timer = current_time;
			chThdSleep(desired_interval);
		}
	}
}
