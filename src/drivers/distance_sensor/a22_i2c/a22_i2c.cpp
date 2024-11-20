/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file a22_i2c.cpp
 * @author Michael Salino-Hugg
 *
 * Driver for the Maxbotix sonar range finders connected via I2C.
 */

#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/device/device.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <perf/perf_counter.h>

using namespace time_literals;

#define A22_I2C_BASE_ADDR                        0x74   // 7-bit address is 0x74. 8-bit address is 0xE8.
#define A22_I2C_BUS_SPEED                        100000 // 100kHz bus speed.


#define A22_I2C_REG_VERSION 			0x00
#define A22_I2C_REG_DISTANCE_VALUE 		0x02
#define A22_I2C_REG_ADDR			0x05
#define A22_I2C_REG_NOISE_REDUCTION_LEVEL 	0x06
#define A22_I2C_REG_ANGLE_LEVEL 		0x07
#define A22_I2C_REG_TEMPERATURE 		0x0A
#define A22_I2C_REG_INSTRUCTION_CTRL 		0x10

#define A22_I2C_CMD_50_CM_DIST			0xBD
#define A22_I2C_CMD_150_CM_DIST			0xBC
#define A22_I2C_CMD_250_CM_DIST			0xB8
#define A22_I2C_CMD_350_CM_DIST			0xB4
#define A22_I2C_CMD_50_CM_TIME			0x05
#define A22_I2C_CMD_150_CM_TIME			0x0A
#define A22_I2C_CMD_250_CM_TIME			0x0F
#define A22_I2C_CMD_350_CM_TIME			0xB2
#define A22_I2C_CMD_RESET			0x5A

#define A22_MIN_DISTANCE (0.02f)
#define A22_MAX_DISTANCE (3.50f)

// Normal conversion wait time.
static constexpr uint32_t A22_CONVERSION_INTERVAL{110_ms};

// Maximum time to wait for a conversion to complete.
static constexpr uint32_t A22_CONVERSION_TIMEOUT{150_ms};

class A22I2c : public device::I2C, public I2CSPIDriver<A22I2c>
{
public:
	A22I2c(const I2CSPIDriverConfig &config);
	virtual ~A22I2c();

	static void print_usage();

	int init();

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void start();


	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void RunImpl();

protected:

	uint32_t get_measure_interval() const { return A22_CONVERSION_INTERVAL; };

	int measure();

private:
	int collect();

	bool _collect_phase{false};
	uint64_t _acquire_time_usec{0};
	PX4Rangefinder	_px4_rangefinder;

	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, "a22_i2c: read")};
};

A22I2c::A22I2c(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_px4_rangefinder(get_device_id(), config.rotation)
{
	_px4_rangefinder.set_min_distance(A22_MIN_DISTANCE);
	_px4_rangefinder.set_max_distance(A22_MAX_DISTANCE);

	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_A22); /// TODO
}

A22I2c::~A22I2c()
{
	perf_free(_sample_perf);
}

int
A22I2c::init()
{
	// Perform I2C init (and probe) first.
	if (I2C::init() != PX4_OK) {
		return PX4_ERROR;
	}

	start();
	return PX4_OK;
}

void
A22I2c::print_usage()
{
	PRINT_MODULE_USAGE_NAME("a22_i2c", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_COMMAND("set_address");
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(A22_I2C_BASE_ADDR);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

int
A22I2c::collect(){
	uint8_t val[2] {};
	int ret = PX4_OK;
	perf_begin(_sample_perf);

	// this should be fairly close to the end of the measurement, so the best approximation of the time
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	const uint8_t cmd[1] = { A22_I2C_REG_DISTANCE_VALUE};
	ret = transfer(&cmd[0], 1, nullptr, 0);
	if (ret == PX4_OK) {
		ret = transfer(nullptr, 0, &val[0], 2);
	}
	if (ret != PX4_OK) {
		return ret;
	}
	uint16_t distance_mm = (((uint16_t) val[0]) << 8) | (((uint16_t) val[1]));
	float distance_m = (float)distance_mm / 1000.0f;
	if (distance_m > 3.6f) {
		//don't update if greater than max distance
		return PX4_OK;
	}

	_px4_rangefinder.update(timestamp_sample, distance_m, 0);

	perf_end(_sample_perf);
	return PX4_OK;
}

int
A22I2c::measure(){
	const uint8_t cmd[2] = { A22_I2C_REG_INSTRUCTION_CTRL, A22_I2C_CMD_350_CM_DIST};
	int ret = transfer(&cmd[0], 2, nullptr, 0);
	// remember when we sent the acquire so we can know when the
	// acquisition has timed out
	_acquire_time_usec = hrt_absolute_time();
	return ret;
}

void A22I2c::start()
{
	// reset the report ring and state machine
	_collect_phase = false;

	// schedule a cycle to start things
	ScheduleNow();
}

void A22I2c::RunImpl()
{
	/* collection phase? */
	if (_collect_phase) {

		/* try a collection */
		if (OK != collect()) {
			PX4_DEBUG("collection error");

			/* if we've been waiting more than 200ms then
			   send a new acquire */
			if (hrt_elapsed_time(&_acquire_time_usec) > (A22_CONVERSION_TIMEOUT * 2)) {
				_collect_phase = false;
			}

		} else {
			/* next phase is measurement */
			_collect_phase = false;
		}
	}

	if (_collect_phase == false) {
		/* measurement phase */
		if (OK != measure()) {
			PX4_DEBUG("measure error");

		} else {
			/* next phase is collection. Don't switch to
			   collection phase until we have a successful
			   acquire request I2C transfer */
			_collect_phase = true;
		}
	}

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(A22_CONVERSION_INTERVAL);
}

extern "C" __EXPORT int a22_i2c_main(int argc, char *argv[])
{
	using ThisDriver = A22I2c;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = A22_I2C_BUS_SPEED;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	cli.i2c_address = A22_I2C_BASE_ADDR;

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIST_DEVTYPE_MB12XX);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	// if (!strcmp(verb, "status")) {
	// 	return ThisDriver::module_status(iterator);
	// }

	// if (!strcmp(verb, "set_address")) {
	// 	return ThisDriver::module_custom_method(cli, iterator);
	// }

	ThisDriver::print_usage();
	return -1;
}
