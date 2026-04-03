/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_Math/AP_Math.h>

#include "AP_InertialSensor_SCH16T.h"
#include <GCS_MAVLink/GCS.h>


//#if defined(HAL_GPIO_PIN_nSPI6_RESET_EXTERNAL1)
#include <hal.h>
//#endif

static constexpr uint16_t EOI = (1 << 1);               // End of Initialization
static constexpr uint16_t EN_SENSOR = (1 << 0);         // Enable RATE and ACC measurement
static constexpr uint16_t DRY_DRV_EN = (1 << 5);        // Enables Data ready function

static constexpr uint16_t FILTER_68HZ = (0x0000);       // 68 Hz default filter
static constexpr uint16_t FILTER_BYPASS = (0b0000000111111111);     // No filtering


// page 46 (53)
// CTRL_RATE - DYN1 (default) '001'

// page 34 (53): 48-bit mode operations
// The SPI binary frame is constructed as follows:
// (TA9 TA8 TA[7:0] RW 0 FrTyp AE[6:0] DATA[19:0] CRC8)
// in doc DATA[19:0] =                      00000 010 010 dec=[000 000 000] for Set RATE Dynamic Range to ±315°/s


// dyn_rate=[001 001] dec=[011 011 011]
static constexpr uint16_t RATE_300DPS_1475HZ = 0b0001001011011011; // Gyro XYZ range 300 deg/s @ 1475Hz - Data 15-bit 
// CTRL_ACC12 - Set ACC Dynamic Range to ±80m/s2
// sample in doc =                          00000 010 010 000 000 000
// Settings for ACC_X12, ACC_Y12, ACC_Z12, post-processing decimation ratio, and shift value (dynamic range)

// dyn_rate=[001 001] decimation=[011, 011, 011]
static constexpr uint16_t ACC12_8G_1475HZ    = 0b0001001011011011;  // Acc XYZ range 8 G and 1475 update rate - Data 15-bit 
// Measurement Range (m/s2 ) = ±80 ; ±260 100 Nominal Sensitivity, 20-bit (LSB/(m/s2)) = 1600

// from PX4: 
// static constexpr uint8_t DECIMATION_1475_HZ =	(0b011);
// static constexpr uint8_t DECIMATION_738_HZ =	(0b100);

// union RATE_CTRL_Register {
// 	struct {
// 		uint16_t DEC_RATE_X2  : 3;
// 		uint16_t DEC_RATE_Y2  : 3;
// 		uint16_t DEC_RATE_Z2  : 3;
//
// 		uint16_t DYN_RATE_XYZ2: 3;
// 		uint16_t DYN_RATE_XYZ1: 3;
// 		uint16_t reserved     : 1;
// 	} bits;

// 	uint16_t value;
// };

// union ACC12_CTRL_Register {
// 	struct {
// 		uint16_t DEC_ACC_X2  : 3;
// 		uint16_t DEC_ACC_Y2  : 3;
// 		uint16_t DEC_ACC_Z2  : 3;
//
// 		uint16_t DYN_ACC_XYZ2: 3;
// 		uint16_t DYN_ACC_XYZ1: 3;
// 		uint16_t reserved     : 1;
// 	} bits;

// 	uint16_t value;
// };

// 	// We always use the maximum dynamic range for gyro and accel
// // Dynamic range settings
// static constexpr uint8_t RATE_RANGE_300 = (0b001);
// static constexpr uint8_t ACC12_RANGE_80 = (0b001);
// static constexpr uint8_t ACC3_RANGE_260 = (0b000);

	// rate_ctrl.bits.DYN_RATE_XYZ1 = 	RATE_RANGE_300;
	// rate_ctrl.bits.DYN_RATE_XYZ2 = 	RATE_RANGE_300;

	// acc12_ctrl.bits.DYN_ACC_XYZ1 = 	ACC12_RANGE_80;
	// acc12_ctrl.bits.DYN_ACC_XYZ2 = 	ACC12_RANGE_80;
	// acc3_ctrl.bits.DYN_ACC_XYZ3 = 	ACC3_RANGE_260;


static constexpr uint16_t ACC3_26G = (0b000 << 0); // Data is 3 bit, default. 
// Writing 4b1010 to this field generates a SPI soft reset. SPI Communication is not
// allowed during 2ms after SPI SOFTRESET.
static constexpr uint16_t SPI_SOFT_RESET = (0b1010);
static constexpr uint32_t POWER_ON_TIME = 250000UL + 5000UL;
static constexpr uint32_t RESET_TIME = 60000UL; // 33 ms max reset time according to datasheet, we wait more to be safe

// Data registers
#define RATE_X1         0x01 // 20 bit
#define RATE_Y1         0x02 // 20 bit
#define RATE_Z1         0x03 // 20 bit
#define ACC_X1          0x04 // 20 bit
#define ACC_Y1          0x05 // 20 bit
#define ACC_Z1          0x06 // 20 bit
#define ACC_X3          0x07 // 20 bit
#define ACC_Y3          0x08 // 20 bit
#define ACC_Z3          0x09 // 20 bit
#define RATE_X2         0x0A // 20 bit
#define RATE_Y2         0x0B // 20 bit
#define RATE_Z2         0x0C // 20 bit
#define ACC_X2          0x0D // 20 bit
#define ACC_Y2          0x0E // 20 bit
#define ACC_Z2          0x0F // 20 bit
#define TEMP            0x10 // 16 bit
// Status registers
#define STAT_SUM        0x14 // 16 bit
#define STAT_SUM_SAT    0x15 // 16 bit
#define STAT_COM        0x16 // 16 bit
#define STAT_RATE_COM   0x17 // 16 bit
#define STAT_RATE_X     0x18 // 16 bit
#define STAT_RATE_Y     0x19 // 16 bit
#define STAT_RATE_Z     0x1A // 16 bit
#define STAT_ACC_X      0x1B // 16 bit
#define STAT_ACC_Y      0x1C // 16 bit
#define STAT_ACC_Z      0x1D // 16 bit
// Control registers
#define CTRL_FILT_RATE  0x25 // 9 bit
#define CTRL_FILT_ACC12 0x26 // 9 bit
#define CTRL_FILT_ACC3  0x27 // 9 bit
#define CTRL_RATE       0x28 // 15 bit
#define CTRL_ACC12      0x29 // 15 bit
#define CTRL_ACC3       0x2A // 3 bit
#define CTRL_USER_IF    0x33 // 16 bit
#define CTRL_ST         0x34 // 13 bit
#define CTRL_MODE       0x35 // 4 bit
#define CTRL_RESET      0x36 // 4 bit
// Misc registers
#define ASIC_ID         0x3B // 12 bit
#define COMP_ID         0x3C // 16 bit
#define SN_ID1          0x3D // 16 bit
#define SN_ID2          0x3E // 16 bit
#define SN_ID3          0x3F // 16 bit

#define T_STALL_US   20U

#define SPI48_DATA_INT32(a)     (((int32_t)(((a) << 4)  & 0xfffff000UL)) >> 12)
#define SPI48_DATA_UINT32(a)    ((uint32_t)(((a) >> 8)  & 0x000fffffUL))
#define SPI48_DATA_UINT16(a)    ((uint16_t)(((a) >> 8)  & 0x0000ffffUL))

extern const AP_HAL::HAL& hal;

AP_InertialSensor_SCH16T::AP_InertialSensor_SCH16T(AP_InertialSensor &imu,
                                                         AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                                         enum Rotation _rotation,
                                                         uint8_t _drdy_pin,
                                                         uint8_t _reset_pin,
                                                         uint8_t _chip_variant
                                                    )
    : AP_InertialSensor_Backend(imu)
    , dev(std::move(_dev))
    , rotation(_rotation)
    , drdy_pin(_drdy_pin)
    , reset_pin(_reset_pin)
    , chip_variant(_chip_variant)
{
    // on K01:  according to datasheet,
    // the update rate is 23.6/X (1 kHz max) where X is the decimation factor set in the control registers. 
    // "1) Decimation ratio X is selectable from the following options: 2, 4, 8, 16 and 32"
    // With the default settings we have decimation of 16, which gives us an update rate of ~1475Hz.
    // Table 15 Selectable decimation ratios and corresponding ODR
    // Decimation factor        Output data rate        Output data rate with nominal F_PRIM (kHz)
    // 1                        F_PRIM/2                11.8
    // 2                        F_PRIM/4                 5.9
    // 4                        F_PRIM/8                 2.95
    // 8                        F_PRIM/16                1.475
    // 16                       F_PRIM/32                0.7375

    // Drawback of decimation is that sampling jitter is increased with the same ratio as the decimation factor.
    // With nominal primary frequency and decimation ratio of 16, the sampling jitter will be up to 85 µs x 16 =
    // 1.36 ms. This means that sample age can be anything between 0 and 1.36 ms. To address this issue,
    // the user can combine decimation with the data ready function. Data ready is explained in chapter 5.4.4.

    // However currently we get ~737-738Hz
    expected_sample_rate_hz = 1475; //FIXME?

    // if using ACC3 with 26G range, the sensitivity is 1600 LSB/(m/s^2), so accel_scale = 1.f / 1600.f;
    // accel_scale = 1.f / 1600.f;

    // if using ACC2 with 8G range, the sensitivity is 3200 LSB/(m/s^2), so accel_scale = 1.f / 3200.f;
    accel_scale = 1.f / 3200.f;

    // if K10 - 100, 200, 400 -> 200 @ 1.475kHz ODR, so gyro_scale = 1.f / 200.f;
        // case 100:
        //     return 0x02;   // 010
        // case 200:
        //     return 0x03;   // 011      
        // case 400:
        //     return 0x04;   // 100
    if (chip_variant == 10) {
        // 200 @ 1.475kHz ODR
        // 200 LSB/(deg/s) ?, so gyro_scale = 1.f / 200.f;
        gyro_scale = 1.f / 200.f;
        //accel_scale = 1.f / 1600.f;
    } else {
        // if K01 - 1600 LSB/(deg/s), so  gyro_scale = 1.f / 1600.f;
        gyro_scale = 1.f / 1600.f;
    }

    _registers[0] = RegisterConfig(CTRL_FILT_RATE,  FILTER_BYPASS);
    _registers[1] = RegisterConfig(CTRL_FILT_ACC12, FILTER_BYPASS);
    _registers[2] = RegisterConfig(CTRL_FILT_ACC3,  FILTER_BYPASS);
    // CTRL_RATE Settings for Gyro post-processing, decimation ratio, and dynamic range RW    15h0028, D0
    _registers[3] = RegisterConfig(CTRL_RATE,       RATE_300DPS_1475HZ); // +/- 300 deg/s, 1600 LSB/(deg/s) -- default, Decimation 8, 1475Hz
    // Settings for ACC_X12, ACC_Y12, ACC_Z12 post-processing decimation ratio, and shift value (dynamic range)
    // ACC12 dynamic range settings
    _registers[4] = RegisterConfig(CTRL_ACC12,      ACC12_8G_1475HZ);    // +/- 80 m/s^2, 3200 LSB/(m/s^2) -- default, Decimation 8, 1475Hz
    // Settings for ACC_X3, ACC_Y3, ACC_Z3 post-processing shift value (dynamic range)
    // ACC3 dynamic range settings
    // K01:
    //  Measurement Range (m/s2 ) = ±80 ; 
    // Electrical headroom = ±260 
    // Nominal Sensitivity, 16-bit (LSB/(m/s2)) = 100 
    // Nominal Sensitivity, 20-bit (LSB/(m/s2)) = 1600
    _registers[5] = RegisterConfig(CTRL_ACC3,       ACC3_26G);           // +/- 260 m/s^2, 1600 LSB/(m/s^2) -- default
}

AP_InertialSensor_Backend *
AP_InertialSensor_SCH16T::probe(AP_InertialSensor &imu,
                                   AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                   enum Rotation rotation,
                                   uint8_t drdy_gpio,
                                   uint8_t reset_gpio,
                                   uint8_t _chip_variant
                                )
{
    if (!dev) {
        return nullptr;
    }

    auto sensor = new AP_InertialSensor_SCH16T(imu, std::move(dev), rotation, drdy_gpio, reset_gpio, _chip_variant);

    // FIXME: sensor should be validated here!!!
    if (!sensor) {
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_SCH16T::start()
{
    if (!_imu.register_accel(accel_instance, expected_sample_rate_hz, dev->get_bus_id_devtype(DEVTYPE_INS_SCH16T)) ||
        !_imu.register_gyro(gyro_instance, expected_sample_rate_hz,   dev->get_bus_id_devtype(DEVTYPE_INS_SCH16T))) {
        return;
    }

    // setup sensor rotations from probe()
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);

    uint32_t period_us = 1000000UL / expected_sample_rate_hz;

    bool use_thread = true;

    if (!use_thread) {
        periodic_handle = dev->register_periodic_callback(period_us, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_SCH16T::run_state_machine, void));
    } else {
        /*
          as the sensor does not have a FIFO we need to jump through some
          hoops to ensure we don't lose any samples. This creates a thread
          to do the capture, running at very high priority
         */
        if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_SCH16T::loop_thread, void),
                                          "SCH16T",
                                          8192,AP_HAL::Scheduler::PRIORITY_BOOST, 1)) {
            AP_HAL::panic("Failed to create SCH16T thread");
        }
        hal.scheduler->delay(300); // give some time for the thread to start and gather some samples before we start the main loop
        while(num_ok_samples < 100) {
            hal.scheduler->delay(300);
        }
    }

}

void sch16t_delay_us(uint32_t usec)
{
    if (usec < 65535UL) {
        hal.scheduler->delay_microseconds((uint16_t) usec);
    } else {
        // for longer delays, we need to split into multiple calls, as it takes a uint16_t
        hal.scheduler->delay((uint16_t) (usec / (uint32_t)1000));
        hal.scheduler->delay_microseconds((uint16_t) (usec % (uint32_t)1000));
    }
}


void AP_InertialSensor_SCH16T::perform_read(){
    const uint32_t period_us = (1000000UL / expected_sample_rate_hz) - 20U;
    ReadStatus read_status = {};
    uint32_t last_reset_time = AP_HAL::micros();
    while(_state == State::Read) {
        uint32_t tstart = AP_HAL::micros();
        bool collect_status = false;
        
        uint32_t event_time = tstart;
        bool drdy_wait_ok = false;
        bool wait_ok = true;
        if (drdy_pin != 0) {
            // when we have a DRDY pin then wait for it to go high
            drdy_wait_ok = hal.gpio->wait_pin(drdy_pin, AP_HAL::GPIO::INTERRUPT_RISING, period_us+50U);
            if (drdy_wait_ok){
                wait_ok = false;
                num_drydry_ok++;
            }
            event_time = AP_HAL::micros();
        }
        
        {
            // minimize the time we hold the semaphore
            WITH_SEMAPHORE(dev->get_semaphore());
            collect_status = collect_and_publish(true, event_time, &read_status);
        }
        // FIMXE: getting huge variation in accel readings...
        if (collect_status) {
            if (failure_count > 0) {
                failure_count--;
            }
            if (!wait_ok) {
                uint32_t dt = AP_HAL::micros() - event_time;
                // moving average
                num_dt_items++;
                total_dt += dt;
                min_dt = min_dt > dt ? dt : min_dt;
                max_dt = max_dt < dt ? dt : max_dt;
            }
            num_ok_samples++;
        } else {
            num_bad_samples++;
            if (read_status.warn_no_new_sample) {
                // we didn't get a new sample, but it's not an error, just wait for the next one
                wait_ok = false;
                num_no_new_samples++;
                // read again almost immediately, expect that we'll get better alignment in time
                // without delay here, sensor keeps restarting
                if (drdy_pin == 0) {
                    sch16t_delay_us(1);
                }
            } else {
                // some other error, we should reset the sensor
                failure_count++;
            }
        }

        if (read_status.err_sample_inconsistent) {
            // we got a sample but it was inconsistent (part of measurements is old), so we should try to read again immediately
            // without waiting, as we might have just caught the sample in the middle of being updated
            // and the next read might be consistent
            wait_ok = false;
            // if we have DRDY, do not restart on inconsistent sample, just wait for another DRDY
            if (drdy_pin == 0){
                failure_count++; 
            }
            num_inconsistent_samples++;
            if (drdy_pin == 0) {
                sch16t_delay_us(1);
            }
        }

        // log status every few seconds
        uint64_t num_samples = num_ok_samples + num_bad_samples;
        if (num_samples % (1500*15) == 0) {
            bool fishy = num_ok_samples != num_drydry_ok || num_missed_samples > 0 || num_no_new_samples > 0;
            bool slightly_fishy = num_inconsistent_samples > 0;
            bool max_counters = num_samples > 1500*60;
            bool report_inconsistent_samples = slightly_fishy && max_counters; // 1min
            bool reset_counters = fishy || max_counters;

            if (fishy || report_inconsistent_samples) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, 
                    "SCH16T: %llu, %llu DRDY, %llu miss, %llu mixed, %llu no new, %d fail, avg_time=%llu us (min=%lu max=%lu, max report=%lu read=%lu crc8=%lu spi=%lu over %llu)",//, avg_wait=%llu us (over %llu)",
                    num_ok_samples, num_drydry_ok, num_missed_samples, num_inconsistent_samples, num_no_new_samples, failure_count, 
                    num_dt_items > 0 ? total_dt / num_dt_items : 0, min_dt, max_dt, max_reporting_dt, max_read_data_dt, max_crc8_dt, max_read_spi_dt, num_dt_items
                    //num_wait > 0 ? total_wait_time_us / num_wait : 0, num_wait
                );
            }
            if (reset_counters) {
                num_ok_samples = 0;
                num_bad_samples = 0;
                num_missed_samples = 0;
                num_inconsistent_samples = 0;
                num_no_new_samples = 0;

                num_dt_items = 0;
                total_dt = 0;
                min_dt = 100000;
                max_dt = 0;
                max_reporting_dt = 0;
                max_read_data_dt = 0;
                max_read_spi_dt = 0;
                max_crc8_dt = 0;

                num_wait = 0;
                total_wait_time_us = 0;

                num_drydry_ok = 0;
            }
        }

        // Reset if successive failures
        if (failure_count > 100) {
            // do not reset more often than every few seconds
            // important during startup
            if (last_reset_time + 5000000UL < AP_HAL::micros()) {
                last_reset_time = AP_HAL::micros();
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SCH16T: resetting");
                _state = State::Reset;
                return;
            }
            failure_count = 0;
        } else {
            if (wait_ok) {
                uint32_t dt = AP_HAL::micros() - tstart;
                // moving average
                num_dt_items++;
                total_dt += dt;
                
                // FIXME: this is strangely sensitive to -20U value          
                if (dt < period_us) {
                    uint32_t wait_us = period_us - dt;
                    if (!drdy_wait_ok || wait_us >= 1U) {
                        num_wait++;
                        total_wait_time_us += wait_us;
                        sch16t_delay_us(wait_us);
                    }
                }
            }
        }
    }
}


void AP_InertialSensor_SCH16T::run_state_machine_non_periodic()
{
    // like run_state_machine, just uses delay instead of periodic callback, 
    // to be used if the driver is configured to run in a thread instead of using periodic callbacks
    switch (_state) {
    case State::PowerOn: {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SCH16T: poweron");
            bool next_step_reset = true;
            if (next_step_reset) {
                _state = State::Reset;
                // at least if using external reset pin,
                // no need to wait that long before initiating a reset?
                if (reset_pin != 0) {
                    sch16t_delay_us(50*1000UL); // 1ms
                } else {
                    sch16t_delay_us(POWER_ON_TIME);
                }
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SCH16T: poweron delay done");
            } else {
                // skip reset and go straight to configuration, to save time on startup
                // if it'll fail, it'll reset at the end of the "flow"
                _state = State::Configure;
                // 1ms voltage + 32ms NVM read and SPI setup
                sch16t_delay_us(RESET_TIME);
            }

            break;
        }

    case State::Reset: {
            //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SCH16T: resetting");
            failure_count = 0;
            {
                WITH_SEMAPHORE(dev->get_semaphore());
                reset_chip();
            }
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SCH16T: reset done");
            _state = State::Configure;
            sch16t_delay_us(POWER_ON_TIME);
            //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SCH16T: reset done");
            break;
        }
    
    case State::Configure: {
            bool product_id_ok = false;
            {
                WITH_SEMAPHORE(dev->get_semaphore());
                product_id_ok = read_product_id();
            }
            
            if (!product_id_ok) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SCH16T: Product ID bad, resetting");
                _state = State::Reset;
                sch16t_delay_us(2000000); // 2s
                break;
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SCH16T: Prod ID ok");
            }

            {
                WITH_SEMAPHORE(dev->get_semaphore());
                configure_registers(); 
                // writes EN_SENSOR
                // datasheet: need to wait 215ms
            }
            _state = State::LockConfiguration;
            sch16t_delay_us(POWER_ON_TIME);
            break;
        }

    case State::LockConfiguration: {
            {
                WITH_SEMAPHORE(dev->get_semaphore());
                read_status_registers(); // Read all status registers once
                register_write(CTRL_MODE, (EOI | EN_SENSOR)); // Write EOI and EN_SENSOR
            }
            _state = State::Validate;
             // datasheet: need to wait 3ms before validating registers
            sch16t_delay_us(50000UL); // 50ms
            break;
        }

    case State::Validate: {
            bool sensor_status_ok = false;
            bool register_config_ok = false;
            {
                WITH_SEMAPHORE(dev->get_semaphore());
                read_status_registers(); // Read all status registers twice
                read_status_registers();
                sensor_status_ok = validate_sensor_status();
                // this reads registers so need to be inside the semaphore
                register_config_ok = validate_register_configuration();
            }

            // Check that registers are configured properly and that the sensor status is OK
            if (sensor_status_ok && register_config_ok) {
                _state = State::Read;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SCH16T: validated, will read");
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, 
                    "SCH16T: not validated (sensor %d, registers %d), resetting", 
                    sensor_status_ok, register_config_ok);
                _state = State::Reset;
                sch16t_delay_us(POWER_ON_TIME);
            }
            break;
        }

    case State::Read: {
            break;
        }
    
    default:
        break;
    } // end switch/case
}

void AP_InertialSensor_SCH16T::loop_thread(void)
{
    
    while (true) {
        if (_state == State::Read) {
            {
                WITH_SEMAPHORE(dev->get_semaphore());
                dev->set_speed(AP_HAL::Device::SPEED_HIGH);
                //dev->set_speed(AP_HAL::Device::SPEED_LOW);

            }
            perform_read();
        } else {
            {
                WITH_SEMAPHORE(dev->get_semaphore());
                dev->set_speed(AP_HAL::Device::SPEED_LOW);
            }
            run_state_machine_non_periodic();
        }
        //hal.scheduler->delay_microseconds(T_STALL_US);
    }
}


void AP_InertialSensor_SCH16T::run_state_machine()
{
    ReadStatus read_status = {};
    // FIXME: is it good practice to use semaphore all the time?
    //WITH_SEMAPHORE(dev->get_semaphore());

    switch (_state) {
    case State::PowerOn: {
            _state = State::Reset;
            dev->adjust_periodic_callback(periodic_handle, POWER_ON_TIME);
            break;
        }

    case State::Reset: {
            failure_count = 0;
            reset_chip();
            _state = State::Configure;
            dev->adjust_periodic_callback(periodic_handle, POWER_ON_TIME);
            break;
        }

    case State::Configure: {
            if (!read_product_id()) {
                _state = State::Reset;
                dev->adjust_periodic_callback(periodic_handle, 2000000); // 2s
                break;
            }

            configure_registers();
            _state = State::LockConfiguration;
            dev->adjust_periodic_callback(periodic_handle, POWER_ON_TIME);
            break;
        }

    case State::LockConfiguration: {
            read_status_registers(); // Read all status registers once
            register_write(CTRL_MODE, (EOI | EN_SENSOR)); // Write EOI and EN_SENSOR
            _state = State::Validate;
            dev->adjust_periodic_callback(periodic_handle, 50000UL); // 50ms
            break;
        }

    case State::Validate: {
            read_status_registers(); // Read all status registers twice
            read_status_registers();

            // Check that registers are configured properly and that the sensor status is OK
            if (validate_sensor_status() && validate_register_configuration()) {
                _state = State::Read;
                dev->adjust_periodic_callback(periodic_handle, 1000000UL / expected_sample_rate_hz);

            } else {
                _state = State::Reset;
                dev->adjust_periodic_callback(periodic_handle, POWER_ON_TIME);
            }

            break;
        }

    case State::Read: {
            if (collect_and_publish(false, AP_HAL::micros(), &read_status)) {
                if (failure_count > 0) {
                    failure_count--;
                }
            } else {
                failure_count++;
            }

            // Reset if successive failures
            if (failure_count > 10) {
                _state = State::Reset;
                return;
            }

            break;
        }

    default:
        break;
    } // end switch/case
}

bool AP_InertialSensor_SCH16T::collect_and_publish(bool use_thread, uint64_t sample_time_us, ReadStatus *read_status)
{
    SensorData data = {};
    bool success = read_data(&data, read_status);
    
    if (success) {
        uint32_t reporting_start = AP_HAL::micros();
        // adjust the periodic callback to be synchronous with the incoming data
        // call this immediately after a read
        if (!use_thread) {
            dev->adjust_periodic_callback(periodic_handle, 1000000UL / expected_sample_rate_hz);
        }

        Vector3f accel{accel_scale*data.acc_x, accel_scale*data.acc_y, accel_scale*data.acc_z};
        Vector3f gyro{gyro_scale*data.gyro_x, gyro_scale*data.gyro_y, gyro_scale*data.gyro_z};

        _rotate_and_correct_accel(accel_instance, accel);
        _notify_new_accel_raw_sample(accel_instance, accel, sample_time_us);

        _rotate_and_correct_gyro(gyro_instance, gyro);
        _notify_new_gyro_raw_sample(gyro_instance, gyro, sample_time_us);

        _publish_temperature(accel_instance, float(data.temp)/100.f);
        uint32_t reporting_end = AP_HAL::micros();
        uint32_t reporting_dt = reporting_end - reporting_start;
        max_reporting_dt = max_reporting_dt < reporting_dt ? reporting_dt : max_reporting_dt;
    }

    return success;
}

void AP_InertialSensor_SCH16T::reset_chip()
{
    if (reset_pin != 0) {
        palClearLine(reset_pin);
        //hal.scheduler->delay(2000);
        sch16t_delay_us(40*1000UL);
        palSetLine(reset_pin);
    } else {
        register_write(CTRL_RESET, SPI_SOFT_RESET);
        // SPI Communication is not allowed during 2ms after SPI SOFTRESET.
        sch16t_delay_us(2*1000UL);
    }
}

inline bool AP_InertialSensor_SCH16T::validate_frame_errors(uint64_t value){
    // Check for frame errors
    // FIXME: need special handling for some errors, like:
    // - out of scale / saturation
    // - still loading / not ready
    static constexpr uint64_t MASK48_ERROR = (uint64_t)0x001E00000000UL;
     if (value & MASK48_ERROR) {
        return false;
    }
    // Validate the CRC
    if (uint8_t(value & 0xff) != calculate_crc8(value)) {
        return false;
    }
    return true;
}


inline bool AP_InertialSensor_SCH16T::validate_received_frame(uint64_t value, int &last_dcnt, ReadStatus *read_status)
{
    // FIXME: shall we allow reporting samples where part of measurement is old?
    // maybe it's OK at least during init when load on MCU is larger?
    bool allow_inconsistent_sample = true;
    if (!validate_frame_errors(value)) {
        return false;
    }

    // bits 29-32 of the 48-bit SPI frame are used for data counter
    //static constexpr uint64_t MASK48_DCNT   = 0b00001111UL << 29;
    int data_counter = (int)(((value >> 29) & 0x00000000000FUL));

    // we should have the same data counter for all values, otherwise we got a mixure of two samples
    if (last_dcnt != -1 && data_counter != last_dcnt) {
        read_status->warn_no_new_sample = false;
        read_status->err_sample_inconsistent = true;
        if (!allow_inconsistent_sample) {
            return false;
        }
    }

    // check that we have a new sample, otherwise we are reading the same sample again, which is not good
    if (data_counter == earlier_dcnt) {
        // we are reading the same sample again
        read_status->warn_no_new_sample = true;
        read_status->err_sample_inconsistent = false;
        return false;
    }

    last_dcnt = data_counter;
    return true;
}


bool AP_InertialSensor_SCH16T::read_data(SensorData *data, ReadStatus *read_status)
{
    uint32_t read_data_start = AP_HAL::micros();
    
    // clean status
    read_status->warn_no_new_sample = false;
    read_status->err_sample_inconsistent = false;

    // Data counter is supported for decimated outputs RATE_XYZ2 and ACC_XYZ2. Value of data counter is
    // increased by one when a new sample is available from corresponding RATE/ACC channel. It can be
    // understood as an index for the data output values. Using the data counter, the user can monitor that
    // every wanted sample has been acquired and that the same sample has not been read twice.

    // When using 48-bit SPI protocol, 4-bit data counter value is included in MISO response frame. 



    // 5.4.6 Frequency counter
    // Using frequency counter, user can acquire accurate clock information from component internal MCLK
    // via SPI. The value of frequency counter register is increased by one with every 16th rising edge of
    // master clock. 

    // 5.4.7 Calculating exact time stamp
    // The data counter value can be combined with the frequency counter value to calculate the exact time
    // stamp of a sample when the MCU clock of the host system is used as reference. This combination is
    // recommended if integration operations are performed to sensor data and timing uncertainty or data jitter
    // of the interpolated data do not fulfill system accuracy requirements.

    // FIXME: this doesn't check if we got an old sample!
    // FIXME: reads look weird here, or is it expected?

    // DCNT A wrapping 4-bit sensor data counter.

    // Check errors & Validate data counter early on, so we can read again later with minimum delay
    int last_dcnt = -1;
    // FIXME: is it better to first read gyro or accel?
    register_read(RATE_X2);
    uint64_t gyro_x = register_read(RATE_Y2);
    //if (!validate_received_frame(gyro_x, last_dcnt, read_status)) return false;
    uint64_t gyro_y = register_read(RATE_Z2);
    //if (!validate_received_frame(gyro_y, last_dcnt, read_status)) return false;
    uint64_t gyro_z = register_read(ACC_X2);
    //if (!validate_received_frame(gyro_z, last_dcnt, read_status)) return false;
    uint64_t acc_x  = register_read(ACC_Y2);
    //if (!validate_received_frame(acc_x, last_dcnt, read_status)) return false;
    uint64_t acc_y  = register_read(ACC_Z2);
    //if (!validate_received_frame(acc_y, last_dcnt, read_status)) return false;
    uint64_t acc_z  = register_read(TEMP);
    //if (!validate_received_frame(acc_z, last_dcnt, read_status)) return false;

    // FIXME: could report temperature at lower rate to save some SPI time... ?
    uint64_t temp   = register_read(TEMP);

    uint32_t read_spi_end = AP_HAL::micros();
    uint32_t read_spi_dt = read_spi_end - read_data_start;
    max_read_spi_dt = max_read_spi_dt < read_spi_dt ? read_spi_dt : max_read_spi_dt;

    if (!validate_received_frame(gyro_x, last_dcnt, read_status)) return false;
    if (!validate_received_frame(gyro_y, last_dcnt, read_status)) return false;
    if (!validate_received_frame(gyro_z, last_dcnt, read_status)) return false;
    // FIXME: it seems DATA_COUNTER (DCNT) probably should be the same for gyro and accel,
    // but most time I was getting different data counter for gyro and accel,
    // though within gyro, or within the accel, the data counter was consistent. 
    // So for now I'm checking data counter consistency only within gyro and accel.
    // Is this due to scheduler/interrupt delay after DRY? or IMU config issue? 
    last_dcnt = -1;
    if (!validate_received_frame(acc_x, last_dcnt, read_status)) return false;
    if (!validate_received_frame(acc_y, last_dcnt, read_status)) return false;
    if (!validate_received_frame(acc_z, last_dcnt, read_status)) return false;


    if (last_dcnt != -1) {
        if (earlier_dcnt != -1) {
            // check for missed samples
            if ((earlier_dcnt + 1) % 16 != last_dcnt % 16){
                num_missed_samples++;
            }
        }
        // we got a valid data counter, save it for the next read
        earlier_dcnt = last_dcnt;
    }

    if (!validate_frame_errors(temp)) {
        return false;
    }

    uint32_t crc8_end = AP_HAL::micros();
    uint32_t crc8_dt = crc8_end - read_spi_end;
    max_crc8_dt = max_crc8_dt < crc8_dt ? crc8_dt : max_crc8_dt;

    // Data registers are 20bit 2s complement
    data->acc_x    = SPI48_DATA_INT32(acc_x);
    data->acc_y    = SPI48_DATA_INT32(acc_y);
    data->acc_z    = SPI48_DATA_INT32(acc_z);
    data->gyro_x   = SPI48_DATA_INT32(gyro_x);
    data->gyro_y   = SPI48_DATA_INT32(gyro_y);
    data->gyro_z   = SPI48_DATA_INT32(gyro_z);
    // Temperature data is always 16 bits wide. Drop 4 LSBs as they are not used.
    data->temp    = SPI48_DATA_INT32(temp) >> 4;

    // Convert to RH coordinate system (FLU to FRD)
    data->acc_x = data->acc_x;
    data->acc_y = -data->acc_y;
    data->acc_z = -data->acc_z;
    data->gyro_x = data->gyro_x;
    data->gyro_y = -data->gyro_y;
    data->gyro_z = -data->gyro_z;

    uint32_t read_data_end = AP_HAL::micros();
    uint32_t read_data_dt = read_data_end - read_data_start;
    max_read_data_dt = max_read_data_dt < read_data_dt ? read_data_dt : max_read_data_dt;

    return true;
}

bool AP_InertialSensor_SCH16T::read_product_id()
{
    register_read(COMP_ID);
    uint16_t comp_id = SPI48_DATA_UINT16(register_read(ASIC_ID));
    uint16_t asic_id = SPI48_DATA_UINT16(register_read(ASIC_ID));

    // Debug code
    // register_read(SN_ID1);
    // uint16_t sn_id1 = SPI48_DATA_UINT16(register_read(SN_ID2));
    // uint16_t sn_id2 = SPI48_DATA_UINT16(register_read(SN_ID3));
    // uint16_t sn_id3 = SPI48_DATA_UINT16(register_read(SN_ID3));

    //char serial_str[14];
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, serial_str, 14, "%05d%01X%04X", sn_id2, sn_id1 & 0x000F, sn_id3);
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Serial:\t %s", serial_str);
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "COMP_ID:\t 0x%0x ASIC_ID:\t 0x%0x", comp_id, asic_id);

    // SCH16T-K01   -   ID hex = 0x0020
    // SCH1633-B13  -   ID hex = 0x0017
    //bool success = asic_id == 0x20 && comp_id == 0x17;
    // asic_id - ASIC revision
    // comp_id - Component type

    bool ok = asic_id == 0x21 || comp_id == 0x23;
    // FIXME: could also auto-detect the variant here based on the IDs, but for now passing _chip_variant from the constructor (hwdef)
    if ((asic_id == 0x21 && comp_id == 0x23) || (asic_id == 0x20 && comp_id == 0x17)) {
		// ASIC_ID = 0x21, COMP_ID = 0x23 is a K01 variant of REV_1
		// ASIC_ID = 0x20, COMP_ID = 0x17 is a B13 variant of REV_1
		//_detected_version = ChipVersion::REV_1;
        ok = true;
	} else if ((asic_id == 0x21 && comp_id == 0x24) || (asic_id == 0x21 && comp_id == 0x21)) {
		// ASIC_ID = 0x21, COMP_ID = 0x24 is a B10 variant of REV_2
		// ASIC_ID = 0x21, COMP_ID = 0x21 is a production K10 variant of REV_2
		//_detected_version = ChipVersion::REV_2;
        ok = true;
    }

    // FIXME why? Some users report that the chip returns 0 for both ASIC_ID and COMP_ID after power on, but then works fine after a reset. This may be a silicon bug or an issue with the power supply ramping up too slowly. In either case, treat this as a non-fatal error and allow the state machine to continue to the configuration step, where we will check the IDs again and reset if they are still wrong.
    return ok;
}

void AP_InertialSensor_SCH16T::configure_registers()
{
    for (auto &r : _registers) {
        register_write(r.addr, r.value);
    }

    uint16_t reg_value;
    reg_value = SPI48_DATA_UINT16(register_read(CTRL_USER_IF));
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CTRL_USER_IF before DRDY: 0x%04X", reg_value);

    // this has read 0000
    // Maybe we need to reset the registers to defaults?

    // Force/reset default IMU config values (according to K01 datasheet):

    // [13:12] 
    // Typical delay time of 1st level status clearance. When user reads data register, the
    // associated 1st level status register is cleared after TDEL.
    // 00 - 0.078 ms
    // 01 - 0.625 ms
    // 10 - 2.5 ms (default)
    // 11 – 5 ms
    #define FTREE_TDEL_DEFAULT (1 << 13) // 2.5ms (default)
    reg_value |= FTREE_TDEL_DEFAULT;

    // MISO_SR_CTRL MISO Slew Rate control [3:3] 
    // 0 - SR control disabled without static current (fast rise/fall time ~<1ns). (Contact
    // sales office before enabling)
    // 1 - SR control enabled with static current (default)
    #define MISO_SR_CTRL_DEFAULT (1 << 3)
    reg_value |= MISO_SR_CTRL_DEFAULT;

    // DRY_SR_CTRL DRY Slew Rate control [2:2]
    // 0 - SR control disabled without static current (fast rise/fall time ~<1ns). (Contact
    // sales office before enabling)
    // 1 - SR control enabled with static current (default)
    #define DRY_SR_CTRL_DEFAULT (1 << 2)
    reg_value |= DRY_SR_CTRL_DEFAULT;

    if (drdy_pin != 0) {
        reg_value |= DRY_DRV_EN; // Enable data ready function on the DRDY pin
    } else {
        reg_value &= ~DRY_DRV_EN; // Disable data ready function, we will poll instead for now
    }
    register_write(CTRL_USER_IF, reg_value);
    register_write(CTRL_MODE, EN_SENSOR); // Enable the sensor
}

bool AP_InertialSensor_SCH16T::validate_sensor_status()
{
    auto &s = _sensor_status;
    uint16_t values[] = { s.summary, s.saturation, s.common, s.rate_common, s.rate_x, s.rate_y, s.rate_z, s.acc_x, s.acc_y, s.acc_z };

    for (auto v : values) {
        if (v != 0xFFFF) {
            return false;
        }
    }

    return true;
}

bool AP_InertialSensor_SCH16T::validate_register_configuration()
{
    bool success = true;

    for (auto &r : _registers) {
        register_read(r.addr); // double read, wasteful but makes the code cleaner, not high rate so doesn't matter anyway
        auto value = SPI48_DATA_UINT16(register_read(r.addr));

        if (value != r.value) {
            success = false;
        }
    }

    return success;
}

void AP_InertialSensor_SCH16T::read_status_registers()
{
    register_read(STAT_SUM);
    _sensor_status.summary      = SPI48_DATA_UINT16(register_read(STAT_SUM_SAT));
    _sensor_status.saturation   = SPI48_DATA_UINT16(register_read(STAT_COM));
    _sensor_status.common       = SPI48_DATA_UINT16(register_read(STAT_RATE_COM));
    _sensor_status.rate_common  = SPI48_DATA_UINT16(register_read(STAT_RATE_X));
    _sensor_status.rate_x       = SPI48_DATA_UINT16(register_read(STAT_RATE_Y));
    _sensor_status.rate_y       = SPI48_DATA_UINT16(register_read(STAT_RATE_Z));
    _sensor_status.rate_z       = SPI48_DATA_UINT16(register_read(STAT_ACC_X));
    _sensor_status.acc_x        = SPI48_DATA_UINT16(register_read(STAT_ACC_Y));
    _sensor_status.acc_y        = SPI48_DATA_UINT16(register_read(STAT_ACC_Z));
    _sensor_status.acc_z        = SPI48_DATA_UINT16(register_read(STAT_ACC_Z));
}

inline uint64_t AP_InertialSensor_SCH16T::register_read(uint8_t addr)
{
    uint64_t frame = {};
    frame |= uint64_t(addr) << 38; // Target address offset
    frame |= uint64_t(1) << 35; // FrameType: SPI48BF
    frame |= uint64_t(calculate_crc8(frame));

    return transfer_spi_frame(frame);
}

// Non-data registers are the only writable ones and are 16 bit or less
void AP_InertialSensor_SCH16T::register_write(uint8_t addr, uint16_t value)
{
    uint64_t frame = {};
    frame |= uint64_t(1) << 37; // Write bit
    frame |= uint64_t(addr) << 38; // Target address offset
    frame |= uint64_t(1) << 35; // FrameType: SPI48BF
    frame |= uint64_t(value) << 8;
    frame |= uint64_t(calculate_crc8(frame));

    // We don't care about the return frame on a write
    (void)transfer_spi_frame(frame);
}

// The SPI protocol (SafeSPI) is 48bit out-of-frame. This means read return frames will be received on the next transfer.
uint64_t AP_InertialSensor_SCH16T::transfer_spi_frame(uint64_t frame)
{
    uint16_t buf[3];
    for (int index = 0; index < 3; index++) {
        uint16_t lower_byte = (frame >> (index << 4)) & 0xFF;
        uint16_t upper_byte = (frame >> ((index << 4) + 8)) & 0xFF;
        buf[3 - index - 1] = (lower_byte << 8) | upper_byte;
    }

    dev->transfer((uint8_t*)buf, 6, (uint8_t*)buf, 6);

    uint64_t value = {};
    for (int index = 0; index < 3; index++) {
        uint16_t lower_byte = buf[index] & 0xFF;
        uint16_t upper_byte = (buf[index] >> 8) & 0xFF;
        value |= (uint64_t)(upper_byte | (lower_byte << 8)) << ((3 - index - 1) << 4);
    }

    return value;
}

inline uint8_t AP_InertialSensor_SCH16T::calculate_crc8(uint64_t frame)
{
    uint64_t data = frame & 0xFFFFFFFFFF00LL;
    uint8_t crc = 0xFF;

    for (int i = 47; i >= 0; i--) {
        uint8_t data_bit = data >> i & 0x01;
        crc = crc & 0x80 ? (uint8_t)((crc << 1) ^ 0x2F) ^ data_bit : (uint8_t)(crc << 1) | data_bit;
    }

    return crc;
}

bool AP_InertialSensor_SCH16T::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}
