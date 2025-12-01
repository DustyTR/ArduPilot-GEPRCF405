#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_AK09911C_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

// AK09911C I2C Address
// The AK09911C has a configurable address based on CAD0 pin:
// CAD0 = GND (0): Address = 0x18 (0x0C in 7-bit addressing)
// CAD0 = VCC (1): Address = 0x1A (0x0D in 7-bit addressing)
// Note: ArduPilot uses 7-bit addressing, so actual addresses are 0x0C or 0x0D
#ifndef HAL_COMPASS_AK09911C_I2C_ADDR
# define HAL_COMPASS_AK09911C_I2C_ADDR 0x0C  // 7-bit address (0x18 >> 1)
#endif

#ifndef HAL_COMPASS_AK09911C_I2C_ADDR2
# define HAL_COMPASS_AK09911C_I2C_ADDR2 0x0D  // 7-bit address (0x1A >> 1)
#endif

// AK09911C register definitions
#define AK09911C_REG_WIA1                   0x00
#define AK09911C_REG_WIA2                   0x01
#define AK09911C_REG_INFO1                  0x02
#define AK09911C_REG_INFO2                  0x03
#define AK09911C_REG_ST1                    0x10
#define AK09911C_REG_HXL                    0x11
#define AK09911C_REG_HXH                    0x12
#define AK09911C_REG_HYL                    0x13
#define AK09911C_REG_HYH                    0x14
#define AK09911C_REG_HZL                    0x15
#define AK09911C_REG_HZH                    0x16
#define AK09911C_REG_TMPS                   0x17
#define AK09911C_REG_ST2                    0x18
#define AK09911C_REG_CNTL1                  0x30
#define AK09911C_REG_CNTL2                  0x31
#define AK09911C_REG_CNTL3                  0x32

// Fuse ROM access registers (ASA - Sensitivity Adjustment)
#define AK09911C_REG_ASAX                   0x60
#define AK09911C_REG_ASAY                   0x61
#define AK09911C_REG_ASAZ                   0x62

// AK09911C Device IDs
#define AK09911C_WIA1_VALUE                 0x48  // Company ID
#define AK09911C_WIA2_VALUE                 0x05  // Device ID

// AK09911C Control modes (written to CNTL2 register)
enum AK09911C_Mode {
    AK09911C_POWER_DOWN_MODE = 0x00,    // Power-down mode
    AK09911C_SINGLE_MODE     = 0x01,    // Single measurement mode
    AK09911C_CONTI_MODE_1    = 0x02,    // Continuous mode 1 (10Hz)
    AK09911C_CONTI_MODE_2    = 0x04,    // Continuous mode 2 (20Hz)
    AK09911C_CONTI_MODE_3    = 0x06,    // Continuous mode 3 (50Hz)
    AK09911C_CONTI_MODE_4    = 0x08,    // Continuous mode 4 (100Hz)
    AK09911C_SELF_TEST       = 0x10,    // Self-test mode
    AK09911C_FUSE_ROM        = 0x1F     // Fuse ROM access mode
};

// CNTL3 register values
#define AK09911C_SOFT_RESET                 0x01

// Sensitivity (0.6 μT/LSB for AK09911C = 6.0 milligauss/LSB)
// Reference: 1 μT = 10 milligauss, so 0.6 μT/LSB * 10 = 6.0 milligauss/LSB
#define AK09911C_MILLIGAUSS_SCALE           6.0f

class AP_AK09911C_BusDriver;

class AP_Compass_AK09911C : public AP_Compass_Backend
{
public:
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                     bool force_external,
                                     enum Rotation rotation = ROTATION_NONE);
    
    static constexpr const char *name = "AK09911C";
    
    void read() override;
    
    /* Must be public so the BusDriver can access its definition */
    struct PACKED sample_regs {
        uint8_t st1;
        int16_t val[3];
        uint8_t tmps;
        uint8_t st2;
    };

private:
    AP_Compass_AK09911C(AP_AK09911C_BusDriver *bus,
                        bool force_external,
                        enum Rotation rotation);
    
    bool init();
    bool _check_id();
    bool _setup_mode();
    bool _reset();
    bool _read_asa();
    void _update();
    void _make_factory_sensitivity_adjustment(Vector3f &field) const;
    
    AP_AK09911C_BusDriver *_bus;
    bool _force_external;
    enum Rotation _rotation;
    bool _initialized;
    
    float _asa[3]; // ASA compensation values
};

// Bus driver abstraction
class AP_AK09911C_BusDriver
{
public:
    virtual ~AP_AK09911C_BusDriver() { }

    virtual bool block_read(uint8_t reg, uint8_t *buf, uint32_t size) = 0;
    virtual bool register_read(uint8_t reg, uint8_t *val) = 0;
    virtual bool register_write(uint8_t reg, uint8_t val) = 0;

    virtual AP_HAL::Semaphore *get_semaphore() = 0;
    virtual AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb) = 0;

    virtual void set_device_type(uint8_t devtype) = 0;
    virtual uint32_t get_bus_id(void) const = 0;
};

// HAL device implementation
class AP_AK09911C_BusDriver_HALDevice : public AP_AK09911C_BusDriver
{
public:
    AP_AK09911C_BusDriver_HALDevice(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    bool block_read(uint8_t reg, uint8_t *buf, uint32_t size) override;
    bool register_read(uint8_t reg, uint8_t *val) override;
    bool register_write(uint8_t reg, uint8_t val) override;

    AP_HAL::Semaphore *get_semaphore() override;
    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;

    void set_device_type(uint8_t devtype) override {
        _dev->set_device_type(devtype);
    }

    uint32_t get_bus_id(void) const override {
        return _dev->get_bus_id();
    }

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};

#endif  // AP_COMPASS_AK09911C_ENABLED
