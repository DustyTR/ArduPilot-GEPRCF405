/*
TYSM to https://github.com/mjx4mois/AK09911/tree/master
*/

#include "AP_Compass_AK09911C.h"

#if AP_COMPASS_AK09911C_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

// Bus Driver Implementation
AP_AK09911C_BusDriver_HALDevice::AP_AK09911C_BusDriver_HALDevice(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : _dev(std::move(dev))
{
}

bool AP_AK09911C_BusDriver_HALDevice::block_read(uint8_t reg, uint8_t *buf, uint32_t size)
{
    return _dev->read_registers(reg, buf, size);
}

bool AP_AK09911C_BusDriver_HALDevice::register_read(uint8_t reg, uint8_t *val)
{
    return _dev->read_registers(reg, val, 1);
}

bool AP_AK09911C_BusDriver_HALDevice::register_write(uint8_t reg, uint8_t val)
{
    return _dev->write_register(reg, val);
}

AP_HAL::Semaphore *AP_AK09911C_BusDriver_HALDevice::get_semaphore()
{
    return _dev->get_semaphore();
}

AP_HAL::Device::PeriodicHandle AP_AK09911C_BusDriver_HALDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return _dev->register_periodic_callback(period_usec, cb);
}

// Compass Backend Implementation
AP_Compass_Backend *AP_Compass_AK09911C::probe(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                                bool force_external,
                                                enum Rotation rotation)
{
    if (!dev) return nullptr;

    dev->set_retries(3);

    auto bus = new AP_AK09911C_BusDriver_HALDevice(std::move(dev));
    if (!bus) return nullptr;
    
    AP_Compass_AK09911C *sensor = new AP_Compass_AK09911C(bus, force_external, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_AK09911C::AP_Compass_AK09911C(AP_AK09911C_BusDriver *bus,
                                         bool force_external,
                                         enum Rotation rotation)
    : AP_Compass_Backend(),
      _bus(bus),
      _force_external(force_external),
      _rotation(rotation),
      _initialized(false)
{
    _asa[0] = 1.0f;
    _asa[1] = 1.0f;
    _asa[2] = 1.0f;
}

bool AP_Compass_AK09911C::init()
{
    if (!_bus) return false;
    
    AP_HAL::Semaphore *sem = _bus->get_semaphore();
    if (!sem) return false;
    
    WITH_SEMAPHORE(*sem);
    
    if (!_reset()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AK09911C: Reset NACK (Ignored)");
    }

    hal.scheduler->delay(50);
    
    if (!_bus->register_write(AK09911C_REG_CNTL2, AK09911C_POWER_DOWN_MODE)) {
         GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AK09911C: PowerDown failed");
         return false;
    }
    
    hal.scheduler->delay(2);

    // Check Device ID (NOW we check ID)
    if (!_check_id()) {
        return false;
    }

    // Read ASA (Sensitivity Adjustment)
    if (!_read_asa()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AK09911C: ASA read failed");
        return false;
    }
    
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AK09911C: Found! ASA X:%.2f Y:%.2f Z:%.2f", 
                  (double)_asa[0], (double)_asa[1], (double)_asa[2]);
    
    _bus->register_write(AK09911C_REG_CNTL2, AK09911C_POWER_DOWN_MODE);
    hal.scheduler->delay(2);

    if (!_setup_mode()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AK09911C: Mode setup failed");
        return false;
    }
    
    _initialized = true;
    _bus->set_device_type(DEVTYPE_AK09911C);
    if (!register_compass(_bus->get_bus_id())) {
        return false;
    }
    
    if (_force_external) set_external(true);
    set_rotation(_rotation);
    
    _bus->register_periodic_callback(10000, FUNCTOR_BIND_MEMBER(&AP_Compass_AK09911C::_update, void));
    
    return true;
}

bool AP_Compass_AK09911C::_reset()
{
    // Write 0x01 to CNTL3 (0x32)
    return _bus->register_write(AK09911C_REG_CNTL3, AK09911C_SOFT_RESET);
}

bool AP_Compass_AK09911C::_check_id()
{
    uint8_t wia1 = 0;
    uint8_t wia2 = 0;
    static bool printed_error = false;

    // Read WIA1 (Company ID)
    if (!_bus->register_read(AK09911C_REG_WIA1, &wia1)) {
        if (!printed_error) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AK09911C: ID Read NACK");
            printed_error = true;
        }
        return false;
    }
    
    if (wia1 != AK09911C_WIA1_VALUE) {
        if (!printed_error) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AK09911C: Bad WIA1: 0x%02X", wia1);
            printed_error = true;
        }
        return false;
    }

    // Read WIA2 (Device ID)
    if (!_bus->register_read(AK09911C_REG_WIA2, &wia2) || wia2 != AK09911C_WIA2_VALUE) {
        if (!printed_error) {
             GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AK09911C: Bad WIA2: 0x%02X", wia2);
             printed_error = true;
        }
        return false;
    }
    
    return true;
}

bool AP_Compass_AK09911C::_read_asa()
{
    // Enter Fuse ROM access mode
    if (!_bus->register_write(AK09911C_REG_CNTL2, AK09911C_FUSE_ROM)) return false;
    
    hal.scheduler->delay(10);
    
    uint8_t asa_data[3];
    if (!_bus->block_read(AK09911C_REG_ASAX, asa_data, 3)) return false;
    
    for (int i = 0; i < 3; i++) {
        _asa[i] = ((float)asa_data[i] / 128.0f) + 1.0f;
    }
    return true;
}

bool AP_Compass_AK09911C::_setup_mode()
{
    return _bus->register_write(AK09911C_REG_CNTL2, AK09911C_CONTI_MODE_4);
}

void AP_Compass_AK09911C::_update()
{
    if (!_initialized) return;
    
    AP_HAL::Semaphore *sem = _bus->get_semaphore();
    if (!sem || !sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) return;
    
    uint8_t st1 = 0;
    if (!_bus->register_read(AK09911C_REG_ST1, &st1)) {
        sem->give();
        return;
    }
    
    // Check DRDY bit (0x01)
    if (!(st1 & 0x01)) {
        sem->give();
        return;
    }
    
    // Read 8 bytes: HXL(0x11) through ST2(0x18)
    uint8_t raw_data[8];
    if (!_bus->block_read(AK09911C_REG_HXL, raw_data, 8)) {
        sem->give();
        return;
    }
    
    sem->give();
    
    // Check ST2 (Last byte) for Overflow (Bit 3)
    if (raw_data[7] & 0x08) return; 
    
    Vector3f field;
    field.x = (int16_t)((raw_data[1] << 8) | raw_data[0]);
    field.y = (int16_t)((raw_data[3] << 8) | raw_data[2]);
    field.z = (int16_t)((raw_data[5] << 8) | raw_data[4]);
    
    _make_factory_sensitivity_adjustment(field);
    field *= AK09911C_MILLIGAUSS_SCALE;
    accumulate_sample(field);
}

void AP_Compass_AK09911C::_make_factory_sensitivity_adjustment(Vector3f &field) const
{
    field.x *= _asa[0];
    field.y *= _asa[1];
    field.z *= _asa[2];
}

void AP_Compass_AK09911C::read()
{
    if (!_initialized) return;
    drain_accumulated_samples();
}

#endif
