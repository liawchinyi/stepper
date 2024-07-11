#include "DRV8711.h"
#include <SPI.h>

// Constructor
DRV8711::DRV8711(SPIClass &spi, byte CSpin) : _spi(spi), _CSpin(CSpin), control_reg(0), drive_level(0) {}

void DRV8711::begin(byte drive, unsigned int microsteps) {
    begin(drive, microsteps, DRV8711DEC_SLOW_DECAY, DRV8711DRV_HIGH_50mA, DRV8711DRV_LOW_100mA, DRV8711CTL_DEADTIME_850ns);
}

void DRV8711::begin(byte drive, unsigned int microsteps, byte decay_mode, byte gate_speed, byte gate_drive, byte deadtime) {
    pinMode(_CSpin, OUTPUT);
    digitalWrite(_CSpin, HIGH);
    setup_spi();

    drive_level = drive;
    
    // Configure initial settings
    write_reg_field(MODE_FIELD, microsteps);
    write_reg_field(DECMOD_FIELD, decay_mode);
    write_reg_field(IDRIVEP_FIELD, gate_speed);
    write_reg_field(IDRIVEN_FIELD, gate_drive);
    write_reg_field(DTIME_FIELD, deadtime);

    // Enable the motor driver
    enable_motor();
}

void DRV8711::set_reg(byte reg, int val) {
    // Combine the write bit (0), the 3-bit address, and the 12-bit data
    unsigned int data = (reg << 12) | (val & 0x0FFF);

    digitalWrite(_CSpin, HIGH);
    _spi.transfer16(data);
    digitalWrite(_CSpin, LOW);
}

int DRV8711::get_reg(byte reg) {
    // Combine the read bit (1) and the 3-bit address
    unsigned int address = 0x8000 | (reg << 12);

    digitalWrite(_CSpin, HIGH);
    _spi.transfer16(address);
    int value = _spi.transfer16(0x0000) & 0x0FFF;
    digitalWrite(_CSpin, LOW);

    return value;
}

byte DRV8711::get_status() {
    return get_reg(STATUS_REG) & 0xFF;
}

void DRV8711::clear_status() {
    set_reg(STATUS_REG, 0);
}

void DRV8711::set_enable(bool enable) {
    write_reg_field(ENBL_FIELD, enable ? 1 : 0);
}

void DRV8711::end() {
    _spi.end();
}

byte DRV8711::reg_field(int field) {
    int reg, shf, bits, mask;
    decode_field(field, reg, shf, bits, mask);
    return (get_reg(reg) & mask) >> shf;
}

void DRV8711::write_reg_field(int field, byte val) {
    int reg, shf, bits, mask;
    decode_field(field, reg, shf, bits, mask);
    int reg_val = get_reg(reg);
    reg_val &= ~mask;
    reg_val |= (val << shf) & mask;
    set_reg(reg, reg_val);
}

void DRV8711::decode_field(int field, int &reg, int &shf, int &bits, int &mask) {
    reg = (field >> 8) & 0x7;
    shf = (field >> 4) & 0xF;
    bits = (field & 0x7) + 1;
    mask = ((1 << bits) - 1) << shf;
}

void DRV8711::set_torque(uint8_t value) {
    write_reg_field(TORQUE_FIELD, value & 0xFF);
}

void DRV8711::set_gain(uint8_t gain) {
    write_reg_field(ISGAIN_FIELD, gain & 0x03);
}

void DRV8711::configureCtrlRegister() {
    control_reg = DRV8711CTL_DEADTIME_400ns | DRV8711CTL_IGAIN_5 | DRV8711CTL_STALL_INTERNAL | (DRV8711_FULL << 3);
    set_reg(CTRL_REG, control_reg);
}

void DRV8711::enable_motor() {
    write_reg_field(ENBL_FIELD, 1); // Set bit 0 of CTRL register to 1
}

void DRV8711::disable_motor() {
    write_reg_field(ENBL_FIELD, 0); // Set bit 0 of CTRL register to 0
}
void DRV8711::setup_spi() {
    _spi.begin();
    _spi.setClockDivider(SPI_CLOCK_DIV8);
    _spi.setBitOrder(MSBFIRST);
    _spi.setDataMode(SPI_MODE0);
}

char DRV8711::transfer(char data) {
    return _spi.transfer(data);
}
