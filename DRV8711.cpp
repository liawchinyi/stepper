#include "DRV8711.h"

// Constructor
DRV8711::DRV8711(SPIClass &spi, byte CSpin)
  : _spi(spi), _CSpin(CSpin), CTRL_reg(0), drive_level(0) {}

void DRV8711::begin(byte drive, unsigned int microsteps) {
  begin(drive, microsteps, DRV8711DEC_SLOW_DECAY, DRV8711DRV_HIGH_50mA, DRV8711DRV_LOW_100mA, DRV8711CTL_DEADTIME_450ns);
}

// Begin function with specified values
void DRV8711::begin(byte drive, unsigned int microsteps, unsigned int decay_mode, unsigned int gate_speed, unsigned int gate_drive, unsigned int deadtime) {
  pinMode(_CSpin, OUTPUT);
  digitalWrite(_CSpin, LOW);
  setup_spi();

  drive_level = drive;
  CTRL_reg = DRV8711CTL_DEADTIME_450ns | DRV8711CTL_IGAIN_10 | DRV8711CTL_STALL_INTERNAL | (DRV8711CTL_MODE_4 << 3) | DRV8711CTL_ENABLE;
  set_reg(CTRL_REG, CTRL_reg);

  TORQUE_reg = DRV8711TRQ_BEMF_100us | DRV8711TRQ_TORQUE_MASK & 93;
  set_reg(TORQUE_REG, TORQUE_reg);

  OFF_reg = 0x30;  //0x00: 500 ns
  set_reg(OFF_REG, OFF_reg);

  BLANK_reg = DRV8711BLNK_ADAPTIVE_BLANK | 0x10;
  set_reg(BLANK_REG, BLANK_reg);

  DECAY_reg = DRV8711DEC_AUTOMIX;
  set_reg(DECAY_REG, DECAY_reg);

  STALL_reg = DRV8711STL_DIVIDE_4 | (0x03 << 8) | 0x10;
  set_reg(STALL_REG, STALL_reg);

  DRIVE_reg = DRV8711DRV_HIGH_50mA | DRV8711DRV_LOW_100mA | DRV8711DRV_HIGH_250ns | DRV8711DRV_LOW_250ns | DRV8711DRV_OCP_2us | DRV8711DRV_OCP_500mV;
  set_reg(DRIVE_REG, DRIVE_reg);

}


// Set register function
void DRV8711::set_reg(byte reg, unsigned int val) {
  unsigned int data = (reg << 12) | (val & 0x0FFF);
  digitalWrite(_CSpin, HIGH);
  delayMicroseconds(1);
  _spi.transfer16(data);
  delayMicroseconds(1);
  digitalWrite(_CSpin, LOW);
}

// Get register function

unsigned int DRV8711::get_reg(byte reg) {
  unsigned int address = 0x8000 | (reg << 12);

  digitalWrite(_CSpin, HIGH);
  delayMicroseconds(1);
  unsigned int value = _spi.transfer16(address) & 0x0FFF;  //_spi.transfer16(0x0000) & 0x0FFF;
  delayMicroseconds(1);
  digitalWrite(_CSpin, LOW);

  return value;
}


// Get status function
unsigned int DRV8711::get_status() {
  return get_reg(STATUS_REG) & 0xFF;
}

// Clear status function
void DRV8711::clear_status() {
  set_reg(STATUS_REG, 0);
}

// Enable/disable motor
void DRV8711::set_enable(bool enable) {
  write_reg_field(ENBL_FIELD, enable ? 1 : 0);
}

// End function
void DRV8711::end() {
  _spi.end();
}

// Get register field function
byte DRV8711::reg_field(int field) {
  int reg, shf, bits, mask;
  decode_field(field, reg, shf, bits, mask);
  return (get_reg(reg) & mask) >> shf;
}

// Write register field function
void DRV8711::write_reg_field(int field, byte val) {
  int reg, shf, bits, mask;
  decode_field(field, reg, shf, bits, mask);
  int reg_val = get_reg(reg);
  reg_val &= ~mask;
  reg_val |= (val << shf) & mask;
  set_reg(reg, reg_val);
}

// Decode field function
void DRV8711::decode_field(int field, int &reg, int &shf, int &bits, int &mask) {
  reg = (field >> 8) & 0x7;
  shf = (field >> 4) & 0xF;
  bits = (field & 0x7) + 1;
  mask = ((1 << bits) - 1) << shf;
}

// Set torque function
void DRV8711::set_torque(uint8_t value) {
  write_reg_field(TORQUE_FIELD, value & 0xFF);
}

// Set gain function
void DRV8711::set_gain(uint8_t gain) {
  write_reg_field(ISGAIN_FIELD, gain & 0x03);
}

// Enable motor function
void DRV8711::enable_motor() {
  write_reg_field(ENBL_FIELD, 1);  // Set bit 0 of CTRL register to 1
}

// Disable motor function
void DRV8711::disable_motor() {
  write_reg_field(ENBL_FIELD, 0);  // Set bit 0 of CTRL register to 0
}

// Setup SPI function
void DRV8711::setup_spi() {
  _spi.begin();
  _spi.setClockDivider(SPI_CLOCK_DIV8);  //0.5MHz
  _spi.setBitOrder(MSBFIRST);
  _spi.setDataMode(SPI_MODE0);
}

// Transfer function
char DRV8711::transfer(char data) {
  return _spi.transfer(data);
}
