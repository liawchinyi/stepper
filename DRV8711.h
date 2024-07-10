#ifndef DRV8711_h
#define DRV8711_h

#include <Arduino.h>
#include <SPI.h>

// Pin definitions
#define MOSI_PIN   PB5   // MOSI Pin
#define MISO_PIN   PB4   // MISO Pin
#define SCLK_PIN   PB3   // Clock Pin
#define CS_PIN     PA15  // Chip Select or Slave Select Pin

// Register definitions
#define CTRL_REG   0
#define TORQUE_REG 1
#define OFF_REG    2
#define BLANK_REG  3
#define DECAY_REG  4
#define STALL_REG  5
#define DRIVE_REG  6
#define STATUS_REG 7

// CTRL_REG settings
#define DRV8711CTL_DEADTIME_400ns 0x000
#define DRV8711CTL_DEADTIME_450ns 0x400
#define DRV8711CTL_DEADTIME_650ns 0x800
#define DRV8711CTL_DEADTIME_850ns 0xC00

#define DRV8711CTL_IGAIN_5        0x000
#define DRV8711CTL_IGAIN_10       0x100
#define DRV8711CTL_IGAIN_20       0x200
#define DRV8711CTL_IGAIN_40       0x300

#define DRV8711CTL_STALL_INTERNAL 0x000
#define DRV8711CTL_STALL_EXTERNAL 0x080

#define DRV8711CTL_STEPMODE_MASK  0x078
#define DRV8711CTL_FORCESTEP      0x004
#define DRV8711CTL_REV_DIRECTION  0x002
#define DRV8711CTL_ENABLE         0x001

// TORQUE_REG settings
#define DRV8711TRQ_BEMF_50us      0x000
#define DRV8711TRQ_BEMF_100us     0x100
#define DRV8711TRQ_BEMF_200us     0x200
#define DRV8711TRQ_BEMF_300us     0x300
#define DRV8711TRQ_BEMF_400us     0x400
#define DRV8711TRQ_BEMF_600us     0x500
#define DRV8711TRQ_BEMF_800us     0x600
#define DRV8711TRQ_BEMF_1ms       0x700
#define DRV8711TRQ_TORQUE_MASK    0x0FF

// OFF_REG settings
#define DRV8711OFF_STEPMOTOR      0x000
#define DRV8711OFF_DUALMOTORS     0x100
#define DRV8711OFF_OFFTIME_MASK   0x0FF

// BLANK_REG settings
#define DRV8711BLNK_ADAPTIVE_BLANK 0x100
#define DRV8711BLNK_BLANKTIME_MASK 0x0FF

// DECAY_REG settings
#define DRV8711DEC_SLOW_DECAY     0x000
#define DRV8711DEC_SLOW_MIXED     0x100
#define DRV8711DEC_FAST_DECAY     0x200
#define DRV8711DEC_MIXED_DECAY    0x300
#define DRV8711DEC_SLOW_AUTOMIX   0x400
#define DRV8711DEC_AUTOMIX        0x500
#define DRV8711DEC_DECAYTIME_MASK 0x0FF

// STALL_REG settings
#define DRV8711STL_DIVIDE_32      0x000
#define DRV8711STL_DIVIDE_16      0x400
#define DRV8711STL_DIVIDE_8       0x800
#define DRV8711STL_DIVIDE_4       0xC00
#define DRV8711STL_STEPS_1        0x000
#define DRV8711STL_STEPS_2        0x100
#define DRV8711STL_STEPS_4        0x200
#define DRV8711STL_STEPS_8        0x300
#define DRV8711STL_THRES_MASK     0x0FF

// DRIVE_REG settings
#define DRV8711DRV_HIGH_50mA      0x000
#define DRV8711DRV_HIGH_100mA     0x400
#define DRV8711DRV_HIGH_150mA     0x800
#define DRV8711DRV_HIGH_200mA     0xC00

#define DRV8711DRV_LOW_100mA      0x000
#define DRV8711DRV_LOW_200mA      0x100
#define DRV8711DRV_LOW_300mA      0x200
#define DRV8711DRV_LOW_400mA      0x300

#define DRV8711DRV_HIGH_250ns     0x000
#define DRV8711DRV_HIGH_500ns     0x040
#define DRV8711DRV_HIGH_1us       0x080
#define DRV8711DRV_HIGH_2us       0x0C0

#define DRV8711DRV_LOW_250ns      0x000
#define DRV8711DRV_LOW_500ns      0x010
#define DRV8711DRV_LOW_1us        0x020
#define DRV8711DRV_LOW_2us        0x030

#define DRV8711DRV_OCP_1us        0x000
#define DRV8711DRV_OCP_2us        0x004
#define DRV8711DRV_OCP_4us        0x008
#define DRV8711DRV_OCP_8us        0x00C

#define DRV8711DRV_OCP_250mV      0x000
#define DRV8711DRV_OCP_500mV      0x001
#define DRV8711DRV_OCP_750mV      0x002
#define DRV8711DRV_OCP_1000mV     0x003

// STATUS_REG settings
#define DRV8711STS_LATCHED_STALL  0x080
#define DRV8711STS_STALL          0x040
#define DRV8711STS_PREDRIVE_B     0x020
#define DRV8711STS_PREDRIVE_A     0x010
#define DRV8711STS_UNDERVOLT      0x008
#define DRV8711STS_OVERCUR_B      0x004
#define DRV8711STS_OVERCUR_A      0x002
#define DRV8711STS_OVERTEMP       0x001
// Define DRV8711 register fields
#define ENBL_FIELD    0x0001

// Define DRV8711 control register bits
#define DRV8711CTL_ENBL      0x0001
// Power mode settings
#define DRV8711_HALF  0
#define DRV8711_FULL  1

// Field definitions
#define FIELD(reg, shf, bits) (((reg & 7) << 8) | ((shf & 0xF) << 4) | ((bits - 1) & 7))
#define ENBL_FIELD     FIELD(CTRL_REG, 0, 1)
#define RDIR_FIELD     FIELD(CTRL_REG, 1, 1)
#define MODE_FIELD     FIELD(CTRL_REG, 3, 4)
#define EXSTALL_FIELD  FIELD(CTRL_REG, 7, 1)
#define ISGAIN_FIELD   FIELD(CTRL_REG, 8, 2)
#define DTIME_FIELD    FIELD(CTRL_REG, 10, 2)

#define TORQUE_FIELD   FIELD(TORQUE_REG, 0, 8)
#define SAMPLTH_FIELD  FIELD(TORQUE_REG, 8, 3)

#define TOFF_FIELD     FIELD(OFF_REG, 0, 8)
#define PWMMODE_FIELD  FIELD(OFF_REG, 8, 1)

#define TBLANK_FIELD   FIELD(BLANK_REG, 0, 8)
#define ABT_FIELD      FIELD(BLANK_REG, 8, 1)

#define TDECAY_FIELD   FIELD(DECAY_REG, 0, 8)
#define DECMOD_FIELD   FIELD(DECAY_REG, 8, 3)

#define SDTHR_FIELD    FIELD(STALL_REG, 0, 8)
#define SDCNT_FIELD    FIELD(STALL_REG, 8, 2)
#define VDIV_FIELD     FIELD(STALL_REG, 10, 2)

#define OCPTH_FIELD    FIELD(DRIVE_REG, 0, 2)
#define OCPDEG_FIELD   FIELD(DRIVE_REG, 2, 2)
#define TDRIVEN_FIELD  FIELD(DRIVE_REG, 4, 2)
#define TDRIVEP_FIELD  FIELD(DRIVE_REG, 6, 2)
#define IDRIVEN_FIELD  FIELD(DRIVE_REG, 8, 2)
#define IDRIVEP_FIELD  FIELD(DRIVE_REG, 10, 2)

class DRV8711 {
public:
    DRV8711(SPIClass &spi, byte CSpin);

    void begin(byte drive, unsigned int microsteps);
    void begin(byte drive, unsigned int microsteps, byte decay_mode, byte gate_speed, byte gate_drive, byte deadtime);
    void set_reg(byte reg, int val);
    int get_reg(byte reg);
    byte get_status();
    void clear_status();
    void set_enable(bool enable);
    void power(byte code);
    void end();
    byte reg_field(int field);
    void write_reg_field(int field, byte val);
    void set_torque(uint8_t value);
    void set_gain(uint8_t gain);
    void configureCtrlRegister();
    void enable_motor();
    void decode_field(int field, int &reg, int &shf, int &bits, int &mask);
   
private:
    void setup_spi();
    char transfer(char data);

    SPIClass &_spi;
    byte _CSpin;
    int control_reg;
    byte drive_level;
};

#endif
