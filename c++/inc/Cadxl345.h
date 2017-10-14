#ifndef CADXL345_H
#define CADXL345_H

#include "hardware/Ciic_device.h"
#include "hardware/CChipsetConfig.h"

#include <bitset>
#include <string>
#include <map>
#include <tuple>
#include <iostream>
using namespace std;

#define ADXL345_SCALE_FACTOR    0.0039

class Cadxl345Config : public CChipsetConfig {
public:

    typedef enum
    {
	ADXL345_DATARATE_3200_HZ    = 0b1111, // 1600Hz Bandwidth   140µA IDD
	ADXL345_DATARATE_1600_HZ    = 0b1110, //  800Hz Bandwidth    90µA IDD
	ADXL345_DATARATE_800_HZ     = 0b1101, //  400Hz Bandwidth   140µA IDD
	ADXL345_DATARATE_400_HZ     = 0b1100, //  200Hz Bandwidth   140µA IDD
	ADXL345_DATARATE_200_HZ     = 0b1011, //  100Hz Bandwidth   140µA IDD
	ADXL345_DATARATE_100_HZ     = 0b1010, //   50Hz Bandwidth   140µA IDD
	ADXL345_DATARATE_50_HZ      = 0b1001, //   25Hz Bandwidth    90µA IDD
	ADXL345_DATARATE_25_HZ      = 0b1000, // 12.5Hz Bandwidth    60µA IDD
	ADXL345_DATARATE_12_5_HZ    = 0b0111, // 6.25Hz Bandwidth    50µA IDD
	ADXL345_DATARATE_6_25HZ     = 0b0110, // 3.13Hz Bandwidth    45µA IDD
	ADXL345_DATARATE_3_13_HZ    = 0b0101, // 1.56Hz Bandwidth    40µA IDD
	ADXL345_DATARATE_1_56_HZ    = 0b0100, // 0.78Hz Bandwidth    34µA IDD
	ADXL345_DATARATE_0_78_HZ    = 0b0011, // 0.39Hz Bandwidth    23µA IDD
	ADXL345_DATARATE_0_39_HZ    = 0b0010, // 0.20Hz Bandwidth    23µA IDD
	ADXL345_DATARATE_0_20_HZ    = 0b0001, // 0.10Hz Bandwidth    23µA IDD
	ADXL345_DATARATE_0_10_HZ    = 0b0000  // 0.05Hz Bandwidth    23µA IDD (default value)
    } dataRate_t;

    typedef enum Numerology {

	ADXL345_MAGIC           = 0xe5,

	// registers map
	ADXL345_DEVID           = 0x00 ,// R   Device ID.
	ADXL345_THRESH_TAP      = 0x1D ,// R/W Tap threshold.

	ADXL345_OFSX            = 0x1E ,// R/W X-axis offset.
	ADXL345_OFSY            = 0x1F ,// R/W Y-axis offset.
	ADXL345_OFSZ            = 0x20 ,// R/W Z-axis offset.
	ADXL345_CALIBRATION_OFFSET = ADXL345_OFSX,

	ADXL345_DUR             = 0x21 ,// R/W Tap duration.
	ADXL345_LATENT          = 0x22 ,// R/W Tap latency.
	ADXL345_WINDOW          = 0x23 ,// R/W Tap window.
	ADXL345_THRESH_ACT      = 0x24 ,// R/W Activity threshold.
	ADXL345_THRESH_INACT    = 0x25 ,// R/W Inactivity threshold.
	ADXL345_TIME_INACT      = 0x26 ,// R/W Inactivity time.
	ADXL345_ACT_INACT_CTL   = 0x27 ,// R/W Axis enable control for activity and inactivity detection.
	ADXL345_THRESH_FF       = 0x28 ,// R/W Free-fall threshold.
	ADXL345_TIME_FF         = 0x29 ,// R/W Free-fall time.
	ADXL345_TAP_AXES        = 0x2A ,// R/W Axis control for tap/double tap.
	ADXL345_ACT_TAP_STATUS  = 0x2B ,// R   Source of tap/double tap.
	ADXL345_BW_RATE         = 0x2C ,// R/W Data rate and power mode control.
	ADXL345_POWER_CTL       = 0x2D ,// R/W Power saving features control.
	ADXL345_INT_ENABLE      = 0x2E ,// R/W Interrupt enable control.
	ADXL345_INT_MAP         = 0x2F ,// R/W Interrupt mapping control.
	ADXL345_INT_SOURCE      = 0x30 ,// R   Source of interrupts.
	ADXL345_DATA_FORMAT     = 0x31 ,// R/W Data format control.
	ADXL345_DATAX0          = 0x32 ,// R   X-Axis Data 0.
	ADXL345_DATAX1          = 0x33 ,// R   X-Axis Data 1.
	ADXL345_DATAY0          = 0x34 ,// R   Y-Axis Data 0.
	ADXL345_DATAY1          = 0x35 ,// R   Y-Axis Data 1.
	ADXL345_DATAZ0          = 0x36 ,// R   Z-Axis Data 0.
	ADXL345_DATAZ1          = 0x37 ,// R   Z-Axis Data 1.
	ADXL345_FIFO_CTL        = 0x38 ,// R/W FIFO control.
	ADXL345_FIFO_STATUS     = 0x39 ,// R   FIFO status.

	/* virtual stuff */
	ADXL345_XYZ             = 0xfe,
	ADXL345_CHIPSET         = 0xff ,// Full chipset

	/* Ranges */
	PlusMinus2G  = 0b00,
	PlusMinus4G  = 0b01,
	PlusMinus8G  = 0b10,
	PlusMinus16G = 0b11,

	/* Freefall  stuff */
	FreeFallPin1 = 0,
	FreeFallPin2 = 1,
	FreeFallNoPin = 2,

	ADXL345_NO_ERR = 0,
	ERR_ADXL345_PAYLOAD_OOSYNC = -1,
	ERR_ADXL345_XMITT_BUS_ERR = -2,
	ERR_ADXL345_ACQUISITION_TIMEOUT = -3,
	ERR_ADXL345_ACQUISITION_SYNC = -4,
	ERR_ADXL345_RANGE_NOT_FOUND = -5,
	ERR_OFFSET_OOSPEC = -6,
    }Numerology;

    Cadxl345Config( string file_spec );
    int sync_configuration();

protected:

};



class Cadxl345 : public CiicDevice, public Cadxl345Config
{
public:
    Cadxl345(int iic_address, string name, string config_spec = "./config/adxl345.xml");
    vector <float> state();

    int offset   ( const vector<int> *offset );
    int range    ( Numerology probe_range, bool full_resolution );
    int sleep    ( bool val );
    int freefall ( float mg_threshold = 500, int msec_window = 200, Numerology pin = FreeFallPin1 );
    int tapping  ( Numerology mode, bitset<3> mask, int msec_duration, int msec_latency, int threshold, Numerology int_mapping);

protected:
    int c2ToDec(int num, int num_size);
    uint8_t DecToc2(int num);
    string report();
    int active_range;
    bool full_resolver;

    string err( int index = -1 );
    void ip_callback(socket_header_t *header, void *payload );

    uint8_t dev_signature, rate_power_mode, int_source, data_format;
    uint8_t lsb_x, msb_x, lsb_y, msb_y, lsb_z, msb_z;
    uint8_t receive(Numerology offset);

    typedef string (*register_decoder_t)(uint16_t);
    typedef tuple <  uint8_t*, string, register_decoder_t> register_spec_t;
    map <Numerology, register_spec_t> registers;

};


#endif


