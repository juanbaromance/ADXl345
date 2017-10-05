#include <bitset>
#include <map>
#include <tuple>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include "adxl345/Cadxl345.h"

#if __cplusplus==201103L
                typedef std::chrono::high_resolution_clock Clock;
                typedef std::chrono::microseconds MyTimeUnits;
#endif

using namespace std;

Cadxl345Config::Cadxl345Config(string file_spec) : CChipsetConfig( file_spec )
{
    sync_configuration();
}

int Cadxl345Config::sync_configuration()
{

}

Cadxl345::Cadxl345(int iic_address, string name, string config_spec) :
    CiicDevice( 0x53 | ( iic_address & 0xf ) ),
    Cadxl345Config( config_spec )
{
    CiicDevice::bus_address( Cadxl345Config::CChipsetConfig::_bus_address );
    registers[ ADXL345_DEVID      ] = register_spec_t( & dev_signature  , "Signature", 0 );
    registers[ ADXL345_BW_RATE    ] = register_spec_t( & rate_power_mode, "Rate-PowerMode", 0 );
    registers[ ADXL345_INT_SOURCE ] = register_spec_t( & int_source     , "Interrupt source", 0 );
    registers[ ADXL345_DATAX0     ] = register_spec_t( & lsb_x, "Xlow"  , 0 );
    registers[ ADXL345_DATAX1     ] = register_spec_t( & msb_x, "Xhigh" , 0 );
    registers[ ADXL345_DATAY0     ] = register_spec_t( & lsb_y, "Ylow"  , 0 );
    registers[ ADXL345_DATAY1     ] = register_spec_t( & msb_y, "Yhigh" , 0 );
    registers[ ADXL345_DATAZ0     ] = register_spec_t( & lsb_z, "Zlow"  , 0 );
    registers[ ADXL345_DATAZ1     ] = register_spec_t( & msb_z, "Zhigh" , 0 );

    int elapsed = 0;

    {
#if __cplusplus==201103L
        Clock::time_point t0 = Clock::now();
#else
        struct timeval start, end;
        gettimeofday(&start, NULL);
#endif
        receive( ADXL345_CHIPSET );

        int latency;
#if __cplusplus==201103L
        Clock::time_point t1 = Clock::now();
        auto experiment = std::chrono::duration_cast<MyTimeUnits>(t1 - t0);
        latency = experiment.count();
#else
        gettimeofday(&end, NULL);
        double elapsed = ((end.tv_sec - start.tv_sec) * 1000)
                + (end.tv_usec / 1000 - start.tv_usec / 1000);
        latency = static_cast<int>(elapsed);
#endif
        elapsed += latency;
    }

    string tmp = ( dev_signature == ADXL345_MAGIC ) ? " :: supported" : "";
    cout << "Actual chipset(0x" << hex << CChipsetConfig::_bus_address << ") state(" << _err << ")" << tmp + "(" + to_string(elapsed) + ")usec" << endl
         << report().c_str() << endl;
}

string Cadxl345::report()
{
    std::ostringstream oss;
    for( auto r: registers )
    {
        uint8_t *r_val;
        string r_name;
        tie( r_val, r_name, ignore ) = r.second;
        oss << setfill(' ') << setw(25) << r_name
            << " register( $"
            << setw(3) << setfill('0') << hex << r.first  << ") :: $"
            << setw(3) << setfill('0') << hex << bitset<8>(*r_val).to_ulong() << endl;
    }
    return( CChipsetConfig::report() + oss.str() );
}

string Cadxl345::err(int index)
{
    static map <int,string> err_table = {
        {ERR_ADXL345_XMITT_BUS_ERR ," # BusError on transmission"},
        {ERR_ADXL345_PAYLOAD_OOSYNC," # Expected Payload :: out of sync"}
    };
    index = ( index < 0 ) ? _err : index;
    return( ( err_table.find( index ) == err_table.end() ) ? ( "unknown err(" + std::to_string(index) + ")" ) : err_table.find( index )->second );

}


uint8_t Cadxl345::receive( Cadxl345Config::Numerology offset )
{

    if( offset == ADXL345_CHIPSET )
    {
         uint8_t tmp = -1;
         for( auto r: registers )
             receive( r.first );
         return( tmp );
    }

    uint8_t tmp = -1;
    uint8_t *ptr = & tmp;
    if( registers.find( offset ) != registers.end() )
        tie( ptr, ignore, ignore ) = registers.find( offset )->second;

    vector< uint8_t> register_address = { static_cast<uint8_t>( offset & 0xff ) };
    if( xmitt( register_address ) == 0 )
    {
        vector <uint8_t> probe = CiicDevice::receive();
        if( probe.size() )
        {
            _err = ADXL345_NO_ERR;
            return( *ptr = probe.at( 0 ) );
        }
        else
            _err = ERR_ADXL345_PAYLOAD_OOSYNC;
    }
    else
        _err = ERR_ADXL345_XMITT_BUS_ERR;
    return( tmp );
}
