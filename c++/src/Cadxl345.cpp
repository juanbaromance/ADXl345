#include <bitset>
#include <map>
#include <tuple>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include "adxl345/Cadxl345.h"
#include "common/raspberry_iface.h"
#include "common/CNetworkSlot.h"
#include <stdint.h>
#include <cmath>
#if __cplusplus==201103L
                typedef std::chrono::high_resolution_clock Clock;
                typedef std::chrono::microseconds MyTimeUnits;
#endif

using namespace std;

Cadxl345Config::Cadxl345Config(string file_spec) : CChipsetConfig( file_spec )
{
    rate = ADXL345_DATARATE_800_HZ;
    calibration.resize(3);
    for( auto & calib : calibration )
        calib = CalibDataT(0,1);

    sync_configuration();
}

int Cadxl345Config::sync_configuration()
{

}

Cadxl345 *Cadxl345::ghost;
Cadxl345::Cadxl345(int iic_address, string name, string config_spec) :
    CiicDevice( 0x53 | ( iic_address & 0xf ) ),
    Cadxl345Config( config_spec )
{
    ghost = this;
    CiicDevice::bus_address( Cadxl345Config::CChipsetConfig::_bus_address );
    registers[ ADXL345_DEVID       ] = register_spec_t( & dev_signature  , "Signature", 0 );

    registers[ ADXL345_BW_RATE     ] = register_spec_t( & rate_power_mode, "Rate-PowerMode", 0 );
    registers[ ADXL345_INT_SOURCE  ] = register_spec_t( & int_source     , "Interrupt source", 0 );

    registers[ ADXL345_DATAX0      ] = register_spec_t( & lsb_x, "Xlow"  , 0 );
    registers[ ADXL345_DATAX1      ] = register_spec_t( & msb_x, "Xhigh" , 0 );
    registers[ ADXL345_DATAY0      ] = register_spec_t( & lsb_y, "Ylow"  , 0 );
    registers[ ADXL345_DATAY1      ] = register_spec_t( & msb_y, "Yhigh" , 0 );
    registers[ ADXL345_DATAZ0      ] = register_spec_t( & lsb_z, "Zlow"  , 0 );
    registers[ ADXL345_DATAZ1      ] = register_spec_t( & msb_z, "Zhigh" , 0 );
    registers[ ADXL345_DATA_FORMAT ] = register_spec_t( & data_format, "DataFormat" , 0 );
    active_range = 2;
    sampling = 100;



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

    profile_specification[ ACQUISITION_SPEC ] =  new AcquisitionSettings();
    t = new std::thread( & Cadxl345::monitor, this );
}

vector<float> Cadxl345::state()
{
#define GtoMsec2 9.8

    static vector <float> tmp(3);
    receive( ADXL345_XYZ );
    if( _err == ADXL345_NO_ERR  )
    {
        float scaling = ADXL345_SCALE_FACTOR;
        scaling = full_resolver ? scaling : ( scaling * ( active_range >> 1 ) );
//        printf("$%02x%02x:%02x%02x:%02x%02x(%4.4f)( $%08x ) :: ",
//               msb_x, lsb_x, msb_y, lsb_y, msb_z, lsb_z, scaling, data_format  );

        tmp[0] = ( c2ToDec( ( static_cast<short>(msb_x) << 8) + lsb_x, 16 ) * scaling * GtoMsec2);
        tmp[1] = ( c2ToDec( ( static_cast<short>(msb_y) << 8) + lsb_y, 16 ) * scaling * GtoMsec2);
        tmp[2] = ( c2ToDec( ( static_cast<short>(msb_z) << 8) + lsb_z, 16 ) * scaling * GtoMsec2);

        int i;
        for( auto & acc : tmp )
        {
            CalibDataT *c = & calibration[ i++ ];
            acc = acc * c->slope + c->offset;
        }
        _state( tmp[ X ], tmp[ Y ], tmp[ Z ] );

    }
    return( tmp );
}

string Cadxl345::report(Numerology mux)
{

    std::ostringstream oss;
    if( mux == ADXL345_CHIPSET )
    {
        for( const auto & r: registers )
        {
            uint8_t *r_val;
            string r_name;
            tie( r_val, r_name, ignore ) = r.second;
            oss << setfill(' ') << setw(25) << r_name
                << " register( $"
                << setw(3) << setfill('0') << hex << r.first  << ") :: $"
                << setw(3) << setfill('0') << hex << bitset<8>(*r_val).to_ulong() << endl;
        }
    }
    else if ( mux == ADXL345_GEOMETRY )
    {

        CiicDevice::TraceLevelEnum report =
                ( _trace_level > -2  ) ? CiicDevice::Informer : CiicDevice::Silent;
        oss << typeid(this).name() <<  " g(x,y,z,pitch,roll) :: ";
        for( const auto & sample : _state.v )
            oss << setw(10) << std::fixed << std::right << sample << " ";
        operation_log( oss.str(), report );

    }
    return( CChipsetConfig::report() + oss.str() );
}

void Cadxl345::monitor()
{
    {
        std::ostringstream oss;
        oss << typeid(this).name() <<  __FUNCTION__ << "running device sampling :: " + to_string(sampling) + "(msec)";
        operation_log( oss.str(), CiicDevice::Informer );
    }

    while( 1 )
    {
        int err_id;
        std::this_thread::sleep_for(std::chrono::milliseconds( sampling ));


        if( state().size() )
        {
            profile.payload.accelerometer.x     = _state.x;
            profile.payload.accelerometer.y     = _state.y;
            profile.payload.accelerometer.z     = _state.z;
            profile.payload.accelerometer.pitch = _state.pitch;
            profile.payload.accelerometer.roll  = _state.roll;
            report(ADXL345_GEOMETRY);
        }

        if( sr[ connection ] == 0 )
            continue;

        bitset<32> mask;
        mask.set(ACCELEROMETER_STATE);
        profile.operation_mask = mask.to_ulong();
        if( ( err_id = sync_peer() ) < 0 )
        {
            std::ostringstream oss;
            oss << typeid(this).name() <<  ".sync_peer : f a i l u r e d : err_id(" + to_string(err_id) + ")";
            operation_log( oss.str(), CiicDevice::Warning );
            continue;
        }

    }

}

string Cadxl345::err(int index)
{
    static map <int,string> err_table = {
        {ERR_ADXL345_XMITT_BUS_ERR ," # BusError on transmission"},
        {ERR_ADXL345_PAYLOAD_OOSYNC," # Expected Payload :: out of sync"},
        {ERR_ADXL345_RANGE_NOT_FOUND," # Dynamic G-Range unknown"}
    };

    index = ( index < 0 ) ? _err : index;
    return( ( err_table.find( index ) == err_table.end() )
            ? ( "unknown err(" + std::to_string(index) + ")" )
            : ( string("(ADXL345)") + err_table.find( index )->second ));

}

Cadxl345Config::dataRate_t Cadxl345::rate( Cadxl345Config::dataRate_t probe )
{

    uint8_t tmp = receive(ADXL345_BW_RATE);
    if( _err != ADXL345_NO_ERR )
        return( Cadxl345Config::rate );

    vector <uint8_t> payload(2);
    tmp &= 0x10;
    tmp |= probe;
    payload[0] = ADXL345_BW_RATE ;
    payload[1] = tmp;
    xmitt( payload );

    return( Cadxl345Config::rate = static_cast<Cadxl345Config::dataRate_t>( receive(ADXL345_BW_RATE) ) );
}


/* POWER_CTL Bits */
#define PCTL_LINK	    (1 << 5)
#define PCTL_AUTO_SLEEP (1 << 4)
#define PCTL_MEASURE	(1 << 3)
#define PCTL_SLEEP	    (1 << 2)
#define PCTL_WAKEUP(x)	((x) & 0x3)

int Cadxl345::sleep( bool val )
{
    uint8_t tmp = receive( ADXL345_POWER_CTL );
    if( _err != ADXL345_NO_ERR )
        return( _err );

    if( val )
    {
        tmp &= ~(PCTL_AUTO_SLEEP | PCTL_LINK);
    }
    else
        tmp |=  PCTL_AUTO_SLEEP | PCTL_LINK;
    tmp |= PCTL_MEASURE * ( val ? 0 : 1 );
    vector <uint8_t> payload(2);
    payload[0] = ADXL345_POWER_CTL;
    payload[1] = tmp;
    xmitt( payload );

    rate( Cadxl345Config::rate );


    return( _err );
}


int Cadxl345::tapping( Numerology mode, bitset<3> mask, int msec_duration , int msec_latency, int threshold, Numerology int_mapping )
{

}

void Cadxl345::settings( adxl345_payload::acquisition_t tmp )
{
    if( tmp.msec < 10 )
        return;

    static map <float,dataRate_t> rates = {
        { 3200, ADXL345_DATARATE_3200_HZ },
        { 1600 ,ADXL345_DATARATE_1600_HZ },
        {  800, ADXL345_DATARATE_800_HZ  },
        {  400, ADXL345_DATARATE_400_HZ  },
        {  200, ADXL345_DATARATE_200_HZ  },
        {  100, ADXL345_DATARATE_100_HZ  },
        {   50, ADXL345_DATARATE_50_HZ   },
        {   25, ADXL345_DATARATE_25_HZ   },
        {   12.5, ADXL345_DATARATE_12_5_HZ },
        {    6.25, ADXL345_DATARATE_6_25HZ  },
        {    3.13, ADXL345_DATARATE_3_13_HZ },
        {    1.56, ADXL345_DATARATE_1_56_HZ },
        {    0.78, ADXL345_DATARATE_0_78_HZ },
        {    0.39, ADXL345_DATARATE_0_39_HZ },
        {    0.20, ADXL345_DATARATE_0_20_HZ },
        {    0.10, ADXL345_DATARATE_0_10_HZ }
    };

    dataRate_t probe_rate = ADXL345_DATARATE_0_10_HZ;
    for( const auto & rate : rates )
        if( rate.first < tmp.bandwidth )
            probe_rate = rate.second;
        else
            break;

    std::ostringstream oss;
    oss << typeid(ghost).name() + string(".settings tweak :: ")
        << std::setw(4) << ghost->sampling << " to " << std::setw(4) << tmp.msec << "(msec)";
    oss << " bandwidth ( " <<  std::scientific << tmp.bandwidth << " ) encoded as : " << probe_rate;
    operation_log( oss.str(), CiicDevice::Informer );

    ghost->sampling = tmp.msec;
    ghost->rate( probe_rate );

}

int Cadxl345::freefall(float mg_threshold, int msec_window, Cadxl345Config::Numerology pin )
{
    vector <uint8_t> payload;
    payload.push_back( ADXL345_THRESH_FF );
    payload.push_back( static_cast<int>( mg_threshold / 62.5) );
    if( xmitt( payload ) < 0 )
        return( _err );

    payload.clear();
    payload.push_back( ADXL345_THRESH_FF );
    payload.push_back( static_cast<int>( msec_window / 5.0 ) );
    if( xmitt( payload ) < 0 )
        return( _err );

    uint8_t aux, tmp = receive( ADXL345_INT_MAP );
    aux = tmp & ( ~( 1 << 2 ) );
    bool int_enabled = false;
    if( ( pin == FreeFallPin1 ) || ( pin == FreeFallPin2 ) )
    {
        aux |= ( pin << 2 );
        payload.clear();
        payload.push_back( ADXL345_INT_MAP );
        payload.push_back( aux);
        if( xmitt( payload ) < 0 )
            return( _err );
        int_enabled = true;
    }

    tmp = receive( ADXL345_INT_ENABLE );
    aux = tmp & ( ~( 1 << 2 ) );
    aux |= ( int_enabled ? 1 : 0 ) << 2;
    payload.clear();
    payload.push_back( ADXL345_INT_ENABLE );
    payload.push_back( aux );
    xmitt( payload ) ;
    return( _err );

}

int Cadxl345::offset(const vector<int> *offset )
{
    if( offset->size() < 3 )
        return ( _err = ERR_OFFSET_OOSPEC );

    vector <uint8_t> payload(2);
    int i = 0;
    for( const auto & val : *offset )
    {
        float tmp = val / 15.6;
        payload[0] = ADXL345_CALIBRATION_OFFSET + i;
        payload[1] = DecToc2( static_cast<int>( round( tmp ) ) );
        if( xmitt( payload ) < 0 )
            break;
        i++;
    }
    return( _err );

}

int Cadxl345::range( Numerology probe_range, bool full_resolution )
{
    static map <Numerology,uint8_t> ranges = {
        { PlusMinus2G , 0},
        { PlusMinus4G , 1},
        { PlusMinus8G , 2},
        { PlusMinus16G, 3},
    };

    if( ranges.find( probe_range ) == ranges.end() )
        return ( _err = ERR_ADXL345_RANGE_NOT_FOUND );

    uint8_t range = ranges.find( probe_range )->second;
    uint8_t r_val = 0;

    data_format = receive( ADXL345_DATA_FORMAT );
    if( _err != ADXL345_NO_ERR )
        return ( _err = ERR_ADXL345_PAYLOAD_OOSYNC );


    r_val  = data_format & ( ~( 0x3 | ( 1 << 3 ) ) );
    r_val |= range | ( ( full_resolution ? 1 : 0 ) << 3 );
    vector< uint8_t> payload;
    payload.push_back( ADXL345_DATA_FORMAT );
    payload.push_back( r_val );
    if( xmitt( payload ) == 0 )
    {
        active_range = ( 1 << ( range + 1 ) );
        full_resolver = full_resolution;
        data_format = receive( ADXL345_DATA_FORMAT );
    }
    else
        _err = ERR_ADXL345_XMITT_BUS_ERR;


    return( _err );
}

int  Cadxl345::c2ToDec( int num, int num_size )
{
    num_size-- ;
    int cast = 0;
    for( int i = 0; i < num_size; i++ )
        cast |= ( 1 << i );

    int ret = num;
    if( num & ( 1 << num_size ) )
        ret = -( ( ( ~ ( num & cast ) ) +1 ) & cast );

    return( ret );
}

uint8_t Cadxl345::DecToc2( int val )
{
    return -(unsigned int)val;
}


uint8_t Cadxl345::receive( Cadxl345Config::Numerology offset )
{

    if( offset == ADXL345_CHIPSET )
    {
         uint8_t tmp = -1;
         for( const auto & r: registers )
             receive( r.first );
         return( tmp );
    }

    if( offset == ADXL345_XYZ )
    {
        uint8_t tmp = -1;
        vector< uint8_t> register_address = { ADXL345_DATAX0 };
         if( xmitt( register_address ) == 0 )
         {
             vector< uint8_t> raw_data = CiicDevice::receive( 6 );
             if( raw_data.size() == 6 )
             {
                 lsb_x = raw_data[0];
                 msb_x = raw_data[1];
                 lsb_y = raw_data[2];
                 msb_y = raw_data[3];
                 lsb_z = raw_data[4];
                 msb_z = raw_data[5];
                 _err = ADXL345_NO_ERR;
             }
         }
         else
             _err = ERR_ADXL345_XMITT_BUS_ERR;
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




void Cadxl345::ip_callback(socket_header_t *header, void *payload)
{
    if( 0 )
    {
        std::ostringstream oss;
        printf("%d::", header->bit.msg_id );
        oss << _name + "." + string( __FUNCTION__ ) << "  :: id received :: " << hex << header->bit.msg_id;
        operation_log( oss.str(), CiicDevice::Informer );
    }

    switch( header->bit.msg_id )
    {

    case __PEER_HEARTBEAT_id:
        break;

    case ADXL345Profile:
    {
        ADXL345ProfileT *profile = (ADXL345ProfileT*)payload;
        bitset<ProfileSpec> bits( profile->operation_mask );

        for( int i = 0; i < bits.size(); i++ )
        {
            if( bits[ i ] )
            {
                adxl345_operation profile_id = static_cast<adxl345_operation>(i);
                if( profile_specification.find( profile_id ) == profile_specification.end() )
                    continue;
                profile_specification.find( profile_id )->second->resolver( payload );
            }
        }
    }
        break;

    case __KILL_CONNECTION_id:
    {
        sr.reset( connection );
        std::ostringstream oss;
        oss << typeid(this).name() + string(" # peer relase s a m p l e d");
        operation_log( oss.str(), CiicDevice::Informer );
     }
        break;

    case __FULL_DUPLEX_COMPLETED_id:
    {
        sr.set( connection );

    }
        break;

    }


}

void AcquisitionSettings::resolver( void *payload )
{
   ADXL345ProfileT *profile_data = (ADXL345ProfileT *)payload;
   Cadxl345::settings( profile_data->payload.acquisition );
}



Cadxl345IPProfile::Cadxl345IPProfile(RaspBerryIPSpecification profile_id) : _profile_id(profile_id)
{

}

int Cadxl345IPProfile::sync_peer()
{
    return( CNetworkSlot::send( _profile_id, & profile, sizeof( profile ) ) );
}
