/*
 * JB#11112017
 * Todo list ::
 * 1.- IP Readout For the Activity and Inactivity, AutoSleep features
 * 2.- Import from the XML configuration file the calibration parameters
 * 3.- Import from the XML configuration the default acquisition, freefall, tapping and autosleep parameters
 * 4.- Connect with a real interrupt service
*/


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
#include "common/CStatistic.h"
#include <stdint.h>
#include <cmath>
#include <list>
#include <algorithm>
#include <mutex>
#include <condition_variable>


#if __cplusplus==201103L
                typedef std::chrono::high_resolution_clock Clock;
                typedef std::chrono::microseconds MyTimeUnits;
#endif

using namespace std;


static std::mutex              mtx;
static std::condition_variable cv;

template<class T>
bool check_range(T value, T min, T max)
{
    return (value >= min) && (value <= max);
}


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


class TapSettings : public Activity {
public:
    void resolver( void *payload )
    {
        ADXL345ProfileT *profile_data = (ADXL345ProfileT *)payload;
        bitset<32>operation(profile_data->payload.tap.masking);
        if( operation[30] )
        {
            Cadxl345::tapping_state();
            return;
        }

        if( operation[31])
            Cadxl345::tapping( profile_data->payload.tap );
        else
            Cadxl345::freefall_specs( profile_data->payload.tap.threshold, profile_data->payload.tap.msec_window );
    }
};

class AcquisitionSettings : public Activity {
public:
    void resolver( void *payload )
    {
        ADXL345ProfileT *profile_data = (ADXL345ProfileT *)payload;
        Cadxl345::settings( profile_data->payload.acquisition );
    }
};

class SleepSettings : public Activity {
public:
    void resolver( void *payload )
    {
        ADXL345ProfileT *tmp = ( ADXL345ProfileT* )payload;
        Cadxl345::sleep( tmp->payload.sleep );
    }
};

class SelfTesting: public Activity {
public:
    void resolver( void *payload )
    {
        ADXL345ProfileT *tmp = ( ADXL345ProfileT* )payload;
        Cadxl345::autoprobe( tmp->payload.enable );
    }
};


Cadxl345 *Cadxl345::ghost;


Cadxl345::Cadxl345(int iic_address, string name, string config_spec) :
    CiicDevice( 0x53 | ( iic_address & 0xf ) ),
    Cadxl345Config( config_spec )
{
    ghost = this;
    CiicDevice::bus_address( Cadxl345Config::CChipsetConfig::_bus_address );
    registers[ ADXL345_DEVID       ] = register_spec_t( & dev_signature  , "Signature", 0 );

    registers[ ADXL345_BW_RATE     ] = register_spec_t( & rate_power_mode, "Rate-PowerMode", 0 );
    registers[ ADXL345_INT_SOURCE  ] = register_spec_t( & int_source     , "int source", 0 );

    registers[ ADXL345_DATAX0      ] = register_spec_t( & lsb_x, "Xlow"  , 0 );
    registers[ ADXL345_DATAX1      ] = register_spec_t( & msb_x, "Xhigh" , 0 );
    registers[ ADXL345_DATAY0      ] = register_spec_t( & lsb_y, "Ylow"  , 0 );
    registers[ ADXL345_DATAY1      ] = register_spec_t( & msb_y, "Yhigh" , 0 );
    registers[ ADXL345_DATAZ0      ] = register_spec_t( & lsb_z, "Zlow"  , 0 );
    registers[ ADXL345_DATAZ1      ] = register_spec_t( & msb_z, "Zhigh" , 0 );
    registers[ ADXL345_DATA_FORMAT ] = register_spec_t( & data_format, "DataFormat" , 0 );

    registers[ ADXL345_ACT_INACT_CTL  ] = register_spec_t( & autosleep_control   , "AutoSleep control" , 0 );
    registers[ ADXL345_THRESH_ACT     ] = register_spec_t( & threshold_activity  , "Activity(mg)" , 0 );
    registers[ ADXL345_THRESH_INACT   ] = register_spec_t( & threshold_inactivity, "Inactivity(mg)" , 0 );
    registers[ ADXL345_WINDOW_INACT   ] = register_spec_t( & window_inactivity   , "Rest(sec)" , 0 );
    registers[ ADXL345_INT_ENABLE     ] = register_spec_t( & int_enable          , "int enable" , 0 );
    registers[ ADXL345_INT_MAP        ] = register_spec_t( & int_map             , "int mapping" , 0 );
    registers[ ADXL345_POWER_CTL      ] = register_spec_t( & power_ctrl            ,"Power Control", 0 );
    registers[ ADXL345_ACT_TAP_STATUS ] = register_spec_t( & tap_status, "tap-status", 0 );

    registers[ ADXL345_TIME_FF        ] = register_spec_t( & freefall.window    , "FreeFall-window", 0 );
    registers[ ADXL345_THRESH_FF      ] = register_spec_t( & freefall.threshold , "FreeFall-threshold", 0 );
    registers[ ADXL345_TAP_AXES       ] = register_spec_t( & tap.mask           , "Tap-axes", 0 );
    registers[ ADXL345_DUR            ] = register_spec_t( & tap.duration       , "Tap-duration", 0 );
    registers[ ADXL345_LATENT         ] = register_spec_t( & tap.latency        , "Tap-latency", 0 );
    registers[ ADXL345_WINDOW         ] = register_spec_t( & tap.window         , "Tap-window", 0 );
    registers[ ADXL345_THRESH_TAP     ] = register_spec_t( & tap.threshold      , "Tap-threshold", 0 );

    scaling = ADXL345_SCALE_FACTOR;
    offset_buffer = { CStatistic("Xoff"), CStatistic("Yoff"), CStatistic("Zoff") };

    /* see page 22 of 36 Table-16 8g.10bit case of use */
    self_test = { range_t(12,148),range_t(-148,-12), range_t(19,232) };

    active_range = 2;
    sampling = 100;
    raw_acquistition = false;
    autoprobing = false;

    int elapsed = 0;

    /* Features not exposed on runtime */
    init_chipset();
    alias = 0;

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
    if( dev_signature ^ ADXL345_MAGIC )
    {
        operation_log( "Chipset detection :: f a i l u r e", CiicDevice::Warning );
        return;
    }

    cout << "Actual chipset(0x" << hex << CChipsetConfig::_bus_address << ") state(" << _err << ")" << tmp + "(" + to_string(elapsed) + ")usec" << endl
         << report().c_str() << endl;

    /* TCP/IP interface */
    profile_specification[ ACQUISITION_SPEC ] = new AcquisitionSettings();
    profile_specification[ SELF_TESTING     ] = new SelfTesting();
    profile_specification[ SLEEP_SPEC       ] = new SleepSettings();
    profile_specification[ TAP_SPEC         ] = new TapSettings();

    t = new std::thread( & Cadxl345::monitor, this );
}


void Cadxl345::init_chipset()
{
    // Interrupts mapping for the AutoSleep states
    {
        bitset<8> tmp(int_map);
        /* Int2 pin */
        tmp.set( Activity_IEbit );
        tmp.set( InActivity_IEbit );

        /* Int1 pin */
        tmp.reset( DoubleTap_IEbit );
        tmp.reset( SingleTap_IEbit );
        tmp.reset( FreeFalling_IEbit );
        xmitt( ADXL345_INT_MAP, static_cast<uint8_t>( tmp.to_ulong() ));
    }

    // Interrupts Enabled for AutoSleep states
    {
        bitset<8> tmp(int_enable);
        tmp = 0;
        tmp.set( DoubleTap_IEbit );
        tmp.set( SingleTap_IEbit );
        tmp.set( FreeFalling_IEbit );
        int_enable = static_cast<uint8_t>(tmp.to_ulong());
        xmitt( ADXL345_INT_ENABLE, static_cast<uint8_t>( tmp.to_ulong() ) );
    }

    // AutoSleep mode is disabled by default
    {
        adxl345_payload::sleep_t parameters;
        parameters.enable = false;
        autosleep(parameters);
    }

    /* initialise freefall stuff with default parameters 30msec/500mg */
    freefall_specs();
    tapping();
}

vector<float> Cadxl345::state()
{
#define GtoMsec2 9.8

    static vector <float> tmp(3);
    receive( ADXL345_XYZ );
    if( _err == ADXL345_NO_ERR  )
    {

        scaling = full_resolver ? scaling : ( scaling * ( active_range >> 1 ) );
//        printf("$%02x%02x:%02x%02x:%02x%02x(%4.4f)( $%08x ) :: ",
//               msb_x, lsb_x, msb_y, lsb_y, msb_z, lsb_z, scaling, data_format  );

        tmp[ X ] = c2ToDec( ( static_cast<short>(msb_x) << 8) + lsb_x, 16 );
        tmp[ Y ] = c2ToDec( ( static_cast<short>(msb_y) << 8) + lsb_y, 16 );
        tmp[ Z ] = c2ToDec( ( static_cast<short>(msb_z) << 8) + lsb_z, 16 );

        if( raw_acquistition == false )
        {
            tmp[ X ] *= scaling * GtoMsec2;
            tmp[ Y ] *= scaling * GtoMsec2;
            tmp[ Z ] *= scaling * GtoMsec2;
            int i = 0;
            for( auto & acc : tmp )
            {
                CalibDataT *c = & calibration[ i++ ];
                acc = acc * c->slope + c->offset;
            }
        }
        else
        {
            std::ostringstream oss;
            oss << " counts(x,y,z) :: " << setw( 4 ) << tmp[X] << " "  << setw( 4 ) << tmp[ Y ] << " "  << setw( 4 ) << tmp[ Z ];
            operation_log( oss.str(),( _trace_level > -2  ) ? CiicDevice::Informer : CiicDevice::Silent );
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
                << setw(2) << setfill('0') << hex << r.first  << ") :: $"
                << setw(2) << setfill('0') << hex << bitset<8>(*r_val).to_ulong() << " :: "
                << setw(3) << setfill('0') << dec << bitset<8>(*r_val).to_ulong() << " :: "
                << bitset<8>(*r_val)
                << endl;
        }
    }
    else if ( mux == ADXL345_GEOMETRY )
    {

        CiicDevice::TraceLevelEnum report =
                ( _trace_level > -2  ) ? CiicDevice::Informer : CiicDevice::Silent;
        oss << typeid(this).name();

        if( bitset<8>(alias)[ASYNCHRONOUS_OPERATION_bit] == 0 )
        {
            if( raw_acquistition == false )
            {
                oss << " g(x,y,z,pitch,roll,yaw) :: ";
                for( const auto & sample : _state.v )
                    oss << setw(10) << std::fixed << std::right << sample << " ";
            }
            else
            {
                oss << " counts(x,y,z) :: " ;
                for( auto const & p : _state.r )
                    oss << p << " ";
            }
        }
        else
        {
            oss << " mg(x,y,z) :: " ;
            for( auto const & p : _state.r )
                oss << p << " ";
        }
        operation_log( oss.str(), report );

    }
    return( CChipsetConfig::report() + oss.str() );
}

void Cadxl345::irq_handler(int elapsed )
{
    uint8_t tap_state = tap_status;
    uint8_t xor_val = receive( ADXL345_ACT_TAP_STATUS ) ^ tap_state;
    uint8_t int_state = int_source;
    if( xor_val )
    {
        receive( ADXL345_INT_SOURCE );

        std::ostringstream oss;
        uint8_t mask;

        mask = ( ( 1 << XActivity_TSbit )|( 1 << YActivity_TSbit )|(1 << ZActivity_TSbit ));
        oss << setw(20) << "Activity( b" << bitset<8>(xor_val) << " ) versus ( b"<< bitset<8>(mask) << " ) ";
        oss << endl;
        oss << setw(20) << " tap-status( b" << bitset<8>(tap_status)<< " ) ";
        oss << setw(20) << " int_source( b" << bitset<8>(int_source)<< " ) ";
        uint8_t activity_event = xor_val & mask & tap_status;
        if( activity_event )
        {
            bitset<8> tmp( receive( ADXL345_POWER_CTL ) );
            if( tmp[ SLEEP_MODE_POWER_CSR_bit ] )
                sleep( false );
            {
                std::unique_lock<std::mutex>(ip_mtx);
                profile.payload.tap.masking = tap_status;
                profile.payload.tap.masking |= ( tmp[ SLEEP_MODE_POWER_CSR_bit ] << 8 );
                profile.payload.tap.masking |= 1 << 31;
                oss << "PowerCtrl( b" << tmp << " )( b" << bitset<8>( profile.payload.tap.masking ) << " )";
                operation_log( oss.str(), CiicDevice::Informer );
                profile.operation_mask = 1 << TAP_SPEC;
                int err_id;
                if( ( err_id = sync_peer() ) < 0 )
                {
                    std::ostringstream oss;
                    oss << typeid(this).name() <<  ".sync_peer : f a i l u r e d : err_id(" + to_string(err_id) + ")";
                    operation_log( oss.str(), CiicDevice::Warning );
                }
            }
        }

        mask = FullTapping;
        uint8_t tap_event =  xor_val & mask & tap_status;
        if( tap_event )
        {
            {
                std::unique_lock<std::mutex>(ip_mtx);
                profile.payload.tap.masking = bitset<3>(tap_event).to_ulong();
                profile.payload.tap.masking |= int_source << 8;
                if( int_source & TapIntMasking )
                    profile.payload.tap.masking |=
                            1 << ( bitset<8>(int_source)[ SingleTap_IEbit ] ? 30 : 29 );
                profile.operation_mask = 1 << TAP_SPEC;
                int err_id;
                if( ( err_id = sync_peer() ) < 0 )
                {
                    std::ostringstream oss;
                    oss << typeid(this).name() <<  ".sync_peer : f a i l u r e d : err_id(" + to_string(err_id) + ")";
                    operation_log( oss.str(), CiicDevice::Warning );
                }
            }

            std::ostringstream oss;
            oss << typeid(this).name() <<  "." + string(__FUNCTION__) << endl;
            oss << setw(20) << " tap_event   ( b" << bitset<3>(tap_event)  << " ) " << endl;
            oss << setw(20) << " int_source  ( b" << bitset<8>(int_source) << " ) " << endl;
            oss << setw(20) << " tap_masking ( b" << bitset<32>(profile.payload.tap.masking) << " )" << endl;
            operation_log( oss.str(), CiicDevice::Informer );
        }
    }
    else
      receive( ADXL345_INT_SOURCE );

    xor_val =  int_source ^ int_state;
    if( xor_val )
    {
        std::ostringstream oss;
        if( bitset<8>(xor_val)[ FreeFalling_IEbit ] )
        {
            oss << typeid(this).name() <<  ".freefall : s a m p l e d";
            std::unique_lock<std::mutex>(ip_mtx);
            profile.payload.tap.masking = 1 << 28;
            profile.operation_mask = 1 << TAP_SPEC;
            int err_id;
            if( ( err_id = sync_peer() ) < 0 )
            {
                std::ostringstream oss;
                oss << typeid(this).name() <<  ".sync_peer : f a i l u r e d : err_id(" + to_string(err_id) + ")";
                operation_log( oss.str(), CiicDevice::Warning );
            }
        }
        operation_log( oss.str(), CiicDevice::Informer );
    }
    return;
}

void Cadxl345::monitor()
{
    {
        std::ostringstream oss;
        oss << typeid(this).name() << "." << __FUNCTION__ << " :: running device sampling :: " + to_string(sampling) + "(msec)";
        operation_log( oss.str(), CiicDevice::Informer );
    }


    int elapsed = 0;
    while( 1 )
    {
        int err_id;
        std::this_thread::sleep_for(std::chrono::milliseconds( sampling ));
        elapsed += sampling;

        if( autoprobing == true )
        {
            std::unique_lock<std::mutex>(mtx);
            cv.notify_one();
            continue;

        }

        irq_handler(elapsed);

        if( state().size() )
        {
            if( sr[ connection ] == 0 )
            {
                if( ! ( elapsed % 1000 ) )
                    report(ADXL345_GEOMETRY);
            }
        }

        if( sr[ connection ] == 0 )
            continue;

        {
            std::unique_lock<std::mutex>(ip_mtx);
            profile.payload.accelerometer.x     = _state.x;
            profile.payload.accelerometer.y     = _state.y;
            profile.payload.accelerometer.z     = _state.z;
            profile.payload.accelerometer.pitch = _state.pitch;
            profile.payload.accelerometer.roll  = _state.roll;
            profile.payload.accelerometer.yaw   = _state.yaw;
            bitset<32> mask;
            mask.set(ACCELEROMETER_STATE);
            profile.operation_mask = mask.to_ulong();
//            if( ! ( elapsed % 1000 ) )
            {
                if( ( err_id = sync_peer() ) < 0 )
                {
                    std::ostringstream oss;
                    oss << typeid(this).name() <<  ".sync_peer : f a i l u r e d : err_id(" + to_string(err_id) + ")";
                    operation_log( oss.str(), CiicDevice::Warning );
                }
            }
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
    tmp &= 0x10;
    tmp |= probe;
    xmitt( ADXL345_BW_RATE, tmp );
    return( Cadxl345Config::rate = static_cast<Cadxl345Config::dataRate_t>( receive(ADXL345_BW_RATE) ) );
}


/* POWER_CTL Bits */
#define PCTL_AUTO_SLEEP (1 << 4)
#define PCTL_MEASURE	(1 << 3)
#define PCTL_SLEEP	    (1 << 2)
#define PCTL_WAKEUP(x)	((x) & 0x3)

int Cadxl345::sleep( bool val )
{
    Numerology r = ADXL345_POWER_CTL;
    uint8_t tmp = receive( r);
    if( _err != ADXL345_NO_ERR )
        return( _err );

    if( val )
    {
        tmp |= (PCTL_MEASURE );
        tmp |= (PCTL_SLEEP);
    }
    else
    {
        tmp &= ~(PCTL_SLEEP|PCTL_AUTO_SLEEP|PCTL_MEASURE);
        xmitt( ADXL345_POWER_CTL, tmp );
        wait(1000);
        tmp |= PCTL_MEASURE;
    }

    xmitt( ADXL345_POWER_CTL, tmp );
    rate( Cadxl345Config::rate );

    std::ostringstream oss;
    oss << typeid(ghost).name() << ".operation # "
        << ( val ? "disabled" : "e n a b l e d" )
        << " # Power.ctrl( " << bitset<8>(tmp) << ")(" << bitset<8>(alias) << ")"
        << endl << endl;
    operation_log(oss.str(), Informer );
    return( _err );
}

int Cadxl345::autosleep( adxl345_payload::sleep_t parameters )
{
    std::ostringstream oss;
    receive( ADXL345_AUTOSLEEP );
    oss << typeid(ghost).name() << ".autosleep # "
        << ( parameters.enable ? "e n a b l e d" : "disabled" )
        << " # Power.ctrl( " << bitset<8>(power_ctrl) << ")"
        << endl;

    power_ctrl &= ~( 1 << ASLEEP_ENABLE_POWER_CSR_bit );
    power_ctrl &= ~( 1 << ASLEEP_LINK_POWER_CSR_bit   );
    xmitt( ADXL345_POWER_CTL, power_ctrl );
    if( parameters.enable == false )
    {
        operation_log(oss.str(), Informer );
        return( _err );
    }

    alias = power_ctrl | ( 1 << ASYNCHRONOUS_OPERATION_bit );
    float scaler = ADXL345_THRESHOLD_FACTOR;
    uint8_t threshold = static_cast<uint8_t>( round( parameters.activity / scaler ) );
    oss << setw(15) << "Activity("  << setw(5) << parameters.activity << ")(" << to_string(threshold) << ") -> (" << to_string(threshold_activity) << ")" << endl;
    if( xmitt( ADXL345_THRESH_ACT, threshold) < 0 )
        return( _err );

    threshold = static_cast<uint8_t>( round( parameters.inactivity / scaler ) );
    oss << setw(15) << "InActivity(" << setw(5) << parameters.inactivity << ")(" << to_string(threshold) << ") -> (" << to_string(threshold_inactivity) << ")" << endl;
    if( xmitt( ADXL345_THRESH_INACT, threshold ) < 0 )
        return( _err );

    threshold = static_cast<uint8_t>( round( parameters.msec / 1000 ) );
    oss << setw(15) << "TimeWindow("  << setw(5) << parameters.msec << ")(" << to_string(threshold) << ") -> (" << to_string(window_inactivity) << ")" << endl;
    if( xmitt( ADXL345_WINDOW_INACT, threshold ) < 0 )
        return( _err );

    bitset<32> masking( parameters.masking );
    bitset<3> operation(masking.to_ulong() & 0x7 );

    int relative = ( masking[31] || operation[Z] ) ? 1 : 0;
    uint8_t csr = ( relative << 3 )|( relative << 7 );
    csr |= ( operation[X] << 2 )|( operation[X] << 6 );
    csr |= ( operation[Y] << 1 )|( operation[Y] << 5 );
    csr |= ( operation[Z] << 0 )|( operation[Z] << 4 );
    oss << setw(15) << "Activity.ctrl(0x" << hex << ADXL345_ACT_INACT_CTL << ")(" << bitset<8>(csr) << ") -> (" << bitset<8>(autosleep_control) << ")" << endl;
    if( xmitt( ADXL345_ACT_INACT_CTL, csr ) < 0 )
        return( _err );

    receive( ADXL345_INT_ENABLE);
    bitset<8> tmp(int_enable);
    tmp.set(Activity_IEbit);
    tmp.set(InActivity_IEbit);
    xmitt( ADXL345_INT_ENABLE, static_cast<uint8_t>( tmp.to_ulong() ) );

    oss << setw(15) << "Power.ctrl("  << bitset<8>(power_ctrl) << ")" << endl;
    power_ctrl &= ( ~ ( 0x3 ) );
    power_ctrl |= 1 << ASLEEP_LINK_POWER_CSR_bit;
    power_ctrl |= 1 << ASLEEP_ENABLE_POWER_CSR_bit;
    xmitt( ADXL345_POWER_CTL, power_ctrl );
    operation_log( oss.str(), Informer );


}

int Cadxl345::xmitt( uint8_t r_index, uint8_t val )
{
    vector <uint8_t> payload(2);
    payload[0] = r_index ;
    payload[1] = val;
    return( _err = ( CiicDevice::xmitt( payload ) < 0 ) ? ERR_ADXL345_XMITT_BUS_ERR : ADXL345_NO_ERR );
}


void Cadxl345::sleep( adxl345_payload::sleep_t tmp )
{
    if( tmp.automatic )
    {
        ghost->autosleep( tmp );
        return;
    }

    std::ostringstream oss;
    oss << typeid(ghost).name() <<  ".sleep : " << ( tmp.enable ? "entering" : "exit" );
    operation_log(oss.str(), Informer );
    ghost->sleep( tmp.enable );
}

void Cadxl345::autoprobe( Cadxl345Config::Numerology phase )
{

    buffer.clear();
    std::unique_lock<std::mutex> locker(mtx);
    cv.wait(locker);
    locker.unlock();

    const int snapshot = 10000, deadline = 200000;
    {
        std::ostringstream oss;
        oss << typeid(this).name() + string(".autoprobe(10bits) :: data taking for ") + to_string( deadline / 1000 ) << "msec";
        operation_log( oss.str(), Informer );
        int elapsed = 0;
        do
        {
            buffer.push_back( ghost->state() );
            wait( snapshot);
        }while( ( elapsed += snapshot ) < deadline );
    }

    if( phase == AUTOPROBE_OFFSET )
    {
        for( auto & s : offset_buffer )
            s = s.clear();

        int i = 0;
        for( const auto & b : buffer )
            for( const auto & sample : b )
                offset_buffer[ ( i++ ) % 3 ].update( sample );

        std::ostringstream oss;
        oss << typeid(this).name() + string(" :: - Offset - statistics mean/noise/max/min(") + to_string( buffer.size()) << ")" << endl ;
        for( auto & o : offset_buffer )
            oss << o.report( ) << endl;
        operation_log( oss.str(), Informer );
        return;
    }

    if( phase == AUTOPROBE_LOAD )
    {
        vector <CStatistic> load = { CStatistic("Xload"), CStatistic("Yload"), CStatistic("Zload") };
        {
            int i = 0;
            for( const auto & b : buffer )
                for( const auto & sample : b )
                    load[ ( i++ ) % 3 ].update( sample );
        }

        int pass_masking = 0;
        vector <int> residual;
        vector <bool> pass;
        vector <adxl345_numerics> v = { X, Y, Z };
        for( const auto & i : v )
        {
            residual.push_back( static_cast<int>( round( load[i].mean - offset_buffer[i].mean ) ) );
            int min,max;
            tie(min,max) = self_test[i];
            pass.push_back( check_range(residual[i],min,max) );
            pass_masking |= ( check_range(residual[i],min,max) ? 1 : 0 ) << i;
        }

        {
            std::ostringstream oss;
            oss << typeid(this).name() + string(" :: -  Load  - statistics mean/noise/max/min(") + to_string( buffer.size()) << ")" << endl ;
            {
                int i = 0;
                for( auto & o : load )
                    oss << o.report( ) << " residual(" << setprecision(3) << setw(7) << residual[ i ] << ")" << ( pass[ i++ ] ? " passed " : " checkme please" ) << endl;
            }
            operation_log( oss.str(), Informer );
        }

        autoprobe( autoprobing = false );

        {
            std::unique_lock<std::mutex>(ip_mtx);
            profile.payload.self_test.mask = pass_masking;
            profile.payload.self_test.offset_x = offset_buffer[X].mean;
            profile.payload.self_test.offset_y = offset_buffer[Y].mean;
            profile.payload.self_test.offset_z = offset_buffer[Z].mean;
            profile.payload.self_test.x = residual[X];
            profile.payload.self_test.y = residual[Y];
            profile.payload.self_test.z = residual[Z];
            bitset<32> mask;
            mask.set(SELF_TESTING);
            profile.operation_mask = mask.to_ulong();
            int err_id;
            if( ( err_id = sync_peer() ) < 0 )
            {
                std::ostringstream oss;
                oss << typeid(this).name() <<  ".sync_peer : f a i l u r e d : err_id(" + to_string(err_id) + ")";
                operation_log( oss.str(), CiicDevice::Warning );
            }
            operation_log( typeid(this).name() + string("peer forwarded with autoprobe results"), Informer );
        }
        return;
    }

}


void Cadxl345::autoprobe( bool enable )
{

    ghost->raw_acquistition= enable;
    ghost->autoprobing = enable;
    if( enable )
        ghost->autoprobe( AUTOPROBE_OFFSET );

    cv.notify_all();

    Numerology r = ADXL345_DATA_FORMAT;
    enum {
        FullResolutionBit = 3,
        SelfTestingBit = 7
    };

    uint8_t tmp = ghost->receive( r );
    tmp &= ~( 1 << SelfTestingBit );
    tmp &= ~( 1 << FullResolutionBit );
    tmp |= ( enable ? 1 : 0 ) << SelfTestingBit;
    tmp |= ( enable ? 0 : 1 ) << FullResolutionBit;
    ghost->xmitt( r, tmp );

    std::ostringstream oss;
    oss << typeid(ghost).name() + string(".autoprobe(10bits) :: ") << ( enable ? "toggled" : "untoggled" );
    operation_log( oss.str(), Informer );

    if( enable )
        ghost->autoprobe( AUTOPROBE_LOAD );

}

void Cadxl345::tapping(adxl345_payload::tap_t tmp)
{
    ghost->tapping( bitset<3>(tmp.masking), tmp.duration, tmp.latency, tmp.msec_window, tmp.threshold );
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


void Cadxl345::freefall_specs( float threshold, int window )
{
    {
        std::ostringstream oss;
        oss << typeid(ghost).name() + string(".freefall-settings(mg/msec) :: ");
        oss << static_cast<int>( ghost->freefall.threshold * ADXL345_THRESHOLD_FACTOR ) << "/";
        oss << static_cast<int>( ghost->freefall.window * 5. );
        oss << " -> " << threshold << "/" << window;
        operation_log( oss.str(), CiicDevice::Informer );
    }

    ghost->freefall.threshold = static_cast<uint8_t>( round( threshold  / ADXL345_THRESHOLD_FACTOR ) );
    ghost->freefall.window    = static_cast<uint8_t>( round( window / 5. ) );
    if(  ghost->xmitt( ADXL345_THRESH_FF, ghost->freefall.threshold ) < 0 )
        return;
    if(  ghost->xmitt( ADXL345_TIME_FF, ghost->freefall.window ) < 0 )
        return;
    ghost->alias |= ( 1 << ASYNCHRONOUS_OPERATION_bit );
    return;

}

int Cadxl345::tapping(bitset<3> mask, int msec_duration , int msec_latency, int msec_window, int threshold )
{
    bitset<8> tmp( receive( ADXL345_TAP_AXES ) );
    tmp &= ~0x7;
    tmp |= ( mask.to_ulong() & 0x7 );
    tmp.set(3);
    xmitt( ADXL345_TAP_AXES  , static_cast<uint8_t>( tap.mask  = tmp.to_ulong() ) );

    {
        std::ostringstream oss;
        oss << typeid(ghost).name() + string(".tap-settings(Threshold(mg)/Duration(usec)/Latency(msec)/Window(msec)) :: ");
        oss << static_cast<int>( ghost->tap.threshold * ADXL345_THRESHOLD_FACTOR ) << "/";
        oss << static_cast<int>( ghost->tap.duration  * ADXL345_DURATION_FACTOR ) << "/";
        oss << static_cast<int>( ghost->tap.latency   * ADXL345_TAP_MSEC_FACTOR ) << "/";
        oss << static_cast<int>( ghost->tap.window    * ADXL345_TAP_MSEC_FACTOR );
        oss << " -> " << threshold << "/" << msec_duration << "/" << msec_latency << "/" << msec_window;
        operation_log( oss.str(), CiicDevice::Informer );
    }

    xmitt( ADXL345_THRESH_TAP, static_cast<uint8_t>( tap.threshold = threshold / ADXL345_THRESHOLD_FACTOR ));
    xmitt( ADXL345_DUR       , static_cast<uint8_t>( tap.duration  = msec_duration / ADXL345_DURATION_FACTOR  ) );
    xmitt( ADXL345_LATENT    , static_cast<uint8_t>( tap.latency   = msec_latency / ADXL345_TAP_MSEC_FACTOR ) );
    xmitt( ADXL345_WINDOW    , static_cast<uint8_t>( tap.window    = msec_window / ADXL345_TAP_MSEC_FACTOR ) );
    return( 0 );


}

void Cadxl345::reply( int mask )
{
    bitset<LastReply> bm(mask);

    if( bm[ FreeFallStuff ] || bm[ TapStuff ] )
        receive( ADXL345_FREEFALLAndTAP );

    for( int i = 0; i < bm.size(); i++ )
    {
        if( bm[i] == 0 )
            continue;

        {

            switch (i) {
            case FreeFallStuff:
                profile.operation_mask = 1 << TAP_SPEC;
                profile.payload.tap.masking = FreeFallStuff << 31;
                profile.payload.tap.masking |= 1 << 27;

                profile.payload.tap.threshold   = freefall.threshold * ADXL345_THRESHOLD_FACTOR;
                profile.payload.tap.msec_window = freefall.window * 5. ;

                break;

            case TapStuff:
                profile.operation_mask = 1 << TAP_SPEC;
                profile.payload.tap.masking = TapStuff << 31;
                profile.payload.tap.masking |= 1 << 27;
                profile.payload.tap.threshold   = tap.threshold * ADXL345_THRESHOLD_FACTOR;
                profile.payload.tap.duration    = tap.duration  * ADXL345_DURATION_FACTOR;
                profile.payload.tap.latency     = tap.latency   * ADXL345_TAP_MSEC_FACTOR;
                profile.payload.tap.msec_window = tap.window    * ADXL345_TAP_MSEC_FACTOR;
                profile.payload.tap.masking |= tap.mask;
                break;

            default:
                continue;
            }

            {
                std::unique_lock<std::mutex>(ip_mtx);
                {
                    int err_id;
                    if( ( err_id = sync_peer() ) < 0 )
                    {
                        std::ostringstream oss;
                        oss << typeid(this).name() <<  ".sync_peer : f a i l u r e d : err_id(" + to_string(err_id) + ")";
                        operation_log( oss.str(), CiicDevice::Warning );
                    }


                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }



    }
}

void Cadxl345::tapping_state()
{
    std::ostringstream oss;
    oss << typeid(ghost).name() + string( __FUNCTION__ ) + string(".settings reply :: ");
    operation_log( oss.str(), CiicDevice::Informer );
    ghost->reply( FreeFallAndTapStuff );

}

int Cadxl345::offset(const vector<int> *payload )
{
    if( payload->size() < 3 )
        return ( _err = ERR_OFFSET_OOSPEC );

    int i = 0;
    for( const auto & val : *payload )
        if(  xmitt( ADXL345_CALIBRATION_OFFSET + ( i++ ), DecToc2( static_cast<int>( round(  val / 15.6 ) ) ) ) < 0 )
            break;

    return( _err );

}

int Cadxl345::range( Numerology probe_range, bool full_resolution )
{
    static map <Numerology,uint8_t> ranges = {
        { PlusMinus1G , 0},
        { PlusMinus2G , 1},
        { PlusMinus4G , 2},
        { PlusMinus8G , 3},
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
    if(  xmitt( ADXL345_DATA_FORMAT, r_val ) == 0 )
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


uint8_t Cadxl345::receive( Cadxl345Config::Numerology index )
{

    /* artefacts ------------------------------------------------------------ */
    if( index == ADXL345_CHIPSET )
    {
         uint8_t tmp = -1;
         for( const auto & r: registers )
             receive( r.first );
         return( tmp );
    }

    if( index == ADXL345_XYZ )
    {
        uint8_t tmp = -1;
        vector< uint8_t> register_address = { ADXL345_DATAX0 };
        if(  CiicDevice::xmitt( register_address ) == 0 )
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

    if( index ==  ADXL345_AUTOSLEEP )
    {
        uint8_t tmp = -1;
        static list <Numerology> autosleep_registers = {
            ADXL345_ACT_INACT_CTL,
            ADXL345_THRESH_ACT,
            ADXL345_THRESH_INACT,
            ADXL345_WINDOW_INACT,
            ADXL345_INT_ENABLE,
            ADXL345_INT_MAP,
            ADXL345_POWER_CTL };
        for( const auto & r: autosleep_registers)
        {
            receive( r );
            if( _err != ADXL345_NO_ERR )
                break;
        }
        return( tmp );
    }

    if( index ==  ADXL345_FREEFALLAndTAP )
    {
        uint8_t tmp = -1;
        static list <Numerology> autosleep_registers = {
            ADXL345_TIME_FF,
            ADXL345_THRESH_FF,
            ADXL345_TAP_AXES,
            ADXL345_DUR,
            ADXL345_LATENT,
            ADXL345_WINDOW,
            ADXL345_THRESH_TAP };
        for( const auto & r: autosleep_registers)
        {
            receive( r );
            if( _err != ADXL345_NO_ERR )
                break;
        }
        return( tmp );
    }
    /* ---------------------------------------------------------------------- */

    uint8_t tmp = -1;
    uint8_t *ptr = & tmp;
    if( registers.find( index ) != registers.end() )
        tie( ptr, ignore, ignore ) = registers.find( index )->second;

    vector< uint8_t> register_address = { static_cast<uint8_t>( index & 0xff ) };
    if(  CiicDevice::xmitt( register_address ) == 0 )
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
        alias = 0;
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

Cadxl345IPProfile::Cadxl345IPProfile(RaspBerryIPSpecification profile_id) : _profile_id(profile_id)
{

}

int Cadxl345IPProfile::sync_peer()
{
    return( CNetworkSlot::send( _profile_id, & profile, sizeof( profile ) ) );
}
