/***************************************************************************

    file                 : Axel.hpp
    copyright            : (C) 2019 Miguel Ferreira and Jo�o Aniceto

 ***************************************************************************/

#include "Axel.hpp"

#if defined (AXEL_DRIVER)

#include <iostream>
#include <cassert>

//#define DEBUG_INPUTS
//#define DEBUG_OUTPUTS

/***************************************************************************
 *                                                                         *
 *   Axel interface.                                                       *
 *                                                                         *
 ***************************************************************************/
/// <summary>
/// Initialises robot local constants and reference values.
/// </summary>
Axel::Axel()
    : axel_ai( generateAI( IS_TRAINING ) ), inputs( {} ), outputs( {} ), car_control( CarControl() ), lap_counter( 0 ),
    last_lap_time( 0.0f ), total_lap_times( 0.0f ), distance_raced( 0 )
{}

/// <summary>
/// Deletes all instances created with new on the constructor.
/// </summary> 
Axel::~Axel()
{
    delete this->axel_ai;
}

/// <summary>
/// Called before the beginning of the race and can be used to define a custom configuration of the track sensors.
/// </summary>
void Axel::init( float * angles )
{
    // sets sensor angles to be every 10� from -90� to 90�, array is clock-wise
    for ( int sensor_index = 0; sensor_index <= 18; sensor_index++ ) angles[ sensor_index ] = 10.0f * static_cast<float>( sensor_index ) - 90.0f;
}

/// <summary>
/// Called at the end of the race, before the driver module is unloaded.
/// </summary>
void Axel::onShutdown()
{
    // show information
    showInformation();
    // quit
    std::cout << "The name's Axel, got it memorised? Bye!" << std::endl;
}

/// <summary>
/// Called when the race is restarted upon the driver request.
/// </summary>
void Axel::onRestart()
{
    // show information
    showInformation();
    // reset counters
    this->lap_counter = 0;
    this->last_lap_time = 0.0f;
    // advance gym situation
    this->axel_ai->advanceInTrain();
    // restart
    std::cout << ">>> Restarting the race!" << std::endl;
}

/// <summary>
/// Drive control, calls the wrapper drive function that translates the state string to the CarState and the returned CarControl to string.
/// </summary>
CarControl Axel::wDrive( CarState car_state )
{
    // saves total distance raced
    this->distance_raced = car_state.getDistRaced();
    // appraises progress and checks if race requires a restart and, if so, proceeds to return car control without any evaluation.
    appraise( car_state );
    restartCheck( car_state );
    if ( this->car_control.getMeta() == CarControl::META_RESTART ) return car_control;
    // converts inputs, evaluates outputs, converts to car control and sends to TORCS.
    convertInputs( car_state );
    evaluate();
    convertOutputs();
    return car_control;
}

/***************************************************************************
 *                                                                         *
 *   Axel Utilities.                                                       *
 *                                                                         *
 ***************************************************************************/
/// <summary>
/// Creates a new AI and returns it's pointer.
/// </summary>
RobotAxel::AI * const Axel::generateAI( bool const & training ) const
{
    return new RobotAxel::AI( training );
}

/// <summary>
/// Converts the necessary CarState inputs to an array of floats for AI use.
/// </summary>
/// <param name="car_state">Current sensors state.</param>
void Axel::convertInputs( CarState & car_state )
{
    // temporary input
    float temporary_input = 0.0f;
    // current input index
    size_t index = 0;

    // [0] gear { -1, ..., 6 } --> { GEAR_OFFSET, ..., GEAR_MAX + GEAR_OFFSET } / (GEAR_MAX + GEAR_OFFSET)
    temporary_input = static_cast<float> ( car_state.getGear() ) + GEAR_OFFSET;
    this->inputs[ index ] = temporary_input / ( GEAR_MAX + GEAR_OFFSET );
    index++;
    // [1] rpm [0, +infinity[ --> [0, RPM_MAX] / RPM_MAX
    temporary_input = static_cast<float> ( car_state.getRpm() );
    temporary_input = temporary_input < RPM_MAX ? temporary_input : RPM_MAX;
    this->inputs[ index ] = temporary_input / RPM_MAX;
    index++;
    // [2] angle [-pi, +pi] --> [ANGLE_OFFSET, ANGLE_MAX + ANGLE_OFFSET] / (ANGLE_MAX + ANGLE_OFFSET)
    temporary_input = car_state.getAngle() + ANGLE_OFFSET;
    this->inputs[ index ] = temporary_input / ( ANGLE_MAX + ANGLE_OFFSET );
    index++;
    // [3] fuel [0, +infinity[ --> [0, FUEL_MAX] / FUEL_MAX
    temporary_input = car_state.getFuel();
    temporary_input = temporary_input < FUEL_MAX ? temporary_input : FUEL_MAX;
    this->inputs[ index ] = temporary_input / FUEL_MAX;
    index++;
    // [4,5,6] speed(x, y, z) ]-infinity, +infinity[ --> [SPEED_OFFSET, SPEED_MAX + SPEED_OFFSET] / (SPEED_MAX + SPEED_OFFSET)
    temporary_input = car_state.getSpeedX();
    temporary_input = temporary_input < SPEED_MAX ? temporary_input : SPEED_MAX;
    temporary_input = -SPEED_OFFSET < temporary_input ? temporary_input : -SPEED_OFFSET;
    temporary_input = temporary_input + SPEED_OFFSET;
    this->inputs[ index ] = temporary_input / ( SPEED_MAX + SPEED_OFFSET );
    index++;
    temporary_input = car_state.getSpeedY();
    temporary_input = temporary_input < SPEED_MAX ? temporary_input : SPEED_MAX;
    temporary_input = -SPEED_OFFSET < temporary_input ? temporary_input : -SPEED_OFFSET;
    temporary_input = temporary_input + SPEED_OFFSET;
    this->inputs[ index ] = temporary_input / ( SPEED_MAX + SPEED_OFFSET );
    index++;
    temporary_input = car_state.getSpeedZ();
    temporary_input = temporary_input < SPEED_MAX ? temporary_input : SPEED_MAX;
    temporary_input = -SPEED_OFFSET < temporary_input ? temporary_input : -SPEED_OFFSET;
    temporary_input = temporary_input + SPEED_OFFSET;
    this->inputs[ index ] = temporary_input / ( SPEED_MAX + SPEED_OFFSET );
    index++;
    // [7] z ]-infinity, + infinity[ --> [Z_OFFSET, Z_MAX + Z_OFFSET] / (Z_MAX + Z_OFFSET)
    temporary_input = car_state.getZ();
    temporary_input = temporary_input < Z_MAX ? temporary_input : Z_MAX;
    temporary_input = -Z_OFFSET < temporary_input ? temporary_input : -Z_OFFSET;
    temporary_input = temporary_input + Z_OFFSET;
    this->inputs[ index ] = temporary_input / ( Z_MAX + Z_OFFSET );
    index++;
    // [8]  damage [0, + infinity] --> [0, DAMAGE_MAX] / DAMAGE_MAX
    temporary_input = car_state.getDamage();
    temporary_input = temporary_input < DAMAGE_MAX ? temporary_input : DAMAGE_MAX;
    this->inputs[ index ] = temporary_input / DAMAGE_MAX;
    index++;
    // [9] track position ]-infinity, + infinity[ --> [TRACK_POSITION_OFFSET, TRACK_POSITION_MAX + TRACK_POSITION_OFFSET] / (TRACK_POSITION_MAX + TRACK_POSITION_OFFSET)
    temporary_input = car_state.getTrackPos();
    temporary_input = temporary_input < TRACK_POSITION_MAX ? temporary_input : TRACK_POSITION_MAX;
    temporary_input = -TRACK_POSITION_OFFSET < temporary_input ? temporary_input : -TRACK_POSITION_OFFSET;
    temporary_input = temporary_input + TRACK_POSITION_OFFSET;
    this->inputs[ index ] = temporary_input / ( TRACK_POSITION_MAX + TRACK_POSITION_OFFSET );
    index++;
    // [10 ... 13] wheel spin angular velocity [0, +infinity] --> [SPIN_OFFSET, SPIN_MAX + SPIN_OFFSET] / (SPIN_MAX + SPIN_OFFSET)
    for ( int i = 0; i < 4; i++ )
    {
        temporary_input = car_state.getWheelSpinVel( i );
        temporary_input = temporary_input < SPIN_MAX ? temporary_input : SPIN_MAX;
        temporary_input = -SPIN_OFFSET < temporary_input ? temporary_input : -SPIN_OFFSET;
        temporary_input = temporary_input + SPIN_OFFSET;
        this->inputs[ index + i ] = temporary_input / ( SPIN_MAX + SPIN_OFFSET );
    }
    index += 4;
    // focus [0, 200] --> [0, FOCUS_DISTANCE_MAX] / FOCUS_DISTANCE_MAX
    bool focus_reliability = true;
    for ( int i = 0; i < FOCUS_SENSORS_NUM; i++ )
    {
        temporary_input = car_state.getFocus( i );
        focus_reliability = focus_reliability && temporary_input >= 0.0f;
        if ( !focus_reliability )
        {
            this->inputs[ index + i ] = 0.0f;
        }
        else
        {
            temporary_input = temporary_input < FOCUS_DISTANCE_MAX ? temporary_input : FOCUS_DISTANCE_MAX;
            this->inputs[ index + i ] = temporary_input / FOCUS_DISTANCE_MAX;
        }
    }
    index += FOCUS_SENSORS_NUM;
    this->inputs[ index ] = focus_reliability ? 1.0f : 0.0f;
    index++;
    // track [0, 200] --> [0, FOCUS_DISTANCE_MAX] / FOCUS_DISTANCE_MAX
    bool track_reliability = true;
    for ( int i = 0; i < TRACK_SENSORS_NUM; i++ )
    {
        temporary_input = car_state.getTrack( i );
        track_reliability = track_reliability && temporary_input >= 0.0f;
        if ( !track_reliability )
        {
            this->inputs[ index + i ] = 0.0f;
        }
        else
        {
            temporary_input = temporary_input < TRACK_DISTANCE_MAX ? temporary_input : TRACK_DISTANCE_MAX;
            this->inputs[ index + i ] = temporary_input / TRACK_DISTANCE_MAX;
        }
    }
    index += TRACK_SENSORS_NUM;
    this->inputs[ index ] = track_reliability ? 1.0f : 0.0f;
    index++;
    // opponents [0, 200] --> [0, FOCUS_DISTANCE_MAX] / FOCUS_DISTANCE_MAX
    for ( int i = 0; i < OPPONENTS_SENSORS_NUM; i++ )
    {
        temporary_input = car_state.getOpponents( i );
        temporary_input = temporary_input < OPPONENTS_DISTANCE_MAX ? temporary_input : OPPONENTS_DISTANCE_MAX;
        this->inputs[ index + i ] = temporary_input / OPPONENTS_DISTANCE_MAX;
    }
    index += OPPONENTS_SENSORS_NUM;
#if defined (DEBUG_INPUTS)
    // show inputs and outputs
    for ( int i = 0; i < RobotAxel::Settings::INPUTS; i++ )
    { std::cout << "inputs [" << i << "] : " << this->inputs[ i ] << std::endl; }
#endif
    assert( index == RobotAxel::Settings::INPUTS );
}

/// <summary>
/// Converts AI outputs to CarControl sent into TORCS.
/// </summary>
/// <param name="car_control">CarControl with to be filled with AI outputs.</param>
void Axel::convertOutputs()
{
    // temporary output
    float temporary_output = 0.0f;
    // current output index
    size_t index = 0;

    // gear ( ( [-1, 1] + GEAR_OFFSET ) * ( GEAR_MAX + GEAR_OFFSET ) / ( GEAR_OFFSET + GEAR_OFFSET ) ) - GEAR_OFFSET --> { -1, ..., 6 }
    temporary_output = ( this->outputs[ index ] + 1.0f ) / 2.0f * ( GEAR_MAX + GEAR_OFFSET );
    temporary_output = temporary_output - GEAR_OFFSET;
    temporary_output = std::trunc( temporary_output );
    temporary_output = temporary_output == GEAR_MAX ? 6.0f : temporary_output;
    if ( TRAINING_NO_REVERSE_OR_NEUTRAL )
    {
        temporary_output = temporary_output < 1.0f ? 1.0f : temporary_output;
    }
    this->car_control.setGear( static_cast<int>( temporary_output ) );
    index++;
    // focus [-1, 1] * FOCUS_MAX --> [-90, 90]
    temporary_output = this->outputs[ index ] * FOCUS_MAX;
    this->car_control.setFocus( static_cast<int>( std::trunc( temporary_output ) ) );
    index++;
    // steer [-1, 1] --> [-1, 1]
    this->car_control.setSteer( this->outputs[ index ] );
    index++;
    // acceleration [-1, 1] --> [0, 1]
    temporary_output = ( this->outputs[ index ] + PEDAL_OFFSET ) / ( PEDAL_MAX + PEDAL_OFFSET );
    this->car_control.setAccel( temporary_output );
    index++;
    // brake [-1, 1] --> [0, 1]
    temporary_output = ( this->outputs[ index ] + PEDAL_OFFSET ) / ( PEDAL_MAX + PEDAL_OFFSET );
    this->car_control.setBrake( temporary_output );
    index++;
    // clutch [-1, 1] --> [0, 1]
    temporary_output = ( this->outputs[ index ] + PEDAL_OFFSET ) / ( PEDAL_MAX + PEDAL_OFFSET );
    this->car_control.setClutch( temporary_output );
    index++;
#if defined (DEBUG_OUTPUTS)
    std::cout << "output[0] (gear) = " << car_control.getGear() << std::endl;
    std::cout << "output[1] (focus) = " << car_control.getFocus() << std::endl;
    std::cout << "output[2] (steer) = " << car_control.getSteer() << std::endl;
    std::cout << "output[3] (acceleration) = " << car_control.getAccel() << std::endl;
    std::cout << "output[4] (brake) = " << car_control.getBrake() << std::endl;
    std::cout << "output[5] (clutch) = " << car_control.getClutch() << std::endl;
#endif
    assert( index == RobotAxel::Settings::OUTPUTS );
}

/// <summary>
/// Sends input information to AI.
/// </summary>
void Axel::evaluate()
{
    this->axel_ai->evaluate( outputs, inputs );
}

/// <summary>
/// Evaluates current progress and updates AI.
/// </summary>
/// <param name="car_state">Current sensors state.</param>
void Axel::appraise( CarState & car_state )
{
    // progress appraisal
    float progress = 0.0f;

    // current time average speed
    float time_counter = this->total_lap_times + car_state.getCurLapTime();
    float average_speed = std::sqrt( car_state.getSpeedX() * car_state.getSpeedX() ) / ( time_counter < 1.0f ? 1.0f : time_counter );
    // best average speed, rewards better average speeds [km/h], uses (average_speed)^2
    progress += average_speed * average_speed;
    // distance run, rewards higher distances [m]
    progress += PROGRESS_DISTANCE_MULTIPLIER * ( std::log( car_state.getDistRaced() + std::exp( 1.0f ) ) );
    // current position, rewards higher positions [1 ... N]
    if ( TRAINING && TRAINING_BASE_POSITION )
    {
        progress += PROGRESS_POSITION_REWARD / TRAINING_FORCED_POSITION;
    }
    else
    {
        progress += PROGRESS_POSITION_REWARD / static_cast<float>( car_state.getRacePos() );
    }

    // update with calculated appraisal
    this->axel_ai->appraise( progress );
}

/// <summary>
/// Determines if race should be restarted.
/// </summary>
/// <param name="car_state">Current sensors state.</param>
void Axel::restartCheck( CarState & car_state )
{
    // defaults as no restart
    this->car_control.setMeta( !CarControl::META_RESTART );

    // lap time is different, update lap time and increment lap counter
    if ( this->last_lap_time != car_state.getLastLapTime() )
    {
        this->last_lap_time = car_state.getLastLapTime();
        this->total_lap_times += this->last_lap_time;
        this->lap_counter++;
    }

    // check restart flags
    if ( this->lap_counter >= TRAINING_MAX_LAPS )
    { this->car_control.setMeta( CarControl::META_RESTART ); }
    if ( car_state.getDamage() >= TRAINING_MAX_DAMAGE )
    { this->car_control.setMeta( CarControl::META_RESTART ); }
    if ( TRAINING_TIMEOUT - car_state.getCurLapTime() - car_state.getCurLapTime() + car_state.getDistRaced() < 0.0f )
    { this->car_control.setMeta( CarControl::META_RESTART ); }
}

/// <summary>
/// Show information such as AI generation, progress results and such.
/// </summary>
void Axel::showInformation()
{
    size_t generation = 0;
    size_t species = 0;
    float progress = 0.0f;
    float top_progress = 0.0f;
    size_t top_genes = 0;
    size_t top_neurons = 0;

    // obtain run information
    this->axel_ai->getInformation( generation, species, progress, top_progress, top_genes, top_neurons );

    // show information if training
    if ( Axel::TRAINING )
    {
        std::cout
            << "Raced=" << distance_raced
            << " Generation=" << generation
            << " Species=" << species
            << " Progress=" << progress
            << " Top Progress=" << top_progress
            << " Top Genes=" << top_genes
            << " Top Neurons=" << top_neurons
            << std::endl;
    }
    else
    {
        std::cout
            << "Raced=" << distance_raced
            << " Generation=" << generation
            << " Top Progress=" << top_progress
            << " Top Genes=" << top_genes
            << " Top Neurons=" << top_neurons
            << std::endl;
    }
}

#endif
