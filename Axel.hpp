/***************************************************************************

    file                 : Axel.hpp
    copyright            : (C) 2019 Miguel Ferreira and Jo�o Aniceto

 ***************************************************************************/

#pragma once

#ifndef AXEL_DRIVER
#define AXEL_DRIVER

#include <array>
#include <cmath>
#include "BaseDriver.h"
#include "CarState.h"
#include "CarControl.h"
#include "SimpleParser.h"
#include "WrapperBaseDriver.h"
#include "robot_axel/AI.hpp"
#include "robot_axel/Settings.hpp"

constexpr bool const IS_TRAINING = true;

/// <summary>
/// Axel, an autonomous racing driver for TORCS.
/// </summary>
class Axel : public WrapperBaseDriver
{
    /***************************************************************************
     *                                                                         *
     *   Axel settings.                                                        *
     *                                                                         *
     ***************************************************************************/
    private:
    /// <summary>
    /// Training flag.
    /// </summary>
    constexpr static bool const TRAINING = IS_TRAINING;
    /// <summary>
    /// Training maximum allowed damage.
    /// </summary>
    constexpr static float const TRAINING_MAX_DAMAGE = 1000.0f;
    /// <summary>
    /// Training timeout.
    /// </summary>
    constexpr static float const TRAINING_TIMEOUT = 10.0f;
    /// <summary>
    /// Training maximum allowed laps.
    /// </summary>
    constexpr static size_t const TRAINING_MAX_LAPS = 3;
    /// <summary>
    /// Training min gear is 1.
    /// </summary>
    constexpr static bool const TRAINING_NO_REVERSE_OR_NEUTRAL = false;
    /// <summary>
    /// Training uses a fixed base position.
    /// </summary>
    constexpr static bool const TRAINING_BASE_POSITION = true;
    /// <summary>
    /// Sets training position to be as if in forced position.
    /// </summary>
    constexpr static float const TRAINING_FORCED_POSITION = 5.0f;
    /// <summary>
    /// Multiplier for progress distance, as in multiplier * log ( distance ).
    /// </summary>
    constexpr static float const PROGRESS_DISTANCE_MULTIPLIER = 5000.0f;
    /// <summary>
    /// Training maximum allowed laps.
    /// </summary>
    constexpr static float const PROGRESS_POSITION_REWARD = 50000.0f;
    /// <summary>
    /// Gear maximum value.
    /// </summary>
    constexpr static float const GEAR_MAX = 7.0f;
    /// <summary>
    /// Gear offset, used to compromise gears in a interval of { OFFSET, ..., MAX + OFFSET }.
    /// </summary>
    constexpr static float const GEAR_OFFSET = 1.0f;
    /// <summary>
    /// RPM maximum value, this is the tachometer maximum value from XML, any value over will be set to this.
    /// </summary>
    constexpr static float const RPM_MAX = 10000.0f;
    /// <summary>
    /// Angle maximum value, this is equal to pi.
    /// </summary>
    constexpr static float const ANGLE_MAX = std::atan( 1.0f ) * 4.0f;
    /// <summary>
    /// Angle offset, used to compromise angles in a interval of [ OFFSET, ..., MAX + OFFSET ], this is equal to pi.
    /// </summary>
    constexpr static float const ANGLE_OFFSET = std::atan( 1.0f ) * 4.0f;
    /// <summary>
    /// Fuel maximum value, this is the fuel tank from XML, any value over will be set to this.
    /// </summary>
    constexpr static float const FUEL_MAX = 94.0f;
    /// <summary>
    /// Speed maximum value, this is the speedometer maximum value from XML, any value over will be set to this, in [Km/h].
    /// </summary>
    constexpr static float const SPEED_MAX = 360.0f;
    /// <summary>
    /// Speed offset, used to compromise speed in a interval of [ OFFSET, ..., MAX + OFFSET ], in [Km/h].
    /// </summary>
    constexpr static float const SPEED_OFFSET = 360.0f;
    /// <summary>
    /// Z maximum value, this is capped at 2Km, any value over will be set to this, in [m].
    /// </summary>
    constexpr static float const Z_MAX = 1000.0f;
    /// <summary>
    /// Z offset, used to compromise speed in a interval of [ OFFSET, ..., MAX + OFFSET ], in [m].
    /// </summary>
    constexpr static float const Z_OFFSET = 1000.0f;
    /// <summary>
    /// Spin maximum value, this is capped at 48 rad/s (255/40 R18 [Wheel/Tire specifications from XML] at 360 km/h), any value over will be set to this, in [rad/s].
    /// </summary>
    constexpr static float const SPIN_MAX = 48.0f;
    /// <summary>
    /// Spin offset, used to compromise speed in a interval of [ OFFSET, ..., MAX + OFFSET ].
    /// </summary>
    constexpr static float const SPIN_OFFSET = 48.0f;
    /// <summary>
    /// Damage maximum value, any value over will be set to this.
    /// </summary>
    constexpr static float const DAMAGE_MAX = 10000.0f;
    /// <summary>
    /// Track position maximum value in relation to centre, any value over will be set to this, if [-1, 1] car is in track.
    /// </summary>
    constexpr static float const TRACK_POSITION_MAX = 1.5f;
    /// <summary>
    /// Track position offset, used to compromise speed in a interval of [ OFFSET, ..., MAX + OFFSET ].
    /// </summary>
    constexpr static float const TRACK_POSITION_OFFSET = 1.5f;
    /// <summary>
    /// Focus maximum distance, in [m].
    /// </summary>
    constexpr static float const FOCUS_DISTANCE_MAX = 200.0f;
    /// <summary>
    /// Track maximum distance, in [m].
    /// </summary>
    constexpr static float const TRACK_DISTANCE_MAX = 200.0f;
    /// <summary>
    /// Opponents maximum distance, in [m].
    /// </summary>
    constexpr static float const OPPONENTS_DISTANCE_MAX = 200.0f;
    /// <summary>
    /// Focus angle maximum value, in [degrees].
    /// </summary>
    constexpr static float const FOCUS_MAX = 90.0f;
    /// <summary>
    /// Pedal maximum value.
    /// </summary>
    constexpr static float const PEDAL_MAX = 1.0f;
    /// <summary>
    /// Pedal offset, used to compromise pedal force in a interval of [ OFFSET, ..., MAX + OFFSET ].
    /// </summary>
    constexpr static float const PEDAL_OFFSET = 1.0f;

    /***************************************************************************
     *                                                                         *
     *   Axel local constants and variables.                                   *
     *                                                                         *
     ***************************************************************************/
    private:
    /// <summary>
    /// Robot Axel AI controller.
    /// </summary>
    RobotAxel::AI * const axel_ai;
    /// <summary>
    /// AI inputs, must be compromised by values between [0, 1].
    /// </summary>
    std::array<float, RobotAxel::Settings::INPUTS> inputs;
    /// <summary>
    /// AI outputs, will be compromised by values between [-1, 1].
    /// </summary>
    std::array<float, RobotAxel::Settings::OUTPUTS> outputs;
    /// <summary>
    /// Robot Axel AI controller.
    /// </summary>
    CarControl car_control;
    /// <summary>
    /// Number of laps made.
    /// </summary>
    size_t lap_counter;
    /// <summary>
    /// Time of last lap, for lap counting.
    /// </summary>
    float last_lap_time;
    /// <summary>
    /// Total previous laps times.
    /// </summary>
    float total_lap_times;
    /// <summary>
    /// Distance raced.
    /// </summary>
    float distance_raced;

    /***************************************************************************
     *                                                                         *
     *   Axel interface.                                                       *
     *                                                                         *
     ***************************************************************************/
    public:
    /// <summary>
    /// Initialises local constants and variables.
    /// </summary>
    Axel();

    /// <summary>
    /// Deletes all local constants and variables created with new on the constructor.
    /// </summary> 
    ~Axel();

    /// <summary>
    /// Called before the beginning of the race and can be used to define a custom configuration of the track sensors.
    /// </summary>
    /// <param name="angles">Angles of the 19 range finder sensors, displayed clockwise.< / param>
    virtual void init( float * angles );

    /// <summary>
    /// Called at the end of the race, before the driver module is unloaded.
    /// </summary>
    virtual void onShutdown();

    /// <summary>
    /// Called when the race is restarted upon the driver request.
    /// </summaryA>
    virtual void onRestart();

    /// <summary>
    /// Drive control, calls the wrapper drive function that translates the state string to the CarState and the returned CarControl to string.
    /// </summary>
    /// <param name="car_state">Sensors state.</param>
    /// <returns>Actions taken.</returns>
    virtual CarControl wDrive( CarState car_state );

    /***************************************************************************
     *                                                                         *
     *   Axel Utilities.                                                       *
     *                                                                         *
     ***************************************************************************/
    private:
    /// <summary>
    /// Creates a new AI and returns it's pointer.
    /// </summary>
    RobotAxel::AI * const generateAI( bool const & training ) const;

    /// <summary>
    /// Converts the necessary CarState inputs to an array of floats for AI use.
    /// </summary>
    /// <param name="car_state">Current sensors state.</param>
    void convertInputs( CarState & car_state );

    /// <summary>
    /// Converts AI outputs to CarControl sent into TORCS.
    /// </summary>
    void convertOutputs();

    /// <summary>
    /// Sends input information to AI.
    /// </summary>
    void evaluate();

    /// <summary>
    /// Evaluates current progress.
    /// </summary>
    /// <param name="car_state">Current sensors state.</param>
    void appraise( CarState & car_state );

    /// <summary>
    /// Determines if race should be restarted.
    /// </summary>
    /// <param name="car_state">Current sensors state.</param>
    void restartCheck( CarState & car_state );

    /// <summary>
    /// Show information such as AI generation, progress results and such.
    /// </summary>
    void showInformation();
};

#endif
