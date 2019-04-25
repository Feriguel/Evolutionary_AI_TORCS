/***************************************************************************

    file                 : Random_Generator.cpp
    copyright            : (C) 2019 Miguel Ferreira and Jo�o Aniceto

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   MarI/O source code made by SethBling was used as a creation base.     *
 *                                                                         *
 ***************************************************************************/

#include "Random_Generator.hpp"

#if defined (ROBOT_AXEL_RANDOM_GENERATOR)

namespace RobotAxel
{
    /***************************************************************************
     *                                                                         *
     *   Random_Generator interface.                                           *
     *                                                                         *
     ***************************************************************************/
    /// <summary>
    /// Retrieves the Random number generator instance.
    /// </summary>
    Random_Generator & Random_Generator::getInstance()
    {
        // single generator instance
        static Random_Generator generator;
        // generator instance
        return generator;
    }

    /// <summary>
    /// Generate a random number between 0 and 1.
    /// </summary>
    float Random_Generator::generateRandom()
    {
        return distribution( this->random_number_generator );
    }

   /***************************************************************************
    *                                                                         *
    *   Random_Generator utilities.                                           *
    *                                                                         *
    ***************************************************************************/
   /// <summary>
   /// Builds internal distribution and random number generator.
   /// </summary>
    Random_Generator::Random_Generator()
        : random_number_generator( std::mt19937( std::random_device()( ) ) ), distribution( std::uniform_real_distribution<float>( 0.0f, 1.0f ) ) {}
}

#endif
