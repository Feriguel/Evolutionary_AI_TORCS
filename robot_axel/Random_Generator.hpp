/***************************************************************************

    file                 : Random_Generator.hpp
    copyright            : (C) 2019 Miguel Ferreira and Jo�o Aniceto

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   MarI/O source code made by SethBling was used as a creation base.     *
 *                                                                         *
 ***************************************************************************/

#pragma once

#ifndef ROBOT_AXEL_RANDOM_GENERATOR
#define ROBOT_AXEL_RANDOM_GENERATOR

#include <random>

namespace RobotAxel
{
    /// <summary>
    /// AI random number generator, uses a uniform distribution of real numbers between 0 and 1 as base.
    /// </summary>
    class Random_Generator
    {
        /***************************************************************************
         *                                                                         *
         *   Random_Generator local constants and variables.                       *
         *                                                                         *
         ***************************************************************************/
        private:
        /// <summary>
        /// Random number generator, used in number retrieval form the distribution.
        /// </summary>
        std::mt19937 random_number_generator;
        /// <summary>
        /// Uniform distribution of real numbers between [0, 1[.
        /// </summary>
        std::uniform_real_distribution<float> distribution;

        /***************************************************************************
         *                                                                         *
         *   Random_Generator interface.                                           *
         *                                                                         *
         ***************************************************************************/
        public:
        /// <summary>
        /// Retrieves the Random number generator instance.
        /// </summary>
        static Random_Generator & getInstance();

        /// <summary>
        /// Generate a random number between 0 and 1.
        /// </summary>
        float generateRandom();

        /***************************************************************************
         *                                                                         *
         *   Random_Generator utilities.                                           *
         *                                                                         *
         ***************************************************************************/
        private:
        /// <summary>
        /// Builds internal distribution and random number generator.
        /// </summary>
        Random_Generator();
    };
}

#endif
