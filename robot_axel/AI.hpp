/***************************************************************************

    file                 : AI.hpp
    copyright            : (C) 2019 Miguel Ferreira and Jo�o Aniceto

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   MarI/O source code made by SethBling was used as a creation base.     *
 *                                                                         *
 ***************************************************************************/

#pragma once

#ifndef ROBOT_AXEL_AI
#define ROBOT_AXEL_AI

#include <array>
#include "Genome.hpp"
#include "Gym.hpp"
#include "Settings.hpp"

namespace RobotAxel
{
    /// <summary>
    /// An evolutionary AI implementation based on NEAT and MarI/O.
    /// </summary>
    class AI
    {
        /***************************************************************************
         *                                                                         *
         *   AI local constants and variables.                                     *
         *                                                                         *
         ***************************************************************************/
        private:
        /// <summary>
        /// Ecosystem used for evolution ( Training ).
        /// </summary>
        Gym * const gym;
        /// <summary>
        /// Top breed, best network result for input, output stream ( Evaluation only ).
        /// </summary>
        Genome const * const best_ai;

        /***************************************************************************
         *                                                                         *
         *  AI interface.                                                          *
         *                                                                         *
         ***************************************************************************/
        public:
        /// <summary>
        /// Creates the AI using the intended method depending of the objective.
        /// </summary>
        /// <param name="training">Whenever should the AI train.</param>
        AI( bool const & training );

        /// <summary>
        /// Destroys the created 
        /// </summary>
        ~AI();

        /// <summary>
        /// Calculates outputs according to the current network, using the Gym or the top Genome.
        /// </summary>
        /// <param name="outputs">Array with output neurons values.</param>
        /// <param name="inputs">The input array used for the update.</param>
        void evaluate( std::array <float, Settings::OUTPUTS> & outputs, std::array<float, Settings::INPUTS> const & inputs );

        /// <summary>
        /// Appraises AI progress in current environment, this progress is used for AI evaluation.
        /// </summary>
        /// <param name="progress_rating">Progress evaluation of current AI situation.</param>
        void appraise( float const & progress_rating );

        /// <summary>
        /// Indicates the Gym to advance to the next Genome, for the fitness test.
        /// </summary>
        void advanceInTrain();

        /// <summary>
        /// Retrieves current training generation and species number in generation, as well current top progress.
        /// </summary>
        /// <param name="generation">Current AI generation if training, generation reference if not.</param>
        /// <param name="species">Number of species in generation if training, 0 if not.</param>
        /// <param name="progress">Current fitness if training, 0 if not.</param>
        /// <param name="top_progress">Current maximum fitness if training, AI progress if not.</param>
        /// <param name="top_genes">Top gene number.</param>
        /// <param name="top_neurons">Top neuron number.</param>
        void getInformation( size_t & generation, size_t & species, float & progress, float & top_progress, size_t & top_genes, size_t & top_neurons ) const;

        /***************************************************************************
         *                                                                         *
         *   AI Utilities.                                                         *
         *                                                                         *
         ***************************************************************************/
        private:
        /// <summary>
        /// Generates the Gym for AI training if training is set.
        /// </summary>
        /// <param name="training">Whenever should the AI train.</param>
        /// <returns>A gym instance pointer or a null pointer.</returns>
        Gym * const generateGym( bool const & training ) const;

        /// <summary>
        /// Generates the top AI for the environment if the training is not set.
        /// </summary>
        /// <param name="training">Whenever should the AI train.</param>
        /// <returns>A pointer with the top found Genome, or a null pointer.</returns>
        Genome const * const generateBestAI( bool const & training ) const;
    };
}

#endif
