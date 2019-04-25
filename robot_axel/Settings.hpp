/***************************************************************************

    file                 : Settings.hpp
    copyright            : (C) 2019 Miguel Ferreira and Jo�o Aniceto

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   MarI/O source code made by SethBling was used as a creation base.     *
 *                                                                         *
 ***************************************************************************/

#pragma once

#ifndef ROBOT_AXEL_SETTINGS
#define ROBOT_AXEL_SETTINGS

#include <cstdlib>

namespace RobotAxel
{
    /// <summary>
    /// Axel AI, an evolutionary AI implementation for TORCS, based on NEAT.
    /// </summary>
    class Settings
    {
        public:
        /***************************************************************************
         *                                                                         *
         *   Global settings.                                                      *
         *                                                                         *
         ***************************************************************************/
        /// <summary>
        /// AI input number.
        /// </summary>
        /// <remarks>
        /// Order of inputs must always be the same, any changes requires a new AI creation.
        /// </remarks>
        constexpr static int const INPUTS = 76;
        /// <summary>
        /// AI output number.
        /// </summary>
        /// <remarks>
        /// Order of outputs will stay the same independently of AI evolution.
        /// </remarks>
        constexpr static int const OUTPUTS = 6;
        /// <summary>
        /// Maximum AI decision neurons (have in mind, in a critical case, all neurons are interconnected, except inputs).
        /// </summary>
        /// <remarks>
        /// Numeric limit is: size_t >= (limit - inputs) * limit
        /// </remarks>
        constexpr static size_t const NEURON_LIMIT = 1000;
        /// <summary>
        /// Is basic genome composed by a full linked input/output, or no links at all.
        /// </summary>
        constexpr static bool const BASIC_FULL_LINK = true;

        /***************************************************************************
         *                                                                         *
         *   Gym and species settings.                                             *
         *                                                                         *
         ***************************************************************************/
        /// <summary>
        /// Maximum population between species
        /// </summary>
        constexpr static int const GYM_POPULATION = 100;
        /// <summary>
        /// Minimum breed count for a species to be considered not week.
        /// </summary>
        constexpr static int const SPECIES_BREED_THRESHOLD = 3;
        /// <summary>
        /// Genome minimum for a species to be considered stale.
        /// </summary>
        constexpr static int const SPECIES_STALE_THRESHOLD = 5;
        /// <summary>
        /// Genome disjoint rating multiplier.
        /// </summary>
        constexpr static float const SPECIES_DELTA_DIJOINT = 0.4f;
        /// <summary>
        /// Genome weight comparison multiplier.
        /// </summary>
        constexpr static float const SPECIES_DELTA_WEIGHTS = 0.6f;
        /// <summary>
        /// Max diference to be considered from a specie.
        /// </summary>
        constexpr static float const SPECIES_DELTA_THRESHOLD = 0.5f;

        /***************************************************************************
         *                                                                         *
         *   Node link weight and value normalization settings.                    *
         *                                                                         *
         ***************************************************************************/
        /// <summary>
        /// Gene link weight range, as in [0 to range].
        /// </summary>
        constexpr static float const LINK_RANGE = 2.0f;
        /// <summary>
        /// Gene link weight offset, as in [offset to range + offset].
        /// </summary>
        constexpr static float const LINK_OFFSET = -1.0f;
        /// <summary>
        /// Value sigmoid normalization range [0 to range].
        /// </summary>
        constexpr static float const SIGMOID_RANGE = 2.0f;
        /// <summary>
        /// Value sigmoid growth rate, how fast reaches the limit.
        /// </summary>
        constexpr static float const SIGMOID_GROWTH_RATE = -4.9f;
        /// <summary>
        /// Value sigmoid offset, as in [offset to range + offset].
        /// </summary>
        constexpr static float const SIGMOID_OFFSET = -1.0f;

        /***************************************************************************
         *                                                                         *
         *   Mutation settings.                                                    *
         *                                                                         *
         ***************************************************************************/
        /// <summary>
        /// Chance for a genome structure change, between [0.0f, 1.0f], triggers if below chance value.
        /// </summary>
        constexpr static float const MUTATE_WEIGHT_CHANCE = 0.4f;
        /// <summary>
        /// Chance for a complete gene weight change, between [0.0f, 1.0f], triggers if below chance value.
        /// </summary>
        constexpr static float const MUTATE_WEIGHT_ALL_CHANCE = 0.3f;
        /// <summary>
        /// Chance for a new random weight, between [0.0f, 1.0f], triggers if below chance value.
        /// </summary>
        constexpr static float const MUTATE_WEIGHT_NEW_RANDOM_CHANCE = 0.2f;
        /// <summary>
        /// Deviation range of weight modification.
        /// </summary>
        constexpr static float const MUTATE_WEIGHT_DEVIATION_RANGE = 0.2f;
        /// <summary>
        /// Chance for a node mutation, between [0.0f, 1.0f], triggers if below chance value.
        /// </summary>
        constexpr static float const MUTATE_NODE_CHANCE = 0.3f;
        /// <summary>
        /// Chance for a link mutation, between [0.0f, 1.0f], triggers if below chance value.
        /// </summary>
        constexpr static float const MUTATE_LINK_CHANCE = 0.3f;
        /// <summary>
        /// Chance for a state mutation, between [0.0f, 1.0f], triggers if below chance value.
        /// </summary>
        constexpr static float const MUTATE_STATE_CHANCE = 0.5f;
        /// <summary>
        /// Chance for a full neuron incoming state mutation, between [0.0f, 1.0f], triggers if below chance value.
        /// </summary>
        constexpr static float const MUTATE_STATE_INVERT_ALL_NEURON_GENES_CHANCE = 0.0f;

        /***************************************************************************
         *                                                                         *
         *   Crossover settings.                                                   *
         *                                                                         *
         ***************************************************************************/
        /// <summary>
        /// Chance for a crossover between two genomes of the same species.
        /// </summary>
        constexpr static float const CROSSOVER_CHANCE = 0.75f;
        /// <summary>
        /// Chance for a gene addition in crossover, between [0.0f, 1.0f], triggers if below chance value.
        /// </summary>
        constexpr static float const CROSSOVER_GENE_ADDITION = 0.5f;

        /***************************************************************************
         *                                                                         *
         *   Gene innovation counter.                                              *
         *                                                                         *
         ***************************************************************************/
        /// <summary>
        /// Current innovation number.
        /// </summary>
        static unsigned int current_innovation;

        /// <summary>
        /// Retrieve Gene innovation.
        /// </summary>
        /// <returns>Current innovation number.</returns>
        static unsigned int const newInnovation();
    };
}

#endif
