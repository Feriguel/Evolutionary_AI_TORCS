/***************************************************************************

    file                 : Gym.hpp
    copyright            : (C) 2019 Miguel Ferreira and Jo�o Aniceto

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   MarI/O source code made by SethBling was used as a creation base.     *
 *                                                                         *
 ***************************************************************************/

#pragma once

#ifndef ROBOT_AXEL_GYM
#define ROBOT_AXEL_GYM

#include <array>
#include "Genome.hpp"
#include "Random_Generator.hpp"
#include "Settings.hpp"

namespace RobotAxel
{
    /// <summary>
    /// AI training gym, evolves the AI in order to achieve better fitness results.
    /// </summary>
    class Gym
    {
        /***************************************************************************
         *                                                                         *
         *   Species class.                                                        *
         *                                                                         *
         ***************************************************************************/
        private:
        /// <summary>
        /// 
        /// </summary>
        class Species
        {
            // Friendship declarations
            friend class Gym;

            /// <summary>
            /// Species's current genome mutations.
            /// </summary>
            std::vector<Genome> genomes;

            /// <summary>
            /// Generates a simple species.
            /// </summary>
            Species();

            /// <summary>
            /// Checks if given genome belongs to species.
            /// </summary>
            /// <param name="genome">Genome to check.</param>
            /// <returns>Whenever genome belongs to species.</returns>
            bool const fromSpecies( Genome const & genome ) const;

            /// <summary>
            /// Average species fitness.
            /// </summary>
            float const averageFitness() const;

            /// <summary>
            /// Calculates the species total offspring to reach the target population in comparison to all other species.
            /// </summary>
            /// <param name="species">Species for breed count</param>
            /// <returns>Breed count.</returns>
            size_t const speciesBreedCount( float const & total_average_fitness ) const;

            /// <summary>
            /// Breed a new genome from this species.
            /// </summary>
            /// <returns>Breed child.</returns>
            Genome breedChild() const;
        };

        /***************************************************************************
         *                                                                         *
         *   Gym local constants and variables.                                    *
         *                                                                         *
         ***************************************************************************/
        private:
        /// <summary>
        /// Full spectre of species in current generation.
        /// </summary>
        std::vector<Species> all_species;
        /// <summary>
        /// Current species in training.
        /// </summary>
        size_t current_species;
        /// <summary>
        /// Current species genome in training.
        /// </summary>
        size_t current_genome;
        /// <summary>
        /// Current training generation.
        /// </summary>
        size_t generation;
        /// <summary>
        /// Reference to genome with best fitness.
        /// </summary>
        Genome top_fitness_genome;

        /***************************************************************************
         *                                                                         *
         *   Gym interface.                                                        *
         *                                                                         *
         ***************************************************************************/
        public:
        /// <summary>
        /// Generates gym with initial population
        /// </summary>
        Gym();

        /// <summary>
        /// Save top_genome before deleting gym.
        /// </summary>
        ~Gym();

        /// <summary>
        /// Calculates outputs according to the current network, using the Gym or the top Genome.
        /// </summary>
        /// <param name="outputs">Array with output neurons values.</param>
        /// <param name="inputs">The input array used for the update.</param>
        void evaluateCurrent( std::array <float, Settings::OUTPUTS> & outputs, std::array<float, Settings::INPUTS> const & inputs ) const;

        /// <summary>
        /// Updates current Genome's fitness reference.
        /// </summary>
        /// <param name="fitness">The fitness of the previous evaluation.</param>
        void appraiseCurrent( float const & fitness );

        /// <summary>
        /// Indicates the Gym to advance to the next Genome, for the fitness test.
        /// </summary>
        void advanceInTrain();

        /// <summary>
        /// Retrieves current training generation and species number in generation, as well current top fitness.
        /// </summary>
        /// <param name="generation">Current AI generation.</param>
        /// <param name="species">Number of species in generation.</param>
        /// <param name="fitness">Current fitness.</param>
        /// <param name="top_fitness">Top fitness.</param>
        /// <param name="top_genes">Top gene number.</param>
        /// <param name="top_neurons">Top neuron number.</param>
        void getInformation( size_t & generation, size_t & species, float & fitness, float & top_fitness, size_t & top_genes, size_t & top_neurons ) const;

        /***************************************************************************
         *                                                                         *
         *   Gym utilities.                                                        *
         *                                                                         *
         ***************************************************************************/
        /// <summary>
        /// Retrieves current genome in training.
        /// </summary>
        /// <returns>The current genome reference.</returns>
        Genome const & currentGenome() const;
        /// <summary>
        /// Retrieves current genome in training.
        /// </summary>
        /// <returns>The current genome reference.</returns>
        Genome & currentGenome();

        /// <summary>
        /// Adds genome to a similar species, if none is found creates one for it.
        /// </summary>
        /// <param name="genome">Genome to add.</param>
        void addGenomeToRespectiveSpecies( Genome & genome );

        /// <summary>
        /// Advances generation by:
        ///     - Culling half of every species;
        ///     - Removing stale species;
        ///     - Remove weak species;
        ///     - Breed new children
        ///     - Culling everything but top species;
        ///     - Add children to species.
        /// </summary>
        void advanceGeneration();

        /// <summary>
        /// Eliminates either half or every but the best genome in every species, also orders every specie's genomes by descending order.
        /// </summary>
        /// <param name="half_cull">Trigger half culling or full culling.</param>
        void cullSpecies( bool const & half_cull = false );

        /// <summary>
        /// Removes all stale species from current generation. Also orders species by their best genome fitness.
        /// </summary>
        /// <remarks>
        /// A specie is considered stale if has over Settings::SPECIES_STALE_THRESHOLD genomes and no progress was made.
        /// </remarks>
        void removeStaleSpecies();

        /// <summary>
        /// Removes all weak species from current generation.
        /// </summary>
        /// <remarks>
        /// A specie is considered weak if has no breed chance.
        /// </remarks>
        void removeWeakSpecies();

        /// <summary>
        /// Calculates the total average fitness of all species.
        /// </summary>
        /// <returns>The total average calculated fitness.</returns>
        float const totalAverageFitness() const;

        /// <summary>
        /// Breed all children till population target.
        /// </summary>
        /// <returns>The breed children.</returns>
        std::vector<Genome> const breedChildren() const;
    };
}

#endif
