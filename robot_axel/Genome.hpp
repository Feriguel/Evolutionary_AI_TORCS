/***************************************************************************

    file                 : Genome.hpp
    copyright            : (C) 2019 Miguel Ferreira and Jo�o Aniceto

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   MarI/O source code made by SethBling was used as a creation base.     *
 *                                                                         *
 ***************************************************************************/

#pragma once

#ifndef ROBOT_AXEL_GENOME
#define ROBOT_AXEL_GENOME

#include <array>
#include <vector>
#include "Random_Generator.hpp"
#include "Settings.hpp"

namespace RobotAxel
{
    /// <summary>
    /// AI Brain, contains network that processes the inputs to outputs.
    /// </summary>
    class Genome
    {
        /***************************************************************************
         *                                                                         *
         *   Gene, Neuron and Network classes.                                     *
         *                                                                         *
         ***************************************************************************/
        private:
        /// <summary>
        /// Node connection.
        /// </summary>
        class Gene
        {
            // Friendship declarations
            friend class Genome;

            private:
            /// <summary>
            /// Innovation reference.
            /// </summary>
            unsigned int innovation;
            /// <summary>
            /// Origin neuron index.
            /// </summary>
            size_t from;
            /// <summary>
            /// Destination neuron index.
            /// </summary>
            size_t to;
            /// <summary>
            /// Link weight.
            /// </summary>
            float weight;
            /// <summary>
            /// Activation state.
            /// </summary>
            bool enabled;

            public:
            /// <summary>
            /// Generates a simple gene, with everything at 0, not enabled and without innovation number.
            /// </summary>
            Gene();

            private:
            /// <summary>
            /// Generates a gene with given settings.
            /// </summary>
            /// <param name="from">Input neuron index.</param>
            /// <param name="to">Output neuron index.</param>
            /// <param name="enabled">Link connection status.</param>
            Gene( size_t const & from, size_t const & to, bool const & enabled );

            /// <summary>
            /// Deviates the current gene degree by a certain range.
            /// </summary>
            void deviateGeneWeight();

            /// <summary>
            /// Randomizes a new weight value for a gene.
            /// </summary>
            /// <returns>A weight created according to settings.</returns>
            void randomGeneWeight();

            /// <summary>
            /// Checks if both neurons have the same reference.
            /// </summary>
            /// <param name="other">The one for comparison.</param>
            /// <returns>Is equal.</returns>
            bool const operator== ( Gene const & other ) const;

        };

        /// <summary>
        /// AI node, and list of incoming connections.
        /// </summary>
        class Neuron
        {
            // Friendship declarations
            friend class Genome;

            private:
            /// <summary>
            /// List of incoming genes indexes (empty in initial neurons).
            /// </summary>
            std::vector<Gene const *> incoming;
            /// <summary>
            /// Neuron output, (input in initial neurons).
            /// </summary>
            float value;

            /// <summary>
            /// Generates a simple Neuron.
            /// </summary>
            /// <param name="index">Neuron index.</param>
            /// <param name="network">Gene network.</param>
            Neuron( size_t const & index, std::vector<Gene> const & network );

            /// <summary>
            /// Checks if neuron already has link.
            /// </summary>
            /// <param name="neuron_index">Index of link to find.</param>
            /// <returns>If neuron has link.</returns>
            bool const hasLink( size_t const & neuron_index ) const;
        };

        /***************************************************************************
         *                                                                         *
         *   Genome local constants and variables.                                 *
         *                                                                         *
         ***************************************************************************/
        private:
        /// <summary>
        /// Genome fitness.
        /// </summary>
        float fitness;
        /// <summary>
        /// AI Generation reference.
        /// </summary>
        size_t generation;
        /// <summary>
        /// Number of neurons.
        /// </summary>
        size_t total_neurons;
        /// <summary>
        /// AI brain structure.
        /// </summary>
        std::vector<Gene> network;

        /***************************************************************************
         *                                                                         *
         *   Genome interface.                                                     *
         *                                                                         *
         ***************************************************************************/
        public:
        /// <summary>
        /// Generates a simple Genome if top genome isn't set. 
        /// Otherwise either generates one from previous top from a file, if exists, or a basic one.
        /// </summary>
        Genome( bool const & generate_top = false );

        /// <summary>
        /// Retrieves AI gene number.
        /// </summary>
        /// <returns>AI total genes.</returns>
        size_t getTotalGenes() const;

        /// <summary>
        /// Retrieves AI neuron number.
        /// </summary>
        /// <returns>AI total neurons.</returns>
        size_t getTotalNeurons() const;

        /// <summary>
        /// Retrieves AI generation.
        /// </summary>
        /// <returns>AI fitness.</returns>
        size_t getGeneration() const;

        /// <summary>
        /// Retrieves AI fitness.
        /// </summary>
        /// <returns>AI fitness.</returns>
        float getFitness() const;

        /// <summary>
        /// Updates Genome's generation reference.
        /// </summary>
        /// <param name="generation">The new generation value.</param>
        void setGeneration( size_t const & generation );

        /// <summary>
        /// Updates Genome's fitness reference.
        /// </summary>
        /// <param name="fitness">The new fitness value.</param>
        void setFitness( float const & fitness );

        /// <summary>
        /// Calculates outputs according to the current network.
        /// </summary>
        /// <param name="outputs">Array with output neurons values.</param>
        /// <param name="inputs">The input array used for the update.</param>
        void evaluate( std::array <float, Settings::OUTPUTS> & outputs, std::array<float, Settings::INPUTS> const & inputs ) const;

        /// <summary>
        /// Does one random mutation depending on the settings given.
        /// </summary>
        void mutate();

        /// <summary>
        /// Compares two genomes.
        /// </summary>
        /// <param name="other">Genome for comparison..</param>
        /// <param name="disjoint">Disjoint rating.</param>
        /// <param name="weights">Weight difference.</param>
        void compare( Genome const & other, float & disjoint, float & weights ) const;

        /// <summary>
        /// Creates a new genome form a crossover between this genome and another one given, with characteristics of both.
        /// </summary>
        /// <remarks>
        /// The higher fitness genome should be the caller.
        /// </remarks>
        /// <param name="other">The genome for crossover</param>
        /// <returns>A new genome with characteristics of both parents.</returns>
        Genome crossover( Genome const & other ) const;

        void serialize( bool finish_training );

        /***************************************************************************
         *                                                                         *
         *   Genome utilities.                                                     *
         *                                                                         *
         ***************************************************************************/
        private:
        /// <summary>
        /// Either randomizes weight or makes a small mutation in weight of all genes.
        /// </summary>
        void mutateWeight();

        /// <summary>
        /// Changes a random gene enabled state.
        /// </summary>
        void mutateState();

        /// <summary>
        /// Creates a new link between two neurons if there is none.
        /// </summary>
        /// <remarks>
        /// To maintain the evaluations correct, only creates links from lower neuron indexes to higher ones (prevents use of non yet evaluated neurons).
        /// </remarks>
        void mutateLink();

        /// <summary>
        /// Generates a new node mutation, by transforming a gene into the same connection with a "middle-man". Original gene is disabled.
        /// </summary>
        /// <remarks>
        /// Updates every other neuron gene connections from selected neuron with the changes in gene.from indexes.
        /// </remarks>
        void mutateNode();
    };
}

#endif
