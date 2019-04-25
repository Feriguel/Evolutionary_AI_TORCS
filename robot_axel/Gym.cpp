/***************************************************************************

    file                 : Gym.cpp
    copyright            : (C) 2019 Miguel Ferreira and Jo�o Aniceto

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   MarI/O source code made by SethBling was used as a creation base.     *
 *                                                                         *
 ***************************************************************************/

#include "Gym.hpp"

#if defined (ROBOT_AXEL_GYM)

#include <algorithm>
#include <iostream>
#include <cassert>

namespace RobotAxel
{
    /***************************************************************************
     *                                                                         *
     *   Gym interface.                                                        *
     *                                                                         *
     ***************************************************************************/
    /// <summary>
    /// Generates gym with initial population
    /// </summary>
    Gym::Gym() : all_species( {} ), current_species( 0 ), current_genome( 0 ), generation( 0 ), top_fitness_genome( Genome( true ) )
    {
        this->generation = top_fitness_genome.getGeneration();
        // create a base population of top genome mutation
        for ( size_t individual = 0; individual < Settings::GYM_POPULATION; individual++ )
        {
            Genome new_genome = Genome( top_fitness_genome );
            assert( new_genome.getTotalGenes() == top_fitness_genome.getTotalGenes() && new_genome.getTotalNeurons() == top_fitness_genome.getTotalNeurons() );
            // mutate and add
            new_genome.mutate();
            addGenomeToRespectiveSpecies( new_genome );
        }
    }

    /// <summary>
    /// Save top_genome before deleting gym.
    /// </summary>
    Gym::~Gym()
    {
        // if last genome before quitting is better save as top
        if ( top_fitness_genome.getFitness() < currentGenome().getFitness() )
        {
            currentGenome().serialize( true );
        }
        // if not serializes top
        else
        {
            top_fitness_genome.serialize( true );
        }
    }

    /// <summary>
    /// Calculates outputs according to the current network, using the Gym or the top Genome.
    /// </summary>
    /// <param name="outputs">Array with output neurons values.</param>
    /// <param name="inputs">The input array used for the update.</param>
    void Gym::evaluateCurrent( std::array <float, Settings::OUTPUTS> & outputs, std::array<float, Settings::INPUTS> const & inputs ) const
    {
        currentGenome().evaluate( outputs, inputs );
    }

    /// <summary>
    /// Updates current Genome's fitness reference.
    /// </summary>
    /// <param name="fitness">The fitness of the previous evaluation.</param>
    void Gym::appraiseCurrent( float const & fitness )
    {
        currentGenome().setFitness( fitness );
    }

    /// <summary>
    /// Indicates the Gym to advance to the next Genome, for the fitness test. If all species were evaluated advances generation.
    /// </summary>
    void Gym::advanceInTrain()
    {
        // increment current genome
        this->current_genome++;
        // check if over current species genome list size
        if ( current_genome < this->all_species[ this->current_species ].genomes.size() ) return;
        // increment current species and set current genome to 0
        this->current_species++;
        this->current_genome = 0;
        // check if over species list size
        if ( current_species < this->all_species.size() ) return;
        // set current species to 0
        this->current_species = 0;
        // advance generation if all species were fully evaluated
        advanceGeneration();
    }

    /// <summary>
    /// Retrieves current training generation and species number in generation, as well current top fitness.
    /// </summary>
    /// <param name="generation">Current AI generation.</param>
    /// <param name="species">Number of species in generation.</param>
    /// <param name="fitness">Current fitness.</param>
    /// <param name="top_fitness">Current maximum fitness.</param>
    /// <param name="top_genes">Top gene number.</param>
    /// <param name="top_neurons">Top neuron number.</param>
    void Gym::getInformation( size_t & generation, size_t & species, float & fitness, float & top_fitness, size_t & top_genes, size_t & top_neurons ) const
    {
        generation = this->generation;
        species = this->all_species.size();
        fitness = currentGenome().getFitness();
        top_fitness = this->top_fitness_genome.getFitness();
        top_genes = this->top_fitness_genome.getTotalGenes();
        top_neurons = this->top_fitness_genome.getTotalNeurons();
    }

    /***************************************************************************
     *                                                                         *
     *   Gym utilities.                                                        *
     *                                                                         *
     ***************************************************************************/
    /// <summary>
    /// Retrieves current genome in training.
    /// </summary>
    /// <returns>The current genome reference.</returns>
    Genome const & Gym::currentGenome() const
    {
        return this->all_species[ this->current_species ].genomes[ this->current_genome ];
    }

    /// <summary>
    /// Retrieves current genome in training.
    /// </summary>
    /// <returns>The current genome reference.</returns>
    Genome & Gym::currentGenome()
    {
        return this->all_species[ this->current_species ].genomes[ this->current_genome ];
    }

    /// <summary>
    /// Adds genome to a similar species, if none is found creates one for it.
    /// </summary>
    /// <param name="genome">Genome to add.</param>
    void Gym::addGenomeToRespectiveSpecies( Genome & genome )
    {
        // try to add to an existent species if from same species
        for ( Species & species : this->all_species )
        {
            if ( species.fromSpecies( genome ) )
            {
                species.genomes.push_back( genome );
                return;
            }
        }
        // add to new created species otherwise
        Species new_species = Species();
        new_species.genomes.push_back( genome );
        this->all_species.push_back( new_species );
    }

    /// <summary>
    /// Advances generation by:
    ///     - Culling half of every species;
    ///     - Removing stale species;
    ///     - Remove weak species;
    ///     - Breed new children
    ///     - Culling everything but top species;
    ///     - Add children to species.
    /// </summary>
    void Gym::advanceGeneration()
    {
        assert( !this->all_species.empty() );
        // cull half species
        cullSpecies( true );
        // remove stale species
        removeStaleSpecies();
        // remove weak species
        removeWeakSpecies();
        // set new top fitness genome
        this->top_fitness_genome = Genome( this->all_species[ 0 ].genomes[ 0 ] );
        // serializing in file
        this->top_fitness_genome.serialize( false );
        // children list
        std::vector<Genome> children = breedChildren();
        // cull all but best genome on all species
        cullSpecies();
        // add mutated children to species
        for ( Genome & child : children )
        {
            child.mutate();
            addGenomeToRespectiveSpecies( child );
        }
        // advance generation
        this->generation++;
        for ( Species & species : this->all_species )
        {
            for ( Genome & genome : species.genomes )
            {
                genome.setGeneration( this->generation );
            }
        }
    }

    /// <summary>
    /// Eliminates either half or every but the best genome in every species, also orders every specie's genomes by descending order.
    /// </summary>
    /// <param name="half_cull">Trigger half culling or full culling.</param>
    void Gym::cullSpecies( bool const & half_cull )
    {
        for ( Species & species : this->all_species )
        {
            // single genome species
            if ( species.genomes.size() == 1 )
            {
                return;
            }
            // sort in descending fitness order
            std::sort(
                species.genomes.begin(), species.genomes.end(),
                []( Genome & g1, Genome & g2 ) { return g1.getFitness() > g2.getFitness(); }
            );
            // set cull limit
            size_t remaining = half_cull ? species.genomes.size() / 2 : 1;
            // cull species to remaining count
            species.genomes.erase( species.genomes.begin() + remaining, species.genomes.end() );
        }
    }

    /// <summary>
    /// Removes all stale species from current generation. Also orders species by their best genome fitness.
    /// </summary>
    /// <remarks>
    /// A specie is considered stale if has over Settings::SPECIES_STALE_THRESHOLD genomes and no progress was made.
    /// </remarks>
    void Gym::removeStaleSpecies()
    {
        // single species
        if ( this->all_species.size() == 1 )
        {
            return;
        }
        // sort species in descending order
        std::sort(
            this->all_species.begin(), this->all_species.end(),
            []( Species & s1, Species & s2 ) { return s1.genomes[ 0 ].getFitness() > s2.genomes[ 0 ].getFitness(); }
        );
        // species iterator from beginning
        auto species_iterator = this->all_species.begin() + 1;
        // remove all stale species
        while ( species_iterator != this->all_species.end() )
        {
            if ( species_iterator->genomes.size() < Settings::SPECIES_STALE_THRESHOLD )
            {
                species_iterator = this->all_species.erase( species_iterator );
            }
            else
            {
                species_iterator++;
            }
        }
    }

    /// <summary>
    /// Removes all weak species from current generation.
    /// </summary>
    /// <remarks>
    /// A specie is considered weak if has no breed chance.
    /// </remarks>
    void Gym::removeWeakSpecies()
    {
        // single species
        if ( this->all_species.size() == 1 )
        {
            return;
        }
        // total average fitness
        float total_average_fitness = totalAverageFitness();
        // species iterator from beginning
        auto species_iterator = this->all_species.begin() + 1;
        // remove all weak species
        while ( species_iterator != this->all_species.end() )
        {
            if ( species_iterator->speciesBreedCount( total_average_fitness ) < Settings::SPECIES_BREED_THRESHOLD )
            {
                species_iterator = this->all_species.erase( species_iterator );
            }
            else
            {
                species_iterator++;
            }
        }
    }

    /// <summary>
    /// Calculates the total average fitness of all species.
    /// </summary>
    /// <returns>The total average calculated fitness.</returns>
    float const Gym::totalAverageFitness() const
    {
        // sum of all species's average fitness
        float total = 0.0f;
        // calculate sum
        for ( Species const & species : this->all_species )
        {
            total += species.averageFitness();
        }
        // return the average of all genomes sizes
        return total;
    }

    /// <summary>
    /// Breed all children till population target.
    /// </summary>
    /// <returns>The breed children.</returns>
    std::vector<Genome> const Gym::breedChildren() const
    {
        // bread children
        std::vector<Genome> children = {};
        // total average fitness
        float total_average_fitness = totalAverageFitness();
        // for every species bread it's children
        for ( Species const & species : this->all_species )
        {
            // species breed count
            size_t breedCount = species.speciesBreedCount( total_average_fitness );
            // breed till breed count
            for ( size_t breed = 0; breed < breedCount; breed++ )
            {
                children.push_back( species.breedChild() );
            }
        }
        // return all created children
        return children;
    }

    /***************************************************************************
     *                                                                         *
     *   Species class.                                                        *
     *                                                                         *
     ***************************************************************************/
    /// <summary>
    /// Generates a simple species.
    /// </summary>
    Gym::Species::Species() : genomes( {} ) {}

    /// <summary>
    /// Checks if given genome belongs to species.
    /// </summary>
    /// <param name="genome">Genome to check.</param>
    /// <returns>Whenever genome belongs to species.</returns>
    bool const Gym::Species::fromSpecies( Genome const & genome ) const
    {
        // initialise values
        float disjoint = 0.0f;
        float weights = 0.0f;
        // compare with first
        this->genomes[ 0 ].compare( genome, disjoint, weights );
        // update
        disjoint = Settings::SPECIES_DELTA_DIJOINT * disjoint;
        weights = Settings::SPECIES_DELTA_WEIGHTS * weights;
        // if sum of disjoint and weights over threshold, not from species
        return disjoint + weights < Settings::SPECIES_DELTA_THRESHOLD;
    }

    /// <summary>
    /// Average fitness from a species.
    /// </summary>
    float const Gym::Species::averageFitness() const
    {
        // sum of all genome's fitness
        float total = 0.0f;
        // calculate sum
        for ( Genome const & genome : this->genomes )
        {
            total += genome.getFitness();
        }
        // return the average of all genomes sizes
        return this->genomes.empty() ? total : total / static_cast<float>( this->genomes.size() );
    }

    /// <summary>
    /// Calculates the species total offspring to reach the target population in comparison to all other species.
    /// </summary>
    /// <param name="species">Species for breed count</param>
    /// <returns>Breed count.</returns>
    size_t const Gym::Species::speciesBreedCount( float const & total_average_fitness ) const
    {
        return static_cast<size_t>( std::floor( ( averageFitness() / total_average_fitness ) * static_cast<float>( Settings::GYM_POPULATION ) ) ) - 1;
    }

    /// <summary>
    /// Breed a new genome from this species.
    /// </summary>
    /// <returns>Breed child.</returns>
    Genome Gym::Species::breedChild() const
    {
        // retrieve god
        Random_Generator & rng = Random_Generator::getInstance();

        // uses always top species parent as base
        Genome const & parent_one = this->genomes[ 0 ];
        // if below crossover change, crossover a new gene
        if ( rng.generateRandom() < Settings::CROSSOVER_CHANCE )
        {
            size_t parent_index_two = static_cast<size_t>( std::floor( rng.generateRandom() * static_cast<float>( this->genomes.size() ) ) );
            Genome const & parent_two = this->genomes[ parent_index_two ];
            // return generated child
            return parent_one.crossover( parent_two );
        }
        // otherwise copy a of the species top genome
        return Genome( parent_one );
    }
}

#endif
