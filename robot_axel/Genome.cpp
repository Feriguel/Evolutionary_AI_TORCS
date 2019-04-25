/***************************************************************************

    file                 : Genome.cpp
    copyright            : (C) 2019 Miguel Ferreira and Jo�o Aniceto

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   MarI/O source code made by SethBling was used as a creation base.     *
 *                                                                         *
 ***************************************************************************/

#include "Genome.hpp"

#if defined (ROBOT_AXEL_GENOME)

#include <algorithm>
#include <cmath>
#include <cassert>

#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <dirent.h>
#include <errno.h>

namespace RobotAxel
{
    /***************************************************************************
     *                                                                         *
     *   Genome interface.                                                     *
     *                                                                         *
     ***************************************************************************/
    /// <summary>
    /// Generate basic Genome.
    /// </summary>
    Genome::Genome( bool const & generate_top ) : fitness( 0 ), generation( 0 ), total_neurons( 0 ), network( {} )
    {
        if ( generate_top )
        {
            DIR* dir = opendir( "genomes/" );
            if ( dir )
            {
                ;
            }
            else
            {
                mkdir( "genomes/", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH );
            }

            std::ifstream myfile( "genomes/top_genome_final.txt" );
            std::string line;
            if ( myfile.is_open() )
            /*if(false)*/
            {
                getline( myfile, line );
                this->generation = static_cast<size_t>( std::stof( line ) );
                getline( myfile, line );
                this->fitness = std::stof( line );
                getline( myfile, line );
                this->total_neurons = static_cast<size_t>( std::stof( line ) );

                std::vector<std::string> genomeInfo;
                std::string tmp_value;

                while ( myfile >> tmp_value ) { genomeInfo.push_back( tmp_value ); }
                size_t i = 0;
                for ( i = 0; i < ( genomeInfo.size() / 4 ); i++ )
                {
                    Gene newGene = Gene(
                        static_cast<size_t>( std::stof( genomeInfo[ i * 4 ] ) ),
                        static_cast<size_t>( std::stof( genomeInfo[ ( i * 4 ) + 1 ] ) ),
                        static_cast<bool>( std::stof( genomeInfo[ ( i * 4 ) + 3 ] ) )
                    );
                    newGene.weight = std::stof( genomeInfo[ ( i * 4 ) + 2 ] );
                    this->network.push_back( newGene );
                }
                myfile.close();
            }
            else
            {
                // total neurons is equal to Inputs + Outputs
                this->total_neurons = Settings::INPUTS + Settings::OUTPUTS;
                // generate genes to fully connect each input to all outputs
                if ( Settings::BASIC_FULL_LINK )
                {
                    for ( size_t output_index = Settings::INPUTS; output_index < this->total_neurons; output_index++ )
                    {
                        for ( size_t input_index = 0; input_index < Settings::INPUTS; input_index++ )
                        {
                            this->network.push_back( Gene( input_index, output_index, true ) );
                        }
                    }
                }
            }
        }
    }

    /// <summary>
    /// Retrieves AI gene number.
    /// </summary>
    /// <returns>AI total genes.</returns>
    size_t Genome::getTotalGenes() const { return this->network.size(); }

    /// <summary>
    /// Retrieves AI neuron number.
    /// </summary>
    /// <returns>AI total neurons.</returns>
    size_t Genome::getTotalNeurons() const { return this->total_neurons; }

    /// <summary>
    /// Retrieves AI generation.
    /// </summary>
    /// <returns>AI fitness.</returns>
    size_t Genome::getGeneration() const { return this->generation; }

    /// <summary>
    /// Retrieves AI fitness.
    /// </summary>
    /// <returns>AI fitness.</returns>
    float Genome::getFitness() const { return this->fitness; }

    /// <summary>
    /// Updates Genome's generation reference.
    /// </summary>
    /// <param name="generation">The new generation value.</param>
    void Genome::setGeneration( size_t const & generation ) { this->generation = generation; }

    /// <summary>
    /// Updates Genome's fitness reference.
    /// </summary>
    /// <param name="fitness">The new fitness value.</param>
    void Genome::setFitness( float const & fitness ) { this->fitness = fitness; }

    /// <summary>
    /// Calculates outputs according to the current network.
    /// </summary>
    /// <param name="outputs">Array with output neurons values.</param>
    /// <param name="inputs">The input array used for the update.</param>
    void Genome::evaluate( std::array <float, Settings::OUTPUTS> & outputs, std::array<float, Settings::INPUTS> const & inputs ) const
    {
        for ( size_t index = 0; index < Settings::INPUTS; index++ )
        {
            assert( inputs[ index ] >= 0.0f && inputs[ index ] <= 1.0f );
        }
        // no genes, do nothing
        if ( this->network.empty() ) return;
        // generate neuron network
        std::vector<Neuron> neurons = {};
        // fill with neurons
        for ( size_t index = 0; index < this->total_neurons; index++ )
        {
            neurons.push_back( Neuron( index, this->network ) );
        }
        // update input neurons
        for ( size_t index = 0; index < Settings::INPUTS; index++ )
        {
            neurons[ index ].value = inputs[ index ];
        }

        // update network
        for ( size_t index = Settings::INPUTS; index < this->total_neurons; index++ )
        {
            // get neuron
            Neuron & neuron = neurons[ index ];
            // no incoming, continue
            if ( neuron.incoming.empty() ) continue;
            // calculate sum of all incoming genes values
            for ( Gene const * incoming_gene : neuron.incoming )
            {
                if ( incoming_gene->enabled ) neuron.value += neurons[ incoming_gene->from ].value * incoming_gene->weight;
            }
            // treat sum result
            neuron.value = ( Settings::SIGMOID_RANGE / ( 1 + std::exp( Settings::SIGMOID_GROWTH_RATE * neuron.value ) ) ) + Settings::SIGMOID_OFFSET;
        }

        size_t output_neuron_offset = this->total_neurons - Settings::OUTPUTS;
        // retrieve outputs
        for ( size_t index = 0; index < Settings::OUTPUTS; index++ )
        {
            assert( index < outputs.size() );
            outputs[ index ] = neurons[ index + output_neuron_offset ].value;
        }
        for ( size_t index = 0; index < Settings::OUTPUTS; index++ )
        {
            assert( outputs[ index ] >= Settings::LINK_OFFSET && outputs[ index ] <= Settings::LINK_OFFSET + Settings::LINK_RANGE );
        }
    }

    /// <summary>
    /// Does one random mutation depending on the AI mutation settings.
    /// </summary>
    void Genome::mutate()
    {
        // retrieve god
        Random_Generator & rng = Random_Generator::getInstance();

        // randomizes a chance, if below weight chance, trigger weight mutation
        if ( rng.generateRandom() < Settings::MUTATE_WEIGHT_CHANCE )
        {
            mutateWeight();
        }
        // randomizes a chance, if below node chance, trigger node mutation
        if ( rng.generateRandom() < Settings::MUTATE_NODE_CHANCE && this->total_neurons < Settings::NEURON_LIMIT )
        {
            mutateNode();
        }
        // randomizes a chance, if below link chance, trigger link mutation
        if ( rng.generateRandom() < Settings::MUTATE_LINK_CHANCE )
        {
            mutateLink();
        }
        // randomizes a chance, if below state chance, trigger state mutation
        if ( rng.generateRandom() < Settings::MUTATE_STATE_CHANCE )
        {
            mutateState();
        }
    }

    /// <summary>
    /// Compares two genomes and retrieves % of disjoint genes (how many more new exist), and diference in weights of equal genes (same innovation num).
    /// </summary>
    /// <param name="other">Genome for comparison..</param>
    /// <param name="disjoint">Disjoint rating.</param>
    /// <param name="weights">Weight difference.</param>
    void Genome::compare( Genome const & other, float & disjoint, float & weights ) const
    {
        // no genes in both, do nothing
        if ( this->network.empty() && other.network.empty() ) return;
        // reset given values
        disjoint = 0.0f;
        weights = 0.0f;

        // number coincident
        float coincident = 0.0f;
        // maximum genes
        float maximum_genes = static_cast<float>( std::max( this->network.size(), other.network.size() ) );

        if ( this->network.empty() )
        {
            disjoint = 1.0f;
            weights = 1.0f;
            return;
        }

        // check for gene equalities and diferences ( equals are removed from other genes vector )
        for ( Gene const & gene : this->network )
        {
            // found flag
            bool found = false;
            // cycle trough all other to find if exist equal
            for ( Gene const & other_gene : other.network )
            {
                // check if have the same innovation
                if ( gene.innovation == other_gene.innovation )
                {
                    // add to weights and increment coincident
                    weights += fabsf( gene.weight - other_gene.weight );
                    coincident += 1.0f;
                    // stop cycle and change flag to found
                    found = true;
                    break;
                }
            }
            // no equal found
            if ( !found ) disjoint += 1.0f;
        }
        // update disjoint and weights
        disjoint = ( disjoint + static_cast<float>( other.network.size() ) ) / maximum_genes;
        weights = coincident == 0.0f ? 0.0f : weights / coincident;
    }

    /// <summary>
    /// Creates a new genome form a crossover between this genome and another one given, with characteristics of both.
    /// </summary>
    /// <remarks>
    /// The higher fitness genome should be the caller.
    /// </remarks>
    /// <param name="other">The genome for crossover</param>
    /// <returns>A new genome with characteristics of both parents.</returns>
    Genome Genome::crossover( Genome const & other ) const
    {
        // generated child
        Genome child = Genome( *this );
        // no genes, do nothing
        if ( this->network.empty() && other.network.empty() ) return child;
        // retrieve god
        Random_Generator & rng = Random_Generator::getInstance();

        // child has the maximum number of neurons between parents
        child.total_neurons = other.total_neurons < this->total_neurons ? this->total_neurons : other.total_neurons;
        // looks for possible gene additions from other
        for ( Gene const & other_gene : other.network )
        {
            // is gene found
            bool found = false;
            // find gene
            for ( Gene const & gene : this->network )
            {
                if ( other_gene == gene )
                {
                    found = true;
                    break;
                }
            }
            // if not found chance add
            if ( !found )
            {
                if ( rng.generateRandom() < Settings::CROSSOVER_GENE_ADDITION ) child.network.push_back( other_gene );
            }
        }
        // return generated child
        return child;
    }

    void Genome::serialize( bool finish_training )
    {
        std::ofstream myfile;
        std::cout << "---------------------Writing in file " << "genomes/top_genome_generation_" + std::to_string( this->generation ) + "_fitness_" + std::to_string( this->fitness ) + ".txt" << "!------------------------" << std::endl;
        std::string textFileName;
        if ( finish_training )
        {
            textFileName = "genomes/top_genome_final.txt";
        }
        else
        {
            textFileName = "genomes/top_genome_generation_" + std::to_string( this->generation ) + "_fitness_" + std::to_string( this->fitness ) + ".txt";
        }
        myfile.open( textFileName );
        // first line is generation
        myfile << std::to_string( this->generation ) + "\n";
        // second line fitness
        myfile << std::to_string( this->fitness ) + "\n";
        // third line total neurons
        myfile << std::to_string( this->total_neurons ) + "\n";
        // after all genes
        for ( Gene const & gene : this->network )
        {
            myfile << std::to_string( gene.from ) + " " + std::to_string( gene.to ) + " " + std::to_string( gene.weight ) + " " + std::to_string( gene.enabled ) + "\n";
        }

        myfile.close();
        std::cout << "Finished writing on file" << std::endl;

    }

    /***************************************************************************
     *                                                                         *
     *   Genome utilities.                                                     *
     *                                                                         *
     ***************************************************************************/
    /// <summary>
    /// Either randomizes weight or makes a small mutation in weight of all genes.
    /// </summary>
    void Genome::mutateWeight()
    {
        // no genes, do nothing
        if ( this->network.empty() ) return;
        // retrieve god
        Random_Generator & rng = Random_Generator::getInstance();

        // if chance type is below all chance, mutates all
        if ( rng.generateRandom() < Settings::MUTATE_WEIGHT_ALL_CHANCE )
        {
            // if below new random chance, randomizes all weights
            if ( rng.generateRandom() < Settings::MUTATE_WEIGHT_NEW_RANDOM_CHANCE ) { for ( Gene & gene : this->network ) gene.randomGeneWeight(); }
            // otherwise deviates current chances
            else { for ( Gene & gene : this->network ) gene.deviateGeneWeight(); }
        }
        // apply to a single random target
        else
        {
            // randomizes target
            size_t target_index = static_cast<size_t>( std::floor( rng.generateRandom() * static_cast<float>( this->network.size() ) ) );
            assert( target_index < this->network.size() );

            // change target
            if ( rng.generateRandom() < Settings::MUTATE_WEIGHT_NEW_RANDOM_CHANCE ) { this->network[ target_index ].randomGeneWeight(); }
            // otherwise deviates current chances
            else { this->network[ target_index ].deviateGeneWeight(); }

        }
    }

    /// <summary>
    /// Changes a random gene enabled state.
    /// </summary>
    void Genome::mutateState()
    {
        // no genes, do nothing
        if ( this->network.empty() ) return;
        // retrieve god
        Random_Generator & rng = Random_Generator::getInstance();

        // change all genes in neuron
        if ( rng.generateRandom() < Settings::MUTATE_STATE_INVERT_ALL_NEURON_GENES_CHANCE )
        {
            size_t target_index = static_cast<size_t>( std::floor( rng.generateRandom() * static_cast<float>( this->total_neurons ) ) );
            assert( target_index < this->total_neurons );
            // create target
            Neuron target = Neuron( target_index, this->network );
            // change all incoming enabled state
            for ( Gene const * target_gene : target.incoming )
            {
                for ( Gene & gene : this->network )
                {
                    if ( *target_gene == gene )
                    {
                        gene.enabled = !gene.enabled;
                        break;
                    }
                }
            }
        }
        // change single gene
        else
        {
            // randomizes target
            size_t target_index = static_cast<size_t>( std::floor( rng.generateRandom() * static_cast<float>( this->network.size() ) ) );
            assert( target_index < this->network.size() );
            // change enabled state
            this->network[ target_index ].enabled = !this->network[ target_index ].enabled;
        }
    }

    /// <summary>
    /// Creates a new link between two neurons if there is none.
    /// </summary>
    /// <remarks>
    /// To maintain the evaluations correct, only creates links from lower neuron indexes to higher ones (prevents use of non yet evaluated neurons).
    /// </remarks>
    void Genome::mutateLink()
    {
        // retrieve god
        Random_Generator & rng = Random_Generator::getInstance();

        // maximum range
        size_t range = this->total_neurons;
        // randomizes all neurons besides output
        size_t non_output_index = static_cast<size_t>( std::floor( rng.generateRandom() * static_cast<float>( range - Settings::OUTPUTS ) ) );
        assert( non_output_index < range );
        // randomizes all neurons besides input
        size_t non_input_index = static_cast<size_t>( std::floor( rng.generateRandom() * static_cast<float>( range - Settings::INPUTS ) ) ) + Settings::INPUTS;
        assert( non_output_index < range );

        // if same neuron, does nothing
        if ( non_output_index == non_input_index ) return;

        // random non-output neuron ( input + hidden ) and non-input neuron ( output + hidden )
        Neuron non_output = Neuron( non_output_index, this->network );
        Neuron non_input = Neuron( non_input_index, this->network );

        // if already connected, does nothing
        if ( non_output.hasLink( non_input_index ) || non_input.hasLink( non_output_index ) ) { return; }

        // if non_output has lower index add link from non-input to non-output
        if ( non_output_index < non_input_index ) { this->network.push_back( Gene( non_output_index, non_input_index, true ) ); }
        // if non_input has lower index add link from non-output to non-input
        if ( non_input_index < non_output_index ) { this->network.push_back( Gene( non_input_index, non_output_index, true ) ); }
    }

    /// <summary>
    /// Generates a new node mutation, by transforming a gene into the same connection with a "middle-man". Original gene is disabled.
    /// </summary>
    /// <remarks>
    /// Updates every other neuron gene connections from selected neuron with the changes in gene.from indexes.
    /// </remarks>
    void Genome::mutateNode()
    {
        // no genes, do nothing
        if ( this->network.empty() ) return;
        // retrieve god
        Random_Generator & rng = Random_Generator::getInstance();

        // randomize target
        size_t target_gene_index = static_cast<size_t>( std::floor( rng.generateRandom() * static_cast<float>( this->network.size() ) ) );
        assert( target_gene_index < this->network.size() );

        // if gene is disabled, does nothing
        if ( !this->network[ target_gene_index ].enabled ) return;

        // disable gene
        this->network[ target_gene_index ].enabled = false;

        // target to neuron index in outputs use the last index before outputs for new neuron other wise use the gene.to value
        size_t target_index = this->network[ target_gene_index ].to;
        if ( this->total_neurons - Settings::OUTPUTS < this->network[ target_gene_index ].to ) { target_index = this->total_neurons - Settings::OUTPUTS; }
        // increment total neurons in genome
        this->total_neurons++;

        // create genes
        Gene bot_gene = Gene( this->network[ target_gene_index ].from, target_index, true );
        Gene top_gene = Gene( target_index, this->network[ target_gene_index ].from, true );
        // update all genes indexes
        for ( Gene & gene : this->network )
        {
            if ( target_index <= gene.from ) { gene.from++; }
            if ( target_index <= gene.to ) { gene.to++; }
        }
        // add new genes
        this->network.push_back( bot_gene );
        this->network.push_back( top_gene );
    }

    /***************************************************************************
     *                                                                         *
     *   Gene, Neuron and Network classes.                                     *
     *                                                                         *
     ***************************************************************************/
    /// Generates a simple gene, with everything at 0 and not enabled.
    /// </summary>
    Genome::Gene::Gene()
        : innovation( 0 ), from( 0 ), weight( 0 ), enabled( false ) {}

    /// <summary>
    /// Generates a gene with given settings.
    /// </summary>
    /// <param name="from">Input neuron index.</param>
    /// <param name="enabled">Link connection status.</param>
    Genome::Gene::Gene( size_t const & from, size_t const & to, bool const & enabled )
        : innovation( Settings::newInnovation() ), from( from ), to( to ), weight( 0 ), enabled( enabled )
    {
        // randomizes a gene weight
        randomGeneWeight();
    }

    /// <summary>
    /// Deviates the current gene degree by a certain range.
    /// </summary>
    void Genome::Gene::deviateGeneWeight()
    {
        // retrieve god
        Random_Generator & rng = Random_Generator::getInstance();
        // generates a random gene weight
        this->weight += rng.generateRandom() * Settings::MUTATE_WEIGHT_DEVIATION_RANGE - Settings::MUTATE_WEIGHT_DEVIATION_RANGE / 2.0f;
        // if over limits set to limit
        if ( this->weight < Settings::LINK_OFFSET )
        { this->weight = Settings::LINK_OFFSET; }
        if ( this->weight > Settings::LINK_RANGE + Settings::LINK_OFFSET )
        { this->weight = Settings::LINK_RANGE + Settings::LINK_OFFSET; }
    }

    /// <summary>
    /// Deviates the current gene degree by a certain range.
    /// </summary>
    void Genome::Gene::randomGeneWeight()
    {
        // retrieve god
        Random_Generator & rng = Random_Generator::getInstance();
        // generates a random gene weight
        this->weight = rng.generateRandom() * Settings::LINK_RANGE + Settings::LINK_OFFSET;
    }

    /// <summary>
    /// Checks if both neurons have the same reference.
    /// </summary>
    /// <param name="other">The one for comparison.</param>
    /// <returns>Is equal.</returns>
    bool const Genome::Gene::operator== ( Gene const & other ) const
    {
        if ( this == &other ) return true;
        if ( this->innovation == other.innovation ) return true;
        if ( this->from == other.from && this->to == other.to ) return true;
        return false;
    }

    /// <summary>
    /// Generates a simple Neuron.
    /// </summary>
    /// <param name="index">Neuron index.</param>
    /// <param name="network">Gene network.</param>
    Genome::Neuron::Neuron( size_t const & index, std::vector<Gene> const & network ) : incoming( {} ), value( 0 )
    {
        // if neuron input does nothing
        if ( index < Settings::INPUTS ) return;
        // otherwise find gene
        for ( Gene const & gene : network )
        {
            if ( gene.to == index ) this->incoming.push_back( &gene );
        }
    }

    /// <summary>
    /// Checks if neuron already has link.
    /// </summary>
    /// <param name="neuron_index">Index of link to find.</param>
    /// <returns>If neuron has link.</returns>
    bool const Genome::Neuron::hasLink( size_t const & neuron_index ) const
    {
        for ( Gene const * gene : this->incoming )
        {
            if ( gene->from == neuron_index ) return true;
        }
        return false;
    }
}

#endif
