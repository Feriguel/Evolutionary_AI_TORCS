/***************************************************************************

    file                 : AI.cpp
    copyright            : (C) 2019 Miguel Ferreira and Jo�o Aniceto

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   MarI/O source code made by SethBling was used as a creation base.     *
 *                                                                         *
 ***************************************************************************/

#include "AI.hpp"

#if defined (ROBOT_AXEL_AI)

namespace RobotAxel
{
    /***************************************************************************
     *                                                                         *
     *   AI interface.                                                         *
     *                                                                         *
     ***************************************************************************/
    /// <summary>
    /// Creates the AI using the intended method depending of the objective.
    /// </summary>
    /// <param name="training">Whenever should the AI train.</param>
    AI::AI( bool const & training )
        :gym( generateGym( training ) ), best_ai( generateBestAI( training ) ) {}

    /// <summary>
    /// Destroys the created 
    /// </summary>
    AI::~AI()
    {
        // not training
        if ( this->best_ai != nullptr ) delete this->best_ai;
        // gym
        if ( this->gym != nullptr ) delete this->gym;
    }

    /// <summary>
    /// Calculates outputs according to the current network, using the Gym or the top AI.
    /// </summary>
    /// <param name="outputs">Array with output neurons values.</param>
    /// <param name="inputs">The input array used for the update.</param>
    void AI::evaluate( std::array <float, Settings::OUTPUTS> & outputs, std::array<float, Settings::INPUTS> const & inputs )
    {
        // not training
        if ( this->gym == nullptr ) return this->best_ai->evaluate( outputs, inputs );
        // gym
        return this->gym->evaluateCurrent( outputs, inputs );
    }

    /// <summary>
    /// Appraises AI progress in current environment, this progress is used for AI evaluation, does nothing .
    /// </summary>
    /// <param name="progress_rating">Progress evaluation of current AI situation.</param>
    void AI::appraise( float const & progress_rating )
    {
        // not training
        if ( this->gym == nullptr ) return;
        // gym
        this->gym->appraiseCurrent( progress_rating );
    }

    /// <summary>
    /// Indicates the Gym to advance to the next Genome for the fitness test, if it is a racing AI, does nothing.
    /// </summary>
    void AI::advanceInTrain()
    {
        // not training
        if ( this->gym == nullptr ) return;
        // gym
        this->gym->advanceInTrain();
    }

    /// <summary>
    /// Retrieves current training generation and species number in generation, as well current top progress.
    /// </summary>
    /// <param name="generation">Current AI generation if training, generation reference if not.</param>
    /// <param name="species">Number of species in generation if training, unchanged if not.</param>
    /// <param name="progress">Current fitness if training, unchanged if not.</param>
    /// <param name="top_progress">Current maximum fitness if training, AI progress if not.</param>
    /// <param name="top_genes">Top gene number.</param>
    /// <param name="top_neurons">Top neuron number.</param>
    void AI::getInformation( size_t & generation, size_t & species, float & progress, float & top_progress, size_t & top_genes, size_t & top_neurons ) const
    {
        // not training
        if ( this->gym == nullptr )
        {
            top_progress = this->best_ai->getFitness();
            generation = this->best_ai->getGeneration();
            top_genes = this->best_ai->getTotalGenes();
            top_neurons = this->best_ai->getTotalNeurons();
            return;
        }
        // gym
        this->gym->getInformation( generation, species, progress, top_progress, top_genes, top_neurons );
    }

    /***************************************************************************
     *                                                                         *
     *   AI utilities.                                                         *
     *                                                                         *
     ***************************************************************************/
    /// <summary>
    /// Generates the Gym for AI training if training is set.
    /// </summary>
    /// <param name="training">Whenever should the AI train.</param>
    /// <returns>A gym instance pointer or a null pointer.</returns>
    Gym * const AI::generateGym( bool const & training ) const
    {
        if ( training )
        { return new Gym(); }
        else
        { return nullptr; }
    }

    /// <summary>
    /// Generates the best AI training is not set.
    /// </summary>
    /// <param name="training">Whenever should the AI train.</param>
    /// <returns>A pointer with the top found Genome, or a null pointer.</returns>
    Genome const * const AI::generateBestAI( bool const & training ) const
    {
        if ( training )
        { return nullptr; }
        else
        { return new Genome( true ); }
    }
}

#endif
