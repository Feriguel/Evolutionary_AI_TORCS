/***************************************************************************

    file                 : Settings.cpp
    copyright            : (C) 2019 Miguel Ferreira and Jo�o Aniceto

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   MarI/O source code made by SethBling was used as a creation base.     *
 *                                                                         *
 ***************************************************************************/

#include "Settings.hpp"

#if defined (ROBOT_AXEL_SETTINGS)

namespace RobotAxel
{
    /// <summary>
    /// Current innovation number.
    /// </summary>
    unsigned int Settings::current_innovation = 0;

    /// <summary>
    /// Retrieve Gene innovation.
    /// </summary>
    /// <returns>Current innovation number.</returns>
    unsigned int const Settings::newInnovation()
    {
        current_innovation++;
        return current_innovation;
    }
}

#endif
