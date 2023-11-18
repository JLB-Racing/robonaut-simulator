#ifndef UTILITY_HXX
#define UTILITY_HXX

#include <cmath>

///////////////////////////////////////////////////////////////////////////
//
//      DEFINES
//

#define PARAM static constexpr
#define px_to_m(px) (px * rsim::SQUARE_LENGTH / rsim::env::BITMAP_SIZE)
#define m_to_px(m) (m * rsim::env::BITMAP_SIZE / rsim::SQUARE_LENGTH)

//
//      END DEFINES
//
///////////////////////////////////////////////////////////////////////////

namespace rsim
{
    ///////////////////////////////////////////////////////////////////////////
    //
    //      SIMULATION
    //

    PARAM bool RUN_HEADLESS = false; // -

    PARAM float START_X = 320.0f;                 // px
    PARAM float START_Y = 724.0f;                 // px
    PARAM float START_ORIENTATION = -M_PI / 2.0f; // rad

    PARAM float PIRATE_START_X = 320.0f;                // px
    PARAM float PIRATE_START_Y = 300.0f;                // px
    PARAM float PIRATE_START_ORIENTATION = M_PI / 2.0f; // rad

    PARAM float SQUARE_LENGTH = 0.6; // m

    //
    //      END SIMULATION
    //
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //
    //      ENVIROMENT
    //

    namespace env
    {

        PARAM unsigned BITMAP_SIZE = 64;                       // px
        PARAM unsigned GRID_WIDTH = 17;                        // -
        PARAM unsigned GRID_HEIGHT = 16;                       // -
        PARAM unsigned MAP_WIDTH = GRID_WIDTH * BITMAP_SIZE;   // px
        PARAM unsigned MAP_HEIGHT = GRID_HEIGHT * BITMAP_SIZE; // px

    } // namespace env

    //
    //      END ENVIROMENT
    //
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //
    //      VEHICLE MODEL
    //

    namespace vmodel
    {

        PARAM float MAX_WHEEL_ANGLE = 1.0f; // rad
        PARAM float MAX_VELOCITY = 500.0f;  // px/s
        PARAM float WHEELBASE = 16.0f;      // px

        PARAM float WHEEL_DIAMETER = 1.0f;                                        // px
        PARAM float GEAR_RATIO_MOTOR_TO_WHEEL = static_cast<float>(3 / 2) * 1.0f; // -

        PARAM float YAW_RATE_NOISE = 0.1f;   // rad/s
        PARAM float MOTOR_RPM_NOISE = 20.0f; // rpm
    }                                        // namespace vmodel

    //
    //      END VEHICLE MODEL
    //
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //
    //      SENSOR MODEL
    //

    namespace smodel
    {

        PARAM int SENSOR_COUNT = 16; // -

    } // namespace smodel

    //
    //      END SENSOR MODEL
    //
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //
    //      PIRATE MODEL
    //

    namespace pmodel
    {

        PARAM float Kp = 0.8f;     // -
        PARAM float Ki = 0.01f;    // -
        PARAM float Kd = 0.6f;     // -
        PARAM float SPEED = 20.0f; // px/s

    } // namespace pmodel

    //
    //      END PIRATE MODEL
    //
    ///////////////////////////////////////////////////////////////////////////

} // namespace rsim

#endif // UTILITY_HXX