#ifndef UTILITY_HXX
#define UTILITY_HXX

#include <cmath>

///////////////////////////////////////////////////////////////////////////
//
//      DEFINES
//

#define PARAM       static constexpr
#define px_to_m(px) (px * (rsim::SQUARE_LENGTH * 2.0f) / rsim::env::BITMAP_SIZE)
#define m_to_px(m)  (m * rsim::env::BITMAP_SIZE / (rsim::SQUARE_LENGTH * 2.0f))

#define Q2

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

    PARAM bool RUN_HEADLESS = false;  // -
#ifndef Q2
    PARAM float START_X                  = 320.0f;        // px
    PARAM float START_Y                  = 724.0f;        // px
    PARAM float START_ORIENTATION        = -M_PI / 2.0f;  // rad
    PARAM float BALANCER_END_CENTER_X    = 96.0f;         // px
    PARAM float BALANCER_END_CENTER_Y    = 568.0f;        // px
    PARAM float BALANCER_END_RADIUS      = 8.0f;          // px
    PARAM float PIRATE_START_X           = 320.0f;        // px
    PARAM float PIRATE_START_Y           = 300.0f;        // px
    PARAM float PIRATE_START_ORIENTATION = M_PI / 2.0f;   // rad
#else
    PARAM float START_X                  = 108.0f;   // px
    PARAM float START_Y                  = 256.0f;   // px
    PARAM float START_ORIENTATION        = 0.0f;     // rad
    PARAM float BALANCER_END_CENTER_X    = 1920.0f;  // px
    PARAM float BALANCER_END_CENTER_Y    = 224.0f;   // px
    PARAM float BALANCER_END_RADIUS      = 8.0f;     // px
    PARAM float PIRATE_START_X           = 1812.0f;  // px
    PARAM float PIRATE_START_Y           = 128.0f;   // px
    PARAM float PIRATE_START_ORIENTATION = M_PI;     // rad
#endif
    PARAM float FAST_START_X                 = 224.0f;       // px
    PARAM float FAST_START_Y                 = 320.0f;       // px
    PARAM float FAST_START_ORIENTATION       = M_PI / 2.0f;  // rad
    PARAM float SAFETY_CAR_START_X           = 224.0f;       // px
    PARAM float SAFETY_CAR_START_Y           = 448.0f;       // px
    PARAM float SAFETY_CAR_START_ORIENTATION = M_PI / 2.0f;  // rad

    PARAM float SQUARE_LENGTH = 0.6f;  // m

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

        PARAM unsigned BITMAP_SIZE = 64;  // px
#ifndef Q2
        PARAM unsigned GRID_WIDTH  = 17;  // -
        PARAM unsigned GRID_HEIGHT = 16;  // -
#else
        PARAM unsigned GRID_WIDTH  = 31;  // -
        PARAM unsigned GRID_HEIGHT = 5;   // -
#endif
        PARAM unsigned MAP_WIDTH  = GRID_WIDTH * BITMAP_SIZE;   // px
        PARAM unsigned MAP_HEIGHT = GRID_HEIGHT * BITMAP_SIZE;  // px

    }  // namespace env

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
        PARAM float mu = 1.0f;
        PARAM float m  = 1.85f;
        PARAM float g  = 9.81f;
        PARAM float h  = 0.02f;
        PARAM float lf = 0.133f;
        PARAM float lr = 0.137f;
        PARAM float Iz = 0.09f;

        PARAM float C_f = 30.0f;
        PARAM float C_r = 40.0f;

        PARAM float Cm1 = 41.7960f;
        PARAM float Cm2 = 2.0152f;
        PARAM float Cm3 = 0.4328f;

        PARAM float MAX_WHEEL_ANGLE = 0.5f;    // rad
        PARAM float MAX_VELOCITY    = 500.0f;  // px/s
        PARAM float WHEELBASE       = 16.0f;   // px

        PARAM float WHEEL_DIAMETER            = 1.0f;                              // px
        PARAM float GEAR_RATIO_MOTOR_TO_WHEEL = static_cast<float>(3 / 2) * 1.0f;  // -

        PARAM float YAW_RATE_NOISE  = 0.1f;   // rad/s
        PARAM float MOTOR_RPM_NOISE = 20.0f;  // rpm
    }                                         // namespace vmodel

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

        PARAM int SENSOR_COUNT = 16;  // -

    }  // namespace smodel

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

        PARAM float Kp          = 10.0f;          // -
        PARAM float Ki          = 0.05f;          // -
        PARAM float Kd          = 0.025f;         // -
        PARAM float SPEED       = m_to_px(1.0f);  // px/s
        PARAM bool  USE_SEED    = true;           // -
        PARAM int   RANDOM_SEED = 1706273087;     // -

    }  // namespace pmodel

    //
    //      END PIRATE MODEL
    //
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //
    //      SAFETY CAR MODEL
    //

    namespace scmodel
    {

        PARAM float Kp    = 10.0f;   // -10
        PARAM float Ki    = 0.05f;   // -
        PARAM float Kd    = 0.025f;  // -
        PARAM float SPEED = 50.0f;   // px/s

    }  // namespace scmodel

    //
    //      END SAFETY CAR MODEL
    //
    ///////////////////////////////////////////////////////////////////////////

}  // namespace rsim

#endif  // UTILITY_HXX