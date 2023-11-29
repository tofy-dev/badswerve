#include <units/length.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

namespace k_internal {
    enum enc{reversed, normal};

    struct swerve_wrapper {
        constexpr swerve_wrapper(int driveId, int steeringId, int absoluteId, units::radian_t offset) :
            swerve_wrapper(driveId, steeringId, absoluteId, offset, enc::normal, enc::normal, enc::normal) {};

        constexpr swerve_wrapper(int driveId, int steeringId, int absoluteId, units::radian_t offset, bool driveReversed, bool steeringReversed, bool absoluteReversed)
            : driveId{driveId}, steeringId{steeringId}, absoluteId{absoluteId}, offset{offset},
              driveReversed{driveReversed}, steeringReversed{steeringReversed}, absoluteReversed{absoluteReversed} {};


        const int driveId;
        const int steeringId;
        const int absoluteId;

        const bool driveReversed;
        const bool steeringReversed;
        const bool absoluteReversed;

        const units::radian_t offset;
    };
}

namespace k {
    using namespace k_internal;
    struct data {
        static constexpr double wheelRadius = 0.0508; // 2 in --> meters
        static constexpr int encoderResolution = 42;

        // format                          dr st ab offset dr_reversed  st_reversed  ab_reversed
        static constexpr swerve_wrapper fl{5, 6, 2, 0.188_rad, enc::reversed, enc::reversed, enc::normal};
        static constexpr swerve_wrapper bl{7, 8, 3, 3.000_rad, enc::reversed, enc::reversed, enc::normal};
        static constexpr swerve_wrapper fr{3, 4, 1, 0.688_rad, enc::normal, enc::reversed, enc::normal};
        static constexpr swerve_wrapper br{1, 2, 0, 3.100_rad, enc::reversed, enc::reversed, enc::normal};
    };

    struct caps {
        static constexpr units::meters_per_second_t maxSpeed =
            3.0_mps;  // 3 meters per second
        static constexpr units::radians_per_second_t maxAngularSpeed{
            std::numbers::pi};  // 1/2 rotation per second

        static constexpr auto moduleMaxAngularVelocity =
            std::numbers::pi * 1_rad_per_s;  // radians per second
        static constexpr auto moduleMaxAngularAcceleration =
            std::numbers::pi * 2_rad_per_s / 1_s;  // radians per second^2
    };

    struct io {
        static constexpr int drivePort = 0;
    };
}