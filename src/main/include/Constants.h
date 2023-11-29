#include <units/length.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

namespace k_internal {
    struct swerve_wrapper {
        constexpr swerve_wrapper(int driveId, int steeringId, int absoluteId, units::radian_t offset)
            : driveId{driveId}, steeringId{steeringId}, absoluteId{absoluteId}, offset{offset} {};

        const int driveId;
        const int steeringId;
        // + steeringEncoderReversed
        // + driveEncoderReversed
        // + absoluteEncoderReversed
        const int absoluteId;
        const units::radian_t offset;
    };
}

namespace k {
    struct data {
        static constexpr double wheelRadius = 0.0508;
        static constexpr int encoderResolution = 42;

        static constexpr k_internal::swerve_wrapper fl{1, 2, 0, 0_rad};
        static constexpr k_internal::swerve_wrapper fr{3, 4, 0, 0_rad};
        static constexpr k_internal::swerve_wrapper bl{5, 6, 0, 0_rad};
        static constexpr k_internal::swerve_wrapper br{7, 8, 0, 0_rad};
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