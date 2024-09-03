package org.firstinspires.ftc.teamcode.utils.MapleOdometerWheels;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Odometry;

public class OdometerWheelsOdometry extends Odometry<OdometerWheelsPositions> {
    /**
     * Constructs an Odometry object.
     *
     * @param gyroAngle         The angle reported by the gyroscope.
     * @param wheelPositions    The current encoder readings.
     * @param initialPoseMeters The starting position of the robot on the field.
     */
    public OdometerWheelsOdometry(OdometerWheelsKinematics kinematics, Rotation2d gyroAngle, OdometerWheelsPositions wheelPositions, Pose2d initialPoseMeters) {
        super(kinematics, gyroAngle, wheelPositions, initialPoseMeters);
    }

    public void resetPosition(Rotation2d gyroAngle, double leftWheelMeters, double rightWheelMeters, double centerWheelMeters, Pose2d currentPose) {
        super.resetPosition(
                gyroAngle,
                new OdometerWheelsPositions(leftWheelMeters, rightWheelMeters, centerWheelMeters),
                currentPose
        );
    }

    public Pose2d update(Rotation2d gyroAngle, double leftWheelMeters, double rightWheelMeters, double centerWheelMeter) {
        return super.update(
                gyroAngle,
                new OdometerWheelsPositions(leftWheelMeters, rightWheelMeters, centerWheelMeter)
        );
    }
}
