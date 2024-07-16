package org.firstinspires.ftc.teamcode.Utils.MapleOdometerWheels;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Odometry;

public class OdometerWheelsOdometry extends Odometry<OdometerWheelsPositions> {
    /**
     * Constructs an Odometry object.
     *
     * @param trackWidthMeters
     * @param centerWheelOffsetMeters
     * @param gyroAngle         The angle reported by the gyroscope.
     * @param wheelPositions    The current encoder readings.
     * @param initialPoseMeters The starting position of the robot on the field.
     */
    public OdometerWheelsOdometry(double trackWidthMeters, double centerWheelOffsetMeters, Rotation2d gyroAngle, OdometerWheelsPositions wheelPositions, Pose2d initialPoseMeters) {
        super(
                new OdometerWheelsKinematics(
                        trackWidthMeters, centerWheelOffsetMeters
                ),
                gyroAngle,
                wheelPositions,
                initialPoseMeters
        );
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
