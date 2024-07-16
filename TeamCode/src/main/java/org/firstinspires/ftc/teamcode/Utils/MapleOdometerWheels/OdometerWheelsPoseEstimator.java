package org.firstinspires.ftc.teamcode.Utils.MapleOdometerWheels;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class OdometerWheelsPoseEstimator extends PoseEstimator<OdometerWheelsPositions> {
    /**
     * Constructs an Odometer Wheels Pose Estimator with default standard deviations for the model and vision measurements.
     * The default standard deviations of the model states are:
     *  0.02 meters for x,
     *  0.02 meters for y,
     *  and 0.01 radians for heading.
     * The default standard deviations of the vision measurements are:
     *  0.1 meters for x,
     *  0.1 meters for y,
     *  and 0.1 radians for heading.
     * @param trackWidthMeters
     * @param centerWheelOffsetMeters
     * @param gyroAngle
     * @param leftWheelMeters
     * @param rightWheelMeters
     * @param centerWheelMeters
     * @param initialPose
     */
    public OdometerWheelsPoseEstimator(
            double trackWidthMeters, double centerWheelOffsetMeters,
            Rotation2d gyroAngle, double leftWheelMeters, double rightWheelMeters, double centerWheelMeters,
            Pose2d initialPose) {
        super(
                new OdometerWheelsKinematics(trackWidthMeters, centerWheelOffsetMeters),
                new OdometerWheelsOdometry(
                        trackWidthMeters, centerWheelOffsetMeters, gyroAngle,
                        new OdometerWheelsPositions(leftWheelMeters, rightWheelMeters, centerWheelMeters),
                        initialPose
                ),
                VecBuilder.fill(0.01, 0.01, 0.005), VecBuilder.fill(0.2, 0.2, 0.35)
        );
    }

    /**
     * Constructs a PoseEstimator.
     * @param trackWidthMeters
     * @param centerWheelOffsetMeters
     * @param gyroAngle
     * @param leftWheelMeters
     * @param rightWheelMeters
     * @param centerWheelMeters
     * @param initialPose
     * @param stateStdDevs             Standard deviations of the pose estimate (x position in meters, y position
     *                                 in meters, and heading in radians). Increase these numbers to trust your state estimate
     *                                 less.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
     *                                 in meters, y position in meters, and heading in radians). Increase these numbers to trust
     *                                 the vision pose measurement less.
     */
    public OdometerWheelsPoseEstimator(
            double trackWidthMeters, double centerWheelOffsetMeters,
            Rotation2d gyroAngle, double leftWheelMeters, double rightWheelMeters, double centerWheelMeters,
            Pose2d initialPose,
            Matrix<N3, N1> stateStdDevs, Matrix<N3, N1> visionMeasurementStdDevs) {
        super(
                new OdometerWheelsKinematics(trackWidthMeters, centerWheelOffsetMeters),
                new OdometerWheelsOdometry(
                        trackWidthMeters, centerWheelOffsetMeters, gyroAngle,
                        new OdometerWheelsPositions(leftWheelMeters, rightWheelMeters, centerWheelMeters),
                        initialPose
                ),
                stateStdDevs, visionMeasurementStdDevs
        );
    }

    /**
     * Resets the robot's position on the field.
     *
     * <p>The gyroscope angle does not need to be reset here on the user's robot code. The library
     * automatically takes care of offsetting the gyro angle.
     *
     * @param gyroAngle The angle reported by the gyroscope.
     * @param leftWheelMeters
     * @param rightWheelMeters
     * @param centerWheelMeters
     * @param currentPose The position on the field that your robot is at.
     */
    public void resetPosition(Rotation2d gyroAngle, double leftWheelMeters, double rightWheelMeters, double centerWheelMeters, Pose2d currentPose) {
        resetPosition(
                gyroAngle,
                new OdometerWheelsPositions(leftWheelMeters, rightWheelMeters, centerWheelMeters),
                currentPose
        );
    }

    /**
     * Updates the pose estimator with wheel encoder and gyro information. This should be called every
     * loop.
     *
     * @param gyroAngle The current gyro angle.
     * @param leftWheelMeters
     * @param rightWheelMeters
     * @param centerWheelMeters
     * @return The estimated pose of the robot in meters.
     */
    public Pose2d update(Rotation2d gyroAngle, double leftWheelMeters, double rightWheelMeters, double centerWheelMeters) {
        return update(
                gyroAngle,
                new OdometerWheelsPositions(leftWheelMeters, rightWheelMeters, centerWheelMeters)
        );
    }

    /**
     * Updates the pose estimator with wheel encoder and gyro information. This should be called every
     * loop.
     *
     * @param currentTimeSeconds Time at which this method was called, in seconds.
     * @param gyroAngle The current gyro angle.
     * @return The estimated pose of the robot in meters.
     */
    public Pose2d updateWithTime(double currentTimeSeconds, Rotation2d gyroAngle, double leftWheelMeters, double rightWheelMeters, double centerWheelMeters) {
        return updateWithTime(
                currentTimeSeconds,
                gyroAngle,
                new OdometerWheelsPositions(leftWheelMeters, rightWheelMeters, centerWheelMeters)
        );
    }
}
