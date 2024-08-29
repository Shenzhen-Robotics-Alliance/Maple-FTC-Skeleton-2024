package org.firstinspires.ftc.teamcode.Utils.MapleOdometerWheels;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.Utils.MapleTime;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class MapleOdometerWheelsOdometry implements Subsystem {
    private final OdometerWheelsPoseEstimator poseEstimator;
    private final MapleEncoder leftOdometerWheel, rightOdometerWheel, centerOdometerWheel;
    private final IMU imu;
    private Rotation2d currentRotation;
    private double previousIMUUpdateTimeSeconds = 0;
    private OdometerWheelsPositions previousPositions;

    /**
     * @param leftOdometerWheel the odometer wheel on the LEFT side of the robot, its reading goes POSITIVE when robot drives FORWARD
     * @param rightOdometerWheel the odometer wheel on the RIGHT side of the robot, its reading goes POSITIVE when robot drives FORWARD
     * @param centerOdometerWheel the odometer wheel on the CENTER of the robot, its reading goes POSITIVE when robot drives LEFTWARDS
     * */
    public MapleOdometerWheelsOdometry(
            double trackWidthMeters, double centerWheelOffsetMeters,
            MapleEncoder leftOdometerWheel, MapleEncoder rightOdometerWheel, MapleEncoder centerOdometerWheel, IMU imu,
            Pose2d initialPose) {
        this.leftOdometerWheel = leftOdometerWheel;
        this.rightOdometerWheel = rightOdometerWheel;
        this.centerOdometerWheel = centerOdometerWheel;
        this.imu = imu;

        pollEncodersBlocking();
        poseEstimator = new OdometerWheelsPoseEstimator(
                trackWidthMeters, centerWheelOffsetMeters,
                getIMUAngleBlocking(),
                getLatestPositions(),
                initialPose
        );

        this.currentRotation = initialPose.getRotation();
        this.previousPositions = getLatestPositions();
    }

    private void pollEncodersBlocking() {
        leftOdometerWheel.poll();
        rightOdometerWheel.poll();
        centerOdometerWheel.poll();
    }

    private Rotation2d getIMUAngleBlocking() {
        return Rotation2d.fromRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

    public OdometerWheelsPositions getLatestPositions() {
        return new OdometerWheelsPositions(
                leftOdometerWheel.getDistanceMeters(),
                rightOdometerWheel.getDistanceMeters(),
                centerOdometerWheel.getDistanceMeters()
        );
    }

    public OdometerWheelsSpeeds getLatestSpeeds() {
        return new OdometerWheelsSpeeds(
                leftOdometerWheel.getVelocityMetersPerSecond(),
                rightOdometerWheel.getVelocityMetersPerSecond(),
                centerOdometerWheel.getVelocityMetersPerSecond()
        );
    }

    /**
     * fetches the data cached by encoder thread (if encoder thread enabled) and feed these data to the pose estimator
     * */
    public void periodic() {
        pollEncodersBlocking();
        final Twist2d twist2d = poseEstimator.kinematics.toTwist2d(previousPositions, getLatestPositions());
        previousPositions = getLatestPositions();


        if (MapleTime.getMatchTimeSeconds() - previousIMUUpdateTimeSeconds > 1.0/ Constants.SystemConfigs.IMU_UPDATE_HZ) {
            currentRotation = getIMUAngleBlocking();
            previousIMUUpdateTimeSeconds = MapleTime.getMatchTimeSeconds();
        } else
            currentRotation = currentRotation.plus(Rotation2d.fromRadians(twist2d.dtheta));

        poseEstimator.updateWithTime(
                MapleTime.getMatchTimeSeconds(),
                currentRotation,
                getLatestPositions()
        );
    }

    public void resetPose(Pose2d currentPose) {
        this.poseEstimator.resetPosition(getIMUAngleBlocking(), getLatestPositions(), currentPose);
    }

    public void addVisionMeasurement(Pose2d visionRobotPose, double timeStamp) {
        this.poseEstimator.addVisionMeasurement(visionRobotPose, timeStamp);
    }

    public void addVisionMeasurement(Pose2d visionRobotPose, double timeStamp, Matrix<N3, N1> visionMeasurementStdDev) {
        this.poseEstimator.addVisionMeasurement(visionRobotPose, timeStamp, visionMeasurementStdDev);
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }
}
