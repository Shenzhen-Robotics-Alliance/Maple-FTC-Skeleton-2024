package org.firstinspires.ftc.teamcode.subsystems.drive;

import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.KINEMATICS;
import static org.firstinspires.ftc.teamcode.constants.FieldConstants.FIELD_HEIGHT_METERS;
import static org.firstinspires.ftc.teamcode.constants.FieldConstants.FIELD_WIDTH_METERS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.SystemConstants;
import org.firstinspires.ftc.teamcode.utils.MapleOdometerWheels.MapleEncoder;
import org.firstinspires.ftc.teamcode.utils.MapleOdometerWheels.OdometerWheelsPoseEstimator;
import org.firstinspires.ftc.teamcode.utils.MapleOdometerWheels.OdometerWheelsPositions;
import org.firstinspires.ftc.teamcode.utils.MapleOdometerWheels.OdometerWheelsSpeeds;
import org.firstinspires.ftc.teamcode.utils.MapleTime;

import java.util.Arrays;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

        resetPose(initialPose);
    }

    private void pollEncodersBlocking() {
        leftOdometerWheel.poll();
        rightOdometerWheel.poll();
        centerOdometerWheel.poll();
    }

    /**
    * poll the newest reading from imu
    * note that this method will block the thread for 10~20 ms (because the control hub sucks)
    * so avoid calling it periodically
    * */
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
    private TelemetryPacket packet;
    public void periodic() {
        pollEncodersBlocking();
        final Twist2d twist2d = poseEstimator.kinematics.toTwist2d(previousPositions, getLatestPositions());
        previousPositions = getLatestPositions();

        if (MapleTime.getMatchTimeSeconds() - previousIMUUpdateTimeSeconds > 1.0/ SystemConstants.IMU_UPDATE_HZ) {
            currentRotation = getIMUAngleBlocking();
            previousIMUUpdateTimeSeconds = MapleTime.getMatchTimeSeconds();
        } else
            currentRotation = currentRotation.plus(Rotation2d.fromRadians(twist2d.dtheta));

        poseEstimator.updateWithTime(
                MapleTime.getMatchTimeSeconds(),
                currentRotation,
                getLatestPositions()
        );

        packet = new TelemetryPacket();
        displayPoseOnDashboard(getEstimatedPose(), "black");
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public void resetPose(Pose2d currentPose) {
        pollEncodersBlocking();
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

    public void displayPoseOnDashboard(Pose2d pose, String color) {
        final Translation2d center = pose.getTranslation();
        final Rotation2d robotFacing = pose.getRotation();
        final Translation2d[] corners = new Translation2d[] {
                center.plus(KINEMATICS.getFrontLeft().rotateBy(robotFacing)),
                center.plus(KINEMATICS.getFrontRight().rotateBy(robotFacing)),
                center.plus(KINEMATICS.getRearRight().rotateBy(robotFacing)),
                center.plus(KINEMATICS.getRearLeft().rotateBy(robotFacing))
        };
        System.out.println(Arrays.toString(corners));
        final double[] xCorners = new double[4];
        final double[] yCorners = new double[4];
        for (int i = 0; i < 4; i++) {
            xCorners[i] = (corners[i].getY() / FIELD_WIDTH_METERS * 144) - (double) 144/2;
            yCorners[i] = -(corners[i].getX() / FIELD_HEIGHT_METERS * 144) + (double) 144/2;
        }
        packet.fieldOverlay()
                .setFill(color)
                .fillPolygon(xCorners, yCorners);
    }
}
