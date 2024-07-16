package org.firstinspires.ftc.teamcode.Subsystems.Drive;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utils.InterpolatedMotorFeedForward;
import org.firstinspires.ftc.teamcode.Utils.MapleTime;

import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;

public class MecanumDriveSubsystem extends SubsystemBase implements HolonomicDriveSubsystem {
    private final DcMotor frontLeft, frontRight, backLeft, backRight;
    private final MecanumDriveKinematics mecanumDriveKinematics;
    private final InterpolatedMotorFeedForward motorFeedForward;
    private final MecanumDrivePoseEstimator poseEstimator;
    private final IMU imu;

    public MecanumDriveSubsystem(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, IMU imu) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.imu = imu;

        this.mecanumDriveKinematics = Constants.ChassisConfigs.KINEMATICS;
        this.poseEstimator = new MecanumDrivePoseEstimator(
                mecanumDriveKinematics,
                getIMUAngle(),
                new MecanumDriveWheelPositions(0, 0, 0, 0),
                new Pose2d()
        );
        this.motorFeedForward = new InterpolatedMotorFeedForward(Constants.ChassisConfigs.chassisMotorPower, Constants.ChassisConfigs.chassisSpeedsMetersPerSecond);
    }

    @Override
    public void periodic() {
        poseEstimator.updateWithTime(
                MapleTime.getMatchTimeSeconds(),
                getIMUAngle(),
                new MecanumDriveWheelPositions(0, 0, 0, 0)
        );
    }

    @Override
    public void runRawChassisSpeeds(ChassisSpeeds speeds) {
        final MecanumDriveWheelSpeeds wheelSpeeds = mecanumDriveKinematics.toWheelSpeeds(speeds);
        feedMotorSpeedForward(frontLeft, wheelSpeeds.frontLeftMetersPerSecond * Constants.HardwareConfigs.frontLeftMotorDirection);
        feedMotorSpeedForward(frontRight, wheelSpeeds.frontRightMetersPerSecond * Constants.HardwareConfigs.frontRightMotorDirection);
        feedMotorSpeedForward(backLeft, wheelSpeeds.rearLeftMetersPerSecond * Constants.HardwareConfigs.backLeftMotorDirection);
        feedMotorSpeedForward(backRight, wheelSpeeds.rearRightMetersPerSecond * Constants.HardwareConfigs.backRightMotorDirection);

        Constants.telemetry.addData("Chassis Speeds", speeds);
        Constants.telemetry.addData("Wheel Speeds", wheelSpeeds);
    }

    private void feedMotorSpeedForward(DcMotor motor, double desiredSpeedMetersPerSecond) {
        motor.setPower(motorFeedForward.getMotorPower(desiredSpeedMetersPerSecond));
    }

    @Override
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    private Rotation2d getIMUAngle() {
        return Rotation2d.fromRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

    @Override
    public void setPose(Pose2d currentPose) {
        poseEstimator.resetPose(currentPose);
    }

    @Override
    public ChassisSpeeds getMeasuredChassisSpeedsRobotRelative() {
        return null;
    }

    @Override
    public double getChassisMaxLinearVelocity() {
        return Constants.ChassisConfigs.maxVelocityMetersPerSecond;
    }

    @Override
    public double getChassisMaxAngularVelocity() {
        return Math.toRadians(Constants.ChassisConfigs.maxAngularVelocityDegreesPerSecond);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {

    }
}
