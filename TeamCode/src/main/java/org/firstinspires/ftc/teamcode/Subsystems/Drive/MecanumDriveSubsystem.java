package org.firstinspires.ftc.teamcode.Subsystems.Drive;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utils.InterpolatedMotorFeedForward;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;

public class MecanumDriveSubsystem extends SubsystemBase implements HolonomicDriveSubsystem {
    private final Motor frontLeft, frontRight, backLeft, backRight;
    private final MecanumDriveKinematics mecanumDriveKinematics;
    private final InterpolatedMotorFeedForward motorFeedForward;

    public MecanumDriveSubsystem(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        this.mecanumDriveKinematics = Constants.ChassisConfigs.KINEMATICS;
        this.motorFeedForward = new InterpolatedMotorFeedForward(Constants.ChassisConfigs.chassisMotorPower, Constants.ChassisConfigs.chassisSpeedsMetersPerSecond);
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public void runRawChassisSpeeds(ChassisSpeeds speeds) {
        final MecanumDriveWheelSpeeds wheelSpeeds = mecanumDriveKinematics.toWheelSpeeds(speeds);
        feedMotorSpeedForward(frontLeft, wheelSpeeds.frontLeftMetersPerSecond);
        feedMotorSpeedForward(frontRight, wheelSpeeds.frontRightMetersPerSecond);
        feedMotorSpeedForward(backLeft, wheelSpeeds.rearLeftMetersPerSecond);
        feedMotorSpeedForward(backRight, wheelSpeeds.rearRightMetersPerSecond);
    }

    private void feedMotorSpeedForward(Motor motor, double desiredSpeedMetersPerSecond) {
        motor.set(motorFeedForward.getMotorPower(desiredSpeedMetersPerSecond));
    }

    @Override
    public Pose2d getPose() {
        return null;
    }

    @Override
    public void setPose(Pose2d currentPose) {

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
