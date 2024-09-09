package org.firstinspires.ftc.teamcode.subsystems.drive;

import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.BACK_LEFT_MOTOR_DIRECTION;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.BACK_RIGHT_MOTOR_DIRECTION;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.FRONT_LEFT_MOTOR_DIRECTION;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.FRONT_RIGHT_MOTOR_DIRECTION;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.MAX_VELOCITY_METERS_PER_SECOND;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.DriveTrainConstants;
import org.firstinspires.ftc.teamcode.constants.SystemConstants;
import org.firstinspires.ftc.teamcode.utils.MapleTime;

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
    private final MecanumDrivePoseEstimator poseEstimator;
    private final IMU imu;

    public MecanumDriveSubsystem(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, IMU imu) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.imu = imu;

        this.mecanumDriveKinematics = DriveTrainConstants.KINEMATICS;
        this.poseEstimator = new MecanumDrivePoseEstimator(
                mecanumDriveKinematics,
                getIMUAngle(),
                new MecanumDriveWheelPositions(0, 0, 0, 0),
                new Pose2d()
        );
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
        runDriveOpenLoop(frontLeft, wheelSpeeds.frontLeftMetersPerSecond * FRONT_LEFT_MOTOR_DIRECTION);
        runDriveOpenLoop(frontRight, wheelSpeeds.frontRightMetersPerSecond * FRONT_RIGHT_MOTOR_DIRECTION);
        runDriveOpenLoop(backLeft, wheelSpeeds.rearLeftMetersPerSecond * BACK_LEFT_MOTOR_DIRECTION);
        runDriveOpenLoop(backRight, wheelSpeeds.rearRightMetersPerSecond * BACK_RIGHT_MOTOR_DIRECTION);

        SystemConstants.telemetry.addData("Chassis Speeds", speeds);
        SystemConstants.telemetry.addData("Wheel Speeds", wheelSpeeds);
    }

    private void runDriveOpenLoop(DcMotor motor, double desiredSpeedMetersPerSecond) {
        motor.setPower(desiredSpeedMetersPerSecond);
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
        return MAX_VELOCITY_METERS_PER_SECOND;
    }

    @Override
    public double getChassisMaxAngularVelocity() {
        return Math.toRadians(MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp);
    }
}
