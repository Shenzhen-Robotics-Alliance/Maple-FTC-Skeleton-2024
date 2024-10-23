package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.CENTER_ODOMETER_WHEEL_INVERTED;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.CENTER_ODOMETER_WHEEL_NAME;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.IMU_PARAMS;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.LEFT_ODOMETER_WHEEL_INVERTED;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.LEFT_ODOMETER_WHEEL_NAME;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.ODOMETER_CENTER_WHEELS_OFFSET;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.ODOMETER_ENCODER_TICKS_PER_REVOLUTION;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.ODOMETER_WHEELS_RADIUS_METERS;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.ODOMETER_WHEELS_TRACK_WIDTH_METERS;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.RIGHT_ODOMETER_WHEEL_INVERTED;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.RIGHT_ODOMETER_WHEEL_NAME;

import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.constants.SystemConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.MapleOdometerWheelsOdometry;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.AllianceSide;
import org.firstinspires.ftc.teamcode.utils.MapleOdometerWheels.MapleEncoder;

import java.io.Closeable;
import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * declares all the subsystems of a robot
 * */
public final class RobotContainer implements Closeable {
    public final AllianceSide currentSide;

    public final MecanumDriveSubsystem driveSubsystem;

    public final MapleOdometerWheelsOdometry testOdometry;

    // public final AprilTagVision vision;
    /** create all the subsystem with the hardware map */
    public RobotContainer(HardwareMap hardwareMap, AllianceSide side) {
        this.currentSide = side;

        /* here we creates all the subsystems */
        final DcMotor
                frontLeft = hardwareMap.get(DcMotor.class, "frontLeft"),
                frontRight = hardwareMap.get(DcMotor.class, "frontRight"),
                backLeft = hardwareMap.get(DcMotor.class, "backLeft"),
                backRight = hardwareMap.get(DcMotor.class, "backRight");

        final IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(IMU_PARAMS);

        this.testOdometry = new MapleOdometerWheelsOdometry(
                ODOMETER_WHEELS_TRACK_WIDTH_METERS,
                ODOMETER_CENTER_WHEELS_OFFSET,
                new MapleEncoder(
                        hardwareMap.get(DcMotor.class, LEFT_ODOMETER_WHEEL_NAME),
                        LEFT_ODOMETER_WHEEL_INVERTED,
                        ODOMETER_ENCODER_TICKS_PER_REVOLUTION,
                        1,
                        ODOMETER_WHEELS_RADIUS_METERS,
                        50
                ),
                new MapleEncoder(
                        hardwareMap.get(DcMotor.class, RIGHT_ODOMETER_WHEEL_NAME),
                        RIGHT_ODOMETER_WHEEL_INVERTED,
                        ODOMETER_ENCODER_TICKS_PER_REVOLUTION,
                        1,
                        ODOMETER_WHEELS_RADIUS_METERS,
                        50
                ),
                new MapleEncoder(
                        hardwareMap.get(DcMotor.class, CENTER_ODOMETER_WHEEL_NAME),
                        CENTER_ODOMETER_WHEEL_INVERTED,
                        ODOMETER_ENCODER_TICKS_PER_REVOLUTION,
                        1,
                        ODOMETER_WHEELS_RADIUS_METERS,
                        50
                ),
                imu,
                new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)) // facing forward
        );
        testOdometry.register();
        testOdometry.setDefaultCommand(new FunctionalCommand(
                () -> {},
                () -> SystemConstants.telemetry.addData("Estimated Pose", testOdometry.getEstimatedPose()),
                (Boolean terminated) -> {},
                () -> false,
                testOdometry
        ));

        this.driveSubsystem = new MecanumDriveSubsystem(
                frontLeft, frontRight, backLeft, backRight,
                testOdometry
        );

//        this.vision = new AprilTagVision(
//                new VisionIOFTCVisionPortal(hardwareMap.get(WebcamName.class, "AprilTag Cam")),
//                driveSubsystem
//        );
    }

    @Override
    public void close() throws IOException {
        // vision.close();
    }
}
