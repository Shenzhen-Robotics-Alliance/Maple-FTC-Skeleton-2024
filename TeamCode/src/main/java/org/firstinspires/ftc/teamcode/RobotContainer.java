package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.AllianceSide;
import org.firstinspires.ftc.teamcode.utils.MapleOdometerWheels.MapleEncoder;
import org.firstinspires.ftc.teamcode.utils.MapleOdometerWheels.MapleOdometerWheelsOdometry;

import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * declares all the subsystems of a robot
 * */
public final class RobotContainer {
    public final AllianceSide currentSide;

    public final MecanumDriveSubsystem driveSubsystem;

    public final MapleOdometerWheelsOdometry testOdometry;
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

        this.driveSubsystem = new MecanumDriveSubsystem(
                frontLeft, frontRight, backLeft, backRight,
                imu
        );

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
                () -> Constants.telemetry.addData("Estimated Pose", testOdometry.getEstimatedPose()),
                (Boolean terminated) -> {},
                () -> false,
                testOdometry
        ));
    }
}
