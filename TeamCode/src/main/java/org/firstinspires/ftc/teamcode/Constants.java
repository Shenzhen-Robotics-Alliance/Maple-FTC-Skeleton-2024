package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.parameters.ImuParameters;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.EmptyTelemetry;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;

public class Constants {
    public static long matchStartTimeMillis = System.currentTimeMillis();
    public static Telemetry telemetry = new EmptyTelemetry();
    public static final class SystemConfigs {
        public static final double robotUpdateRateHZ = 60.0;
    }
    public static final class HardwareConfigs {
        public static final String
                HORIZONTAL_ODOMETER_WHEEL_NAME = "backLeft",
                VERTICAL_ODOMETER_WHEEL_1_NAME = "frontLeft",
                VERTICAL_ODOMETER_WHEEL_2_NAME = "frontRight";

        public static final DcMotorSimple.Direction
                frontLeftMotorDirection = DcMotorSimple.Direction.REVERSE,
                frontRightMotorDirection = DcMotorSimple.Direction.FORWARD,
                backLeftMotorDirection = DcMotorSimple.Direction.REVERSE,
                backRightMotorDirection = DcMotorSimple.Direction.FORWARD;

        public static final IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
    }

    public static final class ChassisConfigs {
        public static final double CHASSIS_WIDTH_METERS = 0.3;
        public static final double CHASSIS_LENGTH_METERS = 0.4;

        public static final MecanumDriveKinematics KINEMATICS = new MecanumDriveKinematics(
                new Translation2d(Constants.ChassisConfigs.CHASSIS_LENGTH_METERS /2, Constants.ChassisConfigs.CHASSIS_WIDTH_METERS / 2),
                new Translation2d(Constants.ChassisConfigs.CHASSIS_LENGTH_METERS /2, -Constants.ChassisConfigs.CHASSIS_WIDTH_METERS / 2),
                new Translation2d(-Constants.ChassisConfigs.CHASSIS_LENGTH_METERS /2, Constants.ChassisConfigs.CHASSIS_WIDTH_METERS / 2),
                new Translation2d(-Constants.ChassisConfigs.CHASSIS_LENGTH_METERS /2, -Constants.ChassisConfigs.CHASSIS_WIDTH_METERS / 2)
        );

        public static final double[] chassisMotorPower = new double[] {0, 1}, chassisSpeedsMetersPerSecond = new double[] {0, 2};

        public static final double maxVelocityMetersPerSecond = 2.0, maxAngularVelocityDegreesPerSecond = 360;
    }

    public static final class DriverConfigs {
        public static final double translationalSpeedInputExponent = 1.6;
        public static final double rotationSpeedInputExponent = 2;
        public static final double deadBandWhenOtherAxisEmpty = 0.05;
        public static final double deadBandWhenOtherAxisFull = 0.2;

        public static final double linearAccelerationSmoothOutTime = 0.2;
        public static final double rotationalAccelerationSmoothOutTime = 0.2;
    }
}
