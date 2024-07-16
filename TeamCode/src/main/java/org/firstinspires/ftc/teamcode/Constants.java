package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.EmptyTelemetry;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;

public class Constants {
    public static long matchStartTimeMillis = System.currentTimeMillis();
    public static Telemetry telemetry = new EmptyTelemetry();
    public static final class SystemConfigs {
        public static final double robotUpdateRateHZ = 50.0;
        public static final double odometryUpdateRateHZ = 200.0;
    }
    public static final class ChassisHardwareConfigs {
        public static final String
                CENTER_ODOMETER_WHEEL_NAME = "frontRight",
                LEFT_ODOMETER_WHEEL_NAME = "backLeft",
                RIGHT_ODOMETER_WHEEL_NAME = "backRight";

        public static final double
                frontLeftMotorDirection = -1,
                frontRightMotorDirection = 1,
                backLeftMotorDirection = -1,
                backRightMotorDirection = 1;

        public static final IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));

        public static final double ODOMETER_ENCODER_TICKS_PER_REVOLUTION = 2048;
        public static final double ODOMETER_WHEELS_RADIUS = 48.0/2/1000;
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
