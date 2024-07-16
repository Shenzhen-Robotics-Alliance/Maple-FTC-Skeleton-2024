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
        public static final double ROBOT_UPDATE_RATE_HZ = 50.0;
        public static final double ODOMETRY_UPDATE_RATE_HZ = 200.0;
        public static final double IMU_UPDATE_HZ = 5;
    }
    public static final class ChassisHardwareConfigs {
        public static final String
                CENTER_ODOMETER_WHEEL_NAME = "frontRight",
                LEFT_ODOMETER_WHEEL_NAME = "backLeft",
                RIGHT_ODOMETER_WHEEL_NAME = "backRight";

        /* if the RAW encoder reading is POSITIVE when moving to the left, false; other wise, true */
        public static final boolean CENTER_ODOMETER_WHEEL_INVERTED = true;
        /* if the RAW encoder reading is POSITIVE when moving front, false; other wise, true */
        public static final boolean LEFT_ODOMETER_WHEEL_INVERTED = true;
        public static final boolean RIGHT_ODOMETER_WHEEL_INVERTED = false;

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

        public static final double ODOMETER_WHEELS_TRACK_WIDTH_METERS = 0.257;
        public static final double ODOMETER_CENTER_WHEELS_OFFSET = -0.022;
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
