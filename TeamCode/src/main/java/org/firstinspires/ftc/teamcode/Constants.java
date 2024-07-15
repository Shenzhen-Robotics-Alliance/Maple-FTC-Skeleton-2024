package org.firstinspires.ftc.teamcode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;

public class Constants {
    public static long matchStartTimeMillis = System.currentTimeMillis();
    public static final class SystemConfigs {
        public static final double robotUpdateRateHZ = 60.0;
    }
    public static final class HardwareNames {

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
