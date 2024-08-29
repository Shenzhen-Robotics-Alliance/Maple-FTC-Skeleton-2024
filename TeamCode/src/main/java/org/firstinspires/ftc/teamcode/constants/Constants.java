package org.firstinspires.ftc.teamcode.constants;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.EmptyTelemetry;


public class Constants {
    public static long matchStartTimeMillis = System.currentTimeMillis();
    public static Telemetry telemetry = new EmptyTelemetry();
    public static final class SystemConfigs {
        public static final double ROBOT_UPDATE_RATE_HZ = 50.0;
        public static final double ODOMETRY_UPDATE_RATE_HZ = 200.0;
        public static final double IMU_UPDATE_HZ = 5;
    }

    public static final class DriverConfigs {
        public static final double TRANSLATIONAL_SPEED_INPUT_EXPONENT = 1.6;
        public static final double ROTATION_SPEED_INPUT_EXPONENT = 2;
        public static final double DEAD_BAND_WHEN_OTHER_AXIS_EMPTY = 0.05;
        public static final double DEAD_BAND_WHEN_OTHER_AXIS_FULL = 0.2;

        public static final double LINEAR_ACCELERATION_SMOOTH_OUT_TIME = 0.2;
        public static final double ROTATIONAL_ACCELERATION_SMOOTH_OUT_TIME = 0.2;
    }
}
