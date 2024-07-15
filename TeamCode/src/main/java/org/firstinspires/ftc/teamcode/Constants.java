package org.firstinspires.ftc.teamcode;

public class Constants {
    public static long matchStartTimeMillis = System.currentTimeMillis();
    public static final class SystemConfigs {
        public static final double robotUpdateRateHZ = 60.0;
    }
    public static final class HardwareNames {

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
