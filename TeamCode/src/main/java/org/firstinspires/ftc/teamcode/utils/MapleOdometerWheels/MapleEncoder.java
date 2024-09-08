// By 5516 Iron Maple, original: https://github.com/Shenzhen-Robotics-Alliance/Maple-FTC-Skeleton-2024
// Under MIT License

package org.firstinspires.ftc.teamcode.utils.MapleOdometerWheels;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.constants.SystemConstants;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;

/**
 *
 * */
public class MapleEncoder {
    private final DcMotor encoderInstance;
    private final int encoderReadingRatio;
    private final double wheelRadius, gearRatio, ticksPerRevolution, periodSeconds;
    private final LinearFilter filter;

    private double calibratedPositionTicks, positionTicks, deltaTicks, filteredVelocityTicksPerSecond;

    public MapleEncoder(DcMotor encoderInstance, boolean inverted, double ticksPerRevolution) {
        this(encoderInstance, inverted, ticksPerRevolution, 1);
    }
    public MapleEncoder(DcMotor encoderInstance, boolean inverted, double ticksPerRevolution, double gearRatio) {
        this(encoderInstance, inverted, ticksPerRevolution, gearRatio, 0);
    }
    public MapleEncoder(DcMotor encoderInstance, boolean inverted, double ticksPerRevolution, double gearRatio, double wheelRadius) {
        this(encoderInstance, inverted, ticksPerRevolution, gearRatio, wheelRadius, SystemConstants.ROBOT_UPDATE_RATE_HZ);
    }

    public MapleEncoder(DcMotor encoderInstance, boolean inverted, double ticksPerRevolution, double gearRatio, double wheelRadius, double updateRateHZ) {
        this(
                encoderInstance, inverted, ticksPerRevolution, gearRatio, wheelRadius, updateRateHZ,
                // LinearFilter.movingAverage(10)
                LinearFilter.singlePoleIIR((1.0/updateRateHZ) * 10, 1.0/updateRateHZ)
        );
    }
    public MapleEncoder(DcMotor encoderInstance, boolean inverted, double ticksPerRevolution, double gearRatio, double wheelRadius, double updateRateHZ, LinearFilter filter) {
        this.encoderInstance = encoderInstance;
        this.encoderReadingRatio = inverted ? -1 : 1;
        this.ticksPerRevolution = ticksPerRevolution;
        this.gearRatio = gearRatio;
        this.wheelRadius = wheelRadius;
        this.periodSeconds = 1.0/updateRateHZ;

        this.filter = filter;

        this.calibratedPositionTicks = this.positionTicks = getCurrentPositionTicks();
        this.deltaTicks = this.filteredVelocityTicksPerSecond = 0;
    }

    private int getCurrentPositionTicks() {
        return encoderInstance.getCurrentPosition() * encoderReadingRatio;
    }

    public void calibrate() {
        this.calibratedPositionTicks = getCurrentPositionTicks();
    }

    private double unfilteredVelocityTicksPerSecond = 0;
    public void poll() {
        final int latestTicks = getCurrentPositionTicks();
        this.deltaTicks = latestTicks - this.positionTicks;
        this.positionTicks = latestTicks;
        this.unfilteredVelocityTicksPerSecond = deltaTicks / periodSeconds;
        this.filteredVelocityTicksPerSecond = filter.calculate(this.unfilteredVelocityTicksPerSecond);
    }

    public double getRevolutionsSinceCalibration() {
        return (this.positionTicks - this.calibratedPositionTicks) / ticksPerRevolution / gearRatio;
    }

    public double getVelocityRevolutionsPerSecond() {
        return filteredVelocityTicksPerSecond / ticksPerRevolution / gearRatio;
    }

    /**
     * gets the encoder
     * @return the unfiltered encoder velocity, in rev/sec
     * */
    @Deprecated
    public double getUnfilteredVelocityRevolutionsPerSecond() {
        return unfilteredVelocityTicksPerSecond / ticksPerRevolution / gearRatio;
    }

    public double getDistanceMeters() {
        return Units.rotationsToRadians(getRevolutionsSinceCalibration()) * wheelRadius;
    }

    public double getVelocityMetersPerSecond() {
        return Units.rotationsToRadians(getVelocityRevolutionsPerSecond()) * wheelRadius;
    }
}
