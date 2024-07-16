// By 5516 Iron Maple, original: https://github.com/Shenzhen-Robotics-Alliance/Maple-FTC-Skeleton-2024
// Under MIT License

package org.firstinspires.ftc.teamcode.Utils.MapleOdometerWheels;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;

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
        this(encoderInstance, inverted, ticksPerRevolution, gearRatio, wheelRadius, Constants.SystemConfigs.robotUpdateRateHZ);
    }
    public MapleEncoder(DcMotor encoderInstance, boolean inverted, double ticksPerRevolution, double gearRatio, double wheelRadius, double updateRateHZ) {
        this.encoderInstance = encoderInstance;
        this.encoderReadingRatio = inverted ? -1 : 1;
        this.ticksPerRevolution = ticksPerRevolution;
        this.gearRatio = gearRatio;
        this.wheelRadius = wheelRadius;
        this.periodSeconds = 1.0/updateRateHZ;
        this.filter = LinearFilter.singlePoleIIR(periodSeconds * 5, periodSeconds);

        this.calibratedPositionTicks = this.positionTicks = getCurrentPositionTicks();
        this.deltaTicks = this.filteredVelocityTicksPerSecond = 0;
    }

    private int getCurrentPositionTicks() {
        return encoderInstance.getCurrentPosition() * encoderReadingRatio;
    }

    public void calibrate() {
        this.calibratedPositionTicks = getCurrentPositionTicks();
    }

    public void poll() {
        final int latestTicks = getCurrentPositionTicks();
        this.deltaTicks = latestTicks - this.positionTicks;
        this.positionTicks = latestTicks;
        final double ticksPerSecondUnFiltered = deltaTicks / periodSeconds;
        this.filteredVelocityTicksPerSecond = filter.calculate(ticksPerSecondUnFiltered);
    }

    public double getRevolutionsSinceCalibration() {
        return (this.positionTicks - this.calibratedPositionTicks) / ticksPerRevolution / gearRatio;
    }

    public double getVelocityRevolutionsPerSecond() {
        return filteredVelocityTicksPerSecond / ticksPerRevolution / gearRatio;
    }

    public double getDistanceMeters() {
        return Units.rotationsToRadians(getRevolutionsSinceCalibration()) * wheelRadius;
    }

    public double getVelocityMetersPerSecond() {
        return Units.rotationsToRadians(getVelocityRevolutionsPerSecond()) * wheelRadius;
    }
}
