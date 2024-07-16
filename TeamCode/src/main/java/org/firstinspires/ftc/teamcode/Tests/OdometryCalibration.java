package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utils.MapleLoopClock;
import org.firstinspires.ftc.teamcode.Utils.MapleOdometerWheels.MapleEncoder;

import static org.firstinspires.ftc.teamcode.Constants.ChassisHardwareConfigs.*;

import edu.wpi.first.math.geometry.Rotation2d;

public class OdometryCalibration implements SimpleUnitTest {
    private final Telemetry telemetry;
    private final MapleLoopClock clock = new MapleLoopClock(50.0);
    private final MapleEncoder leftOdometerWheel, rightOdometerWheel, centerOdometerWheel;
    private final DcMotor frontLeft, frontRight, backLeft, backRight;
    private final Gamepad gamepad;
    private double radiansRotated, previousRotationRadians;
    private final IMU imu;
    public OdometryCalibration(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad) {
        this.telemetry = telemetry;

        this.leftOdometerWheel = new MapleEncoder(
                hardwareMap.get(DcMotor.class, LEFT_ODOMETER_WHEEL_NAME),
                LEFT_ODOMETER_WHEEL_INVERTED,
                ODOMETER_ENCODER_TICKS_PER_REVOLUTION,
                1,
                ODOMETER_WHEELS_RADIUS
        );

        this.rightOdometerWheel = new MapleEncoder(
                hardwareMap.get(DcMotor.class, RIGHT_ODOMETER_WHEEL_NAME),
                RIGHT_ODOMETER_WHEEL_INVERTED,
                ODOMETER_ENCODER_TICKS_PER_REVOLUTION,
                1,
                ODOMETER_WHEELS_RADIUS
        );

        this.centerOdometerWheel = new MapleEncoder(
                hardwareMap.get(DcMotor.class, CENTER_ODOMETER_WHEEL_NAME),
                CENTER_ODOMETER_WHEEL_INVERTED,
                ODOMETER_ENCODER_TICKS_PER_REVOLUTION,
                1,
                ODOMETER_WHEELS_RADIUS
        );

        this.frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        this.frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        this.backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        this.backRight = hardwareMap.get(DcMotor.class, "backRight");

        this.imu = hardwareMap.get(IMU.class, "imu");

        this.gamepad = gamepad;

    }
    @Override
    public void testStart() {
        radiansRotated = 0;
        previousRotationRadians = getIMUReading();
    }

    private double getIMUReading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public void testPeriodic() {
        leftOdometerWheel.poll();
        rightOdometerWheel.poll();
        centerOdometerWheel.poll();

        telemetry.addLine("rotate the robot with button A for 10 revolutions on field (3600 degrees) and press button x");
        telemetry.addData("left rev", leftOdometerWheel.getDistanceMeters());
        telemetry.addData("right rev", rightOdometerWheel.getDistanceMeters());
        telemetry.addData("center rev", centerOdometerWheel.getDistanceMeters());

        final double rotaryPower = gamepad.a ? 0.3: -0;
        frontLeft.setPower(-rotaryPower * frontLeftMotorDirection);
        backLeft.setPower(-rotaryPower * backLeftMotorDirection);
        frontRight.setPower(rotaryPower * frontRightMotorDirection);
        backRight.setPower(rotaryPower * backRightMotorDirection);

        final double imuReading = getIMUReading();
        radiansRotated += Rotation2d.fromRadians(imuReading).minus(Rotation2d.fromRadians(previousRotationRadians)).getRadians();
        previousRotationRadians = imuReading;

        if (radiansRotated != 0) {
            final double trackWidth = (rightOdometerWheel.getDistanceMeters() - leftOdometerWheel.getDistanceMeters()) / radiansRotated,
                    centerWheelOffset = centerOdometerWheel.getDistanceMeters() / radiansRotated;
            telemetry.addData("track width (meters)", trackWidth); // 0.257
            telemetry.addData("center wheel offset (meters)", centerWheelOffset); // 0.020
        }

        telemetry.update();
        clock.tick();
    }
}
