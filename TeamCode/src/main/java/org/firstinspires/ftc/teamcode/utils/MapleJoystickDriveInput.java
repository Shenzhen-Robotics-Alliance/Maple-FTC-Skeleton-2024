package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.constants.GamepadConfigs.DEAD_BAND_WHEN_OTHER_AXIS_EMPTY;
import static org.firstinspires.ftc.teamcode.constants.GamepadConfigs.DEAD_BAND_WHEN_OTHER_AXIS_FULL;
import static org.firstinspires.ftc.teamcode.constants.GamepadConfigs.ROTATION_SPEED_INPUT_EXPONENT;
import static org.firstinspires.ftc.teamcode.constants.GamepadConfigs.TRANSLATIONAL_SPEED_INPUT_EXPONENT;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.MapleMaths.MapleCommonMath;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class MapleJoystickDriveInput {
    private final DoubleSupplier joystickXSupplier, joystickYSupplier, joystickOmegaSupplier;

    /**
     * @param joystickXSupplier the supplier of the x-axis of the joystick, positive is RIGHTWARDS
     * @param joystickYSupplier the supplier of the x-axis of the joystick, positive is DOWNWARDS
     * @param joystickOmegaSupplier the supplier of the omega-axis of the joystick, positive is RIGHTWARDS
     * */
    public MapleJoystickDriveInput(DoubleSupplier joystickXSupplier, DoubleSupplier joystickYSupplier, DoubleSupplier joystickOmegaSupplier) {
        this.joystickXSupplier = joystickXSupplier;
        this.joystickYSupplier = joystickYSupplier;
        this.joystickOmegaSupplier = joystickOmegaSupplier;
    }

    /**
     * reads the joystick inputs and calculate the chassis speeds
     * @return the chassis speeds requested by the pilot, driver-station-centric
     * */
    public ChassisSpeeds getJoystickChassisSpeeds(double chassisMaxVelocityMetersPerSec, double maxAngularVelocityRadPerSec) {
        final Translation2d linearSpeedMetersPerSec = getTranslationalSpeedsFromJoystick(chassisMaxVelocityMetersPerSec);
        final double rotationSpeedRadPerSec = getRotationalSpeedFromJoystick(maxAngularVelocityRadPerSec);

        return new ChassisSpeeds(linearSpeedMetersPerSec.getX(), linearSpeedMetersPerSec.getY(), rotationSpeedRadPerSec);
    }

    /**
     * @return the translational speeds, in meters/second
     * */
    public Translation2d getTranslationalSpeedsFromJoystick(double chassisMaxVelocityMetersPerSec) {
        final double
                linearSpeedXComponentRaw = -joystickYSupplier.getAsDouble(),
                linearSpeedYComponentRaw = -joystickXSupplier.getAsDouble(),
                linearSpeedXComponentDeadBanded = applySmartDeadBand(linearSpeedXComponentRaw, linearSpeedYComponentRaw),
                linearSpeedYComponentDeadBanded = applySmartDeadBand(linearSpeedYComponentRaw, linearSpeedXComponentRaw);

        final Translation2d originalTranslationalSpeed = new Translation2d(linearSpeedXComponentDeadBanded, linearSpeedYComponentDeadBanded);
        final double translationalSpeedMagnitudeScaled = Math.pow(originalTranslationalSpeed.getNorm(), TRANSLATIONAL_SPEED_INPUT_EXPONENT);
        return new Translation2d(
                translationalSpeedMagnitudeScaled * chassisMaxVelocityMetersPerSec,
                originalTranslationalSpeed.getAngle()
        );
    }

    public double getRotationalSpeedFromJoystick(double maxAngularVelocityRadPerSec) {
        final double
                rotationSpeedRaw = -joystickOmegaSupplier.getAsDouble(),
                rotationalSpeedDeadBanded = applySmartDeadBand(rotationSpeedRaw, 0),
                rotationalSpeedScaledMagnitude = Math.abs(Math.pow(
                        rotationalSpeedDeadBanded, ROTATION_SPEED_INPUT_EXPONENT
                )) * maxAngularVelocityRadPerSec;
        return Math.copySign(rotationalSpeedScaledMagnitude, rotationSpeedRaw);
    }

    /**
     * apply  a smart dead-band to the given axis value
     * unlike normal dead-banding, the threshold of a smart deadband increases as the value of the other axis increases
     * this will make it easier for the pilot to set request a straight-line driving
     * @param axisValue the value of the axis of interest
     * @param otherAxisValue the value of the other axis on the stick
     * */
    private static double applySmartDeadBand(double axisValue, double otherAxisValue) {
        final double deadBand = MapleCommonMath.linearInterpretationWithBounding(
                0, DEAD_BAND_WHEN_OTHER_AXIS_EMPTY,
                1, DEAD_BAND_WHEN_OTHER_AXIS_FULL,
                Math.abs(otherAxisValue)
        );
        return MathUtil.applyDeadband(axisValue, deadBand, 1);
    }

    public static MapleJoystickDriveInput leftHandedJoystick(Gamepad driverGamePad) {
        return new MapleJoystickDriveInput(
                () -> driverGamePad.left_stick_x,
                () -> driverGamePad.left_stick_y,
                () -> driverGamePad.right_stick_x
        );
    }

    public static MapleJoystickDriveInput leftHandedJoystick(GamepadEx driverGamePad) {
        return new MapleJoystickDriveInput(
                driverGamePad::getLeftX,
                () -> -driverGamePad.getLeftY(), // GamepadEx has reversed y
                driverGamePad::getRightX
        );
    }

    public static MapleJoystickDriveInput rightHandedJoystick(Gamepad driverGamePad) {
        return new MapleJoystickDriveInput(
                () -> driverGamePad.right_stick_x,
                () -> driverGamePad.right_stick_y,
                () -> driverGamePad.left_stick_x
        );
    }

    public static MapleJoystickDriveInput rightHandedJoystick(GamepadEx driverGamePad) {
        return new MapleJoystickDriveInput(
                driverGamePad::getRightX,
                () -> -driverGamePad.getRightY(), // GamepadEx has reversed y
                driverGamePad::getLeftX
        );
    }
}
