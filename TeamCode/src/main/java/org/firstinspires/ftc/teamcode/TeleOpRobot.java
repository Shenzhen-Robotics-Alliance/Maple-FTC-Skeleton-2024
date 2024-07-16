package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Commands.Drive.GamePadDrive;
import org.firstinspires.ftc.teamcode.Utils.MapleJoystickDriveInput;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * robot during teleop stage'
 * here we bind all the commands to a key on the GamePad
 * */
public class TeleOpRobot extends Robot {
    private final RobotCore robotCore;
    private final GamepadEx pilotGamePad, copilotGamePad;
    public TeleOpRobot(RobotCore robotCore, Gamepad pilotGamePad, Gamepad copilotGamePad) {
        super();
        this.robotCore = robotCore;
        this.pilotGamePad = new GamepadEx(pilotGamePad);
        this.copilotGamePad = new GamepadEx(copilotGamePad);

        configureKeyBindings();
    }

    private void configureKeyBindings() {
        robotCore.driveSubsystem.setDefaultCommand(new GamePadDrive(
                robotCore.driveSubsystem,
                MapleJoystickDriveInput.leftHandedJoystick(pilotGamePad),
                () -> true,
                robotCore.currentSide
        ));

        this.pilotGamePad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                () -> robotCore.driveSubsystem.setPose(new Pose2d())
        );
    }

    @Override
    public void reset() {
        super.reset();
    }
}
