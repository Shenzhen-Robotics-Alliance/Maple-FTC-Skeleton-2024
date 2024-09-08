package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.drive.GamePadDrive;
import org.firstinspires.ftc.teamcode.utils.MapleJoystickDriveInput;

import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * robot during teleop stage'
 * here we bind all the commands to a key on the GamePad
 * */
public class TeleOpRobot extends Robot {
    private final RobotContainer robotContainer;
    private final GamepadEx pilotGamePad, copilotGamePad;
    public TeleOpRobot(RobotContainer robotContainer, Gamepad pilotGamePad, Gamepad copilotGamePad) {
        super();
        this.robotContainer = robotContainer;
        this.pilotGamePad = new GamepadEx(pilotGamePad);
        this.copilotGamePad = new GamepadEx(copilotGamePad);

        configureKeyBindings();
    }

    private void configureKeyBindings() {
        robotContainer.driveSubsystem.setDefaultCommand(new GamePadDrive(
                robotContainer.driveSubsystem,
                MapleJoystickDriveInput.leftHandedJoystick(pilotGamePad),
                () -> true,
                robotContainer.currentSide
        ));

        this.pilotGamePad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                () -> robotContainer.driveSubsystem.setPose(new Pose2d())
        );
    }

    @Override
    public void reset() {
        super.reset();
        try {
            robotContainer.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
