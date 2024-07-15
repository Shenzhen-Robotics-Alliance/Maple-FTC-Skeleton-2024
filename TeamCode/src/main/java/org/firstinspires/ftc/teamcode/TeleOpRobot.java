package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Commands.Drive.GamePadDrive;
import org.firstinspires.ftc.teamcode.Utils.MapleJoystickDriveInput;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

/**
 * robot during teleop stage'
 * here we bind all the commands to a key on the GamePad
 * */
public class TeleOpRobot extends Robot {
    private final RobotCore robotCore;
    private final Gamepad pilotGamePad, copilotGamePad;
    public TeleOpRobot(RobotCore robotCore, Gamepad pilotGamePad, Gamepad copilotGamePad) {
        super();
        this.robotCore = robotCore;
        this.pilotGamePad = pilotGamePad;
        this.copilotGamePad = copilotGamePad;

        registerSubsystems();

        configureKeyBindings();
    }


    private void registerSubsystems() {
        register(robotCore.driveSubsystem);
    }

    private void configureKeyBindings() {
        robotCore.driveSubsystem.setDefaultCommand(new GamePadDrive(
                robotCore.driveSubsystem,
                MapleJoystickDriveInput.leftHandedJoystick(pilotGamePad),
                () -> false,
                robotCore.currentSide
        ));
    }

    @Override
    public void reset() {
        super.reset();
    }
}
