package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.hardware.Gamepad;

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

        // TODO: here, register all the subsystems
    }
    public void configureKeyBindings() {
        // TODO: configure the bindings of commands
    }

    @Override
    public void reset() {
        super.reset();
    }
}
