package org.firstinspires.ftc.teamcode.Autos;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;

import org.firstinspires.ftc.teamcode.RobotCore;

public class ExampleAuto implements Auto {
    @Override
    public Command getAutonomousCommands(RobotCore robotCore) {
        return new RunCommand(
                () -> System.out.println("Example Running on " + robotCore.currentSide + " Side...")
        ).beforeStarting(
                () -> System.out.println("Starting Example...")
        );
    }
}
