package org.firstinspires.ftc.teamcode.autos;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;

import org.firstinspires.ftc.teamcode.RobotContainer;

public class ExampleAuto implements Auto {
    @Override
    public Command getAutonomousCommands(RobotContainer robotContainer) {
        return new RunCommand(
                () -> System.out.println("Example Running on " + robotContainer.currentSide + " Side...")
        ).beforeStarting(
                () -> System.out.println("Starting Example...")
        );
    }
}
