package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * defines an autonomous stage
 * it returns a sequential command group, which is scheduled by AutonomousRobot during auto stage
 * */
public interface Auto {
    Command getAutonomousCommands(RobotContainer robotContainer);

    default void beforeStartPeriodic() {}
}
