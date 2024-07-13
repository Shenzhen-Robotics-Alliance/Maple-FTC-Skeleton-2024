package org.firstinspires.ftc.teamcode.Autos;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.RobotCore;

/**
 * defines an autonomous stage
 * it returns a sequential command group, which is scheduled by AutonomousRobot during auto stage
 * */
public interface Auto {
    SequentialCommandGroup getAutonomousCommands(RobotCore robotCore);
}
