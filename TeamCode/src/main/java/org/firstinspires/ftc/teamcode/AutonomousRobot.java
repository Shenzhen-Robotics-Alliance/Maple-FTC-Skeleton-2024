package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Robot;

import org.firstinspires.ftc.teamcode.Autos.Auto;
import org.firstinspires.ftc.teamcode.Utils.MapleLoopClock;

import java.util.function.BooleanSupplier;

/**
 * robot during autonomous period
 * here we schedule the autonomous commands
 * */
public class AutonomousRobot extends Robot {
    public AutonomousRobot(RobotCore robotCore, Auto auto) {
        super();

        super.schedule(auto.getAutonomousCommands(robotCore));
    }
}
