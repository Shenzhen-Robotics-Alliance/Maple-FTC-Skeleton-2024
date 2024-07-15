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
    private final MapleLoopClock loopClock = new MapleLoopClock(Constants.SystemConfigs.robotUpdateRateHZ);

    public AutonomousRobot(RobotCore robotCore, Auto auto) {
        super();
        // TODO: register subsystems from robot core
        super.schedule(auto.getAutonomousCommands(robotCore));
    }

    @Override
    public void run() {
        super.run();
        loopClock.tick();
    }
}
