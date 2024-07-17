package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Robot;

import org.firstinspires.ftc.teamcode.Autos.Auto;
import org.firstinspires.ftc.teamcode.Utils.MapleLoopClock;

/**
 * robot during autonomous period
 * here we schedule the autonomous commands
 * */
public class AutonomousRobot extends Robot {
    private static final MapleLoopClock beforeStartPeriodicClock = new MapleLoopClock(24);
    private final Auto auto;
    private final RobotCore robotCore;
    public AutonomousRobot(RobotCore robotCore, Auto auto) {
        super();
        this.auto = auto;
        this.robotCore = robotCore;
    }

    public void beforeStartPeriodic() {
        auto.beforeStartPeriodic();
        beforeStartPeriodicClock.tick();
    }

    public void startAuto() {
        super.schedule(auto.getAutonomousCommands(robotCore));
    }
}
