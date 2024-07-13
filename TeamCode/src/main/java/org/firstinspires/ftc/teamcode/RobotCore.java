package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.AllianceSide;

/**
 * declares all the subsystems of a robot
 * */
public final class RobotCore {
    public final AllianceSide currentSide;
    /** create all the subsystem with the hardware map */
    public RobotCore(HardwareMap hardwareMap, AllianceSide side) {
        this.currentSide = side;

        /* here we creates all the subsystems */
    }
}
