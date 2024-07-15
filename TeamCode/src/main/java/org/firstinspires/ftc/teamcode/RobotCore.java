package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Drive.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;

/**
 * declares all the subsystems of a robot
 * */
public final class RobotCore {
    public final AllianceSide currentSide;

    public final MecanumDriveSubsystem driveSubsystem;
    /** create all the subsystem with the hardware map */
    public RobotCore(HardwareMap hardwareMap, AllianceSide side) {
        this.currentSide = side;

        /* here we creates all the subsystems */
        final DcMotor
                frontLeft = hardwareMap.get(DcMotor.class, "frontLeft"),
                frontRight = hardwareMap.get(DcMotor.class, "frontRight"),
                backLeft = hardwareMap.get(DcMotor.class, "backLeft"),
                backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft.setDirection(Constants.HardwareConfigs.frontLeftMotorDirection);
        frontRight.setDirection(Constants.HardwareConfigs.frontRightMotorDirection);
        backLeft.setDirection(Constants.HardwareConfigs.backLeftMotorDirection);
        backRight.setDirection(Constants.HardwareConfigs.backRightMotorDirection);
        this.driveSubsystem = new MecanumDriveSubsystem(
                frontLeft, frontRight, backLeft, backRight
        );
    }
}
