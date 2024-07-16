package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.HolonomicDriveSubsystem;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;
import org.firstinspires.ftc.teamcode.Utils.MapleJoystickDriveInput;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

// TODO: test the current logic
//  then, add straight-line maintenance PID
public class GamePadDrive extends CommandBase {
    private final HolonomicDriveSubsystem driveSubsystem;
    private final MapleJoystickDriveInput driveInput;
    private final BooleanSupplier driveStationCentricModeSwitch;
    private final AllianceSide allianceSide;
    private ChassisSpeeds currentChassisSpeedsSetPointDriverStationCentric;

    public GamePadDrive(HolonomicDriveSubsystem driveSubsystem, MapleJoystickDriveInput driveInput, BooleanSupplier driverStationCentricModeSwitch, AllianceSide side) {
        super();
        super.setName("Joystick-Drive");
        super.addRequirements(driveSubsystem);

        this.driveSubsystem = driveSubsystem;
        this.driveInput = driveInput;
        this.driveStationCentricModeSwitch = driverStationCentricModeSwitch;
        this.allianceSide = side;
    }

    @Override
    public void initialize() {
        this.currentChassisSpeedsSetPointDriverStationCentric = new ChassisSpeeds();
    }

    @Override
    public void execute() {
        final ChassisSpeeds desiredDriveStationCentricSpeed = driveInput.getJoystickChassisSpeeds(
                driveSubsystem.getChassisMaxLinearVelocity(), driveSubsystem.getChassisMaxAngularVelocity()
        );
        Constants.telemetry.addData("Driver Station Requested Speed", desiredDriveStationCentricSpeed);
        this.currentChassisSpeedsSetPointDriverStationCentric = HolonomicDriveSubsystem.constrainAcceleration(
                currentChassisSpeedsSetPointDriverStationCentric,
                desiredDriveStationCentricSpeed,
                driveSubsystem.getChassisMaxLinearVelocity() / Constants.DriverConfigs.linearAccelerationSmoothOutTime,
                driveSubsystem.getChassisMaxAngularVelocity() / Constants.DriverConfigs.rotationalAccelerationSmoothOutTime,
                1/Constants.SystemConfigs.ROBOT_UPDATE_RATE_HZ
        );

        if (driveStationCentricModeSwitch.getAsBoolean() && allianceSide != AllianceSide.NOT_GIVEN)
            driveSubsystem.runDriverStationCentricChassisSpeeds(currentChassisSpeedsSetPointDriverStationCentric, allianceSide);
        else
            driveSubsystem.runRobotCentricChassisSpeeds(currentChassisSpeedsSetPointDriverStationCentric);
    }
}
