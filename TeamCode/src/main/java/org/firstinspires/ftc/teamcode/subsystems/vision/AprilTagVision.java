package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.constants.SystemConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.HolonomicDriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.MapleTime;

import java.io.Closeable;
import java.io.IOException;
import java.util.List;

public class AprilTagVision implements Subsystem, Closeable {
    private final VisionIO io;
    private final HolonomicDriveSubsystem driveSubsystem;
    public AprilTagVision(VisionIO io, HolonomicDriveSubsystem driveSubsystem) {
        this.io = io;
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void periodic() {
        final VisionIO.CameraResults latestResult = io.getLatestResults();
        if (!latestResult.cameraConnected) {
            SystemConstants.telemetry.addLine("WARNING!!! april tag camera not connected");
            return;
        }

        final List<VisionIO.AprilTagTarget> targets = latestResult.visibleTargets;
        final double captureTimeStamp = MapleTime.getMatchTimeSeconds() - latestResult.latencySeconds;

        // TODO: using filtering mechanism, estimate the robot pose and feed it to
        //  driveSubsystem.addVisionMeasurement
    }

    @Override
    public void close() {
        try {
            io.close();
        } catch (IOException ignored) {}
    }
}
