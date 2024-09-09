package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOLimeLight implements VisionIO {
    private final Limelight3A limelight;

    public VisionIOLimeLight(Limelight3A limelight) {
        this.limelight = limelight;
        limelight.start();
    }

    @Override
    public CameraResults getLatestResults() {
        LLResult result = limelight.getLatestResult();
        final List<AprilTagTarget> targets = new ArrayList<>();
        for (LLResultTypes.FiducialResult fiducialResult:result.getFiducialResults())
            targets.add(fromLLFiducialResult(fiducialResult));
        return new CameraResults(
                limelight.isConnected(),
                (result.getCaptureLatency() + result.getTargetingLatency()) / 1000.0,
                targets
        );
    }

    @Override
    public void close() throws IOException {
        limelight.stop();
    }

    private static AprilTagTarget fromLLFiducialResult(LLResultTypes.FiducialResult fiducialResult) {
        return new AprilTagTarget(
                fiducialResult.getFiducialId(),
                Transform3d.fromFTCPose3d(fiducialResult.getTargetPoseCameraSpace())
        );
    }
}
