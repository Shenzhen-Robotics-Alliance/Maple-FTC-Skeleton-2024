package org.firstinspires.ftc.teamcode.subsystems.vision;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOFTCVisionPortal implements VisionIO {
    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTagProcessor;

    public VisionIOFTCVisionPortal(WebcamName webcamName) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setNumThreads(4)
                .setLensIntrinsics(514.826, 514.826, 311.864, 176.19)
                .setOutputUnits(DistanceUnit.METER, AngleUnit.RADIANS)
                .setDrawCubeProjection(true)
                .build();

        aprilTagProcessor.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcamName)
                .setCameraResolution(new Size(640, 400))
                .addProcessor(aprilTagProcessor)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        FtcDashboard.getInstance().startCameraStream(visionPortal, 5);
    }

    private boolean setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return false;

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)
            return false;

        // Set camera controls unless we are stopping.
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual)
            exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);

        return true;
    }

    private boolean cameraConfigured = false;
    @Override
    public CameraResults getLatestResults() {
        cameraConfigured |=
                setManualExposure(3, 250);
        if (!cameraConfigured)
            return new CameraResults(false, 0, new ArrayList<>());

        final List<AprilTagTarget> visibleTargets = new ArrayList<>();
        for (AprilTagDetection detection:aprilTagProcessor.getDetections())
            visibleTargets.add(fromOpenCVAprilTagDetection(detection));

        final double delaySeconds = 1.0 / visionPortal.getFps() // frame time
                        + 20.0/1000.0; // pipeline latency
        return new CameraResults(
                cameraConfigured,
                delaySeconds,
                visibleTargets
        );
    }

    @Override
    public void close() throws IOException {
        visionPortal.close();
    }

    private static AprilTagTarget fromOpenCVAprilTagDetection(AprilTagDetection detection) {
        final Transform3d cameraToTarget = new Transform3d(
                detection.ftcPose.x,
                detection.ftcPose.y,
                detection.ftcPose.z,
                new Rotation3d(detection.ftcPose.roll, detection.ftcPose.pitch, detection.ftcPose.yaw)
        );
        return new AprilTagTarget(
                detection.id,
                cameraToTarget
        );
    }
}
