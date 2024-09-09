package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.io.Closeable;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public interface VisionIO extends Closeable {
    final class AprilTagTarget {
        private final int id;
        private final Transform3d bestCameraToTarget;

        public AprilTagTarget(int id, Transform3d bestCameraToTarget) {
            this.id = id;
            this.bestCameraToTarget = bestCameraToTarget;
        }
    }

    final class CameraResults {
        public final boolean cameraConnected;
        public final double latencySeconds;
        public final List<AprilTagTarget> visibleTargets;

        public CameraResults(boolean cameraConnected, double latencySeconds, List<AprilTagTarget> visibleTargets) {
            this.cameraConnected = cameraConnected;
            this.latencySeconds = latencySeconds;
            this.visibleTargets = visibleTargets;
        }
    }

    /**
     * poll the camera inputs
     * this method is called 50 times a second and should not be thread-blocking
     * @return the latest results from camera
     * */
    CameraResults getLatestResults();
}
