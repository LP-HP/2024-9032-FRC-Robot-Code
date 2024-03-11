package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeCameraConstants.*;

public class Photonvision extends SubsystemBase {
    private final ShuffleboardTab photonvisionTab = Shuffleboard.getTab("PhotonVision");

    private final PhotonCamera camera = new PhotonCamera(cameraName);

    private double distance;
    private double xOffset;

    public Photonvision () {
        try {
            photonvisionTab.addCamera("PhotonVision View", cameraName, "camera_server://" + cameraName)
                .withPosition(0, 0).withSize(5, 5);
        } catch (Exception e) {
            System.err.println("PhotonVision view already added!");
        }

        photonvisionTab.addDouble("Distance", () -> distance)
            .withPosition(0, 6).withSize(1, 1);
        photonvisionTab.addDouble("X Offset", () -> xOffset)
            .withPosition(1, 6).withSize(1, 1);
    }

    @Override
    public void periodic() {
        if (camera.getLatestResult().hasTargets()) {
            PhotonTrackedTarget result = camera.getLatestResult().getBestTarget();

            xOffset = result.getYaw();

            /* Calculate distance */
            distance = PhotonUtils.calculateDistanceToTargetMeters(
                    cameraHeight,
                    targetHeight,
                    mountingAngle,
                    Units.degreesToRadians(result.getPitch())
            );
        }
    }
     
}