package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.PhotonvisionConstants.*;

public class Photonvision extends SubsystemBase {
    private final ShuffleboardTab photonvisionTab = Shuffleboard.getTab("Photonvision");

    private final PhotonCamera camera = new PhotonCamera(cameraName);

    private double distance;
    private double xOffset;
    private boolean hasTargets = false;

    public Photonvision () {
        photonvisionTab.addDouble("Distance", () -> distance)
            .withPosition(0, 0).withSize(2, 1);
        photonvisionTab.addDouble("X Offset", () -> xOffset)
            .withPosition(2, 0).withSize(2, 1);
    }

    public void addCameraToTab(ShuffleboardTab tab, int col, int row, int size) {
        try {
            tab.addCamera("PhotonVision View", cameraName, "http://photonvision.local:1182/stream.mjpg")
                .withPosition(col, row).withSize(size, size);
        } catch (Exception e) {
            System.err.println("PhotonVision view already added!");
        }
    }

    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            hasTargets = true;

            xOffset = result.getBestTarget().getYaw();

            /* Calculate distance */
            distance = PhotonUtils.calculateDistanceToTargetMeters(//TODO fix distance readings
                    cameraHeight,
                    targetHeight,
                    mountingAngle,
                    Units.degreesToRadians(result.getBestTarget().getPitch())
            );
        }

        else 
            hasTargets = false;
    }
    
    public double getLatestDistance() {
        return distance;
    }

    public double getLatestXOffset() {
        return xOffset;
    }

    public boolean hasTargets() {
        return hasTargets;
    }
}