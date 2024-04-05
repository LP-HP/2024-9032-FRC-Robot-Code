package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.Swerve;
import frc.robot.util.VisionTargetCache;
import frc.robot.subsystems.LimelightVision.AprilTagTarget;

import static frc.robot.Constants.ClosedLoopConstants.*;

public class DriveToStage extends Command {
    private final Swerve swerve;    
    private final LimelightVision limelight;

    private final PIDController swerveRotController;
    private final PIDController swerveDistanceController;
    private final PIDController swerveYController;

    private final VisionTargetCache<AprilTagTarget> visionCache;

    public DriveToStage(Swerve swerve, LimelightVision limelight) {
        this.swerve = swerve;
        this.limelight = limelight;

        swerveRotController = new PIDController(kPStageRotation, 0.0, kDStageRotation);
        swerveRotController.setSetpoint(0.0);
        swerveRotController.setTolerance(stageRotationTolerance);

        swerveDistanceController = new PIDController(kPStageDistance, 0.0, kDStageDistance);
        swerveDistanceController.setSetpoint(stageDistanceSetpoint);
        swerveDistanceController.setTolerance(stageDistanceTolerance);

        swerveYController = new PIDController(kPStageY, 0.0, kDStageY);
        swerveYController.setSetpoint(0.0);
        swerveYController.setTolerance(stageYTolerance);

        visionCache = new VisionTargetCache<>(cycleAmtSinceAprilTagSeenCutoff);

        addRequirements(swerve, limelight);
    }

    @Override
    public void execute() {
        AprilTagTarget target = limelight.getAprilTagTarget();

        if(target.isValidStageTag()) {
            visionCache.updateTarget(target);
        }

        if(visionCache.targetNotExpired()) {
            target = visionCache.getAndIncrement();

            swerve.driveOpenLoop(
                new Translation2d(swerveDistanceController.calculate(target.distance), -swerveYController.calculate(target.xOffset)),
                -swerveRotController.calculate(target.skew), 
                false
            );
        }

        else {
            reset();

            System.err.println("Lost stage tag while driving!");
        }
    }

    @Override
    public boolean isFinished() {
        return swerveDistanceController.atSetpoint() && swerveYController.atSetpoint() && swerveRotController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        reset();
    }

    private void reset() {
        swerveRotController.reset();
        swerveDistanceController.reset();
        swerveYController.reset();

        swerve.driveOpenLoop(new Translation2d(), 0.0, false);

        visionCache.reset();
    }
}