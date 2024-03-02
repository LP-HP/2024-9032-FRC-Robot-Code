package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClosedLoopConstants;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.Swerve;

public class AlignWithVisionTarget extends Command {
    private final Swerve swerve;
    private final LimelightVision limelight;

    private final PIDController swerveRotController;
    private final PIDController swerveTranslationController;

    private final boolean endAtTarget;
    private final boolean rotateOnly;

    public AlignWithVisionTarget(Swerve swerve, LimelightVision limelight, boolean rotateOnly, boolean endAtTarget) {
        this.swerve = swerve;       
        this.limelight = limelight;
        this.endAtTarget = endAtTarget;
        this.rotateOnly = rotateOnly;

        swerveRotController = new PIDController(ClosedLoopConstants.kPRotationTarget, 0, 0.0);
        swerveRotController.setTolerance(ClosedLoopConstants.rotationSetpointTolerance);
        swerveTranslationController = new PIDController(ClosedLoopConstants.kPTranslationTarget, 0, 0.0); 
        swerveTranslationController.setTolerance(ClosedLoopConstants.translationSetpointTolerance);

        addRequirements(swerve, limelight);
    }

    @Override
    public void execute() {
        if (rotateOnly) {
            swerve.driveClosedLoop(
                new Translation2d(),
                swerveRotController.calculate(limelight.getAprilTagTarget().xOffset));
        }

        else {
            swerve.driveClosedLoop(
                new Translation2d(swerveTranslationController.calculate(limelight.getAprilTagTarget().yOffset), 0.0),
                swerveRotController.calculate(limelight.getAprilTagTarget().xOffset));
        }
    }

    @Override
    public boolean isFinished() {
        if(rotateOnly)
            return endAtTarget ? swerveRotController.atSetpoint() : false;

        else 
            return endAtTarget ? swerveRotController.atSetpoint() && swerveTranslationController.atSetpoint() : false;
    }
}