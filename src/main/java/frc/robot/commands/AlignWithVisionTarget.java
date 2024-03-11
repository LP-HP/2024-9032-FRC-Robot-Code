package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.ClosedLoopConstants.*;

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

        swerveRotController = new PIDController(kPRotationTarget, 0.0, kDRotationTarget);
        swerveRotController.setTolerance(rotationSetpointTolerance);
        swerveRotController.setSetpoint(0.0);
        
        swerveTranslationController = new PIDController(kPTranslationTarget, 0, 0.0); 
        swerveTranslationController.setTolerance(translationSetpointTolerance);
        swerveTranslationController.setSetpoint(0.0);

        addRequirements(swerve, limelight);
    }

    @Override
    public void execute() {
        if(limelight.getAprilTagTarget().isValid) {
            if (rotateOnly) {
                swerve.driveOpenLoop(//TODO use closed loop
                    new Translation2d(),
                    swerveRotController.calculate(limelight.getAprilTagTarget().xOffset), false);
            }

            else {
                swerve.driveClosedLoop(
                    new Translation2d(swerveTranslationController.calculate(limelight.getAprilTagTarget().distance), 0.0),
                    swerveRotController.calculate(limelight.getAprilTagTarget().xOffset));
            }
        }

        else {
            swerveRotController.reset();
            swerve.driveOpenLoop(new Translation2d(), 0.0, false);
        }
    }

    @Override
    public boolean isFinished() {
        if(rotateOnly) 
            return endAtTarget ? swerveRotController.atSetpoint() : false;

        else 
            return endAtTarget ? swerveRotController.atSetpoint() && swerveTranslationController.atSetpoint() : false;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.driveOpenLoop(new Translation2d(), 0.0, false);
        swerveRotController.reset();
    }
}