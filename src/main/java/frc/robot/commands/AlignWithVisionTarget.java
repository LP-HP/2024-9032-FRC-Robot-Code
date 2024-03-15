package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Photonvision;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.ClosedLoopConstants.*;

public class AlignWithVisionTarget extends Command {
    private final Swerve swerve;
    private final Photonvision photonvision;

    private final PIDController swerveRotController;
    private final PIDController swerveTranslationController;

    private final boolean endAtTarget;
    private final boolean rotateOnly;

    public AlignWithVisionTarget(Swerve swerve, Photonvision photonvision, boolean rotateOnly, boolean endAtTarget) {
        this.swerve = swerve;       
        this.photonvision = photonvision;
        this.endAtTarget = endAtTarget;
        this.rotateOnly = rotateOnly;

        swerveRotController = new PIDController(kPRotationTarget, 0.0, kDRotationTarget);
        swerveRotController.setTolerance(rotationSetpointTolerance);
        swerveRotController.setSetpoint(0.0);
        
        swerveTranslationController = new PIDController(kPTranslationTarget, 0, 0.0); 
        swerveTranslationController.setTolerance(translationSetpointTolerance);
        swerveTranslationController.setSetpoint(0.0);

        addRequirements(swerve, photonvision);
    }

    @Override
    public void execute() {
        if(photonvision.hasTargets()) {
            if (rotateOnly) {
                swerve.driveOpenLoop(
                    new Translation2d(),
                    swerveRotController.calculate(photonvision.getLatestXOffset()), false);
            }

            else {
                swerve.driveOpenLoop(
                    new Translation2d(swerveTranslationController.calculate(-Math.abs(photonvision.getLatestDistance())), 0.0),//TODO fix distance readings
                    swerveRotController.calculate(photonvision.getLatestXOffset()), false);
            }
        }

        else {
            swerveRotController.reset();
            swerve.driveOpenLoop(new Translation2d(), 0.0, false);

            System.err.println("Missed target while aligning!");
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