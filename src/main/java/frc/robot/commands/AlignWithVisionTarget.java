package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.ClosedLoopConstants.*;

import java.util.function.DoubleSupplier;

public class AlignWithVisionTarget extends Command {
    private final Swerve swerve;
    private final LimelightVision limelight;

    private final PIDController swerveRotController;
    private final PIDController swerveTranslationController;

    private final boolean endAtTarget;
    private final boolean rotateOnly;

    private DoubleSupplier transSup;
    private DoubleSupplier strafeSup;

    public AlignWithVisionTarget(Swerve swerve, LimelightVision limelight, boolean rotateOnly, boolean endAtTarget) {
        this.swerve = swerve;       
        this.limelight = limelight;
        this.endAtTarget = endAtTarget;
        this.rotateOnly = rotateOnly;

        swerveRotController = new PIDController(kPRotationTarget, kIRotationTarget, kDRotationTarget);
        swerveRotController.setTolerance(rotationSetpointTolerance);
        swerveRotController.setIntegratorRange(-kIZoneRotationTarget, kIZoneRotationTarget);
        swerveRotController.setSetpoint(0.0);
        
        swerveTranslationController = new PIDController(kPTranslationTarget, 0, 0.0); 
        swerveTranslationController.setTolerance(translationSetpointTolerance);
        swerveTranslationController.setSetpoint(0.0);

        addRequirements(swerve, limelight);
    }

    public AlignWithVisionTarget(Swerve swerve, DoubleSupplier transSup, DoubleSupplier strafeSup, LimelightVision limelight, boolean rotateOnly, boolean endAtTarget) {
        this.swerve = swerve;       
        this.limelight = limelight;
        this.endAtTarget = endAtTarget;
        this.rotateOnly = rotateOnly;
        this.strafeSup = strafeSup;
        this.transSup = transSup;

        swerveRotController = new PIDController(kPRotationTarget, kIRotationTarget, kDRotationTarget);
        swerveRotController.setTolerance(rotationSetpointTolerance);
        swerveRotController.setIntegratorRange(-kIZoneRotationTarget, kIZoneRotationTarget);
        swerveRotController.setSetpoint(0.0);
        
        swerveTranslationController = new PIDController(kPTranslationTarget, 0, 0.0); 
        swerveTranslationController.setTolerance(translationSetpointTolerance);
        swerveTranslationController.setSetpoint(0.0);

        addRequirements(swerve, limelight);
    }

    @Override
    public void execute() {
        if(limelight.getAprilTagTarget().isValid) {
            if (rotateOnly && (transSup == null || strafeSup == null)) {
                swerve.driveOpenLoop(//TODO use closed loop
                    new Translation2d(),
                    swerveRotController.calculate(limelight.getAprilTagTarget().xOffset), false);
            }

            else if (rotateOnly) {
                swerve.driveOpenLoop(//TODO use closed loop
                    new Translation2d(transSup.getAsDouble(), strafeSup.getAsDouble()),
                    swerveRotController.calculate(limelight.getAprilTagTarget().xOffset), false);
            }

            else {
                swerve.driveClosedLoop(
                    new Translation2d(swerveTranslationController.calculate(limelight.getAprilTagTarget().yOffset), 0.0),//TODO use distance
                    swerveRotController.calculate(limelight.getAprilTagTarget().xOffset));
            }
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
    }
}