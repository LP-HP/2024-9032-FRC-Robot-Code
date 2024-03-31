package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Photonvision;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.ClosedLoopConstants.*;

public class DriveToNote extends Command {
    private final Swerve swerve;
    private final Photonvision photonvision;

    private final PIDController swerveRotController;
    private final PIDController swerveDistanceController;

    private double lastXOffset;
    private int cycleAmtSinceTargetSeen;
    private boolean hadTarget = false;

    public DriveToNote(Swerve swerve, Photonvision photonvision) {
        this.swerve = swerve;       
        this.photonvision = photonvision;

        swerveRotController = new PIDController(kPNoteRotation, 0.0, kDNoteRotation);
        swerveRotController.setSetpoint(0.0);

        swerveDistanceController = new PIDController(kPNoteDistance, 0.0, kDNoteDistance);
        swerveDistanceController.setSetpoint(0.0);

        addRequirements(swerve, photonvision);
    }

    @Override
    public void execute() {
        /* Find targets - if there is no target, use the last one seen if it has not expired */
        if(photonvision.hasTargets()) {
            lastXOffset = photonvision.getLatestXOffset();

            cycleAmtSinceTargetSeen = 0;
            hadTarget = true;
        }

        else if(hadTarget)
            cycleAmtSinceTargetSeen++;

        if(cycleAmtSinceTargetSeen < cycleAmtSinceTargetSeenCutoff && hadTarget) {
            double noteDrivingSpeed = Math.min(swerveDistanceController.calculate(photonvision.getLatestDistance()), maxNoteDrivingSpeed);

            swerve.driveOpenLoop(
                new Translation2d(noteDrivingSpeed, 0.0),
                swerveRotController.calculate(lastXOffset), 
                false
            );
        }

        else {
            reset();

            System.err.println("Note tracking lost while driving!");
        }
    }

    @Override
    public void end(boolean interrupted) {
        reset();
    }

    private void reset() {
        swerveRotController.reset();
        swerve.driveOpenLoop(new Translation2d(), 0.0, false);
        
        cycleAmtSinceTargetSeen = 0;
        hadTarget = false;
    }
}