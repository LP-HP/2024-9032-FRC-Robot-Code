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

    public DriveToNote(Swerve swerve, Photonvision photonvision) {
        this.swerve = swerve;       
        this.photonvision = photonvision;

        swerveRotController = new PIDController(kPNoteRotation, 0.0, kDNoteRotation);
        swerveRotController.setSetpoint(0.0);

        addRequirements(swerve, photonvision);
    }

    @Override
    public void execute() {
        if(photonvision.hasTargets()) {
            swerve.driveOpenLoop(
                new Translation2d(noteDrivingSpeed, 0.0),
                swerveRotController.calculate(photonvision.getLatestXOffset()), false);
        }

        else {
            swerveRotController.reset();
            swerve.driveOpenLoop(new Translation2d(), 0.0, false);

            System.err.println("Note tracking lost while driving!");
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.driveOpenLoop(new Translation2d(), 0.0, false);
        swerveRotController.reset();
    }
}