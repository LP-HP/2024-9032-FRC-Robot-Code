package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;

public class AlignWithVisionTarget extends Command {
    private Swerve swerve;

    private PIDController swerveRotController;

    public AlignWithVisionTarget(Swerve swerve, double targetRot) {
        this.swerve = swerve;

        swerveRotController = new PIDController(VisionConstants.kPRotation, 0, VisionConstants.kDRotation);
        swerveRotController.setSetpoint(targetRot);

        addRequirements(swerve);
    }

    @Override
    public void execute() {//TODO add telemetry
        swerve.driveClosedLoop(
            new Translation2d(), 
            swerveRotController.calculate(swerve.getPose().getRotation().getDegrees()));
    }

    @Override
    public boolean isFinished() {
        return swerveRotController.atSetpoint();
    }
}