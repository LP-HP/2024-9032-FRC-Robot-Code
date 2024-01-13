package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;

public class AlignWithRotationTarget extends Command {
    private Swerve swerve;

    private PIDController swerveRotController;
    private double targetRot;

    public AlignWithRotationTarget(Swerve swerve, double targetRot) {
        this.swerve = swerve;       
        this.targetRot = targetRot;

        swerveRotController = new PIDController(VisionConstants.kPRotation, 0, VisionConstants.kDRotation);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerveRotController.setSetpoint(targetRot);
    }

    @Override
    public void execute() {
        swerve.driveClosedLoop(
            new Translation2d(), 
            swerveRotController.calculate(swerve.getPose().getRotation().getDegrees()));

        SmartDashboard.putNumber("Target Rot Error", swerveRotController.getPositionError());
    }

    @Override
    public boolean isFinished() {
        return swerveRotController.atSetpoint();
    }
}