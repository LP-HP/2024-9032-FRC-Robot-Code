package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClosedLoopConstants;
import frc.robot.subsystems.Swerve;

public class AlignWithRotationTarget extends Command {
    private Swerve swerve;

    private PIDController swerveRotController;
    private DoubleSupplier targetRotSup;

    public AlignWithRotationTarget(Swerve swerve, DoubleSupplier targetRotSup) {
        this.swerve = swerve;       
        this.targetRotSup = targetRotSup;

        swerveRotController = new PIDController(ClosedLoopConstants.kPRotationTarget, 0, ClosedLoopConstants.kDRotationTarget);
        swerveRotController.enableContinuousInput(-180.0, 180.0);//TODO the units will be messed up

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerveRotController.setSetpoint(targetRotSup.getAsDouble());
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