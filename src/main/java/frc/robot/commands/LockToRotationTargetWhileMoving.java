package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClosedLoopConstants;
import frc.robot.subsystems.Swerve;

public class LockToRotationTargetWhileMoving extends Command {
    private Swerve swerve;

    private PIDController swerveRotController;
    private DoubleSupplier targetRotSup;
    private Supplier<Translation2d> translationSup;

    /* Lets you move using while locking heading to a rotation target */
    public LockToRotationTargetWhileMoving(Swerve swerve, DoubleSupplier targetRotSup, Supplier<Translation2d> translationSup) {
        this.swerve = swerve;       
        this.targetRotSup = targetRotSup;
        this.translationSup = translationSup;

        swerveRotController = new PIDController(ClosedLoopConstants.kPRotationTarget, 0, ClosedLoopConstants.kDRotationTarget);
        swerveRotController.enableContinuousInput(-180.0, 180.0);//TODO the units will be messed up

        addRequirements(swerve);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        swerveRotController.setSetpoint(targetRotSup.getAsDouble());

        swerve.driveClosedLoop(
            translationSup.get(), 
            swerveRotController.calculate(swerve.getPose().getRotation().getDegrees()));

        SmartDashboard.putNumber("Target Rot Error", swerveRotController.getPositionError());
    }

    @Override
    public boolean isFinished() { return false; }
}