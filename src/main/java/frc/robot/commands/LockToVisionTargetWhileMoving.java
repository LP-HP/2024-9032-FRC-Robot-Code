package frc.robot.commands;

import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.LimelightVision.AprilTagTarget;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.Constants.TeleopConstants.*;
import static frc.robot.Constants.ClosedLoopConstants.*;

public class LockToVisionTargetWhileMoving extends Command {    
    private final Swerve swerve;    
    private final LimelightVision limelight;

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;

    private final SlewRateLimiter accelerationLimiterTranslation = new SlewRateLimiter(accelerationLimit);
    private final SlewRateLimiter accelerationLimiterStrafe = new SlewRateLimiter(accelerationLimit);

    private final PIDController swerveRotController;

    public LockToVisionTargetWhileMoving(Swerve swerve, LimelightVision limelight, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
        this.swerve = swerve;
        this.limelight = limelight;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;

        swerveRotController = new PIDController(kPRotationTarget, kIRotationTarget, kDRotationTarget);
        swerveRotController.setIntegratorRange(-kIZoneRotationTarget, kIZoneRotationTarget);
        swerveRotController.setSetpoint(0.0);

        addRequirements(swerve, limelight);
    }

    @Override
    public void execute() {
        /* Use joystick deadband to prevent small drifts */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), stickDeadband);

        /* Curve inputs to allow for more control closer to the lower range of the joystick */
        translationVal = applyInputCurve(translationVal);
        strafeVal = applyInputCurve(strafeVal);
        rotationVal = applyInputCurve(rotationVal);

        /* Multiply by conversion factor to get the joystick value in m/s and apply acceleration limits */
        translationVal *= joystickToSpeedConversionFactor;
        translationVal = accelerationLimiterTranslation.calculate(translationVal);

        strafeVal *= joystickToSpeedConversionFactor;
        strafeVal = accelerationLimiterStrafe.calculate(strafeVal);

        rotationVal *= joystickToAngularVelocityConversionFactor;

        /* Override rotation to tag x-offset */
        AprilTagTarget aprilTag = limelight.getAprilTagTarget();
        if(aprilTag.isValid) 
            rotationVal = swerveRotController.calculate(aprilTag.xOffset);

        else {
            swerveRotController.reset();

            System.err.println("Missed target while locking on!");
        }

        /* Run the open loop drive using speed values  */
        swerve.driveOpenLoop(
            new Translation2d(translationVal, strafeVal), 
            rotationVal, 
            isFieldCentric
        );
    }

    @Override
    public void end(boolean interrupted) {
        swerveRotController.reset();
    }

    private double applyInputCurve(double joystickInput) {
        return Math.copySign(Math.pow(joystickInput, 2), joystickInput);
    }
}