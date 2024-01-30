package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {    
    private Swerve swerve;    

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier fieldCentricSup;

    private final SlewRateLimiter accelerationLimiterTranslation = new SlewRateLimiter(Constants.TeleopConstants.accelerationLimit);
    private final SlewRateLimiter accelerationLimiterStrafe = new SlewRateLimiter(Constants.TeleopConstants.accelerationLimit);

    public TeleopSwerve(Swerve swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.swerve = swerve;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.fieldCentricSup = robotCentricSup;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        //Use joystick deadband to prevent small drifts
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.TeleopConstants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.TeleopConstants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.TeleopConstants.stickDeadband);

        /* Multiply by conversion factor to get the joystick value in m/s and apply acceleration limits */
        translationVal *= Constants.TeleopConstants.joystickToSpeedConversionFactor;
        translationVal = accelerationLimiterTranslation.calculate(translationVal);

        strafeVal *= Constants.TeleopConstants.joystickToSpeedConversionFactor;
        strafeVal = accelerationLimiterStrafe.calculate(strafeVal);

        //Run the open loop drive using speed values and apply rotation conversion factor
        swerve.driveOpenLoop(
            new Translation2d(translationVal, strafeVal), 
            rotationVal * Constants.TeleopConstants.joystickToAngularVelocityConversionFactor, 
            fieldCentricSup.getAsBoolean()
        );
    }
}