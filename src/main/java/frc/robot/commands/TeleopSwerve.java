package frc.robot.commands;

import frc.robot.subsystems.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.Constants.TeleopConstants.*;

public class TeleopSwerve extends Command {    
    private Swerve swerve;    

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;

    private final SlewRateLimiter accelerationLimiterTranslation = new SlewRateLimiter(accelerationLimit, decelerationLimit, 0.0);
    private final SlewRateLimiter accelerationLimiterStrafe = new SlewRateLimiter(accelerationLimit, decelerationLimit, 0.0);

    public TeleopSwerve(Swerve swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
        this.swerve = swerve;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        //Use joystick deadband to prevent small drifts
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), stickDeadband);

        /* Multiply by conversion factor to get the joystick value in m/s and apply acceleration limits */
        translationVal *= joystickToSpeedConversionFactor;
        translationVal = accelerationLimiterTranslation.calculate(translationVal);

        strafeVal *= joystickToSpeedConversionFactor;
        strafeVal = accelerationLimiterStrafe.calculate(strafeVal);

        //Run the open loop drive using speed values and apply rotation conversion factor
        swerve.driveOpenLoop(
            new Translation2d(translationVal, strafeVal), 
            rotationVal * joystickToAngularVelocityConversionFactor, 
            isFieldCentric
        );
    }
}