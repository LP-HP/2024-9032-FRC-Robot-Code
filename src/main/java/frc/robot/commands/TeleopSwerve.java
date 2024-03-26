package frc.robot.commands;

import frc.robot.subsystems.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.Constants.TeleopConstants.*;

public class TeleopSwerve extends Command {    
    private final Swerve swerve;    

    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;
    private final DoubleSupplier rotationSup;

    private final SlewRateLimiter accelerationLimiterTranslation = new SlewRateLimiter(accelerationLimit);
    private final SlewRateLimiter accelerationLimiterStrafe = new SlewRateLimiter(accelerationLimit);

    public TeleopSwerve(Swerve swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
        this.swerve = swerve;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;

        addRequirements(swerve);
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

        /* Run the open loop drive using speed values  */
        swerve.driveOpenLoop(
            new Translation2d(translationVal, strafeVal), 
            rotationVal, 
            isFieldCentric
        );
    }

    private double applyInputCurve(double joystickInput) {
        return Math.copySign(Math.pow(joystickInput, 2), joystickInput);
    }
}