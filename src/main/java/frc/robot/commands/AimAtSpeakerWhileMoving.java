package frc.robot.commands;

import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.LimelightVision.AprilTagTarget;
import frc.robot.util.VisionTargetCache;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.Constants.TeleopConstants.*;
import static frc.robot.Constants.ClosedLoopConstants.*;

public class AimAtSpeakerWhileMoving extends Command {    
    private final Swerve swerve;    
    private final LimelightVision limelight;

    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;
    private final DoubleSupplier rotationSup;

    private final PIDController swerveRotController;

    private final VisionTargetCache<AprilTagTarget> visionCache;

    private boolean shouldEnd = false;

    public AimAtSpeakerWhileMoving(Swerve swerve, LimelightVision limelight, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
        this.swerve = swerve;
        this.limelight = limelight;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;

        swerveRotController = new PIDController(kPSpeakerRotation, 0.0, kDSpeakerRotation);
        swerveRotController.setSetpoint(0.0);

        visionCache = new VisionTargetCache<>(cycleAmtSinceAprilTagSeenCutoff);

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

        /* Multiply by conversion factor to get the joystick value in m/s */
        translationVal *= joystickToSpeedConversionFactor;
        strafeVal *= joystickToSpeedConversionFactor;
        rotationVal *= joystickToAngularVelocityConversionFactor;

        /* Override rotation to velocity compensated tag x-offset using PID */
        AprilTagTarget aprilTag = limelight.getAprilTagTarget();
        if(aprilTag.isValidSpeakerTag()) 
            visionCache.updateTarget(aprilTag);

        if(visionCache.targetNotExpired()) {
            aprilTag = visionCache.getAndIncrement();

            /* Aim slightly further than the tag based on current strafe velocity and target skew */
            double targetOffset = swerve.getSpeeds().vyMetersPerSecond * xOffsetVelocityCompAmt;
            if(Math.abs(aprilTag.skew) > skewCompAmtCutoff) 
                targetOffset += aprilTag.skew * skewCompAmt;

            swerveRotController.setSetpoint(targetOffset);

            rotationVal = swerveRotController.calculate(aprilTag.xOffset);
        }

        else {
            System.err.println("Speaker tag tracking lost while locking on!");

            shouldEnd = true;
        }

        /* Run the open loop drive using speed values  */
        swerve.driveOpenLoop(
            new Translation2d(translationVal, strafeVal), 
            rotationVal, 
            isFieldCentric
        );
    }

    @Override
    public boolean isFinished() {
        return shouldEnd;
    }

    @Override
    public void end(boolean interrupted) {
        reset();
    }

    private void reset() {
        swerveRotController.reset();
        visionCache.reset();
        shouldEnd = false;
    }

    private double applyInputCurve(double joystickInput) {
        return Math.copySign(Math.pow(joystickInput, 2), joystickInput);
    }
}