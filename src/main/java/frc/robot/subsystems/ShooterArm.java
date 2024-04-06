package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkMaxWrapper;

import static frc.robot.Constants.ShooterConstants.*;

import java.util.function.DoubleSupplier;

public class ShooterArm extends SubsystemBase {
    private final SparkMaxWrapper armMotor;    
    private final SparkMaxWrapper armMotorFollower;    

    private final ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter Arm");

    public ShooterArm() {
        armMotor = new SparkMaxWrapper(shooterArmConstants);
        armMotor.configAbsoluteEncoder(invertAbsoluteEncoder, absoluteEncoderConversionFactor, absoluteEncoderOffset);
        armMotor.setAbsoluteEncoderAsFeedback();
        armMotor.config();

        armMotorFollower = new SparkMaxWrapper(shooterArmFolllowerConstants);
        armMotorFollower.follow(armMotor, invertArmFollower);
        armMotorFollower.config();

        /* Add Telemetry */
        shooterTab.add(armMotor)
            .withPosition(0, 0).withSize(2, 2);
        shooterTab.addBoolean("Arm At Setpoint", this::armAtSetpoint)
            .withPosition(6, 1).withSize(2, 1);
        shooterTab.add(this)
            .withPosition(3, 4).withSize(2, 1);

        /* Add Command Testing Butons */
        shooterTab.add(resetCommand())
            .withPosition(0, 3).withSize(1, 1);
        shooterTab.add(setToUnderStagePosition(true))
            .withPosition(1, 3).withSize(1, 1);
        shooterTab.add(setToPassthroughPosition(true))
            .withPosition(2, 3).withSize(1, 1);
        shooterTab.add(setToUpPosition(true))
            .withPosition(3, 3).withSize(1, 1);
        shooterTab.add(setToAmpPosition(true))
            .withPosition(4, 3).withSize(1, 1);
        /* Add widget to modify the arm setpoint */
        GenericEntry armSetpointEntry = shooterTab.add("Arm Override", 0.0)
            .withPosition(6, 3).withSize(1, 1)
            .getEntry();
        shooterTab.add(runOnce(() -> setArmSetpoint(armSetpointEntry.getDouble(armMotor.getSetpoint()))).withName("Override Arm"))
            .withPosition(7, 3).withSize(1, 1);

        /* Prevent moving to a previous setpoint */
        reset();
    }

    /* Sets the target and if blocking, waits until the setpoint is achieved */
    private Command setTargetPosition(double setpoint, boolean blocking) {
        Command setTargetCommand = runOnce(() -> setArmSetpoint(setpoint));

        return blocking ? setTargetCommand.andThen(Commands.waitUntil(this::armAtSetpoint)) : setTargetCommand;
    }

    private void setArmSetpoint(double setpoint) {
        if(setpoint > maxArmSetpoint || setpoint < minArmSetpoint) {
            System.err.println("Shooter arm setpoint " + setpoint + " is out of bounds!");

            return;
        }

        armMotor.setClosedLoopTarget(setpoint);
    }

    public Command setToUnderStagePosition(boolean waitUntilAchieved) {
        return setTargetPosition(armPositionUnderStage, waitUntilAchieved).withName("To understage"); 
    }

    public Command setToPassthroughPosition(boolean waitUntilAchieved) {
        return setTargetPosition(armPositionPassthrough, waitUntilAchieved).withName("To passthrough"); 
    }

    public Command setToUpPosition(boolean waitUntilAchieved) {
        return setTargetPosition(armPositionUp, waitUntilAchieved).withName("To up"); 
    }

    public Command setToAmpPosition(boolean waitUntilAchieved) {
        return setTargetPosition(armPositionAmp, waitUntilAchieved).withName("To amp"); 
    }

    public Command setToTrapPosition(boolean waitUntilAchieved) {
        return setTargetPosition(armPositionTrap, waitUntilAchieved).withName("To trap"); 
    }

    public Command setToShuttlePosition(boolean waitUntilAchieved) {
        return setTargetPosition(armPositionShuttle, waitUntilAchieved).withName("To shuttle"); 
    }

    public Command setToAutoPosition(double position, boolean waitUntilAchieved) {
        return setTargetPosition(position, waitUntilAchieved).withName("To auto pos");
    }

    /* Moves to the target position from a vision target y offset */
    public Command setToTargetPositionFromDistance(DoubleSupplier distanceSup, DoubleSupplier chassisXVelocitySup, boolean waitUntilAchieved) {
        Command setTargetCommand = runOnce(() -> setArmSetpoint(distanceAndVelocityToArmAngle(distanceSup.getAsDouble(), chassisXVelocitySup.getAsDouble())));
                
        setTargetCommand = waitUntilAchieved ? setTargetCommand.andThen(Commands.waitUntil(this::armAtSetpoint)) : setTargetCommand;

        return setTargetCommand.withName("To distance pos");
    }   

    private double distanceAndVelocityToArmAngle(double distance, double chassisXVelocity) {
        /* Use the interpolated lookup table with the velocity compensated distance value */
        double targetPos = distanceToArmPosTable.get(distance) + (chassisXVelocity * distanceVelocityCompAmt);

        System.out.println("Target pos " + targetPos + " distance " + distance + " velocity " + chassisXVelocity);

        return targetPos;
    }

    public boolean armAtSetpoint() {
        return Math.abs(armMotor.getAbsolutePosition() - armMotor.getSetpoint()) < armSetpointTolerance;
    }

    public void reset() {
        if(getCurrentCommand() != null) 
            getCurrentCommand().cancel();

        armMotor.setClosedLoopTarget(armMotor.getAbsolutePosition());
    }

    public Command resetCommand() {
        return runOnce(this::reset).withName("Reset Arm");
    }
}