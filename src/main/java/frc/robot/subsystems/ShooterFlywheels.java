package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkMaxWrapper;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterFlywheels extends SubsystemBase {
    private final TalonFX leftFlywheelMotor;
    private final TalonFX rightFlywheelMotor;
    private final VelocityVoltage flywheelController = new VelocityVoltage(0.0);

    private final SparkMaxWrapper storageMotor;

    private final DigitalInput beamBreak = new DigitalInput(beamBreakPort);

    private final ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");

    public ShooterFlywheels() { 
        storageMotor = new SparkMaxWrapper(shooterStorageConstants);
        storageMotor.config();

        leftFlywheelMotor = new TalonFX(leftFlywheelMotorID);
        rightFlywheelMotor = new TalonFX(rightFlywheelMotorID);

        /* Flywheel configs */
        TalonFXConfiguration leftFlywheelConfig = new TalonFXConfiguration();
        leftFlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftFlywheelConfig.CurrentLimits.SupplyCurrentLimit = flywheelSupplyCurrentLimit;
        leftFlywheelConfig.CurrentLimits.SupplyTimeThreshold = flywheelSupplyTimeThreshold;
        leftFlywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftFlywheelConfig.Slot0.kP = flywheelkP;
        leftFlywheelConfig.Slot0.kD = flywheelkD;
        leftFlywheelConfig.Slot0.kV = flywheelkV;
        leftFlywheelConfig.Slot0.kS = flywheelkS;
        leftFlywheelConfig.MotorOutput.Inverted = leftFlywheelInvert;

        TalonFXConfiguration rightFlywheelConfig = new TalonFXConfiguration();
        rightFlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightFlywheelConfig.CurrentLimits.SupplyCurrentLimit = flywheelSupplyCurrentLimit;
        rightFlywheelConfig.CurrentLimits.SupplyTimeThreshold = flywheelSupplyTimeThreshold;
        rightFlywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightFlywheelConfig.Slot0.kP = flywheelkP;
        rightFlywheelConfig.Slot0.kD = flywheelkD;
        rightFlywheelConfig.Slot0.kV = flywheelkV;
        rightFlywheelConfig.Slot0.kS = flywheelkS;
        rightFlywheelConfig.MotorOutput.Inverted = rightFlywheelInvert;

        /* Apply the configs and invert */
        leftFlywheelMotor.getConfigurator().apply(
            leftFlywheelConfig
        );
        rightFlywheelMotor.getConfigurator().apply(
            rightFlywheelConfig
        );

        /* Add Telemetry */
        shooterTab.add(leftFlywheelMotor)
            .withPosition(3, 0).withSize(1, 1);
        shooterTab.addDouble("Left Velocity", () -> leftFlywheelMotor.getVelocity().getValueAsDouble())
            .withPosition(3, 1).withSize(1, 1);
        shooterTab.add(rightFlywheelMotor)
            .withPosition(4, 0).withSize(1, 1);
        shooterTab.addDouble("Right Velocity", () -> rightFlywheelMotor.getVelocity().getValueAsDouble())
            .withPosition(4, 1).withSize(1, 1);
        shooterTab.add(storageMotor)
            .withPosition(0, 2).withSize(2, 1);
        shooterTab.addBoolean("Has Note", this::hasNote)
            .withPosition(6, 0).withSize(2, 1);
        shooterTab.addBoolean("Flywheels At Setpoint", this::flywheelsAtSetpoint)
            .withPosition(6, 2).withSize(2, 1);
        shooterTab.add(this)
            .withPosition(0, 4).withSize(2, 1);

        /* Add Command Testing Butons */
        shooterTab.add(enableStorageMotorReceiving())
            .withPosition(0, 3).withSize(1, 1);
        shooterTab.add(enableStorageMotorToFlywheels())
            .withPosition(1, 3).withSize(1, 1);
        shooterTab.add(resetCommand())
            .withPosition(4, 3).withSize(1, 1);
        /* Add widget to modify the flywheel setpoint */
        GenericEntry velocitySetpointEntry = shooterTab.add("Flywheel Override", 0.0)
            .withPosition(6, 4).withSize(1, 1)
            .getEntry();
        shooterTab.add(runOnce(() -> setVelocitySetpoint(velocitySetpointEntry.getDouble(0.0))).withName("Override Velocity"))
            .withPosition(7, 4).withSize(1, 1);

        /* Prevent moving to a previous setpoint */
        reset();
    }

    private Command setStorageMotorPower(double power) {
        return runOnce(() -> storageMotor.set(power));
    }

    /* Sets the target and if blocking, waits until the setpoint is achieved */
    private Command setFlywheelVelocity(double setpoint, boolean blocking) {
        Command setTargetCommand = runOnce(() -> setVelocitySetpoint(setpoint));

        return blocking ? setTargetCommand.andThen(Commands.waitUntil(this::flywheelsAtSetpoint)) : setTargetCommand;
    }

    private void setVelocitySetpoint(double setpoint) {
        if(setpoint > maxFlywheelSetpoint || setpoint < minFlywheelSetpoint) {
            System.err.println("Velocity setpoint " + setpoint + " is out of bounds!");

            return;
        }

        leftFlywheelMotor.setControl(flywheelController.withVelocity(setpoint));
        rightFlywheelMotor.setControl(flywheelController.withVelocity(setpoint));
    }

    private Command enableStorageMotorReceiving() {
        return setStorageMotorPower(storageMotorPowerReceiving).withName("Storage receive");
    }

    private Command enableStorageMotorToFlywheels() {
        return setStorageMotorPower(storageMotorPowerToFlywheels).withName("Storage flywheels");
    }

    public Command disableFlywheels() {
        return runOnce(() -> { 
            leftFlywheelMotor.disable();
            rightFlywheelMotor.disable();
        });
    }

    private Command disableStorageMotor() {
        return runOnce(() -> storageMotor.disable());
    }

    public Command receiveNoteFromIntake() {
        return enableStorageMotorReceiving()
            .andThen(Commands.waitUntil(this::hasNote))
            .andThen(disableStorageMotor())
            .andThen(Commands.print("Shooter received note"));
    }

    public Command spinUpFlywheels(double velocityRPS) {
        return setFlywheelVelocity(velocityRPS, false);
    }

    public Command shoot(double velocityRPS, boolean disableFlywheels) {
        return setFlywheelVelocity(velocityRPS, true)
           .andThen(enableStorageMotorToFlywheels())
           .andThen(Commands.waitSeconds(shotWaitTime))
           .andThen(disableFlywheels ? disableFlywheels() : Commands.none())
           .andThen(disableStorageMotor());
    }

    public boolean hasNote() {
        return !beamBreak.get();
    }

    private boolean flywheelsAtSetpoint() {
        return Math.abs(leftFlywheelMotor.getVelocity().getValueAsDouble() - flywheelController.Velocity) < flywheelVelocityTolerance
        && Math.abs(rightFlywheelMotor.getVelocity().getValueAsDouble() - flywheelController.Velocity) < flywheelVelocityTolerance;
    }

    public void reset() {
        if(getCurrentCommand() != null) 
            getCurrentCommand().cancel();

        leftFlywheelMotor.disable();
        rightFlywheelMotor.disable();
        storageMotor.disable();
    }

    public Command resetCommand() {
        return runOnce(this::reset).withName("Reset Flywheels");
    }
}