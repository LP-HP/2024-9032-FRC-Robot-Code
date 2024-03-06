package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkMaxWrapper;

import static frc.robot.Constants.ClimberConstants.*;

import java.util.function.BooleanSupplier;

public class Climbers extends SubsystemBase {
    private final SparkMaxWrapper leftClimber;
    private final SparkMaxWrapper rightClimber;

    private final ShuffleboardTab climberTab = Shuffleboard.getTab("Climbers");

    public Climbers() {
        leftClimber = new SparkMaxWrapper(leftClimberConstants);
        leftClimber.config();

        rightClimber = new SparkMaxWrapper(rightClimberConstants);
        rightClimber.config();

        /* Add Telemetry */
        climberTab.add(leftClimber)
            .withPosition(0, 0).withSize(2, 1);
        climberTab.add(rightClimber)
            .withPosition(3, 0).withSize(2, 1);
    }

    public Command setLeftClimberPower(double power, BooleanSupplier inverted) {
        return runOnce(() -> leftClimber.set(power * (inverted.getAsBoolean() ? -1 : 1)));
    }

    public Command setRightClimberPower(double power, BooleanSupplier inverted) {
        return runOnce(() -> rightClimber.set(power * (inverted.getAsBoolean() ? -1 : 1)));
    }
}   