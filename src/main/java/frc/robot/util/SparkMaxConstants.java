package frc.robot.util;

import com.revrobotics.CANSparkBase.IdleMode;

public record SparkMaxConstants(int id, String name, ControlMode mode, SparkMaxPIDConstants pidConstants, int currentLimit, boolean inverted, IdleMode idleMode, int nominalVoltage, double positionConversionFactor) {
    public enum ControlMode { position, velocity, positionLeader, velocityLeader, velocityControlWithPositionData, percentOutput }

    public record SparkMaxPIDConstants(double kP, double kI, double kD, double kF) {}
}