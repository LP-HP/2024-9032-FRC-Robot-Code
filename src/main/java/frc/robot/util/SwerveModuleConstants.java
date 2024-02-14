package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

public record SwerveModuleConstants(SparkMaxConstants driveMotorConstants, SparkMaxConstants angleMotorConstants, int canCoderID, Rotation2d angleOffset) {}