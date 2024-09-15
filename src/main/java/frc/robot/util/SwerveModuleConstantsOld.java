package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

public record SwerveModuleConstantsOld(SparkMaxConstants driveMotorConstants, SparkMaxConstants angleMotorConstants, int canCoderID, Rotation2d angleOffset) {}