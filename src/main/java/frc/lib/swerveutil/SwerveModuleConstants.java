package frc.lib.swerveutil;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.SparkMaxConstants;

public class SwerveModuleConstants {
    public final SparkMaxConstants driveMotorConstants;
    public final SparkMaxConstants angleMotorConstants;
    public final int cancoderID;
    public final Rotation2d angleOffset;

    public SwerveModuleConstants(SparkMaxConstants driveMotorConstants, SparkMaxConstants angleMotorConstants, int canCoderID, Rotation2d angleOffset) {
        this.driveMotorConstants = driveMotorConstants;
        this.angleMotorConstants = angleMotorConstants;
        this.cancoderID = canCoderID;
        this.angleOffset = angleOffset;
    }
}
