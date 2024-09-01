package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.absoluteEncoderOffset;
import static frc.robot.Constants.ShooterConstants.shooterArmConstants;
import static frc.robot.Constants.ShooterConstants.shooterArmFolllowerConstants;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkMaxWrapper;

public class ShooterArm extends SubsystemBase {
    SparkMaxWrapper armMotor;
    SparkMaxWrapper followerMotor;
    public ShooterArm() {
        armMotor = new SparkMaxWrapper(shooterArmConstants);
        armMotor.configAbsoluteEncoder(shooterArmConstants.inverted(), shooterArmConstants.positionConversionFactor(), absoluteEncoderOffset);
        armMotor.config();
        
        followerMotor = new SparkMaxWrapper(shooterArmFolllowerConstants);
        followerMotor.
    }
}