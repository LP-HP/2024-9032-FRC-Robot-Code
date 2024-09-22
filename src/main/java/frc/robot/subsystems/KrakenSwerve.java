package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.krakenSwerveConstants;

public class KrakenSwerve extends SubsystemBase {
    public SwerveDrivetrain driveTrain;

    public KrakenSwerve() {
        driveTrain = new SwerveDrivetrain(krakenSwerveConstants.DrivetrainConstants, krakenSwerveConstants.frontLeft, krakenSwerveConstants.frontRight, krakenSwerveConstants.backLeft, krakenSwerveConstants.backRight);
    }
}
