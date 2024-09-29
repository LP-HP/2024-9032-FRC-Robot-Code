package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenSwerveConstants;

public class KrakenSwerve extends SubsystemBase {
    public SwerveDrivetrain driveTrain;

    public KrakenSwerve() {
        driveTrain = new SwerveDrivetrain(KrakenSwerveConstants.DrivetrainConstants, KrakenSwerveConstants.frontLeft, KrakenSwerveConstants.frontRight, KrakenSwerveConstants.backLeft, KrakenSwerveConstants.backRight);
    }
}
