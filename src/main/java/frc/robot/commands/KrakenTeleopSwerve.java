package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.krakenSwerveConstants;
import frc.robot.subsystems.KrakenSwerve;

public class KrakenTeleopSwerve extends Command {
    private final KrakenSwerve swerve;

    private final DoubleSupplier rotSup;
    private final DoubleSupplier transSup;
    private final DoubleSupplier strafeSup;

    public KrakenTeleopSwerve(KrakenSwerve mswerve, DoubleSupplier m_rotSup, DoubleSupplier m_transSup, DoubleSupplier m_strafeSup) {
        swerve = mswerve;

        rotSup = m_rotSup;
        transSup = m_transSup;
        strafeSup = m_strafeSup;

        addRequirements(mswerve);
    }

    @Override
    public void execute() {
        swerve.driveTrain.setControl(
            krakenSwerveConstants.m_driveRequest.withVelocityX(transSup.getAsDouble())
            .withVelocityY(strafeSup.getAsDouble())
            .withRotationalRate(rotSup.getAsDouble())
        );
    }
}
