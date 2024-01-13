package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class SpeakerScoringSequence extends SequentialCommandGroup {
    public SpeakerScoringSequence(Swerve swerve, LimelightVision limelight, Shooter shooter) {
        addCommands(
            new AlignWithRotationTarget(swerve, () -> limelight.getAprilTagTarget().xOffset),
            shooter.moveArmToPosition(3),//TODO lerp lookup table
            shooter.setShooterVelocity(3),
            Commands.waitSeconds(0.25),
            shooter.disableShooterFlywheel()
        );
    }
}