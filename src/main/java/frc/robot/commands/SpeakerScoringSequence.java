package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class SpeakerScoringSequence extends SequentialCommandGroup {
    public SpeakerScoringSequence(Swerve swerve, LimelightVision limelight, Shooter shooter) {
        addCommands(
            new AlignWithRotationTarget(swerve, () -> limelight.getAprilTagTarget().xOffset),
            shooter.moveArmToPositionFromArea(() -> limelight.getAprilTagTarget().area),
            shooter.setShooterVelocityThenWaitThenDisable(5, 0.25),//TODO do lookup table if needed
            shooter.setToPassthroughPosition()
        );
    }
}