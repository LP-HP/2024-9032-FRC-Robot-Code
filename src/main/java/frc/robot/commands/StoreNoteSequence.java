// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class StoreNoteSequence extends Command {
    private Intake intake;
    private Shooter shooter;

    private Trigger noteInIntake = new Trigger(intake::isBeamBreakTriggered).debounce(0.025);
    private Trigger noteInShooter = new Trigger(shooter::isBeamBreakTriggered).debounce(0.025);

    public StoreNoteSequence(Intake intake, Shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;

        noteInIntake.onTrue(noteInIntakeSequence());
        noteInShooter.onTrue(noteInShooterSequence());

        addRequirements(intake, shooter);
    }

    @Override
    public void initialize() {
        intake.setToGroundPositionAndEnable();
        shooter.setToPassthroughPosition();
    }

    private Command noteInIntakeSequence() {
        return intake.disableIntake()
            /* Move ring into shooter once the intake has reached the passthrough position */
            .andThen(intake.moveToPassthroughPosition())            
            /* Move ring into shooter */
            .andThen(shooter.enableStorageMotorReceiving())
            .andThen(intake.shootIntoShooter());
    }

    private Command noteInShooterSequence() {
        /* Make sure to disable motors when the ring arrives in the shooter */
        return shooter.disableStorageMotor()
        .andThen(intake.disableIntake());
    }

    @Override
    public boolean isFinished() { 
        return shooter.isBeamBreakTriggered();
    }
}