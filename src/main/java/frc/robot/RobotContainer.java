package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.util.SparkMaxWrapper;

import static frc.robot.Constants.AutoConstants.*;

import java.util.Optional;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private boolean shouldOverridePathplannerRotation = false;

    /* Controllers */
    private final CommandXboxController driveController = new CommandXboxController(Constants.driveControllerPort);
    private final CommandXboxController mechanismController = new CommandXboxController(Constants.mechanismControllerPort);

    /* Drive Controller Buttons */
    private final Trigger zeroGyroButton = driveController.a().debounce(0.025);

    /* Mechanism Controller Buttons */
    private final Trigger speakerScoreButton = mechanismController.rightBumper().debounce(0.025);
    private final Trigger enableIntakeButton = mechanismController.b().debounce(0.025);
    private final Trigger storeNoteButton = mechanismController.a().debounce(0.025);
    private final Trigger ampScoreButton = mechanismController.y().debounce(0.025);
    private final Trigger storageButton = mechanismController.x().debounce(0.025);
    private final Trigger speakerAimButton = mechanismController.leftBumper().debounce(0.025);
    private final Trigger getNoteButton = mechanismController.rightTrigger(0.25).debounce(0.025);
    private final Trigger resetButton = mechanismController.back().debounce(1.0);

    /* Subsystems */
    private final LimelightVision limelight = new LimelightVision();
    private final Photonvision photonvision = new Photonvision();
    private final Swerve swerve = new Swerve();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Climbers climbers = new Climbers();

    /* Shuffleboard */
    private final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        //Will run the following command when there is no other command set, such as during teleop
        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> -driveController.getLeftY(),//The axes are inverted by default on the xbox controller, so uninvert them
                () -> -driveController.getLeftX(),
                driveController::getRightX
            )
        );

        //Tell pathplanner which commands to associate with named commands in the gui
        registerPathplannerCommands();

        //Configure the button bindings
        configureButtonBindings();

        /* Add auto chooser */
        autoChooser.addOption("Swerve Auto Shakedown", AutoBuilder.buildAuto("SwerveShakedown"));
        autoChooser.addOption("Aiming Auto", 
            swerve.addOptionalVisionPoseSupplier(limelight::getPoseEstimate)
            .andThen(AutoBuilder.buildAuto("Aiming Auto"))
        );

        driverTab.add(autoChooser);
        driverTab.add(
            swerve.addOptionalVisionPoseSupplier(limelight::getPoseEstimate)
            .andThen(swerve.resetOdometryCommand(() -> limelight.getPoseEstimate().get().pose))
            .withName("Add pose sup")
        );

        /* Add driver tab telemetry */
        driverTab.addBoolean("No Motor Errors", () -> SparkMaxWrapper.noMotorErrors());
    }

    /* Only reset variables - don't run any commands here */
    public void autonomousInit() {
        limelight.switchToLocalizationPipeline();//Ensures that the limelight is never stuck in the wrong pipeline 
    }

    /* Only reset variables - don't run any commands here */
    public void teleopInit() {
        limelight.switchToTargetPipeline();//Ensures that the limelight is never stuck in the wrong pipeline
    }

    /* Only reset variables - don't run any commands here */
    public void disabledExit() {
        shooter.reset();
        intake.reset();
    }

    private void registerPathplannerCommands() {
        NamedCommands.registerCommand("ShootAA", 
            shooter.shootSequenceWithDistanceLockOn(shootVelocity, () -> limelight.getAprilTagTarget().distance)
            .andThen(new InstantCommand(() -> shouldOverridePathplannerRotation = false))
        );

        NamedCommands.registerCommand("IntakeAA", 
            intake.getNoteFromGround()
                .deadlineWith(new AlignWithVisionTarget(swerve, photonvision, false, false))
            .andThen(Commands.waitSeconds(passthroughWait))
            .andThen(new StoreNoteSequence(intake, shooter))
            .withTimeout(notePickupTimeout)
        );

        // NamedCommands.registerCommand("Align", 
        //     new InstantCommand(() -> shouldOverridePathplannerRotation = true)
        //     .andThen(shooter.setToTargetPositionFromDistance(() -> limelight.getAprilTagTarget().distance, false)
        //         .repeatedly())
        // );

        // PPHolonomicDriveController.setRotationTargetOverride(this::pathplannerRotationOverride);
    }

    private Optional<Rotation2d> pathplannerRotationOverride() {
        return shouldOverridePathplannerRotation && limelight.getAprilTagTarget().isValid ? 
            Optional.of(Rotation2d.fromDegrees(limelight.getAprilTagTarget().xOffset).plus(swerve.getPose().getRotation())) : 
            Optional.empty();
    }

    private void configureButtonBindings() {
        /* 
         * Current driver controls: 
         * 
         * left stick x and y - controls strafing
         * right stick x - controls rotation 
         * right trigger -> move climbers up by trigger amount
         * left trigger -> move climbers down by trigger amount
         * a -> zero gyro
         * 
         * Current mechanism controls:
         * right bumper [must have a valid speaker target and a note] -> run speaker scoring sequence (align with tag, move shooter arm, shoot, reset arm)
         * left bumper [must have a valid speaker target] -> aim swerve for april tag
         * b -> [must not have a note] set intake to ground position and enable intake - when a note is gained, then move the intake to storage
         * a [must have a note in the intake] -> run store note sequence
         * y [must have a note in the intake] -> move intake to amp position and shoot into amp
         * x -> set intake and shooter to the default positions
         * back [hold 1 second] -> reset states
        */

        /* Driver Controls */
        zeroGyroButton.onTrue(new InstantCommand(swerve::zeroGyro, swerve));

        climbers.setDefaultCommand(
            climbers.setClimberPower(
                () -> driveController.getRightTriggerAxis() - driveController.getLeftTriggerAxis()
            )
        );

        /* Mechanism Controls */
        speakerScoreButton.and(speakerAimButton).onTrue(
            shooter.shootSequenceWithDistanceLockOn(95.0, () -> limelight.getAprilTagTarget().distance)//TODO do lookup table if needed
             /* Only run if there is a valid target and it's a speaker tag and we have a note */
            .onlyIf(() -> limelight.getAprilTagTarget().isValidSpeakerTag() && shooter.hasNote())
        );

        enableIntakeButton.onTrue(
            shooter.setToPassthroughPosition(false)
            .andThen(intake.getNoteFromGround())
            .andThen(setAndDisableRumble())
            .onlyIf(() -> !intake.hasNote() && !shooter.hasNote() && !shooter.isShooting())
        );

        getNoteButton.and(() -> !intake.hasNote() && photonvision.hasTargets()).whileTrue(
            new AlignWithVisionTarget(swerve, photonvision, false, false)
        );   

        storeNoteButton.and(speakerAimButton.negate()).onTrue(
            new StoreNoteSequence(intake, shooter)
            .onlyIf(() -> intake.hasNote() && !shooter.hasNote())
        );
 
        ampScoreButton.onTrue(
            intake.shootIntoAmp()
            .onlyIf(intake::hasNote)
        );

        resetButton.onTrue(
            intake.resetCommand()
            .andThen(shooter.resetCommand())
        );

        speakerAimButton.whileTrue(
            new LockToVisionTargetWhileMoving(swerve, limelight, 
                () -> -driveController.getLeftY(), 
                () -> -driveController.getLeftX(),
                driveController::getRightX)
            .onlyIf(shooter::hasNote)
        );

        speakerAimButton.whileTrue(
            shooter.setToTargetPositionFromDistance(() -> limelight.getAprilTagTarget().distance, false)
                .repeatedly()
            .onlyIf(shooter::hasNote)
        );

        storageButton.onTrue(
            intake.setToPassthroughPosition(false)
            .andThen(intake.disableFlywheels())
            .andThen(shooter.setToStoragePosition(false))
            .onlyIf(() -> !intake.hasNote())
        );
    }

    private Command setAndDisableRumble() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> driveController.getHID().setRumble(RumbleType.kBothRumble, 1)),
            Commands.waitSeconds(0.25),
            new InstantCommand(() -> driveController.getHID().setRumble(RumbleType.kBothRumble, 0))
        );
    }

    /* Only return the auto command here */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
