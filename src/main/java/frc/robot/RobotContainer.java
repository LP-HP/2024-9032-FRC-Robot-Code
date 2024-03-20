package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.Constants.TeleopConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.util.SparkMaxWrapper;

import static frc.robot.Constants.AutoConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController driveController = new CommandXboxController(Constants.driveControllerPort);
    private final CommandXboxController mechanismController = new CommandXboxController(Constants.mechanismControllerPort);

    /* Drive Controller Buttons */
    private final Trigger zeroGyroButton = driveController.a().debounce(0.025);
    private final Trigger underStageButton = driveController.rightBumper().debounce(0.025);
    private final Trigger overrideAutoAim = driveController.leftBumper().debounce(0.025);

    /* Mechanism Controller Buttons */
    private final Trigger shootButton = mechanismController.rightBumper().debounce(0.025);
    private final Trigger enableIntakeButton = mechanismController.b().debounce(0.025);
    private final Trigger storeNoteButton = mechanismController.a().debounce(0.025);
    private final Trigger ampScoreButton = mechanismController.y().debounce(0.025);
    private final Trigger noteAimButton = mechanismController.rightTrigger(0.25).debounce(0.025)
        .and(overrideAutoAim.negate());
    private final Trigger resetIntakeAndShooterButton = mechanismController.x().debounce(0.025);
    private final Trigger ejectButton = mechanismController.leftTrigger(0.25).debounce(0.025);
    private final Trigger resetButton = mechanismController.back().debounce(0.5);

    /* Subsystems */
    private final LimelightVision limelight = new LimelightVision();
    private final Photonvision photonvision = new Photonvision();
    private final Swerve swerve = new Swerve();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Climbers climbers = new Climbers();

    /* Shuffleboard */
    private final ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
    private final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    SendableChooser<Command> autoChooser = new SendableChooser<>();

    /* Teleop Triggers */
    private final Trigger autoAimSpeaker = 
        new Trigger(() -> limelight.getAprilTagTarget().isValidSpeakerTag() && limelight.isTargetPipeline() && shooter.hasNote())
            .and(overrideAutoAim.negate()).and(underStageButton.negate());

    public RobotContainer() {
        /* Will run the following command when there is no other command set, such as during teleop */
        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                /* The axes are inverted by default on the xbox controller, so uninvert them */
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                driveController::getRightX
            )
        );

        /* Tell pathplanner which commands to associate with named commands in the gui */
        registerPathplannerCommands();

        /* Configure the button bindings */
        configureTriggerBindings();

        /* Add auto chooser */
        autoChooser.setDefaultOption("Middle", swerve.getVisionLocalizationAuto("Start Middle", limelight::getPoseEstimate));
        autoChooser.addOption("Swerve Shakedown", AutoBuilder.buildAuto("Swerve Shakedown"));
        autoChooser.addOption("Middle", swerve.getVisionLocalizationAuto("Start Middle", limelight::getPoseEstimate));
        autoChooser.addOption("Wide", swerve.getVisionLocalizationAuto("Start Right", limelight::getPoseEstimate));
        autoChooser.addOption("Skinny", swerve.getVisionLocalizationAuto("Start Left", limelight::getPoseEstimate));
        autoChooser.addOption("Aiming", swerve.getVisionLocalizationAuto("Auto Aiming", limelight::getPoseEstimate));

        /* Add debug tab telemetry */
        debugTab.add(
            swerve.addOptionalVisionPoseSupplier(limelight::getPoseEstimate)
            .andThen(swerve.resetOdometryCommand(() -> limelight.getPoseEstimate().get().pose))
            .withName("Add pose sup")
        );

        /* Add driver tab telemetry */
        driverTab.addBoolean("No Motor Errors", () -> SparkMaxWrapper.noMotorErrors())
        .   withPosition(0, 0).withSize(1, 1);
        driverTab.add("Choose Auto", autoChooser)
            .withPosition(1, 0).withSize(1, 1);
        driverTab.addBoolean("Valid Tag", () -> limelight.getAprilTagTarget().isValid)
            .withPosition(2, 0).withSize(1, 1);
        driverTab.addBoolean("Arms at Setpoint", () -> intake.armAtSetpoint() && shooter.armAtSetpoint())
            .withPosition(3, 0).withSize(1, 1);
        driverTab.addBoolean("Intake Has Note", () -> intake.hasNote())
            .withPosition(4, 0).withSize(1, 1);
        driverTab.addBoolean("Shooter Has Note", () -> shooter.hasNote())
            .withPosition(5, 0).withSize(1, 1);
        driverTab.addDouble("Match Time", () -> DriverStation.getMatchTime())
            .withPosition(6, 0).withSize(2, 1);
        limelight.addCameraToTab(driverTab, 0, 1, 4);
        photonvision.addCameraToTab(driverTab, 5, 1, 4);
        Shuffleboard.selectTab(driverTab.getTitle());
    }

    /* Only reset variables - don't run any commands here */
    public void autonomousInit() {
        limelight.switchToLocalizationPipeline();//Ensures that the limelight is never stuck in the wrong pipeline 
    }

    /* Only reset variables - don't run any commands here */
    public void teleopInit() {
        limelight.switchToTargetPipeline();//Ensures that the limelight is never stuck in the wrong pipeline
        shooter.reset();
        intake.reset();
    }

    /* Only reset variables - don't run any commands here */
    public void disabledExit() {
        shooter.reset();
        intake.reset();
    }

    private void registerPathplannerCommands() {
        NamedCommands.registerCommand("ShootAA", 
            shooter.shootSequenceWithDistanceLockOn(shootVelocity, () -> limelight.getAprilTagTarget().distance)
        );

        NamedCommands.registerCommand("Intake", 
            shooter.setToPassthroughPosition(false)
            .andThen(intake.getNoteFromGround())
            .andThen(Commands.waitSeconds(passthroughWait))
            .andThen(new StoreNoteSequence(intake, shooter))
            .withTimeout(notePickupTimeout)
        );

        NamedCommands.registerCommand("IntakeAA", 
            shooter.setToPassthroughPosition(false)
            .andThen(intake.getNoteFromGround()
                .deadlineWith(new AlignWithVisionTarget(swerve, photonvision, false, false)))
            .andThen(Commands.waitSeconds(passthroughWait))
            .andThen(new StoreNoteSequence(intake, shooter))
            .withTimeout(notePickupTimeout)
        );
    }

    private void configureTriggerBindings() {
        /* 
         * Current driver controls: 
         * 
         * left stick x and y - controls strafing
         * right stick x - controls rotation 
         * right trigger -> move climbers up by trigger amount
         * left trigger -> move climbers down by trigger amount and put the shooter into climber position
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

        underStageButton.onTrue(
            shooter.setToUnderStagePosition(false)
            .andThen(shooter.disableFlywheels())
        );

        underStageButton.onFalse(shooter.setToUpPosition(false));

        /* Mechanism Controls */
        shootButton.onTrue(
            shooter.shootSequenceWithDistanceLockOn(95.0, () -> limelight.getAprilTagTarget().distance)
            .onlyIf(shooter::hasNote)//TODO do velocity lookup table if needed
        );

        enableIntakeButton.onTrue(
            shooter.setToPassthroughPosition(false)
            .andThen(intake.getNoteFromGround())
            .andThen(setAndDisableRumble())
            .onlyIf(() -> !intake.hasNote())
        );

        noteAimButton.whileTrue(
            new AlignWithVisionTarget(swerve, photonvision, false, false)
            .onlyIf(() -> photonvision.hasTargets() && !intake.hasNote())
        );
        
        noteAimButton.onFalse(
            intake.disableRollers()
            .andThen(intake.setToPassthroughPosition(false))
            .andThen(shooter.setToUpPosition(false))
            .onlyIf(() -> !intake.hasNote())
        );   

        storeNoteButton.onTrue(
            new StoreNoteSequence(intake, shooter)
            .onlyIf(() -> intake.hasNote() && !shooter.hasNote())
        );
 
        ampScoreButton.onTrue(
            intake.shootIntoAmp()
            .onlyIf(intake::hasNote)
        );

        /* Reset Buttons */
        ejectButton.onTrue(intake.ejectNote());

        resetIntakeAndShooterButton.or(overrideAutoAim).onTrue(
            intake.setToPassthroughPosition(false)
            .andThen(intake.disableRollers())
            .andThen(shooter.setToUpPosition(false))
        );

        resetButton.onTrue(
            intake.resetCommand()
            .andThen(shooter.resetCommand())
        );

        /* Teleop Triggers */
        autoAimSpeaker.whileTrue(
            new LockToVisionTargetWhileMoving(swerve, limelight, 
                () -> -driveController.getLeftY(), 
                () -> -driveController.getLeftX(),
                driveController::getRightX)
        );

        autoAimSpeaker.and(() -> shooter.getCurrentCommand() == null).whileTrue(
            shooter.spinUpFlywheels(TeleopConstants.flywheelIdleVelocity)
            .andThen(shooter.setToTargetPositionFromDistance(() -> limelight.getAprilTagTarget().distance, false)
                .repeatedly())
        );
    }

    private Command setAndDisableRumble() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> driveController.getHID().setRumble(RumbleType.kBothRumble, 1)),
            Commands.waitSeconds(0.5),
            new InstantCommand(() -> driveController.getHID().setRumble(RumbleType.kBothRumble, 0))
        );
    }

    /* Only return the auto command here */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
