package frc.robot;

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
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.util.SparkMaxWrapper;

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

    /* Mechanism Controller Buttons */
    private final Trigger speakerScoreButton = mechanismController.rightBumper().debounce(0.025);
    private final Trigger enableIntakeButton = mechanismController.b().debounce(0.025);
    private final Trigger storeNoteButton = mechanismController.a().debounce(0.025);
    private final Trigger ampScoreButton = mechanismController.leftBumper().debounce(0.025);
    private final Trigger resetButton = mechanismController.back().debounce(1.0);
    // private final Trigger aprilTagAlignmentTest = driveController.x().debounce(0.025);//TODO remove

    /* Subsystems */
    private final LimelightVision limelight = new LimelightVision();
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
        autoChooser.addOption("Swerve Auto Shakedown", new SwerveShakedown(swerve));
        // autoChooser.addOption("1 Note Test Auto Vision", new MultiNoteAuto(swerve, limelight, shooter, intake, 1));
        // autoChooser.addOption("2 Note Test Auto Vision", new MultiNoteAuto(swerve, limelight, shooter, intake, 2));
        // autoChooser.addOption("3 Note Test Auto Vision", new MultiNoteAuto(swerve, limelight, shooter, intake, 3));
        autoChooser.addOption("Rotate To April Tag", new AlignWithVisionTarget(swerve, limelight, true, false));

        driverTab.add(autoChooser);

        /* Add driver tab telemetry */
        driverTab.addBoolean("Has Any Motor Errors", SparkMaxWrapper::hasAnyMotorErrors);
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
        // NamedCommands.registerCommand("IntakeToGround", intake.setToGroundPosition());
        // NamedCommands.registerCommand("EnableIntake", intake.enableIntake());
        // NamedCommands.registerCommand("ShooterArmNote1", shooter.setArmTargetPosition(4));//TODO maybe don't register here
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
         * b -> [must not have a note] set intake to ground position and enable intake - when a note is gained, then move the intake to storage
         * a [must have a note in the intake] -> run store note sequence
         * left bumper [must have a note in the intake] -> move intake to amp position and shoot into amp
         * back [hold 1 second ] -> reset states
         * 
        */
        zeroGyroButton.onTrue(new InstantCommand(swerve::zeroGyro, swerve));

        speakerScoreButton.onTrue(
            new SpeakerScoringSequence(swerve, limelight, shooter)
             /* Only run if there is a valid target and it's a speaker tag and we have a note */
            .onlyIf(() -> limelight.getAprilTagTarget().isValid && limelight.getAprilTagTarget().isSpeakerTag() && shooter.hasNote())
        );

        enableIntakeButton.onTrue(
            shooter.setToPassthroughPosition(false)
            .andThen(intake.getNoteFromGround())
            .andThen(setAndDisableRumble())
            .onlyIf(() -> !intake.hasNote() && !shooter.hasNote())
        );

        storeNoteButton.onTrue(
            new StoreNoteSequence(intake, shooter)
            .onlyIf(() -> intake.hasNote() && !shooter.hasNote())
        );

        ampScoreButton.onTrue(
            intake.shootIntoAmp()
            .onlyIf(intake::hasNote)
        );

        climbers.setDefaultCommand(
            climbers.setClimberPower(
                () -> driveController.getRightTriggerAxis() - driveController.getLeftTriggerAxis()
            )
        );

        resetButton.onTrue(
            intake.resetState()
            .andThen(shooter.resetState())
        );
        
        // aprilTagAlignmentTest.onTrue(//TODO move to other class
        //     new LockToRotationTargetWhileMoving(swerve, 
        //         () -> limelight.getAprilTagTarget().xOffset, 
        //         () -> 
        //             new Translation2d(driveController.getLeftX(), -driveController.getLeftY())
        //             .times(Constants.TeleopConstants.joystickToSpeedConversionFactor))
        // );
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
