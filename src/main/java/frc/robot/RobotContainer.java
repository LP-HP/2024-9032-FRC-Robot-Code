package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.krakenSwerveConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LEDSubsystem.LEDState;
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
    private final Trigger climberDownButton = driveController.leftTrigger(0.25).debounce(0.025);
    private final Trigger shuttleButton = driveController.x().debounce(0.025);
    private final Trigger underStageButton = driveController.rightBumper().debounce(0.025);
    private final Trigger overrideAutoAim = driveController.leftBumper().debounce(0.025);
    private final Trigger shootButton = driveController.b().debounce(0.025);
    private final Trigger trapButton = driveController.y().debounce(0.025)
        .and(overrideAutoAim.negate());

    /* Mechanism Controller Buttons */
    private final Trigger enableIntakeButton = mechanismController.b().debounce(0.025);
    private final Trigger storeNoteButton = mechanismController.a().debounce(0.025);
    private final Trigger ampScoreButton = mechanismController.rightBumper().debounce(0.025);
    private final Trigger driveToNoteButton = mechanismController.rightTrigger(0.25).debounce(0.025)
        .and(overrideAutoAim.negate());
    private final Trigger resetIntakeAndShooterButton = mechanismController.x().debounce(0.025);
    private final Trigger ejectButton = mechanismController.leftTrigger(0.25).debounce(0.025);
    private final Trigger resetButton = mechanismController.back().debounce(0.5);

    /* Subsystems */
    private final LimelightVision limelight = new LimelightVision();
    private final Photonvision photonvision = new Photonvision();
    private final Swerve swerve = new Swerve();
    private final Intake intake = new Intake();
    private final ShooterFlywheels shooterFlywheels = new ShooterFlywheels();
    private final ShooterArm shooterArm = new ShooterArm();
    private final Climbers climbers = new Climbers();
    private final LEDSubsystem leds = new LEDSubsystem(LEDState.BLUE_GRADIENT);

    /* Shuffleboard */
    private final ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
    private final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    SendableChooser<Command> autoChooser = new SendableChooser<>();

    /* Swerve */
    private final SwerveDrivetrain m_drivetrain = krakenSwerveConstants.DriveTrain;
    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
        .withDeadband(krakenSwerveConstants.maxSpeed * 0.1).withRotationalDeadband(krakenSwerveConstants.maxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    /* Teleop Triggers */
    private final Trigger autoAimSpeaker = 
        new Trigger(() -> limelight.getAprilTagTarget().isValidSpeakerTag() && shooterFlywheels.hasNote() && limelight.isTargetPipeline())
            .and(overrideAutoAim.negate()).and(underStageButton.negate()).debounce(0.025);

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
        autoChooser.setDefaultOption("5 Note Middle", swerve.getVisionLocalizationAuto("Auto Aiming", limelight::getPoseEstimate));
        autoChooser.addOption("Swerve Shakedown", AutoBuilder.buildAuto("Swerve Shakedown"));
        // autoChooser.addOption("Middle", swerve.getVisionLocalizationAuto("Start Middle", limelight::getPoseEstimate));
        autoChooser.addOption("3 Note Wide", swerve.getVisionLocalizationAuto("Start Right", limelight::getPoseEstimate));
        autoChooser.addOption("1 Note Amp Side", swerve.getVisionLocalizationAuto("Start Left", limelight::getPoseEstimate));

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
        driverTab.addBoolean("Arms at Setpoint", () -> intake.armAtSetpoint() && shooterArm.armAtSetpoint())
            .withPosition(3, 0).withSize(1, 1);
        driverTab.addBoolean("Intake Has Note", () -> intake.hasNote())
            .withPosition(4, 0).withSize(1, 1);
        driverTab.addBoolean("Shooter Has Note", () -> shooterFlywheels.hasNote())
            .withPosition(5, 0).withSize(1, 1);
        driverTab.addDouble("Match Time", () -> DriverStation.getMatchTime())
            .withPosition(6, 0).withSize(2, 1);
        driverTab.add(
                shooterArm.setToAutoPosition(148.0, true)
                .andThen(shooterFlywheels.shoot(95.0, true))
                .withName("Shoot Subwoofer"))
            .withPosition(8, 0).withSize(1, 1);
        driverTab.add(shooterFlywheels.shoot(95.0, true))
            .withPosition(9, 0).withSize(1, 1);
        limelight.addCameraToTab(driverTab, 0, 1, 4);
        photonvision.addCameraToTab(driverTab, 5, 1, 4);
        
        Shuffleboard.selectTab(driverTab.getTitle());
    }

    /* Don't run any commands here */
    public void robotPeriodic() {}

    /* Only reset variables - don't run any commands here */
    public void autonomousInit() {
        limelight.switchToLocalizationPipeline();//Ensures that the limelight is never stuck in the wrong pipeline 
    }

    /* Only reset variables - don't run any commands here */
    public void teleopInit() {
        limelight.switchToTargetPipeline();//Ensures that the limelight is never stuck in the wrong pipeline
        shooterArm.reset();
        shooterFlywheels.reset();
        intake.reset();
    }

    /* Only reset variables - don't run any commands here */
    public void disabledExit() {
        shooterArm.reset();
        shooterFlywheels.reset();
        intake.reset();
        setRumble(0.0);
    }

    private void registerPathplannerCommands() {
        NamedCommands.registerCommand("AutoInit", 
            shooterFlywheels.spinUpFlywheels(95.0)
            .andThen(intake.setToGroundPosition(false))
        );

        NamedCommands.registerCommand("Passthrough", 
            new StoreNoteSequence(intake, shooterArm, shooterFlywheels, false)
            .andThen(intake.setToGroundPosition(false))
        );

        NamedCommands.registerCommand("ShootAA", 
            shooterArm.setToTargetPositionFromDistance(() -> limelight.getAprilTagTarget().distance, () -> 0.0, true)
            .andThen(shooterFlywheels.shoot(shootVelocity, false))
        );

        NamedCommands.registerCommand("Intake", 
            shooterArm.setToPassthroughPosition(false)
            .andThen(intake.getNoteFromGround())
            .andThen(new StoreNoteSequence(intake, shooterArm, shooterFlywheels, false))
            .andThen(intake.setToGroundPosition(false))
            .withTimeout(notePickupTimeout)
        );

        NamedCommands.registerCommand("IntakeAA", 
            shooterArm.setToPassthroughPosition(false)
            .andThen(intake.getNoteFromGround()
                .deadlineWith(new DriveToNote(swerve, photonvision)))
            .withTimeout(notePickupTimeout)
        );
    }

    private void configureTriggerBindings() {
        /* 
         * Current driver controls: 
         * 
         * left stick x and y - controls strafing
         * right stick x - controls rotation 
         * 
         * right trigger -> move climbers up by trigger amount
         * left trigger -> move climbers down by trigger amount and put the shooter into climber position
         * left bumper -> override auto aim
         * right bumper (cancelable) -> go under stage
         * a -> zero gyro
         * b [must have a valid speaker target and a note in the shooter] -> shoot
         * y [must have a valid stage target] -> drive to stage
         * 
         * Current mechanism controls:
         * right bumper [must have a note in the shooter] -> score in amp
         * right trigger (cancelable) [must have a note target] -> drive to note
         * left trigger -> eject note
         * b -> [must not have a note] set intake to ground position and enable intake - when a note is gained, then move the intake to storage
         * a [must have a note in the intake] -> run store note sequence
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

        climberDownButton.onTrue(
            Commands.print("Climbers going down")
            .andThen(leds.setState(LEDState.RAINBOW))
        );

        underStageButton.onTrue(
            Commands.print("Going under stage")
            .andThen(shooterArm.setToUnderStagePosition(false))
        );

        underStageButton.onFalse(
            Commands.print("Going under stage released")
            .andThen(shooterArm.setToUpPosition(false))
        );

        /* Mechanism Controls */
        shootButton.and(autoAimSpeaker).onTrue(
            Commands.print("Shooting")
            .andThen(shooterFlywheels.shoot(95.0, true)
                .asProxy())//TODO do velocity lookup table if needed
            .andThen(shooterArm.setToUpPosition(false)
                .asProxy())
            .andThen(leds.setState(LEDState.BLUE_GRADIENT)
                .asProxy())
        );

        enableIntakeButton.onTrue(
            Commands.print("Enabling intake")
            .andThen(shooterArm.setToPassthroughPosition(false))
            .andThen(intake.getNoteFromGround())
            // .andThen(Commands.print("Transfering note"))
            // .andThen(new StoreNoteSequence(intake, shooterArm, shooterFlywheels, true)
            .andThen(setAndDisableRumble())
            .onlyIf(() -> !intake.hasNote() && !shooterFlywheels.hasNote())
        );

        driveToNoteButton.and(() -> !intake.hasNote()).whileTrue(
            Commands.print("Driving to note")
            .andThen(new DriveToNote(swerve, photonvision))
            .onlyIf(() -> !shooterFlywheels.hasNote())
        );
        
        driveToNoteButton.onFalse(
            Commands.print("Canceled driving to note")
            .andThen(intake.disableRollers())
            .andThen(intake.setToPassthroughPosition(false))
            .andThen(shooterArm.setToUpPosition(false))
            .onlyIf(() -> !intake.hasNote())
        );   

        storeNoteButton.and(underStageButton.negate()).onTrue(
            Commands.print("Transfering note")
            .andThen(new StoreNoteSequence(intake, shooterArm, shooterFlywheels, true))
            .andThen(leds.setState(LEDState.GREEN_GRADIENT))
            .onlyIf(() -> intake.hasNote() && !shooterFlywheels.hasNote())
        );

        ampScoreButton.onTrue(
            Commands.print("Shooting in amp")
            .andThen(shooterArm.setToAmpPosition(true))
            .andThen(shooterFlywheels.shootIntoAmp())
            .andThen(shooterArm.setToUpPosition(false))
            .andThen(leds.setState(LEDState.BLUE_GRADIENT))
            .onlyIf(shooterFlywheels::hasNote)
        );

        trapButton.whileTrue(
            Commands.print("Driving to trap")
            .andThen(shooterFlywheels.spinUpFlywheels(ShooterConstants.flywheelTrapSetpoint))
            .andThen(shooterArm.setToTrapPosition(false))
            .andThen(new DriveToStage(swerve, limelight))
            .andThen(Commands.waitSeconds(1.0))
            .andThen(shooterFlywheels.shootIntoTrap())
            .andThen(shooterArm.setToUpPosition(false))
            .andThen(leds.setState(LEDState.BLUE_GRADIENT))
            .onlyIf(() -> limelight.getAprilTagTarget().isValidStageTag() && shooterFlywheels.hasNote())
        );

        shuttleButton.onTrue(
            Commands.print("Shuttling note")
            .andThen(shooterArm.setToShuttlePosition(true))
            .andThen(shooterFlywheels.shoot(95.0, true))
            .andThen(leds.setState(LEDState.BLUE_GRADIENT))
            .onlyIf(shooterFlywheels::hasNote)
        );

        trapButton.onFalse(
            Commands.print("Canceled trap")
            .andThen(shooterArm.setToUpPosition(false))
        );

        /* Reset Buttons */
        ejectButton.onTrue(
            Commands.print("Ejecting note")
            .andThen(intake.ejectNote())
        );

        resetIntakeAndShooterButton.or(overrideAutoAim).onTrue(
            Commands.print("Overriding auto aim or reseting intake and shooter")
            .andThen(intake.setToPassthroughPosition(false))
            .andThen(intake.disableRollers())
            .andThen(shooterArm.setToUpPosition(false))
            .andThen(shooterFlywheels.disableFlywheels())
        );

        resetButton.onTrue(
            Commands.print("Reseting shooter and intake commands")
            .andThen(intake.resetCommand())
            .andThen(shooterArm.resetCommand())
            .andThen(shooterFlywheels.resetCommand())
            .andThen(leds.setState(LEDState.BLUE_GRADIENT))
        );

        /* Teleop Triggers */
        autoAimSpeaker.onTrue(
            new AimAtSpeakerWhileMoving(swerve, limelight, 
                () -> -driveController.getLeftY(), 
                () -> -driveController.getLeftX(),
                driveController::getRightX)
                    .until(() -> overrideAutoAim.getAsBoolean() || underStageButton.getAsBoolean() || !shooterFlywheels.hasNote())
        );

        autoAimSpeaker.and(() -> shooterFlywheels.getCurrentCommand() == null).onTrue(
            shooterFlywheels.spinUpFlywheels(95.0)
        );

        autoAimSpeaker.and(() -> shooterArm.getCurrentCommand() == null).onTrue(
            shooterArm.setToTargetPositionFromDistance(() -> limelight.getAprilTagTarget().distance, () -> swerve.getSpeeds().vxMetersPerSecond, false)
                .repeatedly()
                    .until(() -> !autoAimSpeaker.getAsBoolean() || underStageButton.getAsBoolean())
        );
    }

    private Command setAndDisableRumble() {
        return new InstantCommand(() -> setRumble(1.0))
        .andThen(Commands.waitSeconds(0.5))
        .finallyDo(() -> setRumble(0.0));
    }

    private void setRumble(double power) {
        driveController.getHID().setRumble(RumbleType.kBothRumble, power);
        mechanismController.getHID().setRumble(RumbleType.kBothRumble, power);
    }

    /* Only return the auto command here */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
