package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Auto Chooser */
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    /* Controllers */
    private final CommandXboxController driveController = new CommandXboxController(Constants.driveControllerPort);

    /* Driver Buttons */
    private final Trigger zeroGyroButton = driveController.a().debounce(0.025);
    private final Trigger fieldCentricButton = driveController.x().debounce(0.025);
    // private final Trigger speakerScoreButton = driveController.y().debounce(0.025);
    // private final Trigger enableIntakeButton = driveController.b().debounce(0.025);

    /* Subsystems */
    // private final LimelightVision limelight = new LimelightVision(Constants.VisionConstants.limelightName, true);
    private final Swerve swerve = new Swerve(Optional::empty);//limelight::getPoseEstimate);
    // private final Intake intake = new Intake();
    // private final Shooter shooter = new Shooter();

    /* Subsystem Triggers */
    // private final Trigger intakeBeamBreakTrigger = new Trigger(intake::isBeamBreakTriggered);

    private boolean isFieldCentric = false;
  
    public RobotContainer() {
        //Will run the following command when there is no other command set, such as during teleop
        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> -driveController.getLeftY(),//The y axis is inverted by default on the xbox controller, so uninvert it
                driveController::getLeftX,
                driveController::getRightX,
                this::isFieldCentric
            )
        );

        //Tell pathplanner which commands to associate with named commands in the gui
        registerPathplannerCommands();

        //Configure the button bindings
        configureButtonBindings();

        //Configure the subsystem triggers
        configureSubsystemTriggers();

        autoChooser.addOption("Path Test Auto No Vision", new TestAutoNoVision(swerve));
        // autoChooser.addOption("1 Note Test Auto Vision", new MultiNoteAuto(swerve, limelight, shooter, intake, 1));
        // autoChooser.addOption("2 Note Test Auto Vision", new MultiNoteAuto(swerve, limelight, shooter, intake, 2));
        // autoChooser.addOption("3 Note Test Auto Vision", new MultiNoteAuto(swerve, limelight, shooter, intake, 3));
        // autoChooser.addOption("Align with april tag test", new AlignWithRotationTarget(swerve, () -> limelight.getAprilTagTarget().xOffset));
        SmartDashboard.putData("Choose an Auto:", autoChooser);//Let us choose autos through the dashboard
    }

    /* Only reset variables - don't run any commands here */
    public void autonomousInit() {
        // limelight.switchToLocalizationPipeline();//Ensures that the limelight is never stuck in the wrong pipeline
    }

    /* Only reset variables - don't run any commands here */
    public void teleopInit() {
        // limelight.switchToTargetPipeline();//Ensures that the limelight is never stuck in the wrong pipeline
    }

    private boolean isFieldCentric() {
        return isFieldCentric;
    }

    private void registerPathplannerCommands() {
        // NamedCommands.registerCommand("IntakeToGround", intake.setToGroundPosition());
        // NamedCommands.registerCommand("EnableIntake", intake.enableIntake());
        // NamedCommands.registerCommand("ShooterArmNote1", shooter.setArmTargetPosition(4));//TODO have a constant
    }

    private void configureButtonBindings() {
        /* 
         * Current controls: 
         * 
         * left stick x and y - controls strafing
         * right stick x - controls rotation 
         * 
         * a -> zero gyro
         * x -> toggle field centric
         * y [must have a valid speaker target] -> run speaker scoring sequence (align with tag, move shooter arm, shoot, reset arm)
         * b -> set intake to ground position and enable intake at the same time
         * 
        */
        zeroGyroButton.onTrue(new InstantCommand(swerve::zeroGyro, swerve));

        fieldCentricButton.onTrue(new InstantCommand(() -> isFieldCentric = !isFieldCentric));//Toggle field centric

        // speakerScoreButton.onTrue(
        //     new SpeakerScoringSequence(swerve, limelight, shooter)
        //     /* Only run if there is a valid target and it's a speaker tag */
        //     .onlyIf(() -> limelight.getAprilTagTarget().isValid && limelight.getAprilTagTarget().isSpeakerTag())
        // );

        // enableIntakeButton.onTrue(
        //     intake.setToGroundPosition()
        //     .andThen(intake.enableIntake())
        // );
    }

    private void configureSubsystemTriggers() {
        /* 
         * Current triggers:
         * 
         * intake beam break -> disable intake and move to passthrough position
         * 
         */
        // intakeBeamBreakTrigger.onTrue(
        //     intake.disableIntake()
        //     .andThen(intake.moveToPassthroughPosition())//TODO next this should run something that passes the ring into the shooter and then stops when the next beam break is triggered
        // );
    }

    /* Only return the auto command here */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
