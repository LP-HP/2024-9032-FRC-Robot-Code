package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
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
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    /* Controllers */
    private final CommandXboxController driveController = new CommandXboxController(Constants.driveControllerPort);//TODO remap controls

    /* Driver Buttons */
    private final Trigger zeroGyroButton = driveController.a().debounce(0.1);
    private final Trigger fieldCentricButton = driveController.x().debounce(0.1);

    /* Subsystems */
    private final LimelightVision limelight = new LimelightVision(Constants.VisionConstants.limelightName, true);
    private final Swerve swerve = new Swerve(() -> limelight.getPoseEstimate());

    private boolean isFieldCentric = false;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        swerve.setDefaultCommand(//Will run when there is no command set, such as during teleop
            new TeleopSwerve(
                swerve, 
                () -> -driveController.getLeftY(),//The y axis is inverted by default on the xbox controller
                driveController::getLeftX,
                driveController::getRightX,
                this::isFieldCentric
            )
        );

        //Configure the button bindings
        configureButtonBindings();

        autoChooser.addOption("Test Auto 1 ring", new TestAuto(swerve, limelight));
        autoChooser.addOption("Test Auto 2 rings", new TestAuto(swerve, limelight));
        SmartDashboard.putData("Choose an Auto:", autoChooser);//Let us choose autos through the dashboard
    }

    private boolean isFieldCentric() {
        return isFieldCentric;
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* 
         * Current controls:
         * 
         * left stick controls strafing
         * right stick controls 
         * 
         * a -> zero gyro
         * x -> toggle field centric
         * 
        */
        zeroGyroButton.onTrue(new InstantCommand(swerve::zeroGyro, swerve));
        fieldCentricButton.onTrue(new InstantCommand(() -> isFieldCentric = !isFieldCentric));//Toggle field centric
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
