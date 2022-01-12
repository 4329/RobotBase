package frc.robot;

import frc.robot.subsystems.MySubsystem;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveLock;
import frc.robot.subsystems.Utilities;
import frc.robot.commands.MyCommand;
import frc.robot.commands.SwerveDriveCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer
{
    // The robot's subsystems
    private final MySubsystem mySubsystem = new MySubsystem();
    public static final OI oi = new OI();

    // The autonomous routines
    public static final SwerveLock swerveLock = new SwerveLock();
    public static final SwerveDrive swerveDrive = new SwerveDrive();
    public static final Utilities utilities = new Utilities();
    public static final Pigeon pigeonSub = new Pigeon();

    // The controllers
    public static final XboxController driverController = new XboxController(OIConstants.DRIVER_CONTROLLER);
    public static final XboxController operatorController = new XboxController(OIConstants.OPERATOR_CONTROLLER);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        //  ####mySubsystem.setDefaultCommand(new MyDefaultCommand(mySubsystem));
        swerveDrive.setDefaultCommand(new SwerveDriveCommand());
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings()
    {
        // Run MyCommand whenever the 'A' button is pressed
        new JoystickButton(driverController, OIConstants.A_BUTTON).whenPressed(new MyCommand(mySubsystem));
    }
}
