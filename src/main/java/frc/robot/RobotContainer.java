package frc.robot;

import frc.robot.subsystems.MySubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.MyAutonomousCommand;
import frc.robot.commands.MyCommand;
import frc.robot.commands.MyDefaultCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
    // The robot's subsystems
    private final MySubsystem mySubsystem = new MySubsystem();

    // The autonomous routines
    private final Command myAuto = new MyAutonomousCommand();

    // A chooser for autonomous commands
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    // The controllers
    XboxController driverController = new XboxController(Constants.OIConstants.DRIVER_CONTROLLER);
    XboxController operatorController = new XboxController(Constants.OIConstants.OPERATOR_CONTROLLER);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        mySubsystem.setDefaultCommand(new MyDefaultCommand());

        // Add commands to the autonomous command chooser
        m_chooser.setDefaultOption("Simple Auto", myAuto);
        // m_chooser.addOption("Complex Auto", m_complexAuto);

        // Put the chooser on the dashboard
        Shuffleboard.getTab("Autonomous").add(m_chooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Run MyCommand whenever the 'A' button is pressed
        new JoystickButton(driverController, edu.wpi.first.wpilibj.XboxController.Button.kA.value)
                .whenPressed(new MyCommand());
    }
}
