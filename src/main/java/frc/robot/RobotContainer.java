package frc.robot;

import frc.robot.subsystems.MySubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.MyAutonomousCommand;
import frc.robot.commands.MyDefaultCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class RobotContainer {
    // The robot's subsystems
    private final MySubsystem mySubsystem = new MySubsystem();

    // The autonomous routines
    private final Command myAuto = new MyAutonomousCommand();

    // A chooser for autonomous commands
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    // The controllers
    XboxController driverController = new XboxController(0);
    XboxController operatorController = new XboxController(1);

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
        // Grab the hatch when the 'A' button is pressed.
        new Button(driverController, ).whenPressed(new GrabHatch(m_hatchSubsystem));
        // Release the hatch when the 'B' button is pressed.
        new JoystickButton(m_driverController, Button.kB.value).whenPressed(new ReleaseHatch(m_hatchSubsystem));
        // While holding the shoulder button, drive at half speed
        new JoystickButton(m_driverController, Button.kBumperRight.value).whenHeld(new HalveDriveSpeed(m_robotDrive));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
}
