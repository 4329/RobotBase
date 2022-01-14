/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AutonomousRoutineOptions;
import frc.robot.subsystems.Pigeon;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private CommandBase m_autoSelected;
  private final SendableChooser<CommandBase> m_chooser = new SendableChooser<>();
  RobotContainer robotContainer = new RobotContainer();
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    AutonomousRoutineOptions autonomousRoutineOptions = new AutonomousRoutineOptions();

    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Simple Auto", autonomousRoutineOptions.driveForwardAutonomous());
    // m_chooser.addOption("Complex Auto", m_complexAuto);

    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);

    //RobotContainer.swerveDrive.resetEncoders();
    Configrun.loadconfig();
    //Experimenting with Shuffleboard below
    Shuffleboard.getTab("Tab Test").add("test",7).withWidget(BuiltInWidgets.kTextView).withPosition(0, 0).getEntry();
    Shuffleboard.selectTab("Tab Test").add("test",8).withWidget(BuiltInWidgets.kTextView).withPosition(3, 5).getEntry();
    Shuffleboard.getTab("Size Test").add("test",4329).withWidget(BuiltInWidgets.kTextView).withPosition(0, 0).withSize(3,3).getEntry();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and  integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    Pigeon.resetPigeon();
    RobotContainer.swerveDrive.resetEncoders();
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {/* TODO FIX THIS
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }*/
  }

  /**
   * This function is called once when teleop is enabled.
   */
  @Override
  public void teleopInit()
   {
    RobotContainer.swerveDrive.resetEncoders();
    Pigeon.resetPigeon();
    RobotContainer.swerveLock.unsetLock();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  /**
   * This function is called once when the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  /**
   * This function is called periodically when disabled.
   */
  @Override
  public void disabledPeriodic() {
  }

  /**
   * This function is called once when test mode is enabled.
   */
  @Override
  public void testInit() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
