package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class RunTestMotorCommand extends InstantCommand {
    public RunTestMotorCommand() {
        super(RobotContainer.currentTestMotor::runTestMotor, RobotContainer.currentTestMotor);
    
    }
}