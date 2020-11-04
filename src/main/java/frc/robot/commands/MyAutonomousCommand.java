package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MySubsystem;

public class MyAutonomousCommand extends CommandBase {

    private MySubsystem mySubsystem;

    public MyAutonomousCommand() {
        addRequirements(mySubsystem);
    }

    @Override
    public void initialize() {
        mySubsystem.subsystemMethod();
    }

    @Override
    public void execute() {
        mySubsystem.autonomousMethod();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}