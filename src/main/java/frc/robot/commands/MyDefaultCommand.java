package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MySubsystem;

public class MyDefaultCommand extends CommandBase {

    private MySubsystem mySubsystem;

    public MyDefaultCommand() {
        addRequirements(mySubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        mySubsystem.defaultMethod();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}