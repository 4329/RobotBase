package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.SwerveDrive;

public class InitDrivePID extends CommandBase
{
    private final SwerveDrive swerveDrive;
    double angle;

    // Distance in inches
    public InitDrivePID(SwerveDrive swerveDrive, Pigeon pigeon, int angle)
    {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
        addRequirements(pigeon);
        this.angle = angle;
    }

    @Override
    public void execute()
    {
        Pigeon.resetPigeon();
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}