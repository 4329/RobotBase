package frc.robot.commands;

import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunDrivePID extends CommandBase
{
    private final SwerveDrive swerveDrive;
    double setpoint, angle;

    // Distance in inches
    public RunDrivePID(SwerveDrive swerveDrive, double distance, int angle)
    {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
        setpoint = distance;
        this.angle = angle;
    }

    @Override
    public void initialize()
    {
        swerveDrive.resetEncoders();
        Pigeon.resetPigeon();
    }

    @Override
    public void execute()
    {
        swerveDrive.runDrivePID(setpoint, angle);
    }

    @Override
    public boolean isFinished()
    {
        return swerveDrive.isDrivePIDWithinTolerance(); // Uncomment after tuned
        // return false; // Comment after tuned
    }

    @Override
    public void end(boolean interrupted)
    {
        swerveDrive.stopDrive();
    }
}
