package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.RobotContainer;
import frc.robot.OIConstants;

public class SwerveDriveCommand extends CommandBase
{

    private final SwerveDrive swerveDrive;

    public SwerveDriveCommand()
    {
        swerveDrive = RobotContainer.swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        double xTranslation = RobotContainer.oi.getAxis(RobotContainer.driverController, OIConstants.LEFT_STICK_X);
        double yTranslation = -RobotContainer.oi.getAxis(RobotContainer.driverController, OIConstants.LEFT_STICK_Y);
        double rotation = RobotContainer.oi.getAxis(RobotContainer.driverController, OIConstants.RIGHT_STICK_X);
        // SmartDashboard.putNumber("xTranslation", xTranslation);
        // SmartDashboard.putNumber("yTranslation", yTranslation);
        // SmartDashboard.putNumber("rotation", rotation);
        swerveDrive.drive(xTranslation, yTranslation, rotation);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

    @Override
    public void end(boolean interrupted)
    {

    }
}