/*package frc.robot.commands;

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
*/
package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Pigeon;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunDrivePID extends CommandBase {
    double setpoint, angle;

    // Distance in inches
    public RunDrivePID(double distance, int angle) {
        addRequirements(RobotContainer.swerveDrive);
        setpoint = distance;
        this.angle = angle;
    }

    @Override
    public void initialize() {
        RobotContainer.swerveDrive.resetEncoders();
        Pigeon.resetPigeon();
    }

    @Override
    public void execute() {
        RobotContainer.swerveDrive.runDrivePID(setpoint, angle);
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.swerveDrive.isDrivePIDWithinTolerance();
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.swerveDrive.stopDrive();
    }
}