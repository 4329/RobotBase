package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Configrun;
import frc.robot.RobotContainer;

public class AutonomousRoutineOptions
{
    public SequentialCommandGroup driveForwardAutonomous()
    {
        return new SequentialCommandGroup(
                new InitDrivePID(RobotContainer.swerveDrive, RobotContainer.pigeonSub, 0)
                        .withTimeout(Configrun.get(1.0, "driveDelay")),
                new RunDrivePID(RobotContainer.swerveDrive, 5, 0));
    }

}
