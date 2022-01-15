package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Configrun;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;
public class AutonomousRoutineOptions
{
    public SequentialCommandGroup driveForwardAutonomous()
    {
        System.out.println("horhey");
        return new SequentialCommandGroup(
            new Drive(36, SwerveDrive.FORWARD));
               // new InitDrivePID(RobotContainer.swerveDrive, RobotContainer.pigeonSub, 0)
               //         .withTimeout(Configrun.get(1.0, "driveDelay")),
               // new RunDrivePID(RobotContainer.swerveDrive, 5, 0));
        
    }

}
