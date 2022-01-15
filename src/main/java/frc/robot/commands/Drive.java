package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Configrun;

public class Drive extends SequentialCommandGroup {
    public Drive(double distance, int angle) {
        addCommands(new RunDrivePID(distance, angle));
        System.out.println("jalapenoe");
    }
}
