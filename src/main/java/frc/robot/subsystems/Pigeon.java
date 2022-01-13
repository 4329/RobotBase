//https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/advanced-usage/shuffleboard-commands-subsystems.html?highlight=displaying%20shuffleboard
package frc.robot.subsystems;

import frc.robot.Configrun;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pigeon extends SubsystemBase {
    private static PigeonIMU pigeon;
    private double[] yawPitchRole = new double[3];
    private double yaw;

    public Pigeon()
    {
        pigeon = new PigeonIMU(Configrun.get(9, "Pigeon_ID"));
    }

    public void putRawPigeon()
    {
        pigeon.getYawPitchRoll(yawPitchRole);
        //SmartDashboard.putNumber("Pigeon", yawPitchRole[0]);
        Shuffleboard.getTab("Alpha").add("Pidgeon",yawPitchRole[0]).withWidget(BuiltInWidgets.kTextView).withPosition(0, 1).getEntry();
    }

    public void putPigeon()
    {
        SmartDashboard.putNumber("Pigeon", getYaw());
    }

    // Returns yaw Right: 90 Left: 27
    public double getYaw()
    {
        pigeon.getYawPitchRoll(yawPitchRole);
        yaw = yawPitchRole[0];
        yaw = Utilities.resolveAngle(yaw);
        return yaw;
    }

    public static void resetPigeon()
    {
        pigeon.setYaw(0);
    }
}
