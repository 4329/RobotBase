package frc.robot.subsystems;
import frc.robot.Configrun;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
        Shuffleboard.getTab("Pidgeon Info").add("yawPitchRole[0]",yawPitchRole[0]).withWidget(BuiltInWidgets.kTextView).getEntry();

    }

    public void putPigeon()
    {
        //SmartDashboard.putNumber("Pigeon", getYaw());
        Shuffleboard.getTab("Pidgeon Info").add("getYaw",getYaw()).withWidget(BuiltInWidgets.kTextView).withPosition(1,0).getEntry();
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
