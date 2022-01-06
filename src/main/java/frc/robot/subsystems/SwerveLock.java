package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class SwerveLock extends SubsystemBase
{
    public static boolean lock;
    private NetworkTableEntry lockStatus = Shuffleboard.getTab("TestValues").add("Lock Status", false).getEntry();

    public SwerveLock()
    {
        lock = false;
    }

    public void toggleLock()
    {
        if (lock == false)
        {
            setLock();
        } else
        {
            unsetLock();
        }
    }

    public void setLock()
    {
        lock = true;
        lockStatus.setBoolean(true);
    }

    public void unsetLock()
    {
        lock = false;
        lockStatus.setBoolean(false);

    }
}
