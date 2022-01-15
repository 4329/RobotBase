package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configrun;
import frc.robot.RobotContainer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;


public class LimelightSubsystem extends SubsystemBase {
    SmartDashboard table;
    SmartDashboard ledSmartDashboard;
    //default value for the limelight mode
    int defaultvalue = 1;
    PIDController limeLightPid;
    double h1In = Configrun.get(12.3125, "h1In");
    double h2In = Configrun.get(83, "h2In");
    double a1Degree = Configrun.get(0.0, "a1Degree");
    double limeLightDistance;
    int limeLightTolerance = Configrun.get(1, "limeLightTolerance");
    double limelightP = Configrun.get(1.2, "limelightP");
    double limelightI = Configrun.get(0, "limelightI");
    double limelightD = Configrun.get(0.125, "limelightD");
    double staticFeedforward = Configrun.get(0.103, "turnStaticFeedforward");
    NetworkTableEntry targetStatus;
    double taTolerance;
    public double currentDistance = 120;

    public LimelightSubsystem() {
        limeLightPid = new PIDController(limelightP, limelightI, limelightD);
        limeLightPid.setTolerance(limeLightTolerance);
        targetStatus = Shuffleboard.getTab("TestValues").add("Target Acquired", false).getEntry();
        taTolerance = Configrun.get(0.3, "taTolerance");
    }

    // Sets calculation distance for shooter
    public void setDistance() {
        currentDistance = getDistanceFromTarget();
    }

    public double checkTx() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        double x = tx.getDouble(0.0);
        return x;
    }

    // public void runLimelightPID() {
    //     if (!RobotContainer.shooter.isShooterOverriden()) {
    //         double output = limeLightPid.calculate(checkTx(), 0);
    //         output = output / 30;
    //         if (output < 0) {
    //             output = output - staticFeedforward;
    //         } else {
    //             output = output + staticFeedforward;
    //         }
    //         // SmartDashboard.putNumber("LimelightOutput", output);

    //         RobotContainer.swerveDrive.drive(0, 0, output);
    //         setDistance();

    //     }
    // }

    public double checkTa() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ta = table.getEntry("ta");
        double a = ta.getDouble(0.0);
        return a;
    }

    private double checkTy() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double y = ty.getDouble(0.0);
        return y;
    }

    public void putDistance() {
        limeLightDistance = (h2In - h1In) / Math.tan(Math.toRadians(a1Degree) + (Math.toRadians(checkTy())));
        SmartDashboard.putNumber("Limelight Distance", limeLightDistance);
    }

    public double getDistanceFromTarget() {
        limeLightDistance = (h2In - h1In) / Math.tan(Math.toRadians(a1Degree) + (Math.toRadians(checkTy())));
        return limeLightDistance;
    }
    // limelight distance equation
    // d = (h2-h1) / tan(a1+a2)

    public void limeLight() {

        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        // NetworkTableEntry ledSmartDashboard = table.getEntry("ledSmartDashboard");

        NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDouble(0);
        //turns the limelight on using defaulvalue
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").setNumber(defaultvalue);

        // read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        // post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

    }

    public void limeLightStop() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").setNumber(0);
    }

    public void putTargetAcquired() {
        boolean status;
        if (checkTa() >= taTolerance) {
            status = true;
        } else {
            status = false;
        }
        targetStatus.setBoolean(status);
    }
}