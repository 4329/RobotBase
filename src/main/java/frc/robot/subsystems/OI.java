package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import java.lang.Math;
import frc.robot.OIConstants;

public class OI extends SubsystemBase {

  private double axisValue, joystickAngle, x, y, radius;

  public double getAxis(XboxController controller, int axis) {
    if (controller == RobotContainer.driverController) {
      axisValue = RobotContainer.driverController.getRawAxis(axis);
    }
    if (controller == RobotContainer.operatorController) {
      axisValue = RobotContainer.operatorController.getRawAxis(axis);
    }
    if (Math.abs(axisValue) > OIConstants.DEADZONE) {
      return axisValue;
    } else {
      return 0;
    }
  }
}
