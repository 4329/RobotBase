package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

import java.lang.Math;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModuleConstants;
import frc.robot.RobotContainer;

public class SwerveModule extends SubsystemBase
{
  private CANSparkMax translationMotor, rotationMotor;
  private AnalogPotentiometer potentiometer;
  private PID rotationController;
  private double offset, xResultant, yResultant;
  private int potentiometerPort;
  public static boolean fieldOrientedControl = false;

  public SwerveModule(int translationPort, int rotationPort, int potentiometerPort, double offset)
  {
    translationMotor = new CANSparkMax(translationPort, MotorType.kBrushless);
    translationMotor.setInverted(false);
    translationMotor.setSmartCurrentLimit(SwerveModuleConstants.CURRENT_LIMIT_STALL,
        SwerveModuleConstants.CURRENT_LIMIT_FREE);
    translationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, SwerveModuleConstants.PERIODIC_FRAME_PERIOD);
    translationMotor.setCANTimeout(SwerveModuleConstants.CAN_TIMEOUT);
    translationMotor.setIdleMode(IdleMode.kBrake);
    translationMotor.burnFlash();

    rotationMotor = new CANSparkMax(rotationPort, MotorType.kBrushless);
    rotationMotor.setInverted(false);
    rotationMotor.setSmartCurrentLimit(SwerveModuleConstants.CURRENT_LIMIT_STALL,
        SwerveModuleConstants.CURRENT_LIMIT_FREE);
    rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, SwerveModuleConstants.PERIODIC_FRAME_PERIOD);
    rotationMotor.setCANTimeout(SwerveModuleConstants.CAN_TIMEOUT);
    translationMotor.setIdleMode(IdleMode.kBrake);
    rotationMotor.burnFlash();
    // cringe
    potentiometer = new AnalogPotentiometer(potentiometerPort, 360, 0);

    rotationController = new PID();
    this.offset = offset;
    this.potentiometerPort = potentiometerPort;
  }

  public void driveModule(double speed, double angle, SwerveModule module)
  {
    speed *= speed; // Speed is squared for driver
    
    SwerveModuleState state = rotationController.controlRotationExceeds90(angle, module, speed);

    module.translationMotor.set(state.speedMetersPerSecond);
    module.rotationMotor.set(state.angle.getDegrees());
  }

  public void driveModuleTeleop(SwerveModule module, double xTranslation, double yTranslation, double rotation)
  {
    // yTranslation = yTranslation * -1;
    double angleTemp = RobotContainer.utilities.getAngle(xTranslation, yTranslation);
    double radiusTemp = RobotContainer.utilities.getRadius(xTranslation, yTranslation);

    if (fieldOrientedControl)
    {
      angleTemp = angleTemp - RobotContainer.pigeonSub.getYaw();
    }

    angleTemp = Utilities.resolveAngle(angleTemp);
    angleTemp = Math.toRadians(angleTemp);
    xTranslation = radiusTemp * Math.cos(angleTemp);
    yTranslation = radiusTemp * Math.sin(angleTemp);

    if (module == RobotContainer.swerveDrive.frontRight)
    {
      xResultant = xTranslation + rotation / Math.sqrt(2);
      yResultant = yTranslation - rotation / Math.sqrt(2);
    }
    if (module == RobotContainer.swerveDrive.frontLeft)
    {
      xResultant = xTranslation + rotation / Math.sqrt(2);
      yResultant = yTranslation + rotation / Math.sqrt(2);
    }
    if (module == RobotContainer.swerveDrive.backLeft)
    {
      xResultant = xTranslation - rotation / Math.sqrt(2);
      yResultant = yTranslation + rotation / Math.sqrt(2);
    }
    if (module == RobotContainer.swerveDrive.backRight)
    {
      xResultant = xTranslation - rotation / Math.sqrt(2);
      yResultant = yTranslation - rotation / Math.sqrt(2);
    }
    double driveRadius = RobotContainer.utilities.getRadius(xResultant, yResultant);
    double driveAngle = RobotContainer.utilities.getAngle(xResultant, yResultant);
    // SmartDashboard.putNumber("DriveRadius" + module.getPotentiometerPort(),
    // driveRadius);
    // SmartDashboard.putNumber("DriveAngle" + module.getPotentiometerPort(),
    // driveAngle);
    driveModule(driveRadius, driveAngle, module);
  }

  // Returns a double between 0 and 360; left=180, right=0, forward=90
  public double getAngle()
  {
    double potentiometerAngle = potentiometer.get() - offset;
    potentiometerAngle = potentiometerAngle * -1;
    potentiometerAngle = Utilities.resolveAngle(potentiometerAngle);
    return potentiometerAngle;
  }

  public double getRawAngle()
  {
    return potentiometer.get();
  }

  public int getPotentiometerPort()
  {
    return this.potentiometerPort;
  }

  public void stopModule(SwerveModule module)
  {
    module.translationMotor.set(0);
    module.rotationMotor.set(0);
  }

  public double encoderTranslationGet(SwerveModule module)
  {
    double position = module.translationMotor.getEncoder().getPosition();
    return Math.abs(position);
  }

  public void resetEncoder(SwerveModule module)
  {
    module.translationMotor.getEncoder().setPosition(0);
  }
}
