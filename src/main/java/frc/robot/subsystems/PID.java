package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PIDConstants;

public class PID extends SubsystemBase
{

    private double output, error;
    PIDController pidController;

    public PID()
    {
        //PID
        //Porportional = P
        //Intergal = I
        //derivative = D
        pidController = new PIDController(PIDConstants.ROTATION_P, PIDConstants.ROTATION_I, PIDConstants.ROTATION_D);
        // pidController.setInputRange(0.0, 360.0);
        // pidController.setContinuous(true);
        // pidController.setOutputRange(-1.0, 1.0);
        pidController.setTolerance(PIDConstants.TOLERANCE);
        pidController.enableContinuousInput(0, 360);
    }

    // Runs the PID loop for angles within delta 90
    public double controlRotationWithin90(double angle, SwerveModule module)
    {
        // The wheel is commanded directly to the requested angle
        pidController.setSetpoint(angle);
        output = pidController.calculate(module.getAngle());
        output = output / 90; // May need to reverse sign here
        // SmartDashboard.putNumber("Output" + module.getPotentiometerPort(), output);
        error = pidController.getPositionError();
        // SmartDashboard.putNumber("Error" + module.getPotentiometerPort(), error);
        return output;
    }

    // Runs the PID loop for angles that exceed delta 90
    public double controlRotationExceeds90(double angle, SwerveModule module)
    {
        // Rather than turning the wheel all the way around to the requested angle
        // we turn the wheel to the complementary angle and command it backwards
        angle = angle - 180;
        angle = Utilities.resolveAngle(angle);
        pidController.setSetpoint(angle);
        output = pidController.calculate(module.getAngle());
        output = output / 90; // May need to reverse sign here
        // SmartDashboard.putNumber("Output" + module.getPotentiometerPort(), output);
        error = pidController.getPositionError();
        // SmartDashboard.putNumber("Error" + module.getPotentiometerPort(), error);
        // SmartDashboard.putNumber("SwerveAngle" + module.getPotentiometerPort(),
        // module.getAngle());
        return output;
    }

}