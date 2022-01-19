package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PIDConstants;

public class PID extends SubsystemBase
{

    private double output, error;
    PIDController pidController;
    int testUnder90;

    public PID()
    {
        pidController = new PIDController(PIDConstants.ROTATION_P, PIDConstants.ROTATION_I, PIDConstants.ROTATION_D);
        pidController.setTolerance(PIDConstants.TOLERANCE);
        pidController.enableContinuousInput(0, 360);
    }

    // Runs the PID loop for angles within delta 90
    public double controlRotationWithin90(double angle, SwerveModule module)
    {
        // The wheel is commanded directly to the requested angle
        pidController.setSetpoint(angle);
        output = pidController.calculate(module.getAngle());
        output /= 90; // May need to reverse sign here
        error = pidController.getPositionError();
        return output;
    }

    // Runs the PID loop for angles that exceed delta 90
    public SwerveModuleState controlRotationExceeds90(double angle, SwerveModule module, double speed)
    {
        /* Rather than turning the wheel all the way around to the requested angle
        we turn the wheel to the complementary angle and command it backwards */
        SwerveModuleState desiredState = new SwerveModuleState(speed, Rotation2d.fromDegrees(angle));
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(1));
        return state;
    }
}