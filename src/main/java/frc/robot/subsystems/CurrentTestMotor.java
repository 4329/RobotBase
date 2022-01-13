package frc.robot.subsystems;

import frc.robot.Configrun;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CurrentTestMotor extends SubsystemBase {
    private TalonSRX testMotor = new TalonSRX(Configrun.get(12, "TestMotor_ID"));
    int testMotorSpeed = Configrun.get(0, "testMotorSpeed");

    public void runTestMotor() {
        testMotor.set(ControlMode.PercentOutput, -testMotorSpeed);
    }}