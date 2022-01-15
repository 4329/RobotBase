package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModuleConstants;
import frc.robot.Configrun;
import frc.robot.RobotContainer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class SwerveDrive extends SubsystemBase
{
        public SwerveModule frontLeft, frontRight, backLeft, backRight;

        private double simpleAutoSpeed, simpleAutoAngle, simpleAutoAngle2, driveStaticFeedforward,
                        headingHoldStaticFeedforward, maxSpeed, aDrive, bDrive, turnStaticFeedforward;
        private final double unitsPerFoot = 7.954564056;
        // private final double unitsPerFoot = 8.546620444;
        private PIDController drivePID, headingHoldPID, turnPID;
        public static int LEFT = 180, FORWARD = 90, RIGHT = 0, BACKWARD = 270;
        private double percentOutput = 0;
        private NetworkTableEntry fieldOrientedControlNTE;
        //Display angle after offset
        private NetworkTableEntry frontLeftAngleDisplay;
        private NetworkTableEntry frontRightAngleDisplay;
        private NetworkTableEntry backLeftAngleDisplay;
        private NetworkTableEntry backRightAngleDisplay;
        //Display angle after offset
        private NetworkTableEntry frontLeftRawAngleDisplay;
        private NetworkTableEntry frontRightRawAngleDisplay;
        private NetworkTableEntry backLeftRawAngleDisplay;
        private NetworkTableEntry backRightRawAngleDisplay;
        //Display angle after offset
        private NetworkTableEntry frontLeftOffsetDisplay;
        private NetworkTableEntry frontRightOffsetDisplay;
        private NetworkTableEntry backLeftOffsetDisplay;
        private NetworkTableEntry backRightOffsetDisplay;

        public SwerveDrive()
        {
                frontRight = new SwerveModule(SwerveModuleConstants.TRANSLATION_PORT[2],
                                SwerveModuleConstants.ROTATION_PORT[2], SwerveModuleConstants.POTENTIOMETER_PORT[2],
                                SwerveModuleConstants.OFFSET[2]);
                frontLeft = new SwerveModule(SwerveModuleConstants.TRANSLATION_PORT[3],
                                SwerveModuleConstants.ROTATION_PORT[3], SwerveModuleConstants.POTENTIOMETER_PORT[3],
                                SwerveModuleConstants.OFFSET[3]);
                backLeft = new SwerveModule(SwerveModuleConstants.TRANSLATION_PORT[0],
                                SwerveModuleConstants.ROTATION_PORT[0], SwerveModuleConstants.POTENTIOMETER_PORT[0],
                                SwerveModuleConstants.OFFSET[0]);
                backRight = new SwerveModule(SwerveModuleConstants.TRANSLATION_PORT[1],
                                SwerveModuleConstants.ROTATION_PORT[1], SwerveModuleConstants.POTENTIOMETER_PORT[1],
                                SwerveModuleConstants.OFFSET[1]);
                simpleAutoAngle = (Configrun.get(0, "simpleAutoAngle"));
                simpleAutoSpeed = (Configrun.get(0.0, "simpleAutoSpeed"));
                simpleAutoAngle2 = (Configrun.get(0, "simpleAutoAngle2"));
                drivePID = new PIDController(Configrun.get(0.0, "driveP"), Configrun.get(0.0, "driveI"),
                                Configrun.get(0.0, "driveD"));
                drivePID.setTolerance(unitsPerFoot / 12);
                driveStaticFeedforward = Configrun.get(0.0, "driveStaticFeedforward");
                headingHoldPID = new PIDController(Configrun.get(0.0, "headingHoldP"),
                                Configrun.get(0.0, "headingHoldI"), Configrun.get(0.0, "headingHoldD"));
                headingHoldStaticFeedforward = Configrun.get(0.0, "headingHoldStaticFeedforward");
                headingHoldPID.enableContinuousInput(0, 360);
                turnPID = new PIDController(Configrun.get(0.0, "turnP"), Configrun.get(0.0, "turnI"),
                                Configrun.get(0.0, "turnD"));
                turnPID.setTolerance(Configrun.get(0.0, "turnTolerance"));
                turnPID.enableContinuousInput(0, 360);
                turnStaticFeedforward = Configrun.get(0.0, "turnStaticFeedforward");
                maxSpeed = Configrun.get(0.0, "maxSpeed");
                aDrive = Configrun.get(0.0, "aDrive");
                bDrive = Configrun.get(0.0, "bDrive");
                fieldOrientedControlNTE = Shuffleboard.getTab("TestValues").add("Field Oriented Control", false)
                                .getEntry();
        }

        public void updateShuffleboardAngles()
        {
                //Display angle after offset
                frontLeftAngleDisplay.setDouble(frontLeft.getAngle());
                frontRightAngleDisplay.setDouble(frontRight.getAngle());
                backLeftAngleDisplay.setDouble(backLeft.getAngle());
                backRightAngleDisplay.setDouble(backRight.getAngle());
                //Display angle before offset
                frontLeftRawAngleDisplay.setDouble(frontLeft.getRawAngle());
                frontRightRawAngleDisplay.setDouble(frontRight.getRawAngle());
                backLeftRawAngleDisplay.setDouble(backLeft.getRawAngle());
                backRightRawAngleDisplay.setDouble(backRight.getRawAngle());
                //Display offset
                frontLeftOffsetDisplay.setDouble(frontLeft.getOffset());
                frontRightOffsetDisplay.setDouble(frontRight.getOffset());
                backLeftOffsetDisplay.setDouble(backLeft.getOffset());
                backRightOffsetDisplay.setDouble(backRight.getOffset());
        }
        public void putAngles()
        {
                //Display angle after offset
                frontLeftAngleDisplay = Shuffleboard.getTab("Debug").add("Front Left Angle",frontLeft.getAngle()).withPosition(0,0).withWidget(BuiltInWidgets.kTextView).getEntry();
                frontRightAngleDisplay = Shuffleboard.getTab("Debug").add("Front Right Angle",frontRight.getAngle()).withPosition(1,0).withWidget(BuiltInWidgets.kTextView).getEntry();
                backLeftAngleDisplay = Shuffleboard.getTab("Debug").add("Back Left Angle",backLeft.getAngle()).withPosition(0,1).withWidget(BuiltInWidgets.kTextView).getEntry();
                backRightAngleDisplay = Shuffleboard.getTab("Debug").add("Back Right Angle",backRight.getAngle()).withPosition(1,1).withWidget(BuiltInWidgets.kTextView).getEntry();
                //Display Angle before offset
                frontLeftRawAngleDisplay = Shuffleboard.getTab("Debug").add("Front Left Raw Angle",frontLeft.getRawAngle()).withPosition(0,2).withWidget(BuiltInWidgets.kTextView).getEntry();
                frontRightRawAngleDisplay = Shuffleboard.getTab("Debug").add("Front Right Raw Angle",frontRight.getRawAngle()).withPosition(1,2).withWidget(BuiltInWidgets.kTextView).getEntry();
                backLeftRawAngleDisplay = Shuffleboard.getTab("Debug").add("Back Left Raw Angle",backLeft.getRawAngle()).withPosition(0,3).withWidget(BuiltInWidgets.kTextView).getEntry();
                backRightRawAngleDisplay = Shuffleboard.getTab("Debug").add("Back Right Raw Angle",backRight.getRawAngle()).withPosition(1,3).withWidget(BuiltInWidgets.kTextView).getEntry();
                //Display offset
                frontLeftOffsetDisplay = Shuffleboard.getTab("Debug").add("Front Left Offset",frontLeft.getOffset()).withPosition(0,4).withWidget(BuiltInWidgets.kTextView).getEntry();
                frontRightOffsetDisplay = Shuffleboard.getTab("Debug").add("Front Right Offset",frontRight.getOffset()).withPosition(1,4).withWidget(BuiltInWidgets.kTextView).getEntry();
                backLeftOffsetDisplay = Shuffleboard.getTab("Debug").add("Back Left Offset",backLeft.getOffset()).withPosition(0,5).withWidget(BuiltInWidgets.kTextView).getEntry();
                backRightOffsetDisplay = Shuffleboard.getTab("Debug").add("Back Right Offset",backRight.getOffset()).withPosition(1,5).withWidget(BuiltInWidgets.kTextView).getEntry();
        }

        public void drive(double xTranslation, double yTranslation, double rotation)
        {
                if (SwerveLock.lock)
                {
                        lock();
                        return;
                }
                if ((xTranslation != 0 || yTranslation != 0) && rotation == 0)
                {
                        rotation = headingHoldPID.calculate(RobotContainer.pigeonSub.getYaw(), 0);
                        rotation = -rotation / 360;
                        if (rotation < 0)
                        {
                                rotation = rotation - headingHoldStaticFeedforward;
                        } else
                        {
                                rotation = rotation + headingHoldStaticFeedforward;
                        }
                } else
                {
                        Pigeon.resetPigeon();
                }

                frontRight.driveModuleTeleop(frontRight, xTranslation, yTranslation, rotation);
                frontLeft.driveModuleTeleop(frontLeft, xTranslation, yTranslation, rotation);
                backLeft.driveModuleTeleop(backLeft, xTranslation, yTranslation, rotation);
                backRight.driveModuleTeleop(backRight, xTranslation, yTranslation, rotation);
        }

        // cringe
        public void driveAuto(double xTranslation, double yTranslation, double rotation)
        {
                if (SwerveLock.lock)
                {
                        lock();
                        return;
                }

                frontRight.driveModuleTeleop(frontRight, xTranslation, yTranslation, rotation);
                frontLeft.driveModuleTeleop(frontLeft, xTranslation, yTranslation, rotation);
                backLeft.driveModuleTeleop(backLeft, xTranslation, yTranslation, rotation);
                backRight.driveModuleTeleop(backRight, xTranslation, yTranslation, rotation);
        }

        public void turn(double angle)
        {
                double output = turnPID.calculate(RobotContainer.pigeonSub.getYaw(), angle);
                output = output / 360;
                if (output < 0)
                {
                        output = output - turnStaticFeedforward;
                } else
                {
                        output = output + turnStaticFeedforward;
                }
                // SmartDashboard.putNumber("TurnOutput", output);
                // SmartDashboard.putNumber("TurnError", turnPID.getPositionError());
                RobotContainer.swerveDrive.rotate(output);
        }

        // Rotates the robot as a unit
        public void rotate(double vector)
        {
                if (vector < 0)
                {
                        frontRight.driveModule(vector, 315, frontRight);
                        frontLeft.driveModule(vector, 45, frontLeft);
                        backLeft.driveModule(vector, 135, backLeft);
                        backRight.driveModule(vector, 225, backRight);
                }
                if (vector > 0)
                {
                        frontRight.driveModule(-vector, 135, frontRight);
                        frontLeft.driveModule(-vector, 225, frontLeft);
                        backLeft.driveModule(-vector, 315, backLeft);
                        backRight.driveModule(-vector, 45, backRight);
                }
                if (vector == 0)
                {
                        stopDrive();
                }
        }

        // Points all four wheels inward in order to act as a brake
        // and make the robot harder to push around
        public void lock()
        {
                frontRight.driveModule(0, 45, frontRight);
                frontLeft.driveModule(0, 135, frontLeft);
                backLeft.driveModule(0, 45, backLeft);
                backRight.driveModule(0, 135, backRight);
        }

        public void stopDrive()
        {
                frontRight.stopModule(frontRight);
                frontLeft.stopModule(frontLeft);
                backLeft.stopModule(backLeft);
                backRight.stopModule(backRight);
        }

        public void putAverageEncoderPulses()
        {
                double sum = frontRight.encoderTranslationGet(frontRight) + frontLeft.encoderTranslationGet(frontLeft)
                                + backLeft.encoderTranslationGet(backLeft) + backRight.encoderTranslationGet(backRight);
                SmartDashboard.putNumber("AveragePulses", sum / 4);
        }

        public double getAverageEncoderPulses()
        {
                double sum = frontRight.encoderTranslationGet(frontRight) + frontLeft.encoderTranslationGet(frontLeft)
                                + backLeft.encoderTranslationGet(backLeft) + backRight.encoderTranslationGet(backRight);
                return sum / 4;
        }

        public void resetEncoders()
        {
                frontRight.resetEncoder(frontRight);
                frontLeft.resetEncoder(frontLeft);
                backLeft.resetEncoder(backLeft);
                backRight.resetEncoder(backRight);

        }

        public void simpleAutoDrive()
        {
                frontRight.driveModule(simpleAutoSpeed, simpleAutoAngle, frontRight);
                frontLeft.driveModule(simpleAutoSpeed, simpleAutoAngle, frontLeft);
                backLeft.driveModule(simpleAutoSpeed, simpleAutoAngle, backLeft);
                backRight.driveModule(simpleAutoSpeed, simpleAutoAngle, backRight);
        }

        public void simpleAutoDrive2()
        {
                frontRight.driveModule(simpleAutoSpeed, simpleAutoAngle2, frontRight);
                frontLeft.driveModule(simpleAutoSpeed, simpleAutoAngle2, frontLeft);
                backLeft.driveModule(simpleAutoSpeed, simpleAutoAngle2, backLeft);
                backRight.driveModule(simpleAutoSpeed, simpleAutoAngle2, backRight);
        }

        // Setpoint in inches, angle in degrees (left=180, forward=90. right=0)
        public void runDrivePID(double setpoint, double angle)
        {
                setpoint = setpoint + (setpoint * aDrive + bDrive);
                SmartDashboard.putNumber("Setpoint in Inches", setpoint);
                setpoint = setpoint / 12; // convert to feet
                setpoint = setpoint * unitsPerFoot; // covert to native units
                double output = drivePID.calculate(getAverageEncoderPulses(), setpoint);
                output = output + driveStaticFeedforward;
                output = Math.min(output, maxSpeed);
                angle = Math.toRadians(angle);
                double xTranslation = output * Math.cos(angle);
                double yTranslation = output * Math.sin(angle);
                double rotation = headingHoldPID.calculate(RobotContainer.pigeonSub.getYaw(), 0);
                rotation = -rotation / 360;
                if (rotation < 0)
                {
                        rotation = rotation - headingHoldStaticFeedforward;
                } else
                {
                        rotation = rotation + headingHoldStaticFeedforward;
                }
                // SmartDashboard.putNumber("Drive Error Inches",
                // unitsToInches(drivePID.getPositionError()));
                // SmartDashboard.putNumber("Drive Output", output);
                // SmartDashboard.putNumber("Heading Error", headingHoldPID.getPositionError());
                // SmartDashboard.putNumber("Heading Output", rotation);
                driveAuto(xTranslation, yTranslation, rotation);
        }

        public boolean isDrivePIDWithinTolerance()
        {
                return drivePID.atSetpoint();
        }

        public boolean isTurnPIDWithinTolerance()
        {
                return turnPID.atSetpoint();
        }

        public void checkStaticFeedforwardLimelight()
        {
              /*  percentOutput = percentOutput + 0.001;
                rotate(percentOutput);
                System.out.println("Percent Output: " + percentOutput + "CheckTx: "
                                + RobotContainer.limelightSubsystem.checkTx()); */

        }

        public void checkStaticFeedforwardPigeon()
        {
                percentOutput = percentOutput + 0.001;
                rotate(percentOutput);
                System.out.println("Percent Output: " + percentOutput + "Pigeon: " + RobotContainer.pigeonSub.getYaw());
        }

        public void resetStaticFeedforward()
        {
                percentOutput = 0;
        }

        private double unitsToFeet(double units)
        {
                return units / unitsPerFoot;
        }

        private double unitsToInches(double units)
        {
                return (units / unitsPerFoot) * 12;
        }

        public void toggleFieldOrientedControl()
        {
                if (SwerveModule.fieldOrientedControl)
                {
                        SwerveModule.fieldOrientedControl = false;
                        fieldOrientedControlNTE.setBoolean(false);
                } else
                {
                        SwerveModule.fieldOrientedControl = true;
                        Pigeon.resetPigeon();
                        fieldOrientedControlNTE.setBoolean(true);
                }
        }
}