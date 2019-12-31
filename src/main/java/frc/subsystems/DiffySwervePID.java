/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.mathutil.MathUtil;
import frc.mathutil.Vector2d;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANEncoder;

import java.util.List;
import java.util.ArrayList;

/**
 * Add your docs here.
*/

/*
Ziegler-Nichols method (rule of thumb)
1. set kI and kD to 0
2. increase kP until output is a sustained and stable oscillation
3. Record the critical gain kC (the kP that caused the oscillation) and oscillation period pC
4. Set PID gains as shown below
  kP = 0.6 * kC
  kI = 2 * kP/pC
  kD = 0.125 * kP * pC
5. adjust as needed


*/
public class DiffySwervePID extends PIDSubsystem {
  /**
  * Add your docs here.
  */

  public enum ModuleID {
    FR, FL, BL, BR;
  }

  /*
  two motor rotation: 
    kP = 0.0014;
    kI = 0.0;
    kD = .0021;
    kF = .007;

    
  private static double kP = 0.00067;
  private static double kI = 0.0;
  private static double kD = 0.0015;
  private static double kF = 0.007;

  */

  private static double kP = 2.5;
  private static double kI = 5.5e-2;
  private static double kD = 8.5;
  private static double kF = 9e-3;

  private static double period = .025;

  private static double setpoint = 0;

  private static final double SWERVE_RATIO = 60;
  private static final double MAXRPM = 5700;

  // private CANSparkMax motor0;
  // private CANSparkMax motor1;

  // private CANEncoder motor0Enc;
  // private CANEncoder motor1Enc;

  private NEOMotor2 motor0;
  private NEOMotor2 motor1;

  private ModuleID modID;

  private double output;


  public DiffySwervePID(ModuleID id) {
    // Intert a subsystem name and PID values here
    super("Differential", kP, kI, kD, kF, period);
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.

    switch(id) {
      case FR:
        motor0 = new NEOMotor2(1, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor1 = new NEOMotor2(2, CANSparkMaxLowLevel.MotorType.kBrushless);
        break;
      case FL:
        motor0 = new NEOMotor2(3, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor1 = new NEOMotor2(4, CANSparkMaxLowLevel.MotorType.kBrushless);
        break;
      case BR:
        motor0 = new NEOMotor2(5, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor1 = new NEOMotor2(6, CANSparkMaxLowLevel.MotorType.kBrushless);
        break;
      case BL:
        motor0 = new NEOMotor2(7, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor1 = new NEOMotor2(8, CANSparkMaxLowLevel.MotorType.kBrushless);
        break;
      default:
        System.out.println("id is invalid");
    }

    setOutputRange(-5700, 5700);
    setAbsoluteTolerance(1.5);
  }

  @Override
  public void setSetpoint(double setpoint) {
    super.setSetpoint(setpoint);
  }

  @Override
  public void enable() {
    super.enable();
  }
  
  public double getPosAvg() {
    return (motor0.getPos() + motor1.getPos()) / 2;
  }

  public double getModAngle() {
    return Math.floor(this.getPosAvg() * SWERVE_RATIO);
  }

  public void moveMod(double angle, double power) {
    this.setSetpoint(angle);
    System.out.printf("%nPV: %4.2f%n", this.getModAngle());
    motor0.set(this.output + (power * MAXRPM));
    motor1.set(this.output - power * MAXRPM);
  }

  public void stop() {
    motor0.stop();
    motor1.stop();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return this.getModAngle();
  }

  @Override
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    // e.g. yourMotor.set(output);
    this.output = output;
  }
}
