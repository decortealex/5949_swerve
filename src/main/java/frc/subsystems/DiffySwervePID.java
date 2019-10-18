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

  private static double kP = 0.00065;
  private static double kI = 0.0;
  private static double kD = 0.0017;
  private static double kF = 0.0075;

  private static double period = .025;

  private static double setpoint = 0;

  private static final double SWERVE_RATIO = 75;

  private CANSparkMax motor0;
  private CANSparkMax motor1;

  private CANEncoder motor0Enc;
  private CANEncoder motor1Enc;

  private Follower fMotor;

  private ModuleID modID;

  private double output;


  public DiffySwervePID() {
    // Intert a subsystem name and PID values here
    super("Differential", kP, kI, kD, kF, period);
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.

    motor0 = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    motor1 = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);

    motor0Enc = new CANEncoder(motor0);
    motor1Enc = new CANEncoder(motor1);

    fMotor = new Follower(motor0, motor1);

    // polarities
    motor0.setInverted(false);
    motor1.setInverted(false);
  }

  @Override
  public void setSetpoint(double setpoint) {
    super.setSetpoint(setpoint);
  }

  @Override
  public void enable() {
    super.enable();
  }

  public double getMotor0Pos() {
    return motor0Enc.getPosition();
  }

  public double getMotor1Pos() {
    return motor1Enc.getPosition();
  }

  public double getPosAvg() {
    return (getMotor0Pos() + getMotor1Pos()) / 2;
  }

  public double getModAngle() {
    return Math.floor(this.getPosAvg() * SWERVE_RATIO);
  }

  public void initialize() {
    // this.setSetpoint(setpoint);
    // setOutputRange(-1.0, 1.0);
    // setAbsoluteTolerance(5);
    // this.enable();
    fMotor.init();
  }

  public void test() {
    // double prevAngle = 0;
    // System.out.println("output: " + this.output);
    // if(this.getModAngle() != prevAngle) {
    //   System.out.println("Mod Angle: " + this.getModAngle());
    // }
    // prevAngle = this.getModAngle();
    // motor0.set(this.output + 0.9);
    // motor1.set(this.output - 0.9);
    // motor0.set(this.output);
    // motor1.set(this.output);
    motor0.set(0.1);
    fMotor.follow();
    SmartDashboard.putNumber("Encoder Val", this.getModAngle());
  }

  public void rotModule(double xAxis, double yAxis) {
    double hypot = Math.hypot(xAxis, yAxis);
    setpoint = hypot * 360;
    this.setSetpoint(setpoint);
  }

  public void stop() {
    motor0.set(0.0);
    motor1.set(0.0);
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
