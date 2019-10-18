/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 * Add your docs here.
 */
public class NEOMotor extends PIDSubsystem {

  private static double kP = 0.0;
  private static double kI = 0.0;
  private static double kD = 0.0;
  private static double kF = 0.0;

  private static double setpoint = 0.0;

  private static double output = 0.0;

  private static final double PERIOD = 0.0250;

  private static final double MAXRPM = 5676.0;
  
  private CANSparkMax motor;
  private CANEncoder encoder;

  public NEOMotor(int deviceID, CANSparkMaxLowLevel.MotorType motorType) {
    super("NEOMotor", kP, kI, kD, kF, PERIOD);

    motor = new CANSparkMax(deviceID, motorType);
    encoder = new CANEncoder(motor);

  }

  public void initPID() {
    setAbsoluteTolerance(0.5);
    setInputRange(0, MAXRPM);
    setOutputRange(-1.0, 1.0);
    this.enable();
  }

  public void velToPow(double vel) {
    this.setSetpoint(vel);
    motor.set(this.output);
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
    return encoder.getVelocity();
  }

  @Override
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    // e.g. yourMotor.set(output);
    this.output = output;
  }
  
  @Override
  public void enable() {
    super.enable();
  }

  @Override
  public void setSetpoint(double setpoint) {
    super.setSetpoint(setpoint);
  }
}
