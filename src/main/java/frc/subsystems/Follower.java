/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 * Follower class that implements a PID loop
 */
public class Follower extends PIDSubsystem {
  /**
   * Follower class that implements a PID loop
   */

  private static double kP = 0.00036;
  private static double kI = 0.0;
  private static double kD = 0.0002;
  private static double kF = 0.00013;

  private static double period = .025;

  private static double setpoint = 0;

  private static double leaderSpeed = 0.0;

  private double output;

  private CANSparkMax leader;
  private CANSparkMax follower;

  private CANEncoder leaderEnc;
  private CANEncoder followerEnc;

  /**
   * 
   * @param leader Motor to be followed
   * @param follower Motor that will follow
   */
  public Follower(CANSparkMax leader, CANSparkMax follower) {
    // Intert a subsystem name and PID values here
    super("Follower", kP, kI, kD, kF, period);
    // setOutputRange(-1.0, 1.0);

    this.leader = leader;
    this.follower = follower;

    this.leaderEnc = new CANEncoder(leader);
    this.followerEnc = new CANEncoder(follower);

    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
  }

  public void init() {
    setAbsoluteTolerance(0.1);
    setOutputRange(-1.0, 1.0);
    enable();
  }

  public void follow() {
    leaderSpeed = leaderEnc.getVelocity();
    // System.out.println("Leader Speed: " + leaderSpeed);
    // System.out.println("Follower Speed: " + followerEnc.getVelocity());
    setSetpoint(leaderSpeed);
    follower.set(-this.output);
    System.out.println("leader Vel: " + leaderEnc.getVelocity());
    System.out.println("Follower Vel: " + followerEnc.getVelocity());
    System.out.println("PID Output: " + this.output);
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
    return -(followerEnc.getVelocity());
  }

  @Override
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    // e.g. yourMotor.set(output);
    this.output = output;
    // System.out.println("My PID Output: " + this.output);
  }
}
