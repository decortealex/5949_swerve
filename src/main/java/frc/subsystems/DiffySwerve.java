/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

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
public class DiffySwerve extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public enum ModuleID {
    FR, FL, BL, BR;
  }
  //TODO figure out what the actual values are for these variables
  private static final int POS_GAINS_SLOT = 0;
  public static final int STEERING_COUNTS_PER_REV = 800; // Definitely not correct, fix this number
  private static final double SMALL_NUM = .00001;
  private static final int REMOTE_0 = 0;
  private static final int TIMEOUT = 10;
  public static final double kF = .0006777 * 1023.0; // Fix this number
  // PID Constants
  public static final double kP_POS = 1.0/300.0*1023.0; // Fix this number
  public static final double kI_POS = 0.0; // Fix this maybe
  public static final double kD_POS = 0.0; // Fix this maybe
  public static final int CRUISE_VEL = 1200; // Fix this number
  public static final int ACCEL = 2400; // Fix this number

  public static List<Double> motor0Arr = new ArrayList<Double>();
  public static List<Double> motor1Arr = new ArrayList<Double>();

  public static double motor0VelAvg = 0;
  public static double motor1VelAvg = 0;

  public static List<Double> modVel = new ArrayList<Double>();
  public static double modVelAvg = 0;

  private static double speed = 1.0;
  private static double motor0Speed = speed;
  private static double motor1Speed = -speed;

  private Vector2d positionVec;
  
  private CANSparkMax motor0;
  private CANSparkMax motor1;

  private CANEncoder motor0Enc;
  private CANEncoder motor1Enc;

  private ModuleID modID;

  public DiffySwerve(ModuleID modID) {
    motor0 = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    motor1 = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);

    motor0Enc = new CANEncoder(motor0);
    motor1Enc = new CANEncoder(motor1);

    // polarities (idk what this does so figure it out)
    motor0.setInverted(false);
    motor1.setInverted(false);

  }

  public void populateLists() {
    for(int i = 0; i < 100; i++) {
      motor0Arr.add(0.0);
      motor1Arr.add(0.0);
      modVel.add(0.0);
    }
  }
  public Vector2d getPosVector() {
    return positionVec;
  }

  public double getMotor0Pos() {
    return motor0Enc.getPosition();
  }

  public double getMotor1Pos() {
    return motor1Enc.getPosition();
  }

  public double getEncAvg() {
    return ((getMotor0Pos() + getMotor1Pos()) / 2);
  }

  public void getVelAvg() {
    for(int i = 0; i < 100; i++) {
      modVel.set(i, Math.abs(motor0Arr.get(i)) - Math.abs(motor1Arr.get(i)));
    }

    double sum = 0;
    for(double vel : modVel) {
      sum += vel;
    }

    modVelAvg = sum / (double)modVel.size();

    System.out.println("Module Velocity Average: " + modVelAvg);
  }

  public void test() {
    motor0.set(motor0Speed);
    motor1.set(motor1Speed);
    System.out.println("motor 0: " + motor0Enc.getVelocity());
    // motor0Arr.add(motor0Enc.getVelocity());
    // motor0Arr.remove(0);
    System.out.println("motor 1: " + motor1Enc.getVelocity());
    // motor1Arr.add(motor1Enc.getVelocity());
    // motor1Arr.remove(0);
  }

  /*
  modify left and right speed values by the module velocity avg until the
  module velocity avg < 0.01
  motor0Speed = +
  motor1Speed = -
  */
  public void homebrewPID() {
    if(Math.abs(modVelAvg) > 0.01) {
      motor0Speed -= .0001;
      motor1Speed += .0001;
    }
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
}
