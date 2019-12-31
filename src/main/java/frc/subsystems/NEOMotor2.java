/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
*/
public class NEOMotor2 extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private CANSparkMax m_motor;
  private CANPIDController m_pidControl;
  private CANEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, setPoint;

  public NEOMotor2(int deviceID, CANSparkMaxLowLevel.MotorType type) {
    m_motor = new CANSparkMax(deviceID, type);
    m_pidControl = m_motor.getPIDController();
    m_encoder = m_motor.getEncoder();

    kP = 4.5e-5;
    kI = 1.7e-6;
    kD = 1e-7;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;
    setPoint = 0;

    m_pidControl.setP(kP);
    m_pidControl.setI(kI);
    m_pidControl.setD(kD);
    m_pidControl.setIZone(kIz);
    m_pidControl.setFF(kFF);
    m_pidControl.setOutputRange(kMinOutput, kMaxOutput);

    // SmartDashboard.putNumber("P Gain", kP);
    // SmartDashboard.putNumber("I Gain", kI);
    // SmartDashboard.putNumber("D Gain", kD);
    // SmartDashboard.putNumber("I Zone", kIz);
    // SmartDashboard.putNumber("Feed Forward", kFF);
    // SmartDashboard.putNumber("Max Output", kMaxOutput);
    // SmartDashboard.putNumber("Min Output", kMinOutput);
  }

  public double getPos() {
    return m_encoder.getPosition();
  }

  public void set(double rpm) {
    m_pidControl.setReference((rpm), ControlType.kVelocity);
  }

  public void stop() {
    m_motor.set(0);
    setPoint = 0;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void run(double decimal) {
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidControl.setP(p); kP = p; }
    if((i != kI)) { m_pidControl.setI(i); kI = i; }
    if((d != kD)) { m_pidControl.setD(d); kD = d; }
    if((iz != kIz)) { m_pidControl.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidControl.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidControl.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    
    setPoint = decimal * maxRPM;
    m_pidControl.setReference(setPoint, ControlType.kVelocity);

    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("PV", m_encoder.getVelocity());
  }
}
