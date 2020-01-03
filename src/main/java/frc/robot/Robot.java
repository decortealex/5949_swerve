/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.subsystems.DiffSwerveMod;
import frc.subsystems.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private DiffSwerveMod m_swerveModule;
  private Drivetrain m_drivetrain;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
  */
  @Override
  public void robotInit() {
    m_swerveModule = new DiffSwerveMod(frc.subsystems.DiffSwerveMod.ModuleID.FL);
    m_drivetrain = new Drivetrain();
    
  }

  /**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    // m_swerveModule.enable();
    m_drivetrain.enable();
  }

  public void disabledInit() {
    // diffy.stop();
    m_drivetrain.disable();
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
    // m_swerveModule.moveMod(m_joystick.getRightAxisAngle(), m_joystick.getLeftYAxis());
    // m_drivetrain.drive(m_joystick.getRightAxisAngle(), m_joystick.getLeftYAxis());
    System.out.print(m_swerveModule.getModAngle());

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    System.out.print(m_swerveModule.getModAngle());
  }

}
