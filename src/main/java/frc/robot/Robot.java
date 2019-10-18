/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.controllers.LogitechController;
import frc.subsystems.DiffySwerve;
import frc.subsystems.DiffySwervePID;
import frc.subsystems.DiffySwerve.ModuleID;

import frc.subsystems.NEOMotor;
import com.revrobotics.CANSparkMaxLowLevel;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  // private final DifferentialDrive m_robotDrive
  //     = new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));
  // private final Joystick m_stick = new Joystick(0);
  // private final Timer m_timer = new Timer();

  private final DiffySwervePID diffy = new DiffySwervePID();
  private final Timer m_timer = new Timer();
  private final LogitechController controller = new LogitechController(0);
  private final NEOMotor motor0 = new NEOMotor(1, CANSparkMaxLowLevel.MotorType.kBrushless)
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
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
    // diffy.initialize();
    motor0.initPID();
  }

  public void disabledInit() {
    // diffy.stop();
    motor0.velToPow(500);
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
    // diffy.rotModule(controller.getLeftXAxis(), controller.getLeftYAxis());
    diffy.test();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

}
