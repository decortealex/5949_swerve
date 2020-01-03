/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.subsystems.DiffSwerveMod;
import frc.subsystems.DiffSwerveMod.ModuleID;

/**
 * Combines Swerve modules into single drivetrain
 */

public class Drivetrain extends Subsystem {
  private DiffSwerveMod m_modFR;
  private DiffSwerveMod m_modFL;
  private DiffSwerveMod m_modBR;
  private DiffSwerveMod m_modBL;

  public Drivetrain() {
    this.m_modFR = new DiffSwerveMod(ModuleID.FR);
    this.m_modFL = new DiffSwerveMod(ModuleID.FL);
    // this.m_modBR = new DiffySwervePID(ModuleID.BR);
    // this.m_modBL = new DiffySwervePID(ModuleID.BL);
  }

  /**
   * Enables internal PID controllers for each swerve module
   */
  public void enable() {
    this.m_modFR.enable();
    this.m_modFL.enable();
    // this.m_modBR.enable();
    // this.m_modBL.enable();
  }

  /**
   * Disables the internal PID controllers for each swerve module
   */
  public void disable() {
    this.m_modFR.stop();
    this.m_modFL.stop();
    // this.m_modBR.stop();
    // this.m_modBL.stop();
  }

  /**
   * Uses basic module angle + speed driving scheme
   */
  public void drive(double angle, double power) {
    this.m_modFR.moveMod(angle, power);
    this.m_modFL.moveMod(angle, power);
    // this.m_modBR.moveMod(angle, power);
    // this.m_modBL.moveMod(angle, power);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
