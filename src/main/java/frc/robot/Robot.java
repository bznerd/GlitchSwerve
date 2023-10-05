// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;

public class Robot extends TimedRobot {
  private PS4Controller driverController;
  private System system = new System();
  private Command autoCommand = null;

  private void configureBindings() {
    system.configureTeleopDrive(
      driverController::getLeftX,
      driverController::getLeftY,
      driverController::getRightX,
      driverController::getR2Button);
  }

  @Override
  public void robotInit() {
    configureBindings();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autoCommand = system.getAuto();
    autoCommand.schedule();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    autoCommand.cancel();
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
