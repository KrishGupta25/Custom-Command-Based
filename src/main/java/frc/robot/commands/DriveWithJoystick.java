// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;


public class DriveWithJoystick extends CommandBase {
  private final DriveTrain drivetrain;

  public DriveWithJoystick(DriveTrain drivetrain) 
  {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize(){}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    drivetrain.cheesyDrive(-RobotContainer.controller.getLeftY(), RobotContainer.controller.getRightX(), RobotContainer.controller.rightBumper().getAsBoolean());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(RobotContainer.controller.getLeftY()) <= 0.1 && Math.abs(RobotContainer.controller.getRightX()) <= 0.1);
  }
}
