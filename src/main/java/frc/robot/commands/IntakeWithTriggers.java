// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.RobotContainer;

public class IntakeWithTriggers extends CommandBase 
{
  public final Intake intake;

  public IntakeWithTriggers(Intake intake) 
  {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
<<<<<<< HEAD
    if (RobotContainer.controller.getLeftTriggerAxis() != 0)
      intake.setSpeed(RobotContainer.controller.getLeftTriggerAxis());
    if (RobotContainer.controller.getRightTriggerAxis() != 0)
      intake.setSpeed(RobotContainer.controller.getRightTriggerAxis());
=======
    if (RobotContainer.controller.leftTrigger() != 0)
      intake.setSpeed(lef);
    if (RobotContainer.controller.rightTrigger())
      intake.setSpeed(RobotContainer.controller.rightTrigger());
>>>>>>> a887e3355b245f17204d67c4cf70ca0e7e8908ce
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
