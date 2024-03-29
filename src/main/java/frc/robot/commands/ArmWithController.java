// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubSystem;


public class ArmWithController extends CommandBase
{
  private final ArmSubSystem arm;

  public ArmWithController(ArmSubSystem arm) 
  {
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    arm.setSpeed((RobotContainer.controller.getRightY()) -0.3 / (2));
    //System.out.println(ArmSubSystem.mLeftArm.getEncoder().getPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(RobotContainer.controller.getRightY()) <= 0.1);
  }
}
