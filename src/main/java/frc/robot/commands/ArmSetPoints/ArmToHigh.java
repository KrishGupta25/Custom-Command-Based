// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmSetPoints;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubSystem;

public class ArmToHigh extends CommandBase {
  /** Creates a new ArmtoSetpoint. */
  
  private final ArmSubSystem arm;
  public ArmToHigh(ArmSubSystem arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    arm.setPosition("high");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(Math.abs(arm.mArmEncoder.getPosition()) - Math.abs(arm.getPresetPosition())) == 0)
    {
      System.out.println("Arm to Position High Done!");
      return true;
    }
    return false;
  }
}
