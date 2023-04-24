// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubSystem;

public class ArmtoSetpoint extends CommandBase {
  /** Creates a new ArmtoSetpoint. */
  
  private final ArmSubSystem arm;
  public ArmtoSetpoint(ArmSubSystem arm) {
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
    int armSetPoint = 0;
    if (RobotContainer.controller.b().getAsBoolean())
      armSetPoint = 1000; //stowed
    else if (RobotContainer.controller.x().getAsBoolean())
      armSetPoint = 4000; //human player
    else if (RobotContainer.controller.y().getAsBoolean())
      armSetPoint = 3000; //high goal
    else if (RobotContainer.controller.a().getAsBoolean())
      armSetPoint = 6000; //ground goal
    arm.setPosition(armSetPoint);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(arm.mArmEncoder.getPosition() - arm.getPresetPosition()) <= 2);
 
  }
}
