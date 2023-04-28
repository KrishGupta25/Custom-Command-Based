// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.driveTrainSubSystem;

public class TurnToAngle90 extends CommandBase {
  /** Creates a new DriveDistance. */

  private final driveTrainSubSystem drivetrain;
  public TurnToAngle90(driveTrainSubSystem drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    drivetrain.gyro.zeroYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    drivetrain.turnToAngle(180);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(Math.abs(drivetrain.gyro.getYaw()) - Math.abs(drivetrain.getAngle())) <= 0.5)
    {
      System.out.println("Gyro Angle YAW (Turn) " + drivetrain.gyro.getYaw());
      System.out.println("Drivetrain to " + drivetrain.getAngle() + "done!");
      return true;
    }
    return false;
  }
}
