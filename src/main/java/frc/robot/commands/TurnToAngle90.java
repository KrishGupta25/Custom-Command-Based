// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.driveTrainSubSystem;

public class TurnToAngle90 extends CommandBase {
  /** Creates a new TurnToAngle90. */
  private final driveTrainSubSystem Drivetrain;
  public TurnToAngle90(driveTrainSubSystem drivetrain) {
    this.Drivetrain = drivetrain;
    addRequirements(Drivetrain);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    Drivetrain.gyro.zeroYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    Drivetrain.turnToAngle(180);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(Math.abs(Drivetrain.gyro.getYaw()) - Math.abs(Drivetrain.getAngle())) <= 1)
    {
      System.out.print("Turn to " + Drivetrain.getAngle() + "done \n" + "This is the gyro value right now " + Drivetrain.gyro.getYaw());
      return true;
    }
    return false;
  }
}
