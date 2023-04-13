// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
<<<<<<< HEAD
import frc.robot.subsystems.ArmSubSystem;
=======
import frc.robot.subsystems.Arm;
>>>>>>> a887e3355b245f17204d67c4cf70ca0e7e8908ce


public class ArmWithController extends CommandBase
{
<<<<<<< HEAD
  private final ArmSubSystem arm;

  public ArmWithController(ArmSubSystem arm) 
=======
  private final Arm arm;

  public ArmWithController(Arm arm) 
>>>>>>> a887e3355b245f17204d67c4cf70ca0e7e8908ce
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
<<<<<<< HEAD
    arm.setSpeed((RobotContainer.controller.getRightY()) -0.3 / (0.7));
=======
    arm.setSpeed((RobotContainer.controller.getRightY()) -0/3 / (0.7));

    if (RobotContainer.controller.leftBumper().getAsBoolean())
    {
      System.out.print("HIIIIII");
      arm.brakeMode(true);
    }
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
