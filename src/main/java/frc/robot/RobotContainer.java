// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.ArmWithController;
<<<<<<< HEAD
import frc.robot.subsystems.ArmSubSystem;
=======
import frc.robot.subsystems.Arm;
>>>>>>> a887e3355b245f17204d67c4cf70ca0e7e8908ce
import frc.robot.subsystems.DriveTrain;

import java.lang.ModuleLayer.Controller;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static CommandXboxController controller = new CommandXboxController(1);

  public static DriveTrain drivetrain = new DriveTrain();
  public static DriveWithJoystick drivewithjoystick = new DriveWithJoystick(drivetrain);

<<<<<<< HEAD
  public static ArmSubSystem arm = new ArmSubSystem();
=======
  public static Arm arm = new Arm();
>>>>>>> a887e3355b245f17204d67c4cf70ca0e7e8908ce
  public static ArmWithController armController = new ArmWithController(arm);


  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    drivetrain.setDefaultCommand(drivewithjoystick);
    arm.setDefaultCommand(armController);
  }

  private void configureBindings(){}

  public Command getAutonomousCommand() {
    return drivewithjoystick;
  }
}
