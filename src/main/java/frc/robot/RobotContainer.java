// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.IntakeWithTriggers;
import frc.robot.commands.ArmSetPoints.ArmStowed;
import frc.robot.commands.ArmSetPoints.ArmToGround;
import frc.robot.commands.ArmSetPoints.ArmToHigh;
import frc.robot.commands.ArmSetPoints.ArmToHuman;
import frc.robot.commands.ArmWithController;
import frc.robot.subsystems.ArmSubSystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import edu.wpi.first.wpilibj2.command.button.*;

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

  public static ArmSubSystem arm = new ArmSubSystem();
  public static ArmWithController armController = new ArmWithController(arm);
  public static ArmStowed armStowed = new ArmStowed(arm);
  public static ArmToHigh armHigh = new ArmToHigh(arm);
  public static ArmToHuman armHuman = new ArmToHuman(arm);
  public static ArmToGround armGround = new ArmToGround(arm);


  public static Intake intake = new Intake();
  public static IntakeWithTriggers intakeTriggers = new IntakeWithTriggers(intake); 


  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    drivetrain.setDefaultCommand(drivewithjoystick);
    arm.setDefaultCommand(armController);
    intake.setDefaultCommand(intakeTriggers);
    System.out.println("ARM ENCODER " + arm.mArmEncoder.getPosition());
    //arm.setSpeed(controller.getLeftY());
 
    
  }

  private void configureBindings()
  {
    controller.a().onTrue(armHuman);
    controller.b().onTrue(armGround);
    controller.x().onTrue(armGround);
    controller.y().onTrue(armHigh);
  }

  public Command getAutonomousCommand() {
    return drivewithjoystick;
  }
}
