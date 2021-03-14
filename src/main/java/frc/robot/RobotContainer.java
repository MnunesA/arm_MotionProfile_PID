// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.arm.TrapezoidArmCommandSetGoal;
import frc.robot.constants.Constants;
import frc.robot.constants.ConstantsArm;
import frc.robot.subsystems.TrapezoidPIDArm;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  private TrapezoidPIDArm tArmSubsystem;
  private XboxController xboxController;
  private JoystickButton XBOX_B;
  
  public RobotContainer() {
    
    configureJoysticks();
    configureSubsystems();
    configureCommands();
    configureButtonBindings();
    configureButtonFunctions();
    configureConditionalButtonFuntions();
  }

  
  private void configureButtonBindings() {
    XBOX_B = new JoystickButton(xboxController, Constants.XBOX_B_ID);
  }

  private void configureButtonFunctions() {
  }

  protected void configureConditionalButtonFuntions() {
    if (tArmSubsystem.getGyro().getAngle() >= 80) {
      XBOX_B.whenPressed(new TrapezoidArmCommandSetGoal(tArmSubsystem, ConstantsArm.MIN_ANGLE));
    } else {
      XBOX_B.whenPressed(new TrapezoidArmCommandSetGoal(tArmSubsystem, ConstantsArm.MAX_ANGLE));
    }
  }

  private void configureSubsystems() {
    tArmSubsystem = new TrapezoidPIDArm(Constants.CHANNEL_PWM_2,
                                              Constants.CHANNEL_PWM_3,
                                              ConstantsArm.gyro);
  }

  private void configureCommands() {
  }

  private void configureJoysticks() {
    xboxController = new XboxController(Constants.JOYSTICK_ID);
  }

  public XboxController getXboxController() {
    return xboxController;
  }

  /*
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
  */
}
