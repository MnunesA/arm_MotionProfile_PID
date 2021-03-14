// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ConstantsArm;
import frc.robot.subsystems.TrapezoidPIDArm;

public class TrapezoidArmCommandSetGoal extends CommandBase {
  
  private TrapezoidPIDArm tas;
  private TrapezoidProfile.State goal;
  private double position;
  private double velocity;
  private double prevVelocity = 0.0;
  private double currentPosition;
  private double currentVelocity;
  private double currentAcceleration;
  private double output;

  public TrapezoidArmCommandSetGoal(TrapezoidPIDArm trapezoidProfileSubsystem, double position, double velocity) {
    tas = trapezoidProfileSubsystem;
    this.position = position;
    this.velocity = velocity;
    addRequirements(tas);
  }

  public TrapezoidArmCommandSetGoal(TrapezoidPIDArm trapezoidProfileSubsystem, double position) {
    tas = trapezoidProfileSubsystem;
    this.position = position;
    this.velocity = 0.0;
    addRequirements(tas);
  }

  @Override
  public void initialize() {
    goal = new TrapezoidProfile.State(position, velocity);
    tas.setGoal(goal);
  }

  @Override
  public void execute() {
    this.currentPosition = (tas.getGyro().getAngle()) * (Math.PI / 180); //radians
    this.currentVelocity = (tas.getGyro().getRate()) * (Math.PI / 180); //radians per second
    this.currentAcceleration = (currentVelocity - prevVelocity)/0.02; //radians per second squared
    this.prevVelocity = currentVelocity;
    this.tas.setAcceleration(currentAcceleration);
    this.tas.useOutput(output, new TrapezoidProfile.State(currentPosition, currentVelocity));
  }

  @Override
  public void end(boolean interrupted) {
      tas.getArmMotorsGroup().set(0.0);
  }

  @Override
  public boolean isFinished() {
    boolean stop = (currentPosition == ConstantsArm.MAX_ANGLE) || 
                   (currentPosition == ConstantsArm.MIN_ANGLE);
    return stop;
  }
}