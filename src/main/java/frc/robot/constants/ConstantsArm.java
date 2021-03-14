// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.wpilibj.AnalogGyro;

/**
 * The ConstantsElevator class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class ConstantsArm {

   public static final double 
       Kp = 0,
       Ki = 0,
       Kd = 0,
       Ks = 0, 
       Kcos = 0, 
       Kv = 0, 
       Ka = 0;

   public static final double 
       MAX_VELOCITY = 0, 
       MAX_ACCELERATION = 0, 
       INITIAL_POSITION = 0;

   public static final AnalogGyro gyro = new AnalogGyro(Constants.CHANNEL_AGY_0);

   public static double MAX_ANGLE = 85 * (Math.PI / 180), //radians
                        MIN_ANGLE = 5 * (Math.PI / 180);  //radians
}

