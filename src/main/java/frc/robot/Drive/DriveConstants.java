// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drive;

/** Add your docs here. */
public final class DriveConstants {
    public static final double kS = 0.1849;
    /**volts per meter per second*/
    public static final double kV = 2.5108;
    public static final double kA = 0.24017;

    public static final double MAX_PATH_VELOCITY = 2;
    public static final double MAX_PATH_ACCELERATION = 1;

    public static final double AUTO_X_P =1;// 20;
    public static final double AUTO_X_I = 0;
    public static final double AUTO_X_D = 0;

    public static final double AUTO_Y_P = 1;//20;
    public static final double AUTO_Y_I = 0;
    public static final double AUTO_Y_D = 0;

    public static final double AUTO_THETA_P = .25;// 1;//1.8;//.35;
    public static final double AUTO_THETA_I = 0;//.035;
    public static final double AUTO_THETA_D = 0;//4.5;

}
