// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.core.Mat.Tuple2;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SwerveModule extends ModuleIO {
	/** Creates a new SwerveModule. */
	private CANSparkMax driveMotor, pivotMotor;

	private RelativeEncoder driveEncoder;
	private DutyCycleEncoder pivotEncoder;

	// Gains are for example purposes only - must be determined for your own robot!
	private PIDController drivePIDController, pivotPIDController;
	private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
		DriveConstants.kS, 
		DriveConstants.kV, 
		DriveConstants.kA
	);

	private int encoderID;

	private SwerveModuleState state;

	public SwerveModule(int driveMotorID, boolean driveInverted, int pivotMotorID, int encoderID, boolean pivotInverted) {
		driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
		driveMotor.setInverted(driveInverted);
		pivotMotor = new CANSparkMax(pivotMotorID, MotorType.kBrushless);
		pivotMotor.setInverted(pivotInverted);
		this.encoderID = encoderID;
		/**
		 * We need three encoders, as the sparkmax can only accurately tell
		 * the rpm of the motor, not its position. A third ecnoder needs to handle
		 * that and pass that in to the PIDController
		 **/
		driveEncoder = driveMotor.getEncoder();
		driveEncoder.setPositionConversionFactor(Constants.DRIVE_ROTATIONS_TO_METERS);
		driveEncoder.setVelocityConversionFactor(Constants.RPM_TO_METERS_PER_SEC);
		driveEncoder.setPosition(0);

		pivotEncoder = new DutyCycleEncoder(encoderID);
		pivotEncoder.reset();

		drivePIDController = new PIDController(Constants.MODULEDRIVE_P, Constants.MODULEDRIVE_I, Constants.MODULEDRIVE_D);
		drivePIDController.setP(Constants.MODULEDRIVE_P);
		drivePIDController.setI(Constants.MODULEDRIVE_I);
		drivePIDController.setD(Constants.MODULEDRIVE_D);

		pivotPIDController = new PIDController(Constants.MODULEPIVOT_P, Constants.MODULEPIVOT_I, Constants.MODULEPIVOT_D);
		pivotPIDController.enableContinuousInput(-90,90);
		pivotPIDController.setP(Constants.MODULEPIVOT_P);
		pivotPIDController.setI(Constants.MODULEPIVOT_I);
		pivotPIDController.setD(Constants.MODULEPIVOT_D);

		state = getState();
	}

	@Override
	public SwerveModuleState getState() {
		return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromDegrees(pivotEncoder.getAbsolutePosition() * 360 - Constants.ABS_ENCODER_OFFSETS[this.encoderID]));
	}

	@Override
	public double getDriveVoltage() {
		return driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
	}

	public SwerveModulePosition getPosition() {
		Rotation2d r = Rotation2d.fromDegrees(pivotEncoder.getAbsolutePosition() * 360 - Constants.ABS_ENCODER_OFFSETS[this.encoderID]);
		return new SwerveModulePosition(driveEncoder.getPosition(), r);
	}

	@Override
	public void setDesiredState(SwerveModuleState desiredState) {
		// Optimize the reference state to avoid spinning further than 90 degrees
		double curRotDeg = pivotEncoder.getAbsolutePosition() * 360 - Constants.ABS_ENCODER_OFFSETS[this.encoderID];//-pivotEncoder.getAbsolutePosition() * 360 - Constants.ABS_ENCODER_OFFSETS[this.encoderID];
		//if(this.encoderID == 7) {
		//}"
		state = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(curRotDeg));
		// Different constant need for drivePIDController, convert m/s to rpm
		driveMotor.setVoltage(
			driveFeedforward.calculate(state.speedMetersPerSecond) +
			drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond)
		);
		pivotMotor.set(pivotPIDController.calculate(curRotDeg, state.angle.getDegrees()));
		SmartDashboard.putNumber("DRIVE VEL", driveEncoder.getVelocity());
		//speed = rad per sec * circumference
		//rad per sec = speed / circumference
		SmartDashboard.putNumber("drive vel target rad per sec", Units.radiansToDegrees(state.speedMetersPerSecond / Constants.WHEEL_CIRCUMFERENCE_METERS));
	}

	@Override
	public void setVolts(double driveVolts, double pivotVolts) {
		driveMotor.setVoltage(driveVolts);
		pivotMotor.setVoltage(pivotVolts);
	}

	public void stopModule() {
		driveMotor.set(0);
		pivotMotor.set(0);
	}

	public void resetEncoders() {
		driveEncoder.setPosition(0);
		pivotEncoder.reset();
	}

	public double pivotValue(){
		return pivotEncoder.getAbsolutePosition();
	}

	public double drivePosValue(){
		return driveEncoder.getPosition();
	}

	public double driverVelValue(){
		return driveEncoder.getVelocity();
	}

	public double stateSpeed(){
		return state.speedMetersPerSecond;
	}

	public double stateAngle(){
		return state.angle.getDegrees();
	}

	@Override
	public void periodic() {

	}
}