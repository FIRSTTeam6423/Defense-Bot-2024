// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Rotation;

import javax.lang.model.element.ModuleElement.ProvidesDirective;

import com.pathplanner.lib.path.RotationTarget;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SimModule extends ModuleIO {
	/** Creates a new SwerveModule. */
	//2 states (position, velocity), 1 input (voltage),  2 output (position, velocity)
	private LinearSystemSim<N2, N1, N2> driveMotor = new LinearSystemSim<N2, N1, N2> (
		LinearSystemId.createDCMotorSystem(
			DriveConstants.kV, //volts per meter per second
			DriveConstants.kA
		)
	);
	
	//We aren't sure what the constants are for our pivot motors, but they work fine with just P controller 
	//on real robot so as long as they work fine in the sim too that's okay. We are mostly concerned with
	//our drive motors.
	// private LinearSystemSim<N2, N1, N2> pivotMotor = new LinearSystemSim<N2, N1, N2> (
	// 	LinearSystemId.createDCMotorSystem(
	// 		2, 
	// 		.2
	// 	)
	// );
	private Rotation2d simRotation = Rotation2d.fromDegrees(0);

	// Gains are for example purposes only - must be determined for your own robot!
	private PIDController drivePIDController, pivotPIDController;

	private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
			DriveConstants.kS,
			DriveConstants.kV,
			DriveConstants.kA);

	private int encoderID;

	private SwerveModuleState state;

	public SimModule(int driveMotorID, boolean driveInverted, int pivotMotorID, int encoderID, boolean pivotInverted) {
		this.encoderID = encoderID;
		/**
		 * We need three encoders, as the sparkmax can only accurately tell
		 * the rpm of the motor, not its position. A third ecnoder needs to handle
		 * that and pass that in to the PIDController
		 **/
		// driveEncoder.setPositionConversionFactor(Constants.DRIVE_ROTATIONS_TO_METERS);
		// driveEncoder.setVelocityConversionFactor(Constants.RPM_TO_METERS_PER_SEC);
		// driveEncoder.setPosition(0);

		drivePIDController = new PIDController(Constants.MODULEDRIVE_P, Constants.MODULEDRIVE_I,
				Constants.MODULEDRIVE_D);

		pivotPIDController = new PIDController(.25, Constants.MODULEPIVOT_I,
				Constants.MODULEPIVOT_D);
		pivotPIDController.enableContinuousInput(-90, 90);
		
		state = getState();
	}

	public Rotation2d getPivotRotation() {
		//return Rotation2d.fromDegrees(Math.toDegrees(pivotMotor.getAngularPositionRad())); // -
		return simRotation;																					// Constants.ABS_ENCODER_OFFSETS[this.encoderID]);
	}

	public double getDriveVelocity() {
		//row 1 (second row) is the VELOCITY of the system, in meters per second
		return driveMotor.getOutput(1); 
	}

	public double getDrivePositionMeters() {
		//row 0 (first row) is the POSITION of the system, in meters
		return driveMotor.getOutput(0);
	}

	@Override
	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(getDrivePositionMeters(), getPivotRotation());
	}

	@Override
	public SwerveModuleState getState() {
		return new SwerveModuleState(
				getDriveVelocity(),
				getPivotRotation());
	}

	@Override
	public void setDesiredState(SwerveModuleState desiredState) {
		// Optimize the reference state to avoid spinning further than 90 degrees
		double curRotDeg = getPivotRotation().getDegrees();
		//System.out.println("bruh" + RobotController.getFPGATime());
		state = desiredState;//SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(curRotDeg));
		// Different constant need for drivePIDController, convert m/s to rpm
		driveMotor.setInput(
				driveFeedforward.calculate(desiredState.speedMetersPerSecond) +
						drivePIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond));
		simRotation = desiredState.angle;
		SmartDashboard.putNumber("DRIVE VEL", getDriveVelocity());
		SmartDashboard.putNumber("Desired Drive Vel", desiredState.speedMetersPerSecond);
		SmartDashboard.putNumber("MOTOR INPUT ", pivotPIDController.calculate(curRotDeg, state.angle.getDegrees()));
		// SmartDashboard.putNumber("bruh", );
	}

	public void stopModule() {
		driveMotor.setInput(0);
		//pivotMotor.setInput(0);
	}

	public void resetEncoders() {
		driveMotor.setState(VecBuilder.fill(0, 0));
		simRotation = Rotation2d.fromDegrees(0);
	}

	@Override
	public void simulationPeriodic() {
		driveMotor.update(0.020);
		//pivotMotor.update(0.020);
	}

	@Override
	public void periodic() {

	}
}