package frc.robot.Drive;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ModuleIO extends SubsystemBase{
    
    public void setDesiredState(SwerveModuleState state) {
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState();
    }

    public double getDriveVoltage() {
        return 0;
    }

    public void setVolts(double driveVolts, double pivotVolts) {
    }
    
}
