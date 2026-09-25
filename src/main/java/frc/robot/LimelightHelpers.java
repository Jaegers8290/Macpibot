package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightHelpers {

    public static NetworkTable getLimelightNTTable(String tableName) {
        if (tableName == null || tableName.isEmpty()) {
            return NetworkTableInstance.getDefault().getTable("limelight");
        }
        return NetworkTableInstance.getDefault().getTable(tableName);
    }

    public static NetworkTableEntry getLimelightNTTableEntry(String tableName, String entryName) {
        return getLimelightNTTable(tableName).getEntry(entryName);
    }

    public static double getLimelightNTDouble(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getDouble(0.0);
    }

    public static void setLimelightNTDouble(String tableName, String entryName, double val) {
        getLimelightNTTableEntry(tableName, entryName).setDouble(val);
    }

    public static boolean getTV(String limelightName) {
        return getLimelightNTDouble(limelightName, "tv") == 1.0;
    }

    public static double getTX(String limelightName) {
        return getLimelightNTDouble(limelightName, "tx");
    }

    public static double getTY(String limelightName) {
        return getLimelightNTDouble(limelightName, "ty");
    }

    public static double getTA(String limelightName) {
        return getLimelightNTDouble(limelightName, "ta");
    }

    public static double getFiducialID(String limelightName) {
        return getLimelightNTDouble(limelightName, "tid");
    }

    public static void setPipelineIndex(String limelightName, int pipelineIndex) {
        setLimelightNTDouble(limelightName, "pipeline", pipelineIndex);
    }

    public static void setLEDMode_PipelineControl(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 0);
    }

    public static void setLEDMode_ForceOff(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 1);
    }

    public static void setLEDMode_ForceBlink(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 2);
    }

    public static void setLEDMode_ForceOn(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 3);
    }
}
