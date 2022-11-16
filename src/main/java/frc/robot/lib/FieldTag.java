package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;

public class FieldTag {
    private Pose2d pose;
    private int id;
    public FieldTag(int id, Pose2d pose){
        this.pose = pose;
        this.id = id;
    }
    public int getID(){return id;}
    public Pose2d getPose(){return pose;}
}
