// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import me.wobblyyyy.pathfinder2.geometry.PointXYZ;
import me.wobblyyyy.pathfinder2.robot.AbstractOdometry;

public class Odometry extends AbstractOdometry {


 public class ExampleOdometryTester {
    public void testOdometry() {
        Odometry odometry = new Odometry();
    
        while (true) {
            System.out.println(odometry.getPosition());
            }
        }
    }


    @Override
    public PointXYZ getRawPosition() {
        // your odometry code goes here. this method should return the robot's
        // position, without modifying it in any way

        return new PointXYZ(0, 0, 0);
    }
}