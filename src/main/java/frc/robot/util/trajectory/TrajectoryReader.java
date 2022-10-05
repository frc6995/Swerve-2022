package frc.robot.util.trajectory;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Hashtable;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TrajectoryReader {

    /** CSV format:
     * One frame (0.02 s dt) per line.
     * Velocities are per frame. i.e. 0.001 vx = 0.001 meters per (0.02 s)
     * x(m),vx(m/f),y(m),vy(m/f),theta(rad),omega(rad/f)
     */
    public static List<Pose2d> readFilePoseList(File file) {
        List<Pose2d> poseList = new ArrayList<Pose2d>();
        int time = 0;
        try(BufferedReader fileReader
        = new BufferedReader(new FileReader(file)))
        {
            String line = "";

            //Read the file line by line
            while ((line = fileReader.readLine()) != null)
            {
                //Get all tokens available in line
                String[] tokens = line.split(",");
                poseList.add( time,
                    new Pose2d(
                        new Translation2d(
                            Double.parseDouble(tokens[0]),
                            Double.parseDouble(tokens[1])),
                        new Rotation2d(
                            Double.parseDouble(tokens[2])
                        )
                    )
                );
                time += 1;
            }
        }
        catch (IOException e) {
            e.printStackTrace();
        }

        return poseList;
    }

    public static Hashtable<Integer, State> readFileTrajectory(File file) {

        Hashtable<Integer, State> statesList = new Hashtable<>();

        int time = 0;
        try(BufferedReader fileReader
        = new BufferedReader(new FileReader(file)))
        {
            String line = "";

            //Read the file line by line
            while ((line = fileReader.readLine()) != null)
            {
                //Get all tokens available in line
                String[] tokens = line.split(",");
                statesList.put(time, new TrajectoryReader.State(
                    time,
                    new Pose2d(
                        new Translation2d(
                            Double.parseDouble(tokens[0]),
                            Double.parseDouble(tokens[1])),
                        new Rotation2d(
                            Double.parseDouble(tokens[2])
                        )
                    ),
                    0,
                    0,
                    0
                ));
                time += 1;
            }
            fileReader.close();
        }
        catch (IOException e) {
            e.printStackTrace();
        }
        // trapezoidal integration for velocity
        for(int i = 0; i < time; i++) {
            State currState = getState(i, statesList);
            State prevState = getState(i-1, statesList);
            State nextState = getState(i+1, statesList);
            currState.omegaRadiansPerSecond = 
                (
                    nextState.poseMeters.getRotation().getRadians()
                - prevState.poseMeters.getRotation().getRadians()
                ) / 0.04; //dt = 0.04 s

            currState.vxMetersPerSecond = 
                (
                    nextState.poseMeters.getX()
                - prevState.poseMeters.getX()
                ) / 0.04; //dt = 0.04 s
            
                currState.vyMetersPerSecond = 
                (
                    nextState.poseMeters.getY()
                - prevState.poseMeters.getY()
                ) / 0.04; //dt = 0.04 s

        }

        return statesList;
    }

    public static Pose2d getPose(int i, List<Pose2d> list) {
        if (i < 0) {
            return list.get(0);
        } else if (i >= list.size()) {
            return list.get(list.size()-1);
        } else {
            return list.get(i);
        }
    }

    public static State getState(int i, Hashtable<Integer, State> list) {
        if (i < 0) {
            return list.get(0);
        } else if (i >= list.size()) {
            return list.get(list.size()-1);
        } else {
            return list.get(i);
        }
    }

    public static class State {
        double time;
        Pose2d poseMeters;
        double vxMetersPerSecond;
        double vyMetersPerSecond;
        double omegaRadiansPerSecond;

        public State(double time, Pose2d poseMeters, double vxMetersPerSecond, double vyMetersPerSecond,
                double omegaRadiansPerSecond) {
            this.time = time;
            this.poseMeters = poseMeters;
            this.vxMetersPerSecond = vxMetersPerSecond;
            this.vyMetersPerSecond = vyMetersPerSecond;
            this.omegaRadiansPerSecond = omegaRadiansPerSecond;
        }


    }
}
