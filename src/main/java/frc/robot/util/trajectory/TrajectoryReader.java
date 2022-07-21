package frc.robot.util.trajectory;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
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
                            Double.parseDouble(tokens[2])),
                        new Rotation2d(
                            Double.parseDouble(tokens[4])
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

    public static List<State> readFileTrajectory(File file) {
        List<State> statesList = new ArrayList<>();
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
                statesList.add(time, new TrajectoryReader.State(
                    time,
                    new Pose2d(
                        new Translation2d(
                            Double.parseDouble(tokens[0]),
                            Double.parseDouble(tokens[2])),
                        new Rotation2d(
                            Double.parseDouble(tokens[4])
                        )
                    ),
                    Double.parseDouble(tokens[1]) * 50,
                    Double.parseDouble(tokens[3]) * 50,
                    Double.parseDouble(tokens[5]) * 50
                ));
                time += 1;
            }
        }
        catch (IOException e) {
            e.printStackTrace();
        }

        return statesList;
    }

    private static Pose2d getPose(int i, List<Pose2d> list) {
        if (i < 0) {
            return list.get(0);
        } else if (i >= list.size()) {
            return list.get(list.size()-1);
        } else {
            return list.get(i);
        }
    }

    public static class State {
        public final double time;
        public final Pose2d poseMeters;
        public final double vxMetersPerSecond;
        public final double vyMetersPerSecond;
        public final double omegaRadiansPerSecond;

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
