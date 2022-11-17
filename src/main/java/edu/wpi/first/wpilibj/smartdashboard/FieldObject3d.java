// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.wpilibj.smartdashboard;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableEntry;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;

/** Game field object on a Field3d. */
public class FieldObject3d {
  private final int kPose3dSize = 7;
  /**
   * Package-local constructor.
   *
   * @param name name
   */
  FieldObject3d(String name) {
    m_name = name;
  }

  /**
   * Set the pose from a Pose object.
   *
   * @param pose 3d pose
   */
  public synchronized void setPose(Pose3d pose) {
    setPoses(pose);
  }

  /**
   * Set the pose from x, y, and rotation.
   *
   * @param xMeters X location, in meters
   * @param yMeters Y location, in meters
   * @param rotation rotation
   */
  @SuppressWarnings("ParameterName")
  public synchronized void setPose(double xMeters, double yMeters, double zMeters, Rotation3d rotation) {
    setPose(new Pose3d(xMeters, yMeters, zMeters, rotation));
  }

  /**
   * Get the pose.
   *
   * @return 3d pose
   */
  public synchronized Pose3d getPose() {
    updateFromEntry();
    if (m_poses.isEmpty()) {
      return new Pose3d();
    }
    return m_poses.get(0);
  }

  /**
   * Set multiple poses from an list of Pose objects. The total number of poses is limited to 85.
   *
   * @param poses list of 3d poses
   */
  public synchronized void setPoses(List<Pose3d> poses) {
    m_poses.clear();
    for (Pose3d pose : poses) {
      m_poses.add(pose);
    }
    updateEntry();
  }

  /**
   * Set multiple poses from an list of Pose objects. The total number of poses is limited to 85.
   *
   * @param poses list of 3d poses
   */
  public synchronized void setPoses(Pose3d... poses) {
    m_poses.clear();
    for (Pose3d pose : poses) {
      m_poses.add(pose);
    }
    updateEntry();
  }

  /**
   * Sets poses from a trajectory.
   *
   * @param trajectory The trajectory from which the poses should be added.
   */
  public synchronized void setTrajectory(Trajectory trajectory) {
    m_poses.clear();
    for (Trajectory.State state : trajectory.getStates()) {
      m_poses.add(
        new Pose3d(
            state.poseMeters.getX(), 
            state.poseMeters.getY(),
            0,
            new Rotation3d(0, 0, state.poseMeters.getRotation().getRadians())));
    }
    updateEntry();
  }

  /**
   * Get multiple poses.
   *
   * @return list of 3d poses
   */
  public synchronized List<Pose3d> getPoses() {
    updateFromEntry();
    return new ArrayList<Pose3d>(m_poses);
  }

  void updateEntry() {
    updateEntry(false);
  }

  synchronized void updateEntry(boolean setDefault) {
    if (m_entry == null) {
      return;
    }

    if (m_poses.size() < (255 / kPose3dSize)) {
      double[] arr = new double[m_poses.size() * kPose3dSize];
      int ndx = 0;
      for (Pose3d pose : m_poses) {
        Translation3d translation = pose.getTranslation();
        Rotation3d rotation = pose.getRotation();
        arr[ndx + 0] = translation.getX();
        arr[ndx + 1] = translation.getY();
        arr[ndx + 2] = translation.getZ();
        arr[ndx + 3] = rotation.getQuaternion().getW();
        arr[ndx + 4] = rotation.getQuaternion().getX();
        arr[ndx + 5] = rotation.getQuaternion().getY();
        arr[ndx + 6] = rotation.getQuaternion().getZ();
        ndx += kPose3dSize;
      }

      if (setDefault) {
        m_entry.setDefaultDoubleArray(arr);
      } else {
        m_entry.setDoubleArray(arr);
      }
    } else {
      // send as raw array of doubles if too big for NT array
      ByteBuffer output = ByteBuffer.allocate(m_poses.size() * kPose3dSize * 8);
      output.order(ByteOrder.BIG_ENDIAN);

      for (Pose3d pose : m_poses) {
        Translation3d translation = pose.getTranslation();
        output.putDouble(translation.getX());
        output.putDouble(translation.getY());
        output.putDouble(translation.getZ());
        output.putDouble(pose.getRotation().getQuaternion().getW());
        output.putDouble(pose.getRotation().getQuaternion().getX());
        output.putDouble(pose.getRotation().getQuaternion().getY());
        output.putDouble(pose.getRotation().getQuaternion().getZ());
      }

      if (setDefault) {
        m_entry.setDefaultRaw(output.array());
      } else {
        m_entry.forceSetRaw(output.array());
      }
    }
  }

  private synchronized void updateFromEntry() {
    if (m_entry == null) {
      return;
    }

    double[] arr = m_entry.getDoubleArray((double[]) null);
    if (arr != null) {
      if ((arr.length % kPose3dSize) != 0) {
        return;
      }

      m_poses.clear();
      for (int i = 0; i < arr.length; i += kPose3dSize) {
        m_poses.add(
            new Pose3d(
                arr[i],
                arr[i + 1],
                arr[i + 2],
                new Rotation3d(new Quaternion(
                    arr[i + 3],
                    arr[i + 4],
                    arr[i + 5],
                    arr[i + 6]
                ))
            )
        );
      }
    } else {
      // read as raw array of doubles
      byte[] data = m_entry.getRaw((byte[]) null);
      if (data == null) {
        return;
      }

      // must be triples of doubles
      if ((data.length % (kPose3dSize * 8)) != 0) {
        return;
      }
      ByteBuffer input = ByteBuffer.wrap(data);
      input.order(ByteOrder.BIG_ENDIAN);

      m_poses.clear();
      for (int i = 0; i < (data.length / (3 * 8)); i++) {
        double x = input.getDouble();
        double y = input.getDouble();
        double z = input.getDouble();
        double quatW = input.getDouble();
        double quatX = input.getDouble();
        double quatY = input.getDouble();
        double quatZ = input.getDouble();

        Quaternion quat = new Quaternion(
            quatW, quatX, quatY, quatZ);
        m_poses.add(new Pose3d(x, y, z, new Rotation3d(quat)));
      }
    }
  }

  String m_name;
  NetworkTableEntry m_entry;
  private final List<Pose3d> m_poses = new ArrayList<>();
}
