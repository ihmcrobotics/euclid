package us.ihmc.euclid.rotationConversion;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.tools.YawPitchRollTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollReadOnly;

/**
 * This class gathers all the methods necessary to converts any type of rotation into a rotation
 * matrix.
 * <p>
 * To convert an orientation into other data structure types see:
 * <ul>
 * <li>for quaternion: {@link QuaternionConversion},
 * <li>for axis-angle: {@link AxisAngleConversion},
 * <li>for rotation vector: {@link RotationVectorConversion},
 * <li>for yaw-pitch-roll: {@link YawPitchRollConversion}.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public abstract class RotationMatrixConversion
{
   /**
    * Tolerance used to identify various edge cases, such as to identify when an axis-angle represents
    * a zero orientation.
    */
   public static final double EPS = 1.0e-12;

   /**
    * Sets the given rotation matrix to represent a counter clockwise rotation around the z-axis of an
    * angle {@code yaw}.
    *
    * <pre>
    *        / cos(yaw) -sin(yaw) 0 \
    * this = | sin(yaw)  cos(yaw) 0 |
    *        \    0         0     1 /
    * </pre>
    *
    * @param yaw          the angle to rotate about the z-axis.
    * @param matrixToPack the rotation matrix in which the result is stored. Modified.
    */
   public static void computeYawMatrix(double yaw, RotationMatrix matrixToPack)
   {
      if (EuclidCoreTools.isAngleZero(yaw, EPS))
      {
         matrixToPack.setToZero();
      }
      else
      {
         double sinYaw = Math.sin(yaw);
         double cosYaw = Math.cos(yaw);
         matrixToPack.setUnsafe(cosYaw, -sinYaw, 0.0, sinYaw, cosYaw, 0.0, 0.0, 0.0, 1.0);
      }
   }

   /**
    * Sets the given rotation matrix to represent a counter clockwise rotation around the y-axis of an
    * angle {@code pitch}.
    *
    * <pre>
    *        /  cos(pitch) 0 sin(pitch) \
    * this = |      0      1     0      |
    *        \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    *
    * @param pitch        the angle to rotate about the y-axis.
    * @param matrixToPack the rotation matrix in which the result is stored. Modified.
    */
   public static void computePitchMatrix(double pitch, RotationMatrix matrixToPack)
   {
      if (EuclidCoreTools.isAngleZero(pitch, EPS))
      {
         matrixToPack.setToZero();
      }
      else
      {
         double sinPitch = Math.sin(pitch);
         double cosPitch = Math.cos(pitch);
         matrixToPack.setUnsafe(cosPitch, 0.0, sinPitch, 0.0, 1.0, 0.0, -sinPitch, 0.0, cosPitch);
      }
   }

   /**
    * Sets the given rotation matrix to represent a counter clockwise rotation around the x-axis of an
    * angle {@code roll}.
    *
    * <pre>
    *        / 1     0          0     \
    * this = | 0 cos(roll) -sin(roll) |
    *        \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param roll         the angle to rotate about the x-axis.
    * @param matrixToPack the rotation matrix in which the result is stored. Modified.
    */
   public static void computeRollMatrix(double roll, RotationMatrix matrixToPack)
   {
      if (EuclidCoreTools.isAngleZero(roll, EPS))
      {
         matrixToPack.setToZero();
      }
      else
      {
         double sinRoll = Math.sin(roll);
         double cosRoll = Math.cos(roll);
         matrixToPack.setUnsafe(1.0, 0.0, 0.0, 0.0, cosRoll, -sinRoll, 0.0, sinRoll, cosRoll);
      }
   }

   /**
    * Converts the given axis-angle into a rotation matrix.
    * <p>
    * After calling this method, the axis-angle and the rotation matrix represent the same orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if either component of the axis-angle is {@link Double#NaN}, the rotation matrix is set to
    * {@link Double#NaN}.
    * <li>if the length of the axis is below {@link #EPS}, the rotation matrix is set to identity.
    * </ul>
    * </p>
    *
    * @param axisAngle    the axis-angle to use for the conversion. Not modified.
    * @param matrixToPack the rotation matrix in which the result is stored. Modified.
    */
   public static void convertAxisAngleToMatrix(AxisAngleReadOnly axisAngle, RotationMatrix matrixToPack)
   {
      convertAxisAngleToMatrix(axisAngle.getX(), axisAngle.getY(), axisAngle.getZ(), axisAngle.getAngle(), matrixToPack);
   }

   /**
    * Converts the given axis-angle into a rotation matrix.
    * <p>
    * After calling this method, the axis-angle and the rotation matrix represent the same orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if either component of the axis-angle is {@link Double#NaN}, the rotation matrix is set to
    * {@link Double#NaN}.
    * <li>if the length of the axis is below {@link #EPS}, the rotation matrix is set to identity.
    * </ul>
    * </p>
    *
    * @param ux           the axis x-component of the axis-angle to use for the conversion.
    * @param uy           the axis y-component of the axis-angle to use for the conversion.
    * @param uz           the axis z-component of the axis-angle to use for the conversion.
    * @param angle        the angle of the axis-angle to use for the conversion.
    * @param matrixToPack the rotation matrix in which the result is stored. Modified.
    */
   public static void convertAxisAngleToMatrix(double ux, double uy, double uz, double angle, RotationMatrix matrixToPack)
   {
      if (EuclidCoreTools.containsNaN(ux, uy, uz, angle))
      {
         matrixToPack.setToNaN();
         return;
      }

      if (EuclidCoreTools.isAngleZero(angle, EPS))
      {
         matrixToPack.setToZero();
         return;
      }

      double uNorm = EuclidCoreTools.fastNorm(ux, uy, uz);

      if (uNorm < EPS)
      {
         matrixToPack.setIdentity();
      }
      else
      {
         uNorm = 1.0 / uNorm;
         double ax = ux * uNorm;
         double ay = uy * uNorm;
         double az = uz * uNorm;

         double sinTheta = Math.sin(angle);
         double cosTheta = Math.cos(angle);
         double t = 1.0 - cosTheta;

         double xz = ax * az;
         double xy = ax * ay;
         double yz = ay * az;

         double m00 = t * ax * ax + cosTheta;
         double m01 = t * xy - sinTheta * az;
         double m02 = t * xz + sinTheta * ay;
         double m10 = t * xy + sinTheta * az;
         double m11 = t * ay * ay + cosTheta;
         double m12 = t * yz - sinTheta * ax;
         double m20 = t * xz - sinTheta * ay;
         double m21 = t * yz + sinTheta * ax;
         double m22 = t * az * az + cosTheta;
         matrixToPack.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      }
   }

   /**
    * Converts the given quaternion into a rotation matrix.
    * <p>
    * After calling this method, the quaternion and the rotation matrix represent the same orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if either component of the quaternion is {@link Double#NaN}, the rotation matrix is set to
    * {@link Double#NaN}.
    * <li>if the norm of the quaternion is below {@link #EPS}, the rotation matrix is set to identity.
    * </ul>
    * </p>
    *
    * @param quaternion   the quaternion to use for the conversion. Not modified.
    * @param matrixToPack the rotation matrix in which the result is stored. Modified.
    */
   public static void convertQuaternionToMatrix(QuaternionReadOnly quaternion, RotationMatrix matrixToPack)
   {
      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      convertQuaternionToMatrix(qx, qy, qz, qs, matrixToPack);
   }

   /**
    * Converts the given quaternion into a rotation matrix.
    * <p>
    * After calling this method, the quaternion and the rotation matrix represent the same orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if either component of the quaternion is {@link Double#NaN}, the rotation matrix is set to
    * {@link Double#NaN}.
    * <li>if the norm of the quaternion is below {@link #EPS}, the rotation matrix is set to identity.
    * </ul>
    * </p>
    *
    * @param qx           the x-component of the quaternion.
    * @param qy           the y-component of the quaternion.
    * @param qz           the z-component of the quaternion.
    * @param qs           the s-component of the quaternion.
    * @param matrixToPack the rotation matrix in which the result is stored. Modified.
    */
   public static void convertQuaternionToMatrix(double qx, double qy, double qz, double qs, RotationMatrix matrixToPack)
   {
      if (EuclidCoreTools.containsNaN(qx, qy, qz, qs))
      {
         matrixToPack.setToNaN();
         return;
      }

      if (QuaternionTools.isNeutralQuaternion(qx, qy, qz, qs, EPS))
      {
         matrixToPack.setToZero();
         return;
      }

      double norm = EuclidCoreTools.fastNorm(qx, qy, qz, qs);

      if (norm < EPS)
      {
         matrixToPack.setIdentity();
         return;
      }

      norm = 1.0 / norm;
      qx *= norm;
      qy *= norm;
      qz *= norm;
      qs *= norm;

      double yy2 = 2.0 * qy * qy;
      double zz2 = 2.0 * qz * qz;
      double xx2 = 2.0 * qx * qx;
      double xy2 = 2.0 * qx * qy;
      double sz2 = 2.0 * qs * qz;
      double xz2 = 2.0 * qx * qz;
      double sy2 = 2.0 * qs * qy;
      double yz2 = 2.0 * qy * qz;
      double sx2 = 2.0 * qs * qx;

      double m00 = 1.0 - yy2 - zz2;
      double m01 = xy2 - sz2;
      double m02 = xz2 + sy2;
      double m10 = xy2 + sz2;
      double m11 = 1.0 - xx2 - zz2;
      double m12 = yz2 - sx2;
      double m20 = xz2 - sy2;
      double m21 = yz2 + sx2;
      double m22 = 1.0 - xx2 - yy2;
      matrixToPack.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Converts the given yaw-pitch-roll angles into a rotation matrix.
    * <p>
    * After calling this method, the yaw-pitch-roll angles and the rotation matrix represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if either of the yaw, pitch, or roll angle is {@link Double#NaN}, the rotation matrix is set
    * to {@link Double#NaN}.
    * </ul>
    * </p>
    * <p>
    * Note: the yaw-pitch-roll representation, also called Euler angles, corresponds to the
    * representation of an orientation by decomposing it by three successive rotations around the three
    * axes: Z (yaw), Y (pitch), and X (roll). The equivalent rotation matrix of such representation is:
    *
    * <pre>
    *  R = R<sub>Z</sub>(yaw) * R<sub>Y</sub>(pitch) * R<sub>X</sub>(roll)
    * </pre>
    *
    * <pre>
    *     / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
    * R = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
    *     \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
    * </pre>
    * </p>
    *
    * @param yawPitchRoll the yaw-pitch-roll angles to use in the conversion. Not modified.
    * @param matrixToPack the rotation matrix in which the result is stored. Modified.
    * @deprecated Use {@link #convertYawPitchRollToMatrix(YawPitchRollReadOnly, RotationMatrix)}
    *             instead.
    */
   @Deprecated
   public static void convertYawPitchRollToMatrix(double[] yawPitchRoll, RotationMatrix matrixToPack)
   {
      convertYawPitchRollToMatrix(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2], matrixToPack);
   }

   /**
    * Converts the given yaw-pitch-roll angles into a rotation matrix.
    * <p>
    * After calling this method, the yaw-pitch-roll angles and the rotation matrix represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if either of the yaw, pitch, or roll angle is {@link Double#NaN}, the rotation matrix is set
    * to {@link Double#NaN}.
    * </ul>
    * </p>
    * <p>
    * Note: the yaw-pitch-roll representation, also called Euler angles, corresponds to the
    * representation of an orientation by decomposing it by three successive rotations around the three
    * axes: Z (yaw), Y (pitch), and X (roll). The equivalent rotation matrix of such representation is:
    *
    * <pre>
    *  R = R<sub>Z</sub>(yaw) * R<sub>Y</sub>(pitch) * R<sub>X</sub>(roll)
    * </pre>
    *
    * <pre>
    *     / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
    * R = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
    *     \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
    * </pre>
    * </p>
    *
    * @param yawPitchRoll the yaw-pitch-roll angles to use in the conversion. Not modified.
    * @param matrixToPack the rotation matrix in which the result is stored. Modified.
    */
   public static void convertYawPitchRollToMatrix(YawPitchRollReadOnly yawPitchRoll, RotationMatrix matrixToPack)
   {
      convertYawPitchRollToMatrix(yawPitchRoll.getYaw(), yawPitchRoll.getPitch(), yawPitchRoll.getRoll(), matrixToPack);
   }

   /**
    * Converts the given yaw-pitch-roll angles into a rotation matrix.
    * <p>
    * After calling this method, the yaw-pitch-roll angles and the rotation matrix represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if either of the yaw, pitch, or roll angle is {@link Double#NaN}, the rotation matrix is set
    * to {@link Double#NaN}.
    * </ul>
    * </p>
    * <p>
    * Note: the yaw-pitch-roll representation, also called Euler angles, corresponds to the
    * representation of an orientation by decomposing it by three successive rotations around the three
    * axes: Z (yaw), Y (pitch), and X (roll). The equivalent rotation matrix of such representation is:
    *
    * <pre>
    *  R = R<sub>Z</sub>(yaw) * R<sub>Y</sub>(pitch) * R<sub>X</sub>(roll)
    * </pre>
    *
    * <pre>
    *     / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
    * R = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
    *     \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
    * </pre>
    * </p>
    *
    * @param yaw          the angle to rotate about the z-axis.
    * @param pitch        the angle to rotate about the y-axis.
    * @param roll         the angle to rotate about the x-axis.
    * @param matrixToPack the rotation matrix in which the result is stored. Modified.
    */
   public static void convertYawPitchRollToMatrix(double yaw, double pitch, double roll, RotationMatrix matrixToPack)
   {
      if (EuclidCoreTools.containsNaN(yaw, pitch, roll))
      {
         matrixToPack.setToNaN();
         return;
      }

      if (YawPitchRollTools.isZero(yaw, pitch, roll, EPS))
      {
         matrixToPack.setToZero();
         return;
      }

      double cosc = Math.cos(yaw);
      double sinc = Math.sin(yaw);

      double cosb = Math.cos(pitch);
      double sinb = Math.sin(pitch);

      double cosa = Math.cos(roll);
      double sina = Math.sin(roll);

      // Introduction to Robotics, 2.64
      double m00 = cosc * cosb;
      double m01 = cosc * sinb * sina - sinc * cosa;
      double m02 = cosc * sinb * cosa + sinc * sina;
      double m10 = sinc * cosb;
      double m11 = sinc * sinb * sina + cosc * cosa;
      double m12 = sinc * sinb * cosa - cosc * sina;
      double m20 = -sinb;
      double m21 = cosb * sina;
      double m22 = cosb * cosa;
      matrixToPack.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Converts the given rotation vector into a rotation matrix.
    * <p>
    * After calling this method, the rotation vector and the rotation matrix represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if either component of the rotation vector is {@link Double#NaN}, the rotation matrix is set
    * to {@link Double#NaN}.
    * </ul>
    * </p>
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param rotationVector the rotation vector to use in the conversion. Not modified.
    * @param matrixToPack   the rotation matrix in which the result is stored. Modified.
    */
   public static void convertRotationVectorToMatrix(Vector3DReadOnly rotationVector, RotationMatrix matrixToPack)
   {
      convertRotationVectorToMatrix(rotationVector.getX(), rotationVector.getY(), rotationVector.getZ(), matrixToPack);
   }

   /**
    * Converts the given rotation vector into a rotation matrix.
    * <p>
    * After calling this method, the rotation vector and the rotation matrix represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if either component of the rotation vector is {@link Double#NaN}, the rotation matrix is set
    * to {@link Double#NaN}.
    * </ul>
    * </p>
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param rx           the x-component of the rotation vector to use in the conversion.
    * @param ry           the y-component of the rotation vector to use in the conversion.
    * @param rz           the z-component of the rotation vector to use in the conversion.
    * @param matrixToPack the rotation matrix in which the result is stored. Modified.
    */
   public static void convertRotationVectorToMatrix(double rx, double ry, double rz, RotationMatrix matrixToPack)
   {
      if (EuclidCoreTools.containsNaN(rx, ry, rz))
      {
         matrixToPack.setToNaN();
         return;
      }

      double norm = EuclidCoreTools.norm(rx, ry, rz);

      if (EuclidCoreTools.isAngleZero(norm, EPS))
      {
         matrixToPack.setIdentity();
      }
      else
      {
         double sinTheta = Math.sin(norm);
         double cosTheta = Math.cos(norm);
         double t = 1.0 - cosTheta;

         norm = 1.0 / norm;
         double ax = rx * norm;
         double ay = ry * norm;
         double az = rz * norm;

         double xz = ax * az;
         double xy = ax * ay;
         double yz = ay * az;

         double m00 = t * ax * ax + cosTheta;
         double m01 = t * xy - sinTheta * az;
         double m02 = t * xz + sinTheta * ay;
         double m10 = t * xy + sinTheta * az;
         double m11 = t * ay * ay + cosTheta;
         double m12 = t * yz - sinTheta * ax;
         double m20 = t * xz - sinTheta * ay;
         double m21 = t * yz + sinTheta * ax;
         double m22 = t * az * az + cosTheta;
         matrixToPack.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      }
   }
}
