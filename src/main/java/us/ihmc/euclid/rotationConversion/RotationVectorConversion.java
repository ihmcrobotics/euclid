package us.ihmc.euclid.rotationConversion;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationScaleMatrixReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.Matrix3DFeatures;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollReadOnly;

/**
 * This class gathers all the methods necessary to converts any type of rotation into a rotation
 * vector.
 * <p>
 * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
 * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
 * same axis-angle.
 * </p>
 * <p>
 * To convert an orientation into other data structure types see:
 * <ul>
 * <li>for axis-angle: {@link AxisAngleConversion},
 * <li>for quaternion: {@link QuaternionConversion},
 * <li>for rotation matrix: {@link RotationMatrixConversion},
 * <li>for yaw-pitch-roll: {@link YawPitchRollConversion}.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public abstract class RotationVectorConversion
{
   /**
    * Tolerance used to identify various edge cases, such as to identify when an axis-angle represents
    * a zero orientation.
    */
   public static final double EPS = 1.0e-12;

   /**
    * Converts the given axis-angle into a rotation vector.
    * <p>
    * After calling this method, the axis-angle and the rotation vector represent the same orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if either component of the axis-angle is {@link Double#NaN}, the rotation vector is set to
    * {@link Double#NaN}.
    * <li>if the length of the axis is below {@link #EPS}, the rotation vector is set to zero.
    * </ul>
    * </p>
    *
    * @param axisAngle the axis-angle to use for the conversion. Not modified.
    * @param rotationVectorToPack the vector in which the result is stored. Modified.
    */
   public static void convertAxisAngleToRotationVector(AxisAngleReadOnly axisAngle, Vector3DBasics rotationVectorToPack)
   {
      convertAxisAngleToRotationVectorImpl(axisAngle.getX(), axisAngle.getY(), axisAngle.getZ(), axisAngle.getAngle(), rotationVectorToPack);
   }

   /**
    * Converts the given axis-angle into a rotation vector.
    * <p>
    * After calling this method, the axis-angle and the rotation vector represent the same orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if either component of the axis-angle is {@link Double#NaN}, the rotation vector is set to
    * {@link Double#NaN}.
    * <li>if the length of the axis is below {@link #EPS}, the rotation vector is set to zero.
    * </ul>
    * </p>
    *
    * @param ux the axis x-component of the axis-angle to use for the conversion.
    * @param uy the axis y-component of the axis-angle to use for the conversion.
    * @param uz the axis z-component of the axis-angle to use for the conversion.
    * @param angle the angle of the axis-angle to use for the conversion.
    * @param rotationVectorToPack the vector in which the result is stored. Modified.
    */
   public static void convertAxisAngleToRotationVectorImpl(double ux, double uy, double uz, double angle, Vector3DBasics rotationVectorToPack)
   {
      if (EuclidCoreTools.containsNaN(ux, uy, uz, angle))
      {
         rotationVectorToPack.setToNaN();
         return;
      }

      double uNorm = EuclidCoreTools.norm(ux, uy, uz);

      if (uNorm > EPS)
      {
         uNorm = 1.0 / uNorm;
         rotationVectorToPack.setX(ux * uNorm * angle);
         rotationVectorToPack.setY(uy * uNorm * angle);
         rotationVectorToPack.setZ(uz * uNorm * angle);
      }
      else
      {
         rotationVectorToPack.setToZero();
      }
   }

   /**
    * Converts the given quaternion into a rotation vector.
    * <p>
    * After calling this method, the quaternion and the rotation vector represent the same orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if either component of the quaternion is {@link Double#NaN}, the rotation vector is set to
    * {@link Double#NaN}.
    * <li>if the norm of the quaternion is below {@link #EPS}, the rotation vector is set to zero.
    * </ul>
    * </p>
    *
    * @param quaternion the quaternion to use for the conversion. Not modified.
    * @param rotationVectorToPack the vector in which the result is stored. Modified.
    */
   public static void convertQuaternionToRotationVector(QuaternionReadOnly quaternion, Vector3DBasics rotationVectorToPack)
   {
      if (quaternion.containsNaN())
      {
         rotationVectorToPack.setToNaN();
         return;
      }

      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      double uNorm = Math.sqrt(EuclidCoreTools.normSquared(qx, qy, qz));

      if (uNorm > EPS)
      {
         double angle = 2.0 * Math.atan2(uNorm, qs) / uNorm;
         rotationVectorToPack.setX(qx * angle);
         rotationVectorToPack.setY(qy * angle);
         rotationVectorToPack.setZ(qz * angle);
      }
      else
      {
         // Small angle approximation
         // "A Primer on the Differential Calculus of 3D Orientations" M. Bloesh et al
         double sign = Math.signum(qs);
         rotationVectorToPack.setX(sign * qx);
         rotationVectorToPack.setY(sign * qy);
         rotationVectorToPack.setZ(sign * qz);
      }
   }

   /**
    * Converts the rotation part of the given rotation-scale matrix into a rotation vector.
    * <p>
    * After calling this method, the rotation matrix and the rotation vector represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation matrix contains at least one {@link Double#NaN}, the rotation vector is set
    * to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param rotationScaleMatrix a 3-by-3 matrix representing an orientation and a scale. Only the
    *           orientation part is used during the conversion. Not modified.
    * @param rotationVectorToPack the vector in which the result is stored. Modified.
    */
   public static void convertMatrixToRotationVector(RotationScaleMatrixReadOnly rotationScaleMatrix, Vector3DBasics rotationVectorToPack)
   {
      convertMatrixToRotationVector(rotationScaleMatrix.getRotationMatrix(), rotationVectorToPack);
   }

   /**
    * Converts the given rotation matrix into a rotation vector.
    * <p>
    * After calling this method, the rotation matrix and the rotation vector represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation matrix contains at least one {@link Double#NaN}, the rotation vector is set
    * to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param rotationMatrix a 3-by-3 matrix representing an orientation. Not modified.
    * @param rotationVectorToPack the vector in which the result is stored. Modified.
    */
   public static void convertMatrixToRotationVector(RotationMatrixReadOnly rotationMatrix, Vector3DBasics rotationVectorToPack)
   {
      double m00 = rotationMatrix.getM00();
      double m01 = rotationMatrix.getM01();
      double m02 = rotationMatrix.getM02();
      double m10 = rotationMatrix.getM10();
      double m11 = rotationMatrix.getM11();
      double m12 = rotationMatrix.getM12();
      double m20 = rotationMatrix.getM20();
      double m21 = rotationMatrix.getM21();
      double m22 = rotationMatrix.getM22();

      convertMatrixToRotationVectorImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, rotationVectorToPack);
   }

   /**
    * Converts the given rotation matrix into a rotation vector.
    * <p>
    * <b> This method is for internal use. Use
    * {@link #convertMatrixToRotationVector(RotationMatrixReadOnly, Vector3DBasics)} or
    * {@link #convertMatrixToRotationVector(RotationScaleMatrixReadOnly, Vector3DBasics)} instead. </b>
    * </p>
    * <p>
    * After calling this method, the rotation matrix and the rotation vector represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation matrix contains at least one {@link Double#NaN}, the rotation vector is set
    * to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param m00 the new 1st row 1st column coefficient for the matrix to use for the conversion.
    * @param m01 the new 1st row 2nd column coefficient for the matrix to use for the conversion.
    * @param m02 the new 1st row 3rd column coefficient for the matrix to use for the conversion.
    * @param m10 the new 2nd row 1st column coefficient for the matrix to use for the conversion.
    * @param m11 the new 2nd row 2nd column coefficient for the matrix to use for the conversion.
    * @param m12 the new 2nd row 3rd column coefficient for the matrix to use for the conversion.
    * @param m20 the new 3rd row 1st column coefficient for the matrix to use for the conversion.
    * @param m21 the new 3rd row 2nd column coefficient for the matrix to use for the conversion.
    * @param m22 the new 3rd row 3rd column coefficient for the matrix to use for the conversion.
    * @param rotationVectorToPack the vector in which the result is stored. Modified.
    */
   static void convertMatrixToRotationVectorImpl(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22,
                                                 Vector3DBasics rotationVectorToPack)
   {
      if (EuclidCoreTools.containsNaN(m00, m01, m02, m10, m11, m12, m20, m21, m22))
      {
         rotationVectorToPack.setToNaN();
         return;
      }

      double angle, x, y, z; // variables for result

      x = m21 - m12;
      y = m02 - m20;
      z = m10 - m01;

      double s = Math.sqrt(EuclidCoreTools.normSquared(x, y, z));

      if (s > EPS)
      {
         double sin = 0.5 * s;
         double cos = 0.5 * (m00 + m11 + m22 - 1.0);
         angle = Math.atan2(sin, cos);
         x /= s;
         y /= s;
         z /= s;
      }
      else if (Matrix3DFeatures.isIdentity(m00, m01, m02, m10, m11, m12, m20, m21, m22))
      {
         rotationVectorToPack.setToZero();
         return;
      }
      else
      {
         // otherwise this singularity is angle = 180
         angle = Math.PI;
         double xx = 0.50 * (m00 + 1.0);
         double yy = 0.50 * (m11 + 1.0);
         double zz = 0.50 * (m22 + 1.0);
         double xy = 0.25 * (m01 + m10);
         double xz = 0.25 * (m02 + m20);
         double yz = 0.25 * (m12 + m21);

         if (xx > yy && xx > zz)
         { // m00 is the largest diagonal term
            x = Math.sqrt(xx);
            y = xy / x;
            z = xz / x;
         }
         else if (yy > zz)
         { // m11 is the largest diagonal term
            y = Math.sqrt(yy);
            x = xy / y;
            z = yz / y;
         }
         else
         { // m22 is the largest diagonal term so base result on this
            z = Math.sqrt(zz);
            x = xz / z;
            y = yz / z;
         }
      }

      rotationVectorToPack.setX(x * angle);
      rotationVectorToPack.setY(y * angle);
      rotationVectorToPack.setZ(z * angle);
   }

   /**
    * Converts the given yaw-pitch-roll angles into a rotation vector.
    * <p>
    * After calling this method, the yaw-pitch-roll and the rotation vector represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if either of the yaw, pitch, or roll angle is {@link Double#NaN}, the rotation vector is set
    * to {@link Double#NaN}.
    * </ul>
    * </p>
    * <p>
    * Note: the yaw-pitch-roll representation, also called Euler angles, corresponds to the
    * representation of an orientation by decomposing it by three successive rotations around the three
    * axes: Z (yaw), Y (pitch), and X (roll). The equivalent rotation matrix of such representation is:
    * <br>
    * R = R<sub>Z</sub>(yaw) * R<sub>Y</sub>(pitch) * R<sub>X</sub>(roll) </br>
    * </p>
    *
    * @param yawPitchRoll the yaw-pitch-roll angles to use in the conversion. Not modified.
    * @param rotationVectorToPack the vector in which the result is stored. Modified.
    * @deprecated Use
    *             {@link #convertYawPitchRollToRotationVector(YawPitchRollReadOnly, Vector3DBasics)}
    *             instead.
    */
   public static void convertYawPitchRollToRotationVector(double[] yawPitchRoll, Vector3DBasics rotationVectorToPack)
   {
      convertYawPitchRollToRotationVector(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2], rotationVectorToPack);
   }

   /**
    * Converts the given yaw-pitch-roll angles into a rotation vector.
    * <p>
    * After calling this method, the yaw-pitch-roll and the rotation vector represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if either of the yaw, pitch, or roll angle is {@link Double#NaN}, the rotation vector is set
    * to {@link Double#NaN}.
    * </ul>
    * </p>
    * <p>
    * Note: the yaw-pitch-roll representation, also called Euler angles, corresponds to the
    * representation of an orientation by decomposing it by three successive rotations around the three
    * axes: Z (yaw), Y (pitch), and X (roll). The equivalent rotation matrix of such representation is:
    * <br>
    * R = R<sub>Z</sub>(yaw) * R<sub>Y</sub>(pitch) * R<sub>X</sub>(roll) </br>
    * </p>
    *
    * @param yawPitchRoll the yaw-pitch-roll angles to use in the conversion. Not modified.
    * @param rotationVectorToPack the vector in which the result is stored. Modified.
    */
   public static void convertYawPitchRollToRotationVector(YawPitchRollReadOnly yawPitchRoll, Vector3DBasics rotationVectorToPack)
   {
      convertYawPitchRollToRotationVector(yawPitchRoll.getYaw(), yawPitchRoll.getPitch(), yawPitchRoll.getRoll(), rotationVectorToPack);
   }

   /**
    * Converts the given yaw-pitch-roll angles into a rotation vector.
    * <p>
    * After calling this method, the yaw-pitch-roll and the rotation vector represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if either of the yaw, pitch, or roll angle is {@link Double#NaN}, the rotation vector is set
    * to {@link Double#NaN}.
    * </ul>
    * </p>
    * <p>
    * Note: the yaw-pitch-roll representation, also called Euler angles, corresponds to the
    * representation of an orientation by decomposing it by three successive rotations around the three
    * axes: Z (yaw), Y (pitch), and X (roll). The equivalent rotation matrix of such representation is:
    * <br>
    * R = R<sub>Z</sub>(yaw) * R<sub>Y</sub>(pitch) * R<sub>X</sub>(roll) </br>
    * </p>
    *
    * @param yaw the yaw angle to use in the conversion.
    * @param pitch the pitch angle to use in the conversion.
    * @param roll the roll angle to use in the conversion.
    * @param rotationVectorToPack the vector in which the result is stored. Modified.
    */
   public static void convertYawPitchRollToRotationVector(double yaw, double pitch, double roll, Vector3DBasics rotationVectorToPack)
   {
      if (EuclidCoreTools.containsNaN(yaw, pitch, roll))
      {
         rotationVectorToPack.setToNaN();
         return;
      }

      double halfYaw = yaw / 2.0;
      double cYaw = Math.cos(halfYaw);
      double sYaw = Math.sin(halfYaw);

      double halfPitch = pitch / 2.0;
      double cPitch = Math.cos(halfPitch);
      double sPitch = Math.sin(halfPitch);

      double halfRoll = roll / 2.0;
      double cRoll = Math.cos(halfRoll);
      double sRoll = Math.sin(halfRoll);

      double qs = cYaw * cPitch * cRoll + sYaw * sPitch * sRoll;
      double qx = cYaw * cPitch * sRoll - sYaw * sPitch * cRoll;
      double qy = sYaw * cPitch * sRoll + cYaw * sPitch * cRoll;
      double qz = sYaw * cPitch * cRoll - cYaw * sPitch * sRoll;

      double uNorm = Math.sqrt(EuclidCoreTools.normSquared(qx, qy, qz));

      if (uNorm > EPS)
      {
         double angle = 2.0 * Math.atan2(uNorm, qs) / uNorm;
         rotationVectorToPack.setX(qx * angle);
         rotationVectorToPack.setY(qy * angle);
         rotationVectorToPack.setZ(qz * angle);
      }
      else
      {
         rotationVectorToPack.setToZero();
      }
   }
}
