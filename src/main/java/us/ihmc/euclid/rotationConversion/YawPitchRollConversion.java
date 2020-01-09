package us.ihmc.euclid.rotationConversion;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationScaleMatrixReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollBasics;

/**
 * This class gathers all the methods necessary to converts any type of rotation into the
 * yaw-pitch-roll angles.
 * <p>
 * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
 * sometimes undefined. Prefer using other representations of 3D orientation such as axis-angle,
 * quaternion, or rotation matrix.
 * </p>
 * <p>
 * To convert an orientation into other data structure types see:
 * <ul>
 * <li>for yaw-pitch-roll: {@link AxisAngleConversion},
 * <li>for quaternion: {@link QuaternionConversion},
 * <li>for rotation matrix: {@link RotationMatrixConversion},
 * <li>for rotation vector: {@link RotationVectorConversion}.
 * </ul>
 * </p>
 * <p>
 * Note: the yaw-pitch-roll representation, also called Euler angles, corresponds to the
 * representation of an orientation by decomposing it by three successive rotations around the three
 * axes: Z (yaw), Y (pitch), and X (roll). The equivalent rotation matrix of such representation is:
 *
 * <pre>
 * R = R<sub>Z</sub>(yaw) * R<sub>Y</sub>(pitch) * R<sub>X</sub>(roll)
 *
 * <sub></sub>          / cos(yaw) -sin(yaw) 0 \
 * R<sub>Z</sub>(yaw) = | sin(yaw)  cos(yaw) 0 |
 * <sub></sub>          \    0         0     1 /
 *
 * <sub></sub>            /  cos(pitch) 0 sin(pitch) \
 * R<sub>Y</sub>(pitch) = |      0      1     0      |
 * <sub></sub>            \ -sin(pitch) 0 cos(pitch) /
 *
 * <sub></sub>            / 1     0          0     \
 * R<sub>X</sub>(roll)  = | 0 cos(roll) -sin(roll) |
 * <sub></sub>            \ 0 sin(roll)  cos(roll) /
 * </pre>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public abstract class YawPitchRollConversion
{
   /**
    * Represents the safety margin that
    */
   public static final double SAFE_THRESHOLD_PITCH = Math.toRadians(1.82);
   /**
    * Pitch angle that defines the upper bound of the safe region in which the resulting pitch angle of
    * a conversion is accurate. If the pitch angle from a conversion is beyond this bound, the
    * yaw-pitch-roll becomes inaccurate.
    */
   public static final double MAX_SAFE_PITCH_ANGLE = Math.PI / 2.0 - SAFE_THRESHOLD_PITCH;
   /**
    * Pitch angle that defines the lower bound of the safe region in which the resulting pitch angle of
    * a conversion is accurate. If the pitch angle from a conversion is beyond this bound, the
    * yaw-pitch-roll becomes inaccurate.
    */
   public static final double MIN_SAFE_PITCH_ANGLE = -MAX_SAFE_PITCH_ANGLE;

   private static final double EPS = 1.0e-12;

   /**
    * Computes the yaw from a rotation matrix.
    * <p>
    * <b> This method is for internal use. Use {@link #computeYaw(RotationMatrixReadOnly)} instead.
    * </b>
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation matrix contains at least one {@link Double#NaN}, this method returns
    * {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param m00 the new 1st row 1st column coefficient for the matrix to use for the conversion.
    * @param m10 the new 2nd row 1st column coefficient for the matrix to use for the conversion.
    * @return the yaw angle.
    */
   static double computeYawImpl(double m00, double m10)
   {
      if (EuclidCoreTools.containsNaN(m00, m10))
         return Double.NaN;

      return Math.atan2(m10, m00);
   }

   /**
    * Computes the pitch from a rotation matrix.
    * <p>
    * <b> This method is for internal use. Use {@link #computePitch(RotationMatrixReadOnly)} instead.
    * </b>
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation matrix contains at least one {@link Double#NaN}, this method returns
    * {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param m20 the new 3rd row 1st column coefficient for the matrix to use for the conversion.
    * @return the pitch angle.
    */
   static double computePitchImpl(double m20)
   {
      if (Double.isNaN(m20))
         return Double.NaN;

      if (m20 > 1.0)
         m20 = 1.0;
      else if (m20 < -1.0)
         m20 = -1.0;

      return Math.asin(-m20);
   }

   /**
    * Computes the roll from a rotation matrix.
    * <p>
    * <b> This method is for internal use. Use {@link #computeRoll(RotationMatrixReadOnly)} instead.
    * </b>
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation matrix contains at least one {@link Double#NaN}, this method returns
    * {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param m21 the new 3rd row 2nd column coefficient for the matrix to use for the conversion.
    * @param m22 the new 3rd row 3rd column coefficient for the matrix to use for the conversion.
    * @return the roll angle.
    */
   static double computeRollImpl(double m21, double m22)
   {
      if (EuclidCoreTools.containsNaN(m21, m22))
         return Double.NaN;

      return Math.atan2(m21, m22);
   }

   /**
    * Computes the yaw from a rotation matrix.
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation matrix contains at least one {@link Double#NaN}, this method returns
    * {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param rotationMatrix the rotation matrix to use for the conversion. Not modified.
    * @return the yaw angle.
    */
   public static double computeYaw(RotationMatrixReadOnly rotationMatrix)
   {
      return rotationMatrix.isZeroOrientation() ? 0.0 : computeYawImpl(rotationMatrix.getM00(), rotationMatrix.getM10());
   }

   /**
    * Computes the pitch angle from a rotation matrix.
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation matrix contains at least one {@link Double#NaN}, this method returns
    * {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param rotationMatrix the rotation matrix to use for the conversion. Not modified.
    * @return the pitch angle.
    */
   public static double computePitch(RotationMatrixReadOnly rotationMatrix)
   {
      return rotationMatrix.isZeroOrientation() ? 0.0 : computePitchImpl(rotationMatrix.getM20());
   }

   /**
    * Computes the roll from a rotation matrix.
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation matrix contains at least one {@link Double#NaN}, this method returns
    * {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param rotationMatrix the rotation matrix to use for the conversion. Not modified.
    * @return the roll angle.
    */
   public static double computeRoll(RotationMatrixReadOnly rotationMatrix)
   {
      return rotationMatrix.isZeroOrientation() ? 0.0 : computeRollImpl(rotationMatrix.getM21(), rotationMatrix.getM22());
   }

   /**
    * Computes the yaw from the rotation part of a rotation-scale matrix.
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation matrix contains at least one {@link Double#NaN}, this method returns
    * {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param rotationScaleMatrix a 3-by-3 matrix representing an orientation and a scale. Only the
    *                            orientation part is used during the conversion. Not modified.
    * @return the yaw angle.
    */
   public static double computeYaw(RotationScaleMatrixReadOnly rotationScaleMatrix)
   {
      return computeYaw(rotationScaleMatrix.getRotationMatrix());
   }

   /**
    * Computes the pitch angle from the rotation part of a rotation-scale matrix.
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation matrix contains at least one {@link Double#NaN}, this method returns
    * {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param rotationScaleMatrix a 3-by-3 matrix representing an orientation and a scale. Only the
    *                            orientation part is used during the conversion. Not modified.
    * @return the pitch angle.
    */
   public static double computePitch(RotationScaleMatrixReadOnly rotationScaleMatrix)
   {
      return computePitch(rotationScaleMatrix.getRotationMatrix());
   }

   /**
    * Computes the roll from the rotation part of a rotation-scale matrix.
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation matrix contains at least one {@link Double#NaN}, this method returns
    * {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param rotationScaleMatrix a 3-by-3 matrix representing an orientation and a scale. Only the
    *                            orientation part is used during the conversion. Not modified.
    * @return the roll angle.
    */
   public static double computeRoll(RotationScaleMatrixReadOnly rotationScaleMatrix)
   {
      return computeRoll(rotationScaleMatrix.getRotationMatrix());
   }

   /**
    * Converts the rotation part of the given rotation-scale matrix into yaw-pitch-roll.
    * <p>
    * After calling this method, the rotation part of the rotation-scale matrix and the yaw-pitch-roll
    * angles represent the same orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation matrix contains at least one {@link Double#NaN}, the yaw-pitch-roll angles
    * are set to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param rotationScaleMatrix a 3-by-3 matrix representing an orientation and a scale. Only the
    *                            orientation part is used during the conversion. Not modified.
    * @param yawPitchRollToPack  the yaw-pitch-roll used to store the orientation. Modified.
    */
   public static void convertMatrixToYawPitchRoll(RotationScaleMatrixReadOnly rotationScaleMatrix, YawPitchRollBasics yawPitchRollToPack)
   {
      convertMatrixToYawPitchRoll(rotationScaleMatrix.getRotationMatrix(), yawPitchRollToPack);
   }

   /**
    * Converts the rotation part of the given rotation-scale matrix into yaw-pitch-roll.
    * <p>
    * After calling this method, the rotation part of the rotation-scale matrix and the yaw-pitch-roll
    * angles represent the same orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation matrix contains at least one {@link Double#NaN}, the yaw-pitch-roll angles
    * are set to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param rotationScaleMatrix a 3-by-3 matrix representing an orientation and a scale. Only the
    *                            orientation part is used during the conversion. Not modified.
    * @param yawPitchRollToPack  the array in which the yaw-pitch-roll angles are stored, in the order
    *                            {@code yaw}, {@code pitch}, then {@code roll}. Modified.
    * @deprecated Use
    *             {@link #convertMatrixToYawPitchRoll(RotationScaleMatrixReadOnly, YawPitchRollBasics)}
    *             instead.
    */
   public static void convertMatrixToYawPitchRoll(RotationScaleMatrixReadOnly rotationScaleMatrix, double[] yawPitchRollToPack)
   {
      convertMatrixToYawPitchRoll(rotationScaleMatrix.getRotationMatrix(), yawPitchRollToPack);
   }

   /**
    * Converts the rotation matrix into yaw-pitch-roll.
    * <p>
    * After calling this method, the rotation matrix and the yaw-pitch-roll angles represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation matrix contains at least one {@link Double#NaN}, the yaw-pitch-roll angles
    * are set to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param rotationMatrix     a 3-by-3 matrix representing an orientation. Not modified.
    * @param yawPitchRollToPack the yaw-pitch-roll used to store the orientation. Modified.
    */
   public static void convertMatrixToYawPitchRoll(RotationMatrixReadOnly rotationMatrix, YawPitchRollBasics yawPitchRollToPack)
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

      convertMatrixToYawPitchRoll(m00, m01, m02, m10, m11, m12, m20, m21, m22, yawPitchRollToPack);
   }

   /**
    * Converts the rotation matrix into yaw-pitch-roll.
    * <p>
    * After calling this method, the rotation matrix and the yaw-pitch-roll angles represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation matrix contains at least one {@link Double#NaN}, the yaw-pitch-roll angles
    * are set to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param m00                the 1st row 1st column coefficient of the rotation matrix.
    * @param m01                the 1st row 2nd column coefficient of the rotation matrix.
    * @param m02                the 1st row 3rd column coefficient of the rotation matrix.
    * @param m10                the 2nd row 1st column coefficient of the rotation matrix.
    * @param m11                the 2nd row 2nd column coefficient of the rotation matrix.
    * @param m12                the 2nd row 3rd column coefficient of the rotation matrix.
    * @param m20                the 3rd row 1st column coefficient of the rotation matrix.
    * @param m21                the 3rd row 2nd column coefficient of the rotation matrix.
    * @param m22                the 3rd row 3rd column coefficient of the rotation matrix.
    * @param yawPitchRollToPack the yaw-pitch-roll used to store the orientation. Modified.
    */
   public static void convertMatrixToYawPitchRoll(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22,
                                                  YawPitchRollBasics yawPitchRollToPack)
   {
      double yaw = computeYawImpl(m00, m10);
      double pitch = computePitchImpl(m20);
      double roll = computeRollImpl(m21, m22);
      yawPitchRollToPack.set(yaw, pitch, roll);
   }

   /**
    * Converts the rotation matrix into yaw-pitch-roll.
    * <p>
    * After calling this method, the rotation matrix and the yaw-pitch-roll angles represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation matrix contains at least one {@link Double#NaN}, the yaw-pitch-roll angles
    * are set to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param rotationMatrix     a 3-by-3 matrix representing an orientation. Not modified.
    * @param yawPitchRollToPack the array in which the yaw-pitch-roll angles are stored, in the order
    *                           {@code yaw}, {@code pitch}, then {@code roll}. Modified.
    * @deprecated Use {@link #convertMatrixToYawPitchRoll(RotationMatrixReadOnly, YawPitchRollBasics)}
    *             instead.
    */
   public static void convertMatrixToYawPitchRoll(RotationMatrixReadOnly rotationMatrix, double[] yawPitchRollToPack)
   {
      yawPitchRollToPack[0] = computeYawImpl(rotationMatrix.getM00(), rotationMatrix.getM10());
      yawPitchRollToPack[1] = computePitchImpl(rotationMatrix.getM20());
      yawPitchRollToPack[2] = computeRollImpl(rotationMatrix.getM21(), rotationMatrix.getM22());
   }

   /**
    * Converts the rotation part of the given rotation-scale matrix into yaw-pitch-roll.
    * <p>
    * After calling this method, the rotation part of the rotation-scale matrix and the yaw-pitch-roll
    * angles represent the same orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation matrix contains at least one {@link Double#NaN}, the yaw-pitch-roll angles
    * are set to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param rotationScaleMatrix a 3-by-3 matrix representing an orientation and a scale. Only the
    *                            orientation part is used during the conversion. Not modified.
    * @param eulerAnglesToPack   the tuple in which the yaw-pitch-roll angles are stored, i.e.
    *                            {@code eulerAnglesToPack.set(roll, pitch, yaw)}. Modified.
    */
   public static void convertMatrixToYawPitchRoll(RotationScaleMatrixReadOnly rotationScaleMatrix, Tuple3DBasics eulerAnglesToPack)
   {
      convertMatrixToYawPitchRoll(rotationScaleMatrix.getRotationMatrix(), eulerAnglesToPack);
   }

   /**
    * Converts the given rotation matrix into yaw-pitch-roll.
    * <p>
    * After calling this method, the rotation matrix and the yaw-pitch-roll angles represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation matrix contains at least one {@link Double#NaN}, the yaw-pitch-roll angles
    * are set to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param rotationMatrix    a 3-by-3 matrix representing an orientation. Not modified.
    * @param eulerAnglesToPack the tuple in which the yaw-pitch-roll angles are stored, i.e.
    *                          {@code eulerAnglesToPack.set(roll, pitch, yaw)}. Modified.
    */
   public static void convertMatrixToYawPitchRoll(RotationMatrixReadOnly rotationMatrix, Tuple3DBasics eulerAnglesToPack)
   {
      eulerAnglesToPack.setX(computeRollImpl(rotationMatrix.getM21(), rotationMatrix.getM22()));
      eulerAnglesToPack.setY(computePitchImpl(rotationMatrix.getM20()));
      eulerAnglesToPack.setZ(computeYawImpl(rotationMatrix.getM00(), rotationMatrix.getM10()));
   }

   /**
    * Computes the yaw from a quaternion.
    * <p>
    * <b> This method is for internal use. Use {@link #computeYaw(QuaternionReadOnly)} instead. </b>
    * </p>
    *
    * @param qx the x-component of the quaternion to use in the conversion.
    * @param qy the y-component of the quaternion to use in the conversion.
    * @param qz the z-component of the quaternion to use in the conversion.
    * @param qs the s-component of the quaternion to use in the conversion.
    * @return the yaw angle.
    */
   static double computeYawFromQuaternionImpl(double qx, double qy, double qz, double qs)
   {
      return Math.atan2(2.0 * (qx * qy + qz * qs), 1.0 - 2.0 * (qy * qy + qz * qz));
   }

   /**
    * Computes the pitch from a quaternion.
    * <p>
    * <b> This method is for internal use. Use {@link #computePitch(QuaternionReadOnly)} instead. </b>
    * </p>
    *
    * @param qx the x-component of the quaternion to use in the conversion.
    * @param qy the y-component of the quaternion to use in the conversion.
    * @param qz the z-component of the quaternion to use in the conversion.
    * @param qs the s-component of the quaternion to use in the conversion.
    * @return the pitch angle.
    */
   static double computePitchFromQuaternionImpl(double qx, double qy, double qz, double qs)
   {
      double pitchArgument = 2.0 * (qs * qy - qx * qz);

      if (pitchArgument > 1.0)
         pitchArgument = 1.0;
      else if (pitchArgument < -1.0)
         pitchArgument = -1.0;

      return Math.asin(pitchArgument);
   }

   /**
    * Computes the roll from a quaternion.
    * <p>
    * <b> This method is for internal use. Use {@link #computeRoll(QuaternionReadOnly)} instead. </b>
    * </p>
    *
    * @param qx the x-component of the quaternion to use in the conversion.
    * @param qy the y-component of the quaternion to use in the conversion.
    * @param qz the z-component of the quaternion to use in the conversion.
    * @param qs the s-component of the quaternion to use in the conversion.
    * @return the roll angle.
    */
   static double computeRollFromQuaternionImpl(double qx, double qy, double qz, double qs)
   {
      return Math.atan2(2.0 * (qy * qz + qx * qs), 1.0 - 2.0 * (qx * qx + qy * qy));
   }

   /**
    * Computes the yaw from a quaternion.
    * <p>
    * Edge case:
    * <ul>
    * <li>if the quaternion contains at least one {@link Double#NaN}, this method returns
    * {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param quaternion the quaternion to use in the conversion. Not modified.
    * @return the yaw angle.
    */
   public static double computeYaw(QuaternionReadOnly quaternion)
   {
      if (quaternion.containsNaN())
         return Double.NaN;

      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      double norm = quaternion.norm();
      if (norm < EPS)
         return 0.0;

      norm = 1.0 / norm;
      qx *= norm;
      qy *= norm;
      qz *= norm;
      qs *= norm;

      return computeYawFromQuaternionImpl(qx, qy, qz, qs);
   }

   /**
    * Computes the pitch from a quaternion.
    * <p>
    * Edge case:
    * <ul>
    * <li>if the quaternion contains at least one {@link Double#NaN}, this method returns
    * {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param quaternion the quaternion to use in the conversion. Not modified.
    * @return the pitch angle.
    */
   public static double computePitch(QuaternionReadOnly quaternion)
   {
      if (quaternion.containsNaN())
         return Double.NaN;

      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      double norm = quaternion.norm();
      if (norm < EPS)
         return 0.0;

      norm = 1.0 / norm;
      qx *= norm;
      qy *= norm;
      qz *= norm;
      qs *= norm;

      return computePitchFromQuaternionImpl(qx, qy, qz, qs);
   }

   /**
    * Computes the roll from a quaternion.
    * <p>
    * Edge case:
    * <ul>
    * <li>if the quaternion contains at least one {@link Double#NaN}, this method returns
    * {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param quaternion the quaternion to use in the conversion. Not modified.
    * @return the roll angle.
    */
   public static double computeRoll(QuaternionReadOnly quaternion)
   {
      if (quaternion.containsNaN())
         return Double.NaN;

      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      double norm = quaternion.norm();
      if (norm < EPS)
         return 0.0;

      norm = 1.0 / norm;
      qx *= norm;
      qy *= norm;
      qz *= norm;
      qs *= norm;

      return computeRollFromQuaternionImpl(qx, qy, qz, qs);
   }

   /**
    * Converts the quaternion into yaw-pitch-roll.
    * <p>
    * After calling this method, the quaternion and the yaw-pitch-roll angles represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if the quaternion contains at least one {@link Double#NaN}, the yaw-pitch-roll angles are set
    * to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param quaternion         the quaternion to use in the conversion. Not modified.
    * @param yawPitchRollToPack the yaw-pitch-roll used to store the orientation. Modified.
    */
   public static void convertQuaternionToYawPitchRoll(QuaternionReadOnly quaternion, YawPitchRollBasics yawPitchRollToPack)
   {
      convertQuaternionToYawPitchRoll(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getS(), yawPitchRollToPack);
   }

   /**
    * Converts the quaternion into yaw-pitch-roll.
    * <p>
    * After calling this method, the quaternion and the yaw-pitch-roll angles represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if the quaternion contains at least one {@link Double#NaN}, the yaw-pitch-roll angles are set
    * to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param qx                 the x-component of the quaternion to use in the conversion.
    * @param qy                 the y-component of the quaternion to use in the conversion.
    * @param qz                 the z-component of the quaternion to use in the conversion.
    * @param qs                 the s-component of the quaternion to use in the conversion.
    * @param yawPitchRollToPack the yaw-pitch-roll used to store the orientation. Modified.
    */
   public static void convertQuaternionToYawPitchRoll(double qx, double qy, double qz, double qs, YawPitchRollBasics yawPitchRollToPack)
   {
      if (EuclidCoreTools.containsNaN(qx, qy, qz, qs))
      {
         yawPitchRollToPack.setToNaN();
         return;
      }

      double norm = EuclidCoreTools.fastNorm(qx, qy, qz, qs);

      if (norm < EPS)
      {
         yawPitchRollToPack.setToZero();
         return;
      }

      norm = 1.0 / norm;
      qx *= norm;
      qy *= norm;
      qz *= norm;
      qs *= norm;

      double yaw = computeYawFromQuaternionImpl(qx, qy, qz, qs);
      double pitch = computePitchFromQuaternionImpl(qx, qy, qz, qs);
      double roll = computeRollFromQuaternionImpl(qx, qy, qz, qs);
      yawPitchRollToPack.set(yaw, pitch, roll);
   }

   /**
    * Converts the quaternion into yaw-pitch-roll.
    * <p>
    * After calling this method, the quaternion and the yaw-pitch-roll angles represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if the quaternion contains at least one {@link Double#NaN}, the yaw-pitch-roll angles are set
    * to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param quaternion         the quaternion to use in the conversion. Not modified.
    * @param yawPitchRollToPack the array in which the yaw-pitch-roll angles are stored, in the order
    *                           {@code yaw}, {@code pitch}, then {@code roll}. Modified.
    * @deprecated Use {@link #convertQuaternionToYawPitchRoll(QuaternionReadOnly, YawPitchRollBasics)}
    *             instead.
    */
   public static void convertQuaternionToYawPitchRoll(QuaternionReadOnly quaternion, double[] yawPitchRollToPack)
   {
      if (quaternion.containsNaN())
      {
         yawPitchRollToPack[0] = Double.NaN;
         yawPitchRollToPack[1] = Double.NaN;
         yawPitchRollToPack[2] = Double.NaN;
         return;
      }

      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      double norm = quaternion.norm();
      if (norm < EPS)
      {
         yawPitchRollToPack[0] = 0.0;
         yawPitchRollToPack[1] = 0.0;
         yawPitchRollToPack[2] = 0.0;
         return;
      }

      norm = 1.0 / norm;
      qx *= norm;
      qy *= norm;
      qz *= norm;
      qs *= norm;

      yawPitchRollToPack[0] = computeYawFromQuaternionImpl(qx, qy, qz, qs);
      yawPitchRollToPack[1] = computePitchFromQuaternionImpl(qx, qy, qz, qs);
      yawPitchRollToPack[2] = computeRollFromQuaternionImpl(qx, qy, qz, qs);
   }

   /**
    * Converts the given quaternion into yaw-pitch-roll.
    * <p>
    * After calling this method, the quaternion and the yaw-pitch-roll angles represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if the quaternion contains at least one {@link Double#NaN}, the yaw-pitch-roll angles are set
    * to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param quaternion        the quaternion to use in the conversion. Not modified.
    * @param eulerAnglesToPack the tuple in which the yaw-pitch-roll angles are stored, i.e.
    *                          {@code eulerAnglesToPack.set(roll, pitch, yaw)}. Modified.
    */
   public static void convertQuaternionToYawPitchRoll(QuaternionReadOnly quaternion, Tuple3DBasics eulerAnglesToPack)
   {
      if (quaternion.containsNaN())
      {
         eulerAnglesToPack.setToNaN();
         return;
      }

      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      double norm = quaternion.norm();
      if (norm < EPS)
      {
         eulerAnglesToPack.setToZero();
         return;
      }

      norm = 1.0 / norm;
      qx *= norm;
      qy *= norm;
      qz *= norm;
      qs *= norm;

      eulerAnglesToPack.setZ(computeYawFromQuaternionImpl(qx, qy, qz, qs));
      eulerAnglesToPack.setY(computePitchFromQuaternionImpl(qx, qy, qz, qs));
      eulerAnglesToPack.setX(computeRollFromQuaternionImpl(qx, qy, qz, qs));
   }

   /**
    * Computes the yaw from the given axis-angle.
    * <p>
    * <b> This method is for internal use. Use {@link #computeYaw(AxisAngleReadOnly)} instead. </b>
    * </p>
    *
    * @param ux    the x-component of the axis of the axis-angle to use in the conversion.
    * @param uy    the y-component of the axis of the axis-angle to use in the conversion.
    * @param uz    the z-component of the axis of the axis-angle to use in the conversion.
    * @param angle the angle of the axis-angle to use in the conversion.
    * @return the yaw angle.
    */
   static double computeYawFromAxisAngleImpl(double ux, double uy, double uz, double angle)
   {
      double sinTheta = Math.sin(angle);
      double cosTheta = Math.cos(angle);
      double t = 1.0 - cosTheta;
      double m10 = t * ux * uy + sinTheta * uz;
      double m00 = t * ux * ux + cosTheta;
      return computeYawImpl(m00, m10);
   }

   /**
    * Computes the pitch from the given axis-angle.
    * <p>
    * <b> This method is for internal use. Use {@link #computePitch(AxisAngleReadOnly)} instead. </b>
    * </p>
    *
    * @param ux    the x-component of the axis of the axis-angle to use in the conversion.
    * @param uy    the y-component of the axis of the axis-angle to use in the conversion.
    * @param uz    the z-component of the axis of the axis-angle to use in the conversion.
    * @param angle the angle of the axis-angle to use in the conversion.
    * @return the pitch angle.
    */
   static double computePitchFromAxisAngleImpl(double ux, double uy, double uz, double angle)
   {
      double m20 = (1.0 - Math.cos(angle)) * ux * uz - Math.sin(angle) * uy;
      return computePitchImpl(m20);
   }

   /**
    * Computes the roll from the given axis-angle.
    * <p>
    * <b> This method is for internal use. Use {@link #computeRoll(AxisAngleReadOnly)} instead. </b>
    * </p>
    *
    * @param ux    the x-component of the axis of the axis-angle to use in the conversion.
    * @param uy    the y-component of the axis of the axis-angle to use in the conversion.
    * @param uz    the z-component of the axis of the axis-angle to use in the conversion.
    * @param angle the angle of the axis-angle to use in the conversion.
    * @return the roll angle.
    */
   static double computeRollFromAxisAngleImpl(double ux, double uy, double uz, double angle)
   {
      double sinTheta = Math.sin(angle);
      double cosTheta = Math.cos(angle);
      double t = 1.0 - cosTheta;
      double m21 = t * uy * uz + sinTheta * ux;
      double m22 = t * uz * uz + cosTheta;
      return computeRollImpl(m21, m22);
   }

   /**
    * Computes the yaw from an axis-angle.
    * <p>
    * Edge case:
    * <ul>
    * <li>if the axis-angle contains at least one {@link Double#NaN}, this method returns
    * {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param axisAngle the axis-angle to use in the conversion. Not modified.
    * @return the yaw angle.
    */
   public static double computeYaw(AxisAngleReadOnly axisAngle)
   {
      if (axisAngle.containsNaN())
         return Double.NaN;

      double ux = axisAngle.getX();
      double uy = axisAngle.getY();
      double uz = axisAngle.getZ();
      double angle = axisAngle.getAngle();

      double uNorm = axisAngle.axisNorm();

      if (uNorm < EPS)
         return 0.0;

      uNorm = 1.0 / uNorm;
      ux *= uNorm;
      uy *= uNorm;
      uz *= uNorm;

      return computeYawFromAxisAngleImpl(ux, uy, uz, angle);
   }

   /**
    * Computes the pitch from an axis-angle.
    * <p>
    * Edge case:
    * <ul>
    * <li>if the axis-angle contains at least one {@link Double#NaN}, this method returns
    * {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param axisAngle the axis-angle to use in the conversion. Not modified.
    * @return the pitch angle.
    */
   public static double computePitch(AxisAngleReadOnly axisAngle)
   {
      if (axisAngle.containsNaN())
         return Double.NaN;

      double ux = axisAngle.getX();
      double uy = axisAngle.getY();
      double uz = axisAngle.getZ();
      double angle = axisAngle.getAngle();

      double uNorm = axisAngle.axisNorm();

      if (uNorm < EPS)
         return 0.0;

      uNorm = 1.0 / uNorm;
      ux *= uNorm;
      uy *= uNorm;
      uz *= uNorm;

      return computePitchFromAxisAngleImpl(ux, uy, uz, angle);
   }

   /**
    * Computes the roll from an axis-angle.
    * <p>
    * Edge case:
    * <ul>
    * <li>if the axis-angle contains at least one {@link Double#NaN}, this method returns
    * {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param axisAngle the axis-angle to use in the conversion. Not modified.
    * @return the roll angle.
    */
   public static double computeRoll(AxisAngleReadOnly axisAngle)
   {
      if (axisAngle.containsNaN())
         return Double.NaN;

      double ux = axisAngle.getX();
      double uy = axisAngle.getY();
      double uz = axisAngle.getZ();
      double angle = axisAngle.getAngle();

      double uNorm = axisAngle.axisNorm();

      if (uNorm < EPS)
         return 0.0;

      uNorm = 1.0 / uNorm;
      ux *= uNorm;
      uy *= uNorm;
      uz *= uNorm;

      return computeRollFromAxisAngleImpl(ux, uy, uz, angle);
   }

   /**
    * Converts the axis-angle into yaw-pitch-roll.
    * <p>
    * After calling this method, the axis-angle and the yaw-pitch-roll angles represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if the axis-angle contains at least one {@link Double#NaN}, the yaw-pitch-roll angles are set
    * to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param axisAngle          the axis-angle to use in the conversion. Not modified.
    * @param yawPitchRollToPack the yaw-pitch-roll used to store the orientation. Modified.
    */
   public static void convertAxisAngleToYawPitchRoll(AxisAngleReadOnly axisAngle, YawPitchRollBasics yawPitchRollToPack)
   {

      double ux = axisAngle.getX();
      double uy = axisAngle.getY();
      double uz = axisAngle.getZ();
      double angle = axisAngle.getAngle();

      convertAxisAngleToYawPitchRoll(ux, uy, uz, angle, yawPitchRollToPack);
   }

   /**
    * Converts the axis-angle into yaw-pitch-roll.
    * <p>
    * After calling this method, the axis-angle and the yaw-pitch-roll angles represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if the axis-angle contains at least one {@link Double#NaN}, the yaw-pitch-roll angles are set
    * to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param ux                 the x-component of the axis of the axis-angle to use in the conversion.
    * @param uy                 the y-component of the axis of the axis-angle to use in the conversion.
    * @param uz                 the z-component of the axis of the axis-angle to use in the conversion.
    * @param angle              the angle of the axis-angle to use in the conversion.
    * @param yawPitchRollToPack the yaw-pitch-roll used to store the orientation. Modified.
    */
   public static void convertAxisAngleToYawPitchRoll(double ux, double uy, double uz, double angle, YawPitchRollBasics yawPitchRollToPack)
   {
      if (EuclidCoreTools.containsNaN(ux, uy, uz, angle))
      {
         yawPitchRollToPack.setToNaN();
         return;
      }

      double uNorm = EuclidCoreTools.fastNorm(ux, uy, uz);
      if (uNorm < EPS)
      {
         yawPitchRollToPack.setToZero();
         return;
      }

      uNorm = 1.0 / uNorm;
      ux *= uNorm;
      uy *= uNorm;
      uz *= uNorm;
      double sinTheta = Math.sin(angle);
      double cosTheta = Math.cos(angle);
      double t = 1.0 - cosTheta;
      double m20 = t * ux * uz - sinTheta * uy;
      double m10 = t * ux * uy + sinTheta * uz;
      double m00 = t * ux * ux + cosTheta;
      double m21 = t * uy * uz + sinTheta * ux;
      double m22 = t * uz * uz + cosTheta;

      double yaw = computeYawImpl(m00, m10);
      double pitch = computePitchImpl(m20);
      double roll = computeRollImpl(m21, m22);
      yawPitchRollToPack.set(yaw, pitch, roll);
   }

   /**
    * Converts the axis-angle into yaw-pitch-roll.
    * <p>
    * After calling this method, the axis-angle and the yaw-pitch-roll angles represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if the axis-angle contains at least one {@link Double#NaN}, the yaw-pitch-roll angles are set
    * to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param axisAngle          the axis-angle to use in the conversion. Not modified.
    * @param yawPitchRollToPack the array in which the yaw-pitch-roll angles are stored, in the order
    *                           {@code yaw}, {@code pitch}, then {@code roll}. Modified.
    * @deprecated Use {@link #convertAxisAngleToYawPitchRoll(AxisAngleReadOnly, YawPitchRollBasics)}
    *             instead.
    */
   public static void convertAxisAngleToYawPitchRoll(AxisAngleReadOnly axisAngle, double[] yawPitchRollToPack)
   {
      if (axisAngle.containsNaN())
      {
         yawPitchRollToPack[0] = Double.NaN;
         yawPitchRollToPack[1] = Double.NaN;
         yawPitchRollToPack[2] = Double.NaN;
         return;
      }

      double ux = axisAngle.getX();
      double uy = axisAngle.getY();
      double uz = axisAngle.getZ();
      double angle = axisAngle.getAngle();
      double uNorm = EuclidCoreTools.fastNorm(ux, uy, uz);
      if (uNorm < EPS)
      {
         yawPitchRollToPack[0] = 0.0;
         yawPitchRollToPack[1] = 0.0;
         yawPitchRollToPack[2] = 0.0;
         return;
      }

      uNorm = 1.0 / uNorm;
      ux *= uNorm;
      uy *= uNorm;
      uz *= uNorm;
      convertAxisAngleToYawPitchRollImpl(ux, uy, uz, angle, yawPitchRollToPack);
   }

   /**
    * Converts the axis-angle into yaw-pitch-roll.
    * <p>
    * After calling this method, the axis-angle and the yaw-pitch-roll angles represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if the axis-angle contains at least one {@link Double#NaN}, the yaw-pitch-roll angles are set
    * to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param axisAngle         the axis-angle to use in the conversion. Not modified.
    * @param eulerAnglesToPack the tuple in which the yaw-pitch-roll angles are stored, i.e.
    *                          {@code eulerAnglesToPack.set(roll, pitch, yaw)}. Modified.
    */
   public static void convertAxisAngleToYawPitchRoll(AxisAngleReadOnly axisAngle, Tuple3DBasics eulerAnglesToPack)
   {
      if (axisAngle.containsNaN())
      {
         eulerAnglesToPack.setToNaN();
         return;
      }

      double ux = axisAngle.getX();
      double uy = axisAngle.getY();
      double uz = axisAngle.getZ();
      double angle = axisAngle.getAngle();
      double uNorm = EuclidCoreTools.fastNorm(ux, uy, uz);
      if (uNorm < EPS)
      {
         eulerAnglesToPack.setToZero();
         return;
      }

      uNorm = 1.0 / uNorm;
      ux *= uNorm;
      uy *= uNorm;
      uz *= uNorm;
      convertAxisAngleToYawPitchRollImpl(ux, uy, uz, angle, eulerAnglesToPack);
   }

   /**
    * Converts the axis-angle into yaw-pitch-roll.
    * <p>
    * <b> This method is for internal use. Use {@link #computeYaw(AxisAngleReadOnly)} instead. </b>
    * </p>
    * <p>
    * After calling this method, the axis-angle and the yaw-pitch-roll angles represent the same
    * orientation.
    * </p>
    *
    * @param ux                 the x-component of the axis of the axis-angle to use in the conversion.
    * @param uy                 the y-component of the axis of the axis-angle to use in the conversion.
    * @param uz                 the z-component of the axis of the axis-angle to use in the conversion.
    * @param angle              the angle of the axis-angle to use in the conversion.
    * @param yawPitchRollToPack the array in which the yaw-pitch-roll angles are stored, in the order
    *                           {@code yaw}, {@code pitch}, then {@code roll}. Modified.
    * @deprecated Use
    *             {@link #convertAxisAngleToYawPitchRollImpl(double, double, double, double, double[])}
    *             instead.
    */
   static void convertAxisAngleToYawPitchRollImpl(double ux, double uy, double uz, double angle, double[] yawPitchRollToPack)
   {
      double sinTheta = Math.sin(angle);
      double cosTheta = Math.cos(angle);
      double t = 1.0 - cosTheta;
      double m20 = t * ux * uz - sinTheta * uy;
      double m10 = t * ux * uy + sinTheta * uz;
      double m00 = t * ux * ux + cosTheta;
      double m21 = t * uy * uz + sinTheta * ux;
      double m22 = t * uz * uz + cosTheta;

      yawPitchRollToPack[0] = computeYawImpl(m00, m10);
      yawPitchRollToPack[1] = computePitchImpl(m20);
      yawPitchRollToPack[2] = computeRollImpl(m21, m22);
   }

   /**
    * Converts the axis-angle into yaw-pitch-roll.
    * <p>
    * <b> This method is for internal use. Use {@link #computeYaw(AxisAngleReadOnly)} instead. </b>
    * </p>
    * <p>
    * After calling this method, the axis-angle and the yaw-pitch-roll angles represent the same
    * orientation.
    * </p>
    *
    * @param ux                the x-component of the axis of the axis-angle to use in the conversion.
    * @param uy                the y-component of the axis of the axis-angle to use in the conversion.
    * @param uz                the z-component of the axis of the axis-angle to use in the conversion.
    * @param angle             the angle of the axis-angle to use in the conversion.
    * @param eulerAnglesToPack the tuple in which the yaw-pitch-roll angles are stored, i.e.
    *                          {@code eulerAnglesToPack.set(roll, pitch, yaw)}. Modified.
    */
   static void convertAxisAngleToYawPitchRollImpl(double ux, double uy, double uz, double angle, Tuple3DBasics eulerAnglesToPack)
   {
      double sinTheta = Math.sin(angle);
      double cosTheta = Math.cos(angle);
      double t = 1.0 - cosTheta;
      double m20 = t * ux * uz - sinTheta * uy;
      double m10 = t * ux * uy + sinTheta * uz;
      double m00 = t * ux * ux + cosTheta;
      double m21 = t * uy * uz + sinTheta * ux;
      double m22 = t * uz * uz + cosTheta;

      eulerAnglesToPack.setZ(computeYawImpl(m00, m10));
      eulerAnglesToPack.setY(computePitchImpl(m20));
      eulerAnglesToPack.setX(computeRollImpl(m21, m22));
   }

   /**
    * Computes the yaw from a rotation vector.
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation vector contains at least one {@link Double#NaN}, this method returns
    * {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param rotationVector the rotation vector to use in the conversion. Not modified.
    * @return the yaw angle.
    */
   public static double computeYaw(Vector3DReadOnly rotationVector)
   {
      if (rotationVector.containsNaN())
         return Double.NaN;

      double ux = rotationVector.getX();
      double uy = rotationVector.getY();
      double uz = rotationVector.getZ();
      double angle = 0.0;
      double uNorm = EuclidCoreTools.norm(ux, uy, uz);

      if (uNorm < EPS)
         return 0.0;

      angle = uNorm;
      uNorm = 1.0 / uNorm;
      ux *= uNorm;
      uy *= uNorm;
      uz *= uNorm;

      return computeYawFromAxisAngleImpl(ux, uy, uz, angle);
   }

   /**
    * Computes the pitch from a rotation vector.
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation vector contains at least one {@link Double#NaN}, this method returns
    * {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param rotationVector the rotation vector to use in the conversion. Not modified.
    * @return the pitch angle.
    */
   public static double computePitch(Vector3DReadOnly rotationVector)
   {
      if (rotationVector.containsNaN())
         return Double.NaN;

      double ux = rotationVector.getX();
      double uy = rotationVector.getY();
      double uz = rotationVector.getZ();
      double angle = 0.0;
      double uNorm = EuclidCoreTools.norm(ux, uy, uz);
      angle = uNorm;

      if (uNorm < EPS)
         return 0.0;

      uNorm = 1.0 / uNorm;
      ux *= uNorm;
      uy *= uNorm;
      uz *= uNorm;
      return computePitchFromAxisAngleImpl(ux, uy, uz, angle);
   }

   /**
    * Computes the roll from a rotation vector.
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation vector contains at least one {@link Double#NaN}, this method returns
    * {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param rotationVector the rotation vector to use in the conversion. Not modified.
    * @return the roll angle.
    */
   public static double computeRoll(Vector3DReadOnly rotationVector)
   {
      if (rotationVector.containsNaN())
         return Double.NaN;

      double ux = rotationVector.getX();
      double uy = rotationVector.getY();
      double uz = rotationVector.getZ();
      double angle = 0.0;
      double uNorm = EuclidCoreTools.norm(ux, uy, uz);

      if (uNorm < EPS)
         return 0.0;

      angle = uNorm;
      uNorm = 1.0 / uNorm;
      ux *= uNorm;
      uy *= uNorm;
      uz *= uNorm;

      return computeRollFromAxisAngleImpl(ux, uy, uz, angle);
   }

   /**
    * Converts the rotation vector into yaw-pitch-roll.
    * <p>
    * After calling this method, the rotation vector and the yaw-pitch-roll angles represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation vector contains at least one {@link Double#NaN}, the yaw-pitch-roll angles
    * are set to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param rotationVector     the rotation vector to use in the conversion. Not modified.
    * @param yawPitchRollToPack the yaw-pitch-roll used to store the orientation. Modified.
    */
   public static void convertRotationVectorToYawPitchRoll(Vector3DReadOnly rotationVector, YawPitchRollBasics yawPitchRollToPack)
   {
      convertRotationVectorToYawPitchRoll(rotationVector.getX(), rotationVector.getY(), rotationVector.getZ(), yawPitchRollToPack);
   }

   /**
    * Converts the rotation vector into yaw-pitch-roll.
    * <p>
    * After calling this method, the rotation vector and the yaw-pitch-roll angles represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation vector contains at least one {@link Double#NaN}, the yaw-pitch-roll angles
    * are set to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param rx                 the x-component of the rotation vector to use in the conversion.
    * @param ry                 the y-component of the rotation vector to use in the conversion.
    * @param rz                 the z-component of the rotation vector to use in the conversion.
    * @param yawPitchRollToPack the yaw-pitch-roll used to store the orientation. Modified.
    */
   public static void convertRotationVectorToYawPitchRoll(double rx, double ry, double rz, YawPitchRollBasics yawPitchRollToPack)
   {
      if (EuclidCoreTools.containsNaN(rx, ry, rz))
      {
         yawPitchRollToPack.setToNaN();
         return;
      }

      double angle = EuclidCoreTools.norm(rx, ry, rz);

      if (angle < EPS)
      {
         yawPitchRollToPack.setToZero();
         return;
      }

      double uNorm = 1.0 / angle;
      double ux = rx * uNorm;
      double uy = ry * uNorm;
      double uz = rz * uNorm;

      double sinTheta = Math.sin(angle);
      double cosTheta = Math.cos(angle);
      double t = 1.0 - cosTheta;
      double m20 = t * ux * uz - sinTheta * uy;
      double m10 = t * ux * uy + sinTheta * uz;
      double m00 = t * ux * ux + cosTheta;
      double m21 = t * uy * uz + sinTheta * ux;
      double m22 = t * uz * uz + cosTheta;

      double yaw = computeYawImpl(m00, m10);
      double pitch = computePitchImpl(m20);
      double roll = computeRollImpl(m21, m22);

      yawPitchRollToPack.set(yaw, pitch, roll);
   }

   /**
    * Converts the rotation vector into yaw-pitch-roll.
    * <p>
    * After calling this method, the rotation vector and the yaw-pitch-roll angles represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation vector contains at least one {@link Double#NaN}, the yaw-pitch-roll angles
    * are set to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param rotationVector     the rotation vector to use in the conversion. Not modified.
    * @param yawPitchRollToPack the array in which the yaw-pitch-roll angles are stored, in the order
    *                           {@code yaw}, {@code pitch}, then {@code roll}. Modified.
    * @deprecated Use
    *             {@link #convertRotationVectorToYawPitchRoll(Vector3DReadOnly, YawPitchRollBasics)}
    *             instead.
    */
   public static void convertRotationVectorToYawPitchRoll(Vector3DReadOnly rotationVector, double[] yawPitchRollToPack)
   {
      if (rotationVector.containsNaN())
      {
         yawPitchRollToPack[0] = Double.NaN;
         yawPitchRollToPack[1] = Double.NaN;
         yawPitchRollToPack[2] = Double.NaN;
         return;
      }

      double ux = rotationVector.getX();
      double uy = rotationVector.getY();
      double uz = rotationVector.getZ();
      double angle = 0.0;
      double uNorm = EuclidCoreTools.norm(ux, uy, uz);

      if (uNorm < EPS)
      {
         yawPitchRollToPack[0] = 0.0;
         yawPitchRollToPack[1] = 0.0;
         yawPitchRollToPack[2] = 0.0;
         return;
      }

      angle = uNorm;
      uNorm = 1.0 / uNorm;
      ux *= uNorm;
      uy *= uNorm;
      uz *= uNorm;

      convertAxisAngleToYawPitchRollImpl(ux, uy, uz, angle, yawPitchRollToPack);
   }

   /**
    * Converts the rotation vector into yaw-pitch-roll.
    * <p>
    * After calling this method, the rotation vector and the yaw-pitch-roll angles represent the same
    * orientation.
    * </p>
    * <p>
    * Edge case:
    * <ul>
    * <li>if the rotation vector contains at least one {@link Double#NaN}, the yaw-pitch-roll angles
    * are set to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param rotationVector    the rotation vector to use in the conversion. Not modified.
    * @param eulerAnglesToPack the tuple in which the yaw-pitch-roll angles are stored, i.e.
    *                          {@code eulerAnglesToPack.set(roll, pitch, yaw)}. Modified.
    */
   public static void convertRotationVectorToYawPitchRoll(Vector3DReadOnly rotationVector, Vector3DBasics eulerAnglesToPack)
   {
      if (rotationVector.containsNaN())
      {
         eulerAnglesToPack.setToNaN();
         return;
      }

      double ux = rotationVector.getX();
      double uy = rotationVector.getY();
      double uz = rotationVector.getZ();
      double angle = 0.0;
      double uNorm = EuclidCoreTools.norm(ux, uy, uz);

      if (uNorm < EPS)
      {
         eulerAnglesToPack.setToZero();
         return;
      }

      angle = uNorm;
      uNorm = 1.0 / uNorm;
      ux *= uNorm;
      uy *= uNorm;
      uz *= uNorm;

      convertAxisAngleToYawPitchRollImpl(ux, uy, uz, angle, eulerAnglesToPack);
   }
}
