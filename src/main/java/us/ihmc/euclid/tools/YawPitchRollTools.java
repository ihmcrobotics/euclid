package us.ihmc.euclid.tools;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.exceptions.NotAnOrientation2DException;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollBasics;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollReadOnly;

/**
 * This class provides a collection of static tools to perform operations on yaw-pitch-rolls.
 * <p>
 * This class is mostly used to centralize operations on yaw-pitch-rolls, these operations are
 * available in classes such as {@link YawPitchRoll} and the user should always using this classes
 * instead of using tools classes. The API of these tools classes is more likely to change over
 * time.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class YawPitchRollTools
{
   /** Tolerance used to test if a yaw-pitch-roll is equal to zero. */
   public static final double ZERO_EPS = 1.0e-12;

   private YawPitchRollTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Tests whether the three given angles yaw, pitch, and roll are equal to zero.
    * 
    * @param yaw     the first angle representing the rotation around the z-axis.
    * @param pitch   the second angle representing the rotation around the y-axis.
    * @param roll    the third angle representing the rotation around the x-axis.
    * @param epsilon the tolerance to use for the comparison.
    * @return {@code true} if the three angles are equal to zero, {@code false} otherwise.
    */
   public static boolean isZero(double yaw, double pitch, double roll, double epsilon)
   {
      return EuclidCoreTools.isAngleZero(yaw, epsilon) && EuclidCoreTools.isAngleZero(pitch, epsilon) && EuclidCoreTools.isAngleZero(roll, epsilon);
   }

   /**
    * Tests whether the orientation represented by the given three angles yaw, pitch, and roll
    * represent an orientation 2D, i.e. only the yaw angle is non-zero.
    * 
    * @param yaw     the first angle representing the rotation around the z-axis.
    * @param pitch   the second angle representing the rotation around the y-axis.
    * @param roll    the third angle representing the rotation around the x-axis.
    * @param epsilon the tolerance to use for the comparison.
    * @return {@code true} if the query is considered a 2D orientation, {@code false} otherwise.
    */
   public static boolean isOrientation2D(double yaw, double pitch, double roll, double epsilon)
   {
      return Math.abs(pitch) <= epsilon && Math.abs(roll) <= epsilon;
   }

   /**
    * Computes and returns the distance between the two yaw-pitch-rolls {@code yawPitchRoll1} and
    * {@code yawPitchRoll2}.
    *
    * @param yawPitchRoll1 the first yaw-pitch-roll to measure the distance. Not modified.
    * @param yawPitchRoll2 the second yaw-pitch-roll to measure the distance. Not modified.
    * @return the angle representing the distance between the two yaw-pitch-roll. It is contained in
    *         [0, 2<i>pi</i>]
    */
   public static double distance(YawPitchRollReadOnly yawPitchRoll1, YawPitchRollReadOnly yawPitchRoll2)
   {
      return distance(yawPitchRoll1.getYaw(),
                      yawPitchRoll1.getPitch(),
                      yawPitchRoll1.getRoll(),
                      yawPitchRoll2.getYaw(),
                      yawPitchRoll2.getPitch(),
                      yawPitchRoll2.getRoll());
   }

   /**
    * Computes and returns the distance between the two yaw-pitch-rolls {@code yawPitchRoll1} and
    * {@code yawPitchRoll2}.
    *
    * @param yaw1   the first angle of the first orientation representing the rotation around the
    *               z-axis.
    * @param pitch1 the second angle of the first orientation representing the rotation around the
    *               y-axis.
    * @param roll1  the third angle of the first orientation representing the rotation around the
    *               x-axis.
    * @param yaw2   the first angle of the second orientation representing the rotation around the
    *               z-axis.
    * @param pitch2 the second angle of the second orientation representing the rotation around the
    *               y-axis.
    * @param roll2  the third angle of the second orientation representing the rotation around the
    *               x-axis.
    * @return the angle representing the distance between the two yaw-pitch-roll. It is contained in
    *         [0, 2<i>pi</i>]
    */
   public static double distance(double yaw1, double pitch1, double roll1, double yaw2, double pitch2, double roll2)
   {
      double q1s, q1x, q1y, q1z;

      {
         double halfYaw = 0.5 * yaw1;
         double cYaw = Math.cos(halfYaw);
         double sYaw = Math.sin(halfYaw);

         double halfPitch = 0.5 * pitch1;
         double cPitch = Math.cos(halfPitch);
         double sPitch = Math.sin(halfPitch);

         double halfRoll = 0.5 * roll1;
         double cRoll = Math.cos(halfRoll);
         double sRoll = Math.sin(halfRoll);

         q1s = cYaw * cPitch * cRoll + sYaw * sPitch * sRoll;
         q1x = cYaw * cPitch * sRoll - sYaw * sPitch * cRoll;
         q1y = sYaw * cPitch * sRoll + cYaw * sPitch * cRoll;
         q1z = sYaw * cPitch * cRoll - cYaw * sPitch * sRoll;
      }

      double q2s, q2x, q2y, q2z;

      {
         double halfYaw = 0.5 * yaw2;
         double cYaw = Math.cos(halfYaw);
         double sYaw = Math.sin(halfYaw);

         double halfPitch = 0.5 * pitch2;
         double cPitch = Math.cos(halfPitch);
         double sPitch = Math.sin(halfPitch);

         double halfRoll = 0.5 * roll2;
         double cRoll = Math.cos(halfRoll);
         double sRoll = Math.sin(halfRoll);

         q2s = cYaw * cPitch * cRoll + sYaw * sPitch * sRoll;
         q2x = cYaw * cPitch * sRoll - sYaw * sPitch * cRoll;
         q2y = sYaw * cPitch * sRoll + cYaw * sPitch * cRoll;
         q2z = sYaw * cPitch * cRoll - cYaw * sPitch * sRoll;
      }

      double x = q1s * q2x - q1x * q2s - q1y * q2z + q1z * q2y;
      double y = q1s * q2y + q1x * q2z - q1y * q2s - q1z * q2x;
      double z = q1s * q2z - q1x * q2y + q1y * q2x - q1z * q2s;
      double s = q1s * q2s + q1x * q2x + q1y * q2y + q1z * q2z;

      double sinHalfTheta = EuclidCoreTools.squareRoot(EuclidCoreTools.normSquared(x, y, z));
      return 2.0 * Math.atan2(sinHalfTheta, s);
   }

   /**
    * Calculates the inverse of the orientation represented by the given {@code yaw}, {@code pitch},
    * and {@code roll}.
    * 
    * @param yaw               the first angle representing the rotation around the z-axis.
    * @param pitch             the second angle representing the rotation around the y-axis.
    * @param roll              the third angle representing the rotation around the x-axis.
    * @param orientationToPack the orientation used to store the inverse. Modified.
    */
   public static void invert(double yaw, double pitch, double roll, Orientation3DBasics orientationToPack)
   {
      double halfYaw = 0.5 * yaw;
      double cYaw = Math.cos(halfYaw);
      double sYaw = Math.sin(halfYaw);

      double halfPitch = 0.5 * pitch;
      double cPitch = Math.cos(halfPitch);
      double sPitch = Math.sin(halfPitch);

      double halfRoll = 0.5 * roll;
      double cRoll = Math.cos(halfRoll);
      double sRoll = Math.sin(halfRoll);

      double qs = cYaw * cPitch * cRoll + sYaw * sPitch * sRoll;
      double qx = cYaw * cPitch * sRoll - sYaw * sPitch * cRoll;
      double qy = sYaw * cPitch * sRoll + cYaw * sPitch * cRoll;
      double qz = sYaw * cPitch * cRoll - cYaw * sPitch * sRoll;
      orientationToPack.setQuaternion(-qx, -qy, -qz, qs);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} using the orientation represented by the given yaw,
    * pitch, and roll angles and stores the result in {@code tupleTransformed}.
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    *
    * @param yaw              the first angle representing the rotation around the z-axis.
    * @param pitch            the second angle representing the rotation around the y-axis.
    * @param roll             the third angle representing the rotation around the x-axis.
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   public static void transform(double yaw, double pitch, double roll, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      transformImpl(yaw, pitch, roll, false, tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} using {@code yawPitchRoll} and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    *
    * @param yawPitchRoll     the yaw-pitch-roll used to transform the tuple. Not modified.
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   public static void transform(YawPitchRollReadOnly yawPitchRoll, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      transform(yawPitchRoll.getYaw(), yawPitchRoll.getPitch(), yawPitchRoll.getRoll(), tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform of the tuple {@code tupleOriginal} using the orientation
    * represented by the given angles yaw, pitch, and roll and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(YawPitchRollReadOnly, Tuple3DReadOnly, Tuple3DBasics)} with an yaw-pitch-roll
    * that has been inverted.
    * </p>
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    *
    * @param yaw              the first angle representing the rotation around the z-axis.
    * @param pitch            the second angle representing the rotation around the y-axis.
    * @param roll             the third angle representing the rotation around the x-axis.
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   public static void inverseTransform(double yaw, double pitch, double roll, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      transformImpl(yaw, pitch, roll, true, tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform of the tuple {@code tupleOriginal} using
    * {@code yawPitchRoll} and stores the result in {@code tupleTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(YawPitchRollReadOnly, Tuple3DReadOnly, Tuple3DBasics)} with an yaw-pitch-roll
    * that has an angle of opposite value compared to the given one.
    * </p>
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    *
    * @param yawPitchRoll     the yaw-pitch-roll used to transform the tuple. Not modified.
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   public static void inverseTransform(YawPitchRollReadOnly yawPitchRoll, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      inverseTransform(yawPitchRoll.getYaw(), yawPitchRoll.getPitch(), yawPitchRoll.getRoll(), tupleOriginal, tupleTransformed);
   }

   private static void transformImpl(double yaw, double pitch, double roll, boolean inverseTransform, Tuple3DReadOnly tupleOriginal,
                                     Tuple3DBasics tupleTransformed)
   {
      if (isZero(yaw, pitch, roll, ZERO_EPS))
      {
         tupleTransformed.set(tupleOriginal);
         return;
      }

      if (EuclidCoreTools.containsNaN(yaw, pitch, roll))
      {
         tupleTransformed.setToNaN();
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

      if (inverseTransform)
      {
         double x = m00 * tupleOriginal.getX() + m10 * tupleOriginal.getY() + m20 * tupleOriginal.getZ();
         double y = m01 * tupleOriginal.getX() + m11 * tupleOriginal.getY() + m21 * tupleOriginal.getZ();
         double z = m02 * tupleOriginal.getX() + m12 * tupleOriginal.getY() + m22 * tupleOriginal.getZ();
         tupleTransformed.set(x, y, z);
      }
      else
      {
         double x = m00 * tupleOriginal.getX() + m01 * tupleOriginal.getY() + m02 * tupleOriginal.getZ();
         double y = m10 * tupleOriginal.getX() + m11 * tupleOriginal.getY() + m12 * tupleOriginal.getZ();
         double z = m20 * tupleOriginal.getX() + m21 * tupleOriginal.getY() + m22 * tupleOriginal.getZ();
         tupleTransformed.set(x, y, z);
      }
   }

   /**
    * Transforms the tuple {@code tupleOriginal} with the orientation represented by the given
    * {@code yaw}, {@code pitch}, and {@code roll} and adds the result to {@code tupleTransformed}.
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    *
    * @param yaw              the first angle representing the rotation around the z-axis.
    * @param pitch            the second angle representing the rotation around the y-axis.
    * @param roll             the third angle representing the rotation around the x-axis.
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   public static void addTransform(double yaw, double pitch, double roll, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      double x = tupleTransformed.getX();
      double y = tupleTransformed.getY();
      double z = tupleTransformed.getZ();
      transform(yaw, pitch, roll, tupleOriginal, tupleTransformed);
      tupleTransformed.add(x, y, z);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} using {@code yawPitchRoll} and adds the result to
    * {@code tupleTransformed}.
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    *
    * @param yawPitchRoll     the yaw-pitch-roll used to transform the tuple. Not modified.
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   public static void addTransform(YawPitchRollReadOnly yawPitchRoll, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      addTransform(yawPitchRoll.getYaw(), yawPitchRoll.getPitch(), yawPitchRoll.getRoll(), tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} with the orientation represented by the given
    * {@code yaw}, {@code pitch}, and {@code roll} and stores the result in {@code tupleTransformed}.
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    *
    * @param yaw                       the first angle representing the rotation around the z-axis.
    * @param pitch                     the second angle representing the rotation around the y-axis.
    * @param roll                      the third angle representing the rotation around the x-axis.
    * @param tupleOriginal             the tuple to transform. Not modified.
    * @param tupleTransformed          the tuple in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the yaw-pitch-roll
    *                                  represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the yaw-pitch-roll
    *                               does not represent a transformation in the XY plane.
    */
   public static void transform(double yaw, double pitch, double roll, Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed,
                                boolean checkIfTransformInXYPlane)
   {
      transformImpl(yaw, pitch, roll, false, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} with the orientation represented by the given
    * {@code yaw}, {@code pitch}, and {@code roll} and stores the result in {@code tupleTransformed}.
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    *
    * @param yawPitchRoll              the yaw-pitch-roll used to transform the tuple. Not modified.
    * @param tupleOriginal             the tuple to transform. Not modified.
    * @param tupleTransformed          the tuple in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the yaw-pitch-roll
    *                                  represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the yaw-pitch-roll
    *                               does not represent a transformation in the XY plane.
    */
   public static void transform(YawPitchRollReadOnly yawPitchRoll, Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed,
                                boolean checkIfTransformInXYPlane)
   {
      transform(yawPitchRoll.getYaw(), yawPitchRoll.getPitch(), yawPitchRoll.getRoll(), tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform of the tuple {@code tupleOriginal} using
    * {@code yawPitchRoll} and stores the result in {@code tupleTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(YawPitchRollReadOnly, Tuple2DReadOnly, Tuple2DBasics, boolean)} after having
    * inverted the orientation used to transform.
    * </p>
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    *
    * @param yaw                       the first angle representing the rotation around the z-axis.
    * @param pitch                     the second angle representing the rotation around the y-axis.
    * @param roll                      the third angle representing the rotation around the x-axis.
    * @param tupleOriginal             the tuple to transform. Not modified.
    * @param tupleTransformed          the tuple in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the yaw-pitch-roll
    *                                  represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the yaw-pitch-roll
    *                               does not represent a transformation in the XY plane.
    */
   public static void inverseTransform(double yaw, double pitch, double roll, Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed,
                                       boolean checkIfTransformInXYPlane)
   {
      transformImpl(yaw, pitch, roll, true, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform of the tuple {@code tupleOriginal} using
    * {@code yawPitchRoll} and stores the result in {@code tupleTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(YawPitchRollReadOnly, Tuple2DReadOnly, Tuple2DBasics, boolean)} after having
    * inverted the orientation used to transform.
    * </p>
    * <p>
    * Both tuples can be the same object for performing in place transformation.
    * </p>
    *
    * @param yawPitchRoll              the yaw-pitch-roll used to transform the tuple. Not modified.
    * @param tupleOriginal             the tuple to transform. Not modified.
    * @param tupleTransformed          the tuple in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the yaw-pitch-roll
    *                                  represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the yaw-pitch-roll
    *                               does not represent a transformation in the XY plane.
    */
   public static void inverseTransform(YawPitchRollReadOnly yawPitchRoll, Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed,
                                       boolean checkIfTransformInXYPlane)
   {
      inverseTransform(yawPitchRoll.getYaw(), yawPitchRoll.getPitch(), yawPitchRoll.getRoll(), tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   private static void transformImpl(double yaw, double pitch, double roll, boolean inverseTransform, Tuple2DReadOnly tupleOriginal,
                                     Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      if (isZero(yaw, pitch, roll, ZERO_EPS))
      {
         tupleTransformed.set(tupleOriginal);
         return;
      }

      if (EuclidCoreTools.containsNaN(yaw, pitch, roll))
      {
         tupleTransformed.setToNaN();
         return;
      }

      if (checkIfTransformInXYPlane && !isOrientation2D(yaw, pitch, roll, ZERO_EPS))
         throw new NotAnOrientation2DException("The orientation is not in XY plane: \n" + EuclidCoreIOTools.getYawPitchRollString(yaw, pitch, roll));

      double cosc = Math.cos(yaw);
      double sinc = Math.sin(yaw);

      double cosb = Math.cos(pitch);
      double sinb = Math.sin(pitch);

      double cosa = Math.cos(roll);
      double sina = Math.sin(roll);

      // Introduction to Robotics, 2.64
      double m00 = cosc * cosb;
      double m01 = cosc * sinb * sina - sinc * cosa;
      double m10 = sinc * cosb;
      double m11 = sinc * sinb * sina + cosc * cosa;

      if (inverseTransform)
      {
         double x = m00 * tupleOriginal.getX() + m10 * tupleOriginal.getY();
         double y = m01 * tupleOriginal.getX() + m11 * tupleOriginal.getY();
         tupleTransformed.set(x, y);
      }
      else
      {
         double x = m00 * tupleOriginal.getX() + m01 * tupleOriginal.getY();
         double y = m10 * tupleOriginal.getX() + m11 * tupleOriginal.getY();
         tupleTransformed.set(x, y);
      }
   }

   /**
    * Transforms the matrix {@code matrixOriginal} using {@code yawPitchRoll} and stores the result in
    * {@code matrixTransformed}.
    * <p>
    * Both matrices can be the same object for performing in place transformation.
    * </p>
    * <p>
    * matrixTransformed = R(yawPitchRoll) * matrixOriginal * R(yawPitchRoll)<sup>-1</sup> <br>
    * where R(yawPitchRoll) is the function to convert an yaw-pitch-roll into a 3-by-3 rotation matrix.
    * </p>
    *
    * @param yaw               the first angle representing the rotation around the z-axis.
    * @param pitch             the second angle representing the rotation around the y-axis.
    * @param roll              the third angle representing the rotation around the x-axis.
    * @param matrixOriginal    the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result is stored. Modified.
    */
   public static void transform(double yaw, double pitch, double roll, Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      transformImpl(yaw, pitch, roll, false, matrixOriginal, matrixTransformed);
   }

   /**
    * Transforms the matrix {@code matrixOriginal} using {@code yawPitchRoll} and stores the result in
    * {@code matrixTransformed}.
    * <p>
    * Both matrices can be the same object for performing in place transformation.
    * </p>
    * <p>
    * matrixTransformed = R(yawPitchRoll) * matrixOriginal * R(yawPitchRoll)<sup>-1</sup> <br>
    * where R(yawPitchRoll) is the function to convert an yaw-pitch-roll into a 3-by-3 rotation matrix.
    * </p>
    *
    * @param yawPitchRoll      the yaw-pitch-roll used to transform the matrix. Not modified.
    * @param matrixOriginal    the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result is stored. Modified.
    */
   public static void transform(YawPitchRollReadOnly yawPitchRoll, Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      transform(yawPitchRoll.getYaw(), yawPitchRoll.getPitch(), yawPitchRoll.getRoll(), matrixOriginal, matrixTransformed);
   }

   /**
    * Performs the inverse of the transform of the matrix {@code matrixOriginal} using
    * {@code yawPitchRoll} and stores the result in {@code matrixTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(YawPitchRollReadOnly, Matrix3DReadOnly, Matrix3DBasics)} after having inverted
    * the orientation used to transform.
    * </p>
    * <p>
    * Both matrices can be the same object for performing in place transformation.
    * </p>
    * <p>
    * matrixTransformed = R(yawPitchRoll)<sup>-1</sup> * matrixOriginal * R(yawPitchRoll) <br>
    * where R(yawPitchRoll) is the function to convert an yaw-pitch-roll into a 3-by-3 rotation matrix.
    * </p>
    *
    * @param yaw               the first angle representing the rotation around the z-axis.
    * @param pitch             the second angle representing the rotation around the y-axis.
    * @param roll              the third angle representing the rotation around the x-axis.
    * @param matrixOriginal    the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result is stored. Modified.
    */
   public static void inverseTransform(double yaw, double pitch, double roll, Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      transformImpl(yaw, pitch, roll, true, matrixOriginal, matrixTransformed);
   }

   /**
    * Performs the inverse of the transform of the matrix {@code matrixOriginal} using
    * {@code yawPitchRoll} and stores the result in {@code matrixTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(YawPitchRollReadOnly, Matrix3DReadOnly, Matrix3DBasics)} after having inverted
    * the orientation used to transform.
    * </p>
    * <p>
    * Both matrices can be the same object for performing in place transformation.
    * </p>
    * <p>
    * matrixTransformed = R(yawPitchRoll)<sup>-1</sup> * matrixOriginal * R(yawPitchRoll) <br>
    * where R(yawPitchRoll) is the function to convert an yaw-pitch-roll into a 3-by-3 rotation matrix.
    * </p>
    *
    * @param yawPitchRoll      the yaw-pitch-roll used to transform the matrix. Not modified.
    * @param matrixOriginal    the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result is stored. Modified.
    */
   public static void inverseTransform(YawPitchRollReadOnly yawPitchRoll, Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      inverseTransform(yawPitchRoll.getYaw(), yawPitchRoll.getPitch(), yawPitchRoll.getRoll(), matrixOriginal, matrixTransformed);
   }

   private static void transformImpl(double yaw, double pitch, double roll, boolean inverseTransform, Matrix3DReadOnly matrixOriginal,
                                     Matrix3DBasics matrixTransformed)
   {
      if (isZero(yaw, pitch, roll, ZERO_EPS))
      {
         matrixTransformed.set(matrixOriginal);
         return;
      }

      if (EuclidCoreTools.containsNaN(yaw, pitch, roll))
      {
         matrixTransformed.setToNaN();
         return;
      }

      double cosc = Math.cos(yaw);
      double sinc = Math.sin(yaw);

      double cosb = Math.cos(pitch);
      double sinb = Math.sin(pitch);

      double cosa = Math.cos(roll);
      double sina = Math.sin(roll);

      double ypr00, ypr01, ypr02, ypr10, ypr11, ypr12, ypr20, ypr21, ypr22;
      // Introduction to Robotics, 2.64
      if (inverseTransform)
      {
         ypr00 = cosc * cosb;
         ypr10 = cosc * sinb * sina - sinc * cosa;
         ypr20 = cosc * sinb * cosa + sinc * sina;
         ypr01 = sinc * cosb;
         ypr11 = sinc * sinb * sina + cosc * cosa;
         ypr21 = sinc * sinb * cosa - cosc * sina;
         ypr02 = -sinb;
         ypr12 = cosb * sina;
         ypr22 = cosb * cosa;
      }
      else
      {
         ypr00 = cosc * cosb;
         ypr01 = cosc * sinb * sina - sinc * cosa;
         ypr02 = cosc * sinb * cosa + sinc * sina;
         ypr10 = sinc * cosb;
         ypr11 = sinc * sinb * sina + cosc * cosa;
         ypr12 = sinc * sinb * cosa - cosc * sina;
         ypr20 = -sinb;
         ypr21 = cosb * sina;
         ypr22 = cosb * cosa;
      }

      double yprM00 = ypr00 * matrixOriginal.getM00() + ypr01 * matrixOriginal.getM10() + ypr02 * matrixOriginal.getM20();
      double yprM01 = ypr00 * matrixOriginal.getM01() + ypr01 * matrixOriginal.getM11() + ypr02 * matrixOriginal.getM21();
      double yprM02 = ypr00 * matrixOriginal.getM02() + ypr01 * matrixOriginal.getM12() + ypr02 * matrixOriginal.getM22();
      double yprM10 = ypr10 * matrixOriginal.getM00() + ypr11 * matrixOriginal.getM10() + ypr12 * matrixOriginal.getM20();
      double yprM11 = ypr10 * matrixOriginal.getM01() + ypr11 * matrixOriginal.getM11() + ypr12 * matrixOriginal.getM21();
      double yprM12 = ypr10 * matrixOriginal.getM02() + ypr11 * matrixOriginal.getM12() + ypr12 * matrixOriginal.getM22();
      double yprM20 = ypr20 * matrixOriginal.getM00() + ypr21 * matrixOriginal.getM10() + ypr22 * matrixOriginal.getM20();
      double yprM21 = ypr20 * matrixOriginal.getM01() + ypr21 * matrixOriginal.getM11() + ypr22 * matrixOriginal.getM21();
      double yprM22 = ypr20 * matrixOriginal.getM02() + ypr21 * matrixOriginal.getM12() + ypr22 * matrixOriginal.getM22();

      double yprMrpy00 = yprM00 * ypr00 + yprM01 * ypr01 + yprM02 * ypr02;
      double yprMrpy01 = yprM00 * ypr10 + yprM01 * ypr11 + yprM02 * ypr12;
      double yprMrpy02 = yprM00 * ypr20 + yprM01 * ypr21 + yprM02 * ypr22;
      double yprMrpy10 = yprM10 * ypr00 + yprM11 * ypr01 + yprM12 * ypr02;
      double yprMrpy11 = yprM10 * ypr10 + yprM11 * ypr11 + yprM12 * ypr12;
      double yprMrpy12 = yprM10 * ypr20 + yprM11 * ypr21 + yprM12 * ypr22;
      double yprMrpy20 = yprM20 * ypr00 + yprM21 * ypr01 + yprM22 * ypr02;
      double yprMrpy21 = yprM20 * ypr10 + yprM21 * ypr11 + yprM22 * ypr12;
      double yprMrpy22 = yprM20 * ypr20 + yprM21 * ypr21 + yprM22 * ypr22;

      matrixTransformed.set(yprMrpy00, yprMrpy01, yprMrpy02, yprMrpy10, yprMrpy11, yprMrpy12, yprMrpy20, yprMrpy21, yprMrpy22);
   }

   /**
    * Transforms the rotation matrix {@code rotationMatrixOriginal} using {@code yawPitchRoll} and
    * stores the result in {@code rotationMatrixTransformed}.
    * <p>
    * Both rotation matrices can be the same object for performing in place transformation.
    * </p>
    * <p>
    * rotationMatrixTransformed = R(yawPitchRoll) * rotationMatrixOriginal <br>
    * where R(yawPitchRoll) is the function to convert an yaw-pitch-roll into a 3-by-3 rotation matrix.
    * </p>
    * <p>
    * Note that this transformation is equivalent to concatenating the orientations of
    * {@code yawPitchRoll} and {@code rotationMatrixOriginal}.
    * </p>
    *
    * @param yaw                       the first angle representing the rotation around the z-axis.
    * @param pitch                     the second angle representing the rotation around the y-axis.
    * @param roll                      the third angle representing the rotation around the x-axis.
    * @param rotationMatrixOriginal    the rotation matrix to transform. Not modified.
    * @param rotationMatrixTransformed the rotation matrix in which the result is stored. Modified.
    */
   public static void transform(double yaw, double pitch, double roll, RotationMatrixReadOnly rotationMatrixOriginal, RotationMatrix rotationMatrixTransformed)
   {
      transformImpl(yaw, pitch, roll, false, rotationMatrixOriginal, rotationMatrixTransformed);
   }

   /**
    * Transforms the rotation matrix {@code rotationMatrixOriginal} using {@code yawPitchRoll} and
    * stores the result in {@code rotationMatrixTransformed}.
    * <p>
    * Both rotation matrices can be the same object for performing in place transformation.
    * </p>
    * <p>
    * rotationMatrixTransformed = R(yawPitchRoll) * rotationMatrixOriginal <br>
    * where R(yawPitchRoll) is the function to convert an yaw-pitch-roll into a 3-by-3 rotation matrix.
    * </p>
    * <p>
    * Note that this transformation is equivalent to concatenating the orientations of
    * {@code yawPitchRoll} and {@code rotationMatrixOriginal}.
    * </p>
    *
    * @param yawPitchRoll              the yaw-pitch-roll used to transform the rotation matrix. Not
    *                                  modified.
    * @param rotationMatrixOriginal    the rotation matrix to transform. Not modified.
    * @param rotationMatrixTransformed the rotation matrix in which the result is stored. Modified.
    */
   public static void transform(YawPitchRollReadOnly yawPitchRoll, RotationMatrixReadOnly rotationMatrixOriginal, RotationMatrix rotationMatrixTransformed)
   {
      transform(yawPitchRoll.getYaw(), yawPitchRoll.getPitch(), yawPitchRoll.getRoll(), rotationMatrixOriginal, rotationMatrixTransformed);
   }

   /**
    * Performs the inverse of the transform of the rotation matrix {@code rotationMatrixOriginal} using
    * {@code yawPitchRoll} and stores the result in {@code rotationMatrixTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(YawPitchRollReadOnly, RotationMatrixReadOnly, RotationMatrix)} with an
    * yaw-pitch-roll that has an angle of opposite value compared to the given one.
    * </p>
    * <p>
    * Both rotation matrices can be the same object for performing in place transformation.
    * </p>
    * <p>
    * rotationMatrixTransformed = R(yawPitchRoll)<sup>-1</sup> * rotationMatrixOriginal <br>
    * where R(yawPitchRoll) is the function to convert an yaw-pitch-roll into a 3-by-3 rotation matrix.
    * </p>
    * <p>
    * Note that this transformation is equivalent to concatenating the orientations of
    * {@code yawPitchRoll} and {@code rotationMatrixOriginal}.
    * </p>
    *
    * @param yaw                       the first angle representing the rotation around the z-axis.
    * @param pitch                     the second angle representing the rotation around the y-axis.
    * @param roll                      the third angle representing the rotation around the x-axis.
    * @param rotationMatrixOriginal    the rotation matrix to transform. Not modified.
    * @param rotationMatrixTransformed the rotation matrix in which the result is stored. Modified.
    */
   public static void inverseTransform(double yaw, double pitch, double roll, RotationMatrixReadOnly rotationMatrixOriginal,
                                       RotationMatrix rotationMatrixTransformed)
   {
      transformImpl(yaw, pitch, roll, true, rotationMatrixOriginal, rotationMatrixTransformed);
   }

   /**
    * Performs the inverse of the transform of the rotation matrix {@code rotationMatrixOriginal} using
    * {@code yawPitchRoll} and stores the result in {@code rotationMatrixTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(YawPitchRollReadOnly, RotationMatrixReadOnly, RotationMatrix)} with an
    * yaw-pitch-roll that has an angle of opposite value compared to the given one.
    * </p>
    * <p>
    * Both rotation matrices can be the same object for performing in place transformation.
    * </p>
    * <p>
    * rotationMatrixTransformed = R(yawPitchRoll)<sup>-1</sup> * rotationMatrixOriginal <br>
    * where R(yawPitchRoll) is the function to convert an yaw-pitch-roll into a 3-by-3 rotation matrix.
    * </p>
    * <p>
    * Note that this transformation is equivalent to concatenating the orientations of
    * {@code yawPitchRoll} and {@code rotationMatrixOriginal}.
    * </p>
    *
    * @param yawPitchRoll              the yaw-pitch-roll used to transform the rotation matrix. Not
    *                                  modified.
    * @param rotationMatrixOriginal    the rotation matrix to transform. Not modified.
    * @param rotationMatrixTransformed the rotation matrix in which the result is stored. Modified.
    */
   public static void inverseTransform(YawPitchRollReadOnly yawPitchRoll, RotationMatrixReadOnly rotationMatrixOriginal,
                                       RotationMatrix rotationMatrixTransformed)
   {
      inverseTransform(yawPitchRoll.getYaw(), yawPitchRoll.getPitch(), yawPitchRoll.getRoll(), rotationMatrixOriginal, rotationMatrixTransformed);
   }

   private static void transformImpl(double yaw, double pitch, double roll, boolean inverseTransform, RotationMatrixReadOnly rotationMatrixOriginal,
                                     RotationMatrix rotationMatrixTransformed)
   {
      if (isZero(yaw, pitch, roll, ZERO_EPS))
      {
         rotationMatrixTransformed.set(rotationMatrixOriginal);
         return;
      }

      if (EuclidCoreTools.containsNaN(yaw, pitch, roll))
      {
         rotationMatrixTransformed.setToNaN();
         return;
      }

      double cosc = Math.cos(yaw);
      double sinc = Math.sin(yaw);

      double cosb = Math.cos(pitch);
      double sinb = Math.sin(pitch);

      double cosa = Math.cos(roll);
      double sina = Math.sin(roll);

      double ypr00, ypr01, ypr02, ypr10, ypr11, ypr12, ypr20, ypr21, ypr22;
      // Introduction to Robotics, 2.64
      if (inverseTransform)
      {
         ypr00 = cosc * cosb;
         ypr10 = cosc * sinb * sina - sinc * cosa;
         ypr20 = cosc * sinb * cosa + sinc * sina;
         ypr01 = sinc * cosb;
         ypr11 = sinc * sinb * sina + cosc * cosa;
         ypr21 = sinc * sinb * cosa - cosc * sina;
         ypr02 = -sinb;
         ypr12 = cosb * sina;
         ypr22 = cosb * cosa;
      }
      else
      {
         ypr00 = cosc * cosb;
         ypr01 = cosc * sinb * sina - sinc * cosa;
         ypr02 = cosc * sinb * cosa + sinc * sina;
         ypr10 = sinc * cosb;
         ypr11 = sinc * sinb * sina + cosc * cosa;
         ypr12 = sinc * sinb * cosa - cosc * sina;
         ypr20 = -sinb;
         ypr21 = cosb * sina;
         ypr22 = cosb * cosa;
      }

      double yprM00 = ypr00 * rotationMatrixOriginal.getM00() + ypr01 * rotationMatrixOriginal.getM10() + ypr02 * rotationMatrixOriginal.getM20();
      double yprM01 = ypr00 * rotationMatrixOriginal.getM01() + ypr01 * rotationMatrixOriginal.getM11() + ypr02 * rotationMatrixOriginal.getM21();
      double yprM02 = ypr00 * rotationMatrixOriginal.getM02() + ypr01 * rotationMatrixOriginal.getM12() + ypr02 * rotationMatrixOriginal.getM22();
      double yprM10 = ypr10 * rotationMatrixOriginal.getM00() + ypr11 * rotationMatrixOriginal.getM10() + ypr12 * rotationMatrixOriginal.getM20();
      double yprM11 = ypr10 * rotationMatrixOriginal.getM01() + ypr11 * rotationMatrixOriginal.getM11() + ypr12 * rotationMatrixOriginal.getM21();
      double yprM12 = ypr10 * rotationMatrixOriginal.getM02() + ypr11 * rotationMatrixOriginal.getM12() + ypr12 * rotationMatrixOriginal.getM22();
      double yprM20 = ypr20 * rotationMatrixOriginal.getM00() + ypr21 * rotationMatrixOriginal.getM10() + ypr22 * rotationMatrixOriginal.getM20();
      double yprM21 = ypr20 * rotationMatrixOriginal.getM01() + ypr21 * rotationMatrixOriginal.getM11() + ypr22 * rotationMatrixOriginal.getM21();
      double yprM22 = ypr20 * rotationMatrixOriginal.getM02() + ypr21 * rotationMatrixOriginal.getM12() + ypr22 * rotationMatrixOriginal.getM22();

      rotationMatrixTransformed.set(yprM00, yprM01, yprM02, yprM10, yprM11, yprM12, yprM20, yprM21, yprM22);
   }

   /**
    * Transforms the vector {@code vectorOriginal} using {@code yawPitchRoll} and stores the result in
    * {@code vectorTransformed}.
    * <p>
    * Both vectors can be the same object for performing in place transformation.
    * </p>
    *
    * @param yaw               the first angle representing the rotation around the z-axis.
    * @param pitch             the second angle representing the rotation around the y-axis.
    * @param roll              the third angle representing the rotation around the x-axis.
    * @param vectorOriginal    the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    */
   public static void transform(double yaw, double pitch, double roll, Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      transformImpl(yaw, pitch, roll, false, vectorOriginal, vectorTransformed);
   }

   /**
    * Transforms the vector {@code vectorOriginal} using {@code yawPitchRoll} and stores the result in
    * {@code vectorTransformed}.
    * <p>
    * Both vectors can be the same object for performing in place transformation.
    * </p>
    *
    * @param yawPitchRoll      the yaw-pitch-roll used to transform the tuple. Not modified.
    * @param vectorOriginal    the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    */
   public static void transform(YawPitchRollReadOnly yawPitchRoll, Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      transform(yawPitchRoll.getYaw(), yawPitchRoll.getPitch(), yawPitchRoll.getRoll(), vectorOriginal, vectorTransformed);
   }

   /**
    * Performs the inverse of the transform of the vector {@code vectorOriginal} using
    * {@code yawPitchRoll} and stores the result in {@code vectorTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(YawPitchRollReadOnly, Vector4DReadOnly, Vector4DBasics)} with an yaw-pitch-roll
    * that has an angle of opposite value compared to the given one.
    * </p>
    * <p>
    * Both vectors can be the same object for performing in place transformation.
    * </p>
    *
    * @param yaw               the first angle representing the rotation around the z-axis.
    * @param pitch             the second angle representing the rotation around the y-axis.
    * @param roll              the third angle representing the rotation around the x-axis.
    * @param vectorOriginal    the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    */
   public static void inverseTransform(double yaw, double pitch, double roll, Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      transformImpl(yaw, pitch, roll, true, vectorOriginal, vectorTransformed);
   }

   /**
    * Performs the inverse of the transform of the vector {@code vectorOriginal} using
    * {@code yawPitchRoll} and stores the result in {@code vectorTransformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(YawPitchRollReadOnly, Vector4DReadOnly, Vector4DBasics)} with an yaw-pitch-roll
    * that has an angle of opposite value compared to the given one.
    * </p>
    * <p>
    * Both vectors can be the same object for performing in place transformation.
    * </p>
    *
    * @param yawPitchRoll      the yaw-pitch-roll used to transform the tuple. Not modified.
    * @param vectorOriginal    the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    */
   public static void inverseTransform(YawPitchRollReadOnly yawPitchRoll, Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      inverseTransform(yawPitchRoll.getYaw(), yawPitchRoll.getPitch(), yawPitchRoll.getRoll(), vectorOriginal, vectorTransformed);
   }

   private static void transformImpl(double yaw, double pitch, double roll, boolean inverseTransform, Vector4DReadOnly vectorOriginal,
                                     Vector4DBasics vectorTransformed)
   {
      if (isZero(yaw, pitch, roll, ZERO_EPS))
      {
         vectorTransformed.set(vectorOriginal);
         return;
      }

      if (EuclidCoreTools.containsNaN(yaw, pitch, roll))
      {
         vectorTransformed.setToNaN();
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

      if (inverseTransform)
      {
         double x = m00 * vectorOriginal.getX() + m10 * vectorOriginal.getY() + m20 * vectorOriginal.getZ();
         double y = m01 * vectorOriginal.getX() + m11 * vectorOriginal.getY() + m21 * vectorOriginal.getZ();
         double z = m02 * vectorOriginal.getX() + m12 * vectorOriginal.getY() + m22 * vectorOriginal.getZ();
         vectorTransformed.set(x, y, z, vectorOriginal.getS());
      }
      else
      {
         double x = m00 * vectorOriginal.getX() + m01 * vectorOriginal.getY() + m02 * vectorOriginal.getZ();
         double y = m10 * vectorOriginal.getX() + m11 * vectorOriginal.getY() + m12 * vectorOriginal.getZ();
         double z = m20 * vectorOriginal.getX() + m21 * vectorOriginal.getY() + m22 * vectorOriginal.getZ();
         vectorTransformed.set(x, y, z, vectorOriginal.getS());
      }
   }

   /**
    * Performs the multiplication of {@code orientation1} and {@code orientation2} and stores the
    * result in {@code yawPitchRollToPack}.
    * <p>
    * More precisely, {@code orientation1} and {@code orientation2} are first converted to quaternions,
    * then a quaternion multiplication is performed using the two first arguments as entry, the result
    * is then converted and stored in the given {@code yawPitchRoll}.
    * </p>
    * <p>
    * All three arguments can be the same object for in place operations.
    * </p>
    * 
    * @param orientation1       the first orientation in the multiplication. Not modified.
    * @param inverse1           whether the first orientation should be inverted in the multiplication.
    * @param orientation2       the second orientation in the multiplication. Not modified.
    * @param inverse2           whether the second orientation should be inverted in the
    *                           multiplication.
    * @param yawPitchRollToPack the yaw-pitch-roll in which the result is stored. Modified.
    */
   public static void multiply(Orientation3DReadOnly orientation1, boolean inverse1, Orientation3DReadOnly orientation2, boolean inverse2,
                               YawPitchRollBasics yawPitchRollToPack)
   {
      double q1s, q1x, q1y, q1z;

      if (orientation1 instanceof QuaternionReadOnly)
      {
         QuaternionReadOnly quaternion1 = (QuaternionReadOnly) orientation1;
         q1s = quaternion1.getS();
         q1x = quaternion1.getX();
         q1y = quaternion1.getY();
         q1z = quaternion1.getZ();
      }
      else if (orientation1 instanceof AxisAngleReadOnly)
      {
         AxisAngleReadOnly axisAngle1 = (AxisAngleReadOnly) orientation1;
         double halfTheta = 0.5 * axisAngle1.getAngle();
         double sinHalfTheta = Math.sin(halfTheta) / axisAngle1.axisNorm();
         q1x = axisAngle1.getX() * sinHalfTheta;
         q1y = axisAngle1.getY() * sinHalfTheta;
         q1z = axisAngle1.getZ() * sinHalfTheta;
         q1s = Math.cos(halfTheta);
      }
      else
      {
         double halfYaw = 0.5 * orientation1.getYaw();
         double cYaw = Math.cos(halfYaw);
         double sYaw = Math.sin(halfYaw);

         double halfPitch = 0.5 * orientation1.getPitch();
         double cPitch = Math.cos(halfPitch);
         double sPitch = Math.sin(halfPitch);

         double halfRoll = 0.5 * orientation1.getRoll();
         double cRoll = Math.cos(halfRoll);
         double sRoll = Math.sin(halfRoll);

         q1s = cYaw * cPitch * cRoll + sYaw * sPitch * sRoll;
         q1x = cYaw * cPitch * sRoll - sYaw * sPitch * cRoll;
         q1y = sYaw * cPitch * sRoll + cYaw * sPitch * cRoll;
         q1z = sYaw * cPitch * cRoll - cYaw * sPitch * sRoll;
      }

      double q2s, q2x, q2y, q2z;

      if (orientation2 instanceof QuaternionReadOnly)
      {
         QuaternionReadOnly quaternion2 = (QuaternionReadOnly) orientation2;
         q2s = quaternion2.getS();
         q2x = quaternion2.getX();
         q2y = quaternion2.getY();
         q2z = quaternion2.getZ();
      }
      else if (orientation2 instanceof AxisAngleReadOnly)
      {
         AxisAngleReadOnly axisAngle2 = (AxisAngleReadOnly) orientation2;
         double halfTheta = 0.5 * axisAngle2.getAngle();
         double sinHalfTheta = Math.sin(halfTheta) / axisAngle2.axisNorm();
         q2x = axisAngle2.getX() * sinHalfTheta;
         q2y = axisAngle2.getY() * sinHalfTheta;
         q2z = axisAngle2.getZ() * sinHalfTheta;
         q2s = Math.cos(halfTheta);
      }
      else
      {
         double halfYaw = 0.5 * orientation2.getYaw();
         double cYaw = Math.cos(halfYaw);
         double sYaw = Math.sin(halfYaw);

         double halfPitch = 0.5 * orientation2.getPitch();
         double cPitch = Math.cos(halfPitch);
         double sPitch = Math.sin(halfPitch);

         double halfRoll = 0.5 * orientation2.getRoll();
         double cRoll = Math.cos(halfRoll);
         double sRoll = Math.sin(halfRoll);

         q2s = cYaw * cPitch * cRoll + sYaw * sPitch * sRoll;
         q2x = cYaw * cPitch * sRoll - sYaw * sPitch * cRoll;
         q2y = sYaw * cPitch * sRoll + cYaw * sPitch * cRoll;
         q2z = sYaw * cPitch * cRoll - cYaw * sPitch * sRoll;
      }

      QuaternionTools.multiplyImpl(q1x, q1y, q1z, q1s, inverse1, q2x, q2y, q2z, q2s, inverse2, yawPitchRollToPack);
   }

   /**
    * Prepend a rotation about the z-axis to {@code yawPitchRollOriginal} and stores the result in
    * {@code yawPitchRollToPack}.
    * <p>
    * All the yaw-pitch-rolls can be the same object.
    * </p>
    *
    * @param yaw                  the angle to rotate about the z-axis.
    * @param yawPitchRollOriginal the yaw-pitch-roll on which the yaw rotation is prepended. Not
    *                             modified.
    * @param yawPitchRollToPack   the yaw-pitch-roll in which the result is stored. Modified.
    */
   public static void prependYawRotation(YawPitchRollReadOnly yawPitchRollOriginal, double yaw, YawPitchRollBasics yawPitchRollToPack)
   {
      yaw = EuclidCoreTools.trimAngleMinusPiToPi(yawPitchRollOriginal.getYaw() + yaw);
      double pitch = yawPitchRollOriginal.getPitch();
      double roll = yawPitchRollOriginal.getRoll();
      yawPitchRollToPack.set(yaw, pitch, roll);
   }

   /**
    * Append a rotation about the z-axis to {@code yawPitchRollOriginal} and stores the result in
    * {@code yawPitchRollToPack}.
    * <p>
    * All the yaw-pitch-rolls can be the same object.
    * </p>
    *
    * @param yawPitchRollOriginal the yaw-pitch-roll on which the yaw rotation is appended. Not
    *                             modified.
    * @param yaw                  the angle to rotate about the z-axis.
    * @param yawPitchRollToPack   the yaw-pitch-roll in which the result is stored. Modified.
    */
   public static void appendYawRotation(YawPitchRollReadOnly yawPitchRollOriginal, double yaw, YawPitchRollBasics yawPitchRollToPack)
   {
      double qs, qx, qy, qz;

      {
         double halfYaw = 0.5 * yawPitchRollOriginal.getYaw();
         double cYaw = Math.cos(halfYaw);
         double sYaw = Math.sin(halfYaw);

         double halfPitch = 0.5 * yawPitchRollOriginal.getPitch();
         double cPitch = Math.cos(halfPitch);
         double sPitch = Math.sin(halfPitch);

         double halfRoll = 0.5 * yawPitchRollOriginal.getRoll();
         double cRoll = Math.cos(halfRoll);
         double sRoll = Math.sin(halfRoll);

         qs = cYaw * cPitch * cRoll + sYaw * sPitch * sRoll;
         qx = cYaw * cPitch * sRoll - sYaw * sPitch * cRoll;
         qy = sYaw * cPitch * sRoll + cYaw * sPitch * cRoll;
         qz = sYaw * cPitch * cRoll - cYaw * sPitch * sRoll;
      }

      double halfYaw = 0.5 * yaw;
      double cYaw = Math.cos(halfYaw);
      double sYaw = Math.sin(halfYaw);

      double x = qx * cYaw + qy * sYaw;
      double y = -qx * sYaw + qy * cYaw;
      double z = qs * sYaw + qz * cYaw;
      double s = qs * cYaw - qz * sYaw;
      yawPitchRollToPack.setQuaternion(x, y, z, s);
   }

   /**
    * Prepend a rotation about the y-axis to {@code yawPitchRollOriginal} and stores the result in
    * {@code yawPitchRollToPack}.
    * <p>
    * All the yaw-pitch-rolls can be the same object.
    * </p>
    *
    * @param pitch                the angle to rotate about the y-axis.
    * @param yawPitchRollOriginal the yaw-pitch-roll on which the yaw rotation is prepended. Not
    *                             modified.
    * @param yawPitchRollToPack   the yaw-pitch-roll in which the result is stored. Modified.
    */
   public static void prependPitchRotation(YawPitchRollReadOnly yawPitchRollOriginal, double pitch, YawPitchRollBasics yawPitchRollToPack)
   {
      if (Math.abs(yawPitchRollOriginal.getRoll()) < ZERO_EPS)
      {
         double yaw = yawPitchRollOriginal.getYaw();
         pitch = EuclidCoreTools.trimAngleMinusPiToPi(pitch + yawPitchRollOriginal.getPitch());
         double roll = yawPitchRollOriginal.getRoll();
         yawPitchRollToPack.set(yaw, pitch, roll);
      }
      else
      {
         double qs, qx, qy, qz;

         {
            double halfYaw = 0.5 * yawPitchRollOriginal.getYaw();
            double cYaw = Math.cos(halfYaw);
            double sYaw = Math.sin(halfYaw);

            double halfPitch = 0.5 * yawPitchRollOriginal.getPitch();
            double cPitch = Math.cos(halfPitch);
            double sPitch = Math.sin(halfPitch);

            double halfRoll = 0.5 * yawPitchRollOriginal.getRoll();
            double cRoll = Math.cos(halfRoll);
            double sRoll = Math.sin(halfRoll);

            qs = cYaw * cPitch * cRoll + sYaw * sPitch * sRoll;
            qx = cYaw * cPitch * sRoll - sYaw * sPitch * cRoll;
            qy = sYaw * cPitch * sRoll + cYaw * sPitch * cRoll;
            qz = sYaw * cPitch * cRoll - cYaw * sPitch * sRoll;
         }

         double halfPitch = 0.5 * pitch;
         double cPitch = Math.cos(halfPitch);
         double sPitch = Math.sin(halfPitch);

         double x = cPitch * qx + sPitch * qz;
         double y = cPitch * qy + sPitch * qs;
         double z = cPitch * qz - sPitch * qx;
         double s = cPitch * qs - sPitch * qy;
         yawPitchRollToPack.setQuaternion(x, y, z, s);
      }
   }

   /**
    * Append a rotation about the y-axis to {@code yawPitchRollOriginal} and stores the result in
    * {@code yawPitchRollToPack}.
    * <p>
    * All the yaw-pitch-rolls can be the same object.
    * </p>
    *
    * @param yawPitchRollOriginal the yaw-pitch-roll on which the yaw rotation is appended. Not
    *                             modified.
    * @param pitch                the angle to rotate about the y-axis.
    * @param yawPitchRollToPack   the yaw-pitch-roll in which the result is stored. Modified.
    */
   public static void appendPitchRotation(YawPitchRollReadOnly yawPitchRollOriginal, double pitch, YawPitchRollBasics yawPitchRollToPack)
   {
      if (Math.abs(yawPitchRollOriginal.getRoll()) < ZERO_EPS)
      {
         double yaw = yawPitchRollOriginal.getYaw();
         pitch = EuclidCoreTools.trimAngleMinusPiToPi(pitch + yawPitchRollOriginal.getPitch());
         double roll = yawPitchRollOriginal.getRoll();
         yawPitchRollToPack.set(yaw, pitch, roll);
      }
      else
      {
         double qs, qx, qy, qz;

         {
            double halfYaw = 0.5 * yawPitchRollOriginal.getYaw();
            double cYaw = Math.cos(halfYaw);
            double sYaw = Math.sin(halfYaw);

            double halfPitch = 0.5 * yawPitchRollOriginal.getPitch();
            double cPitch = Math.cos(halfPitch);
            double sPitch = Math.sin(halfPitch);

            double halfRoll = 0.5 * yawPitchRollOriginal.getRoll();
            double cRoll = Math.cos(halfRoll);
            double sRoll = Math.sin(halfRoll);

            qs = cYaw * cPitch * cRoll + sYaw * sPitch * sRoll;
            qx = cYaw * cPitch * sRoll - sYaw * sPitch * cRoll;
            qy = sYaw * cPitch * sRoll + cYaw * sPitch * cRoll;
            qz = sYaw * cPitch * cRoll - cYaw * sPitch * sRoll;
         }

         double halfPitch = 0.5 * pitch;
         double cPitch = Math.cos(halfPitch);
         double sPitch = Math.sin(halfPitch);

         double x = qx * cPitch - qz * sPitch;
         double y = qs * sPitch + qy * cPitch;
         double z = qx * sPitch + qz * cPitch;
         double s = qs * cPitch - qy * sPitch;
         yawPitchRollToPack.setQuaternion(x, y, z, s);
      }
   }

   /**
    * Prepend a rotation about the x-axis to {@code yawPitchRollOriginal} and stores the result in
    * {@code yawPitchRollToPack}.
    * <p>
    * All the yaw-pitch-rolls can be the same object.
    * </p>
    *
    * @param roll                 the angle to rotate about the x-axis.
    * @param yawPitchRollOriginal the yaw-pitch-roll on which the yaw rotation is prepended. Not
    *                             modified.
    * @param yawPitchRollToPack   the yaw-pitch-roll in which the result is stored. Modified.
    */
   public static void prependRollRotation(YawPitchRollReadOnly yawPitchRollOriginal, double roll, YawPitchRollBasics yawPitchRollToPack)
   {
      double qs, qx, qy, qz;

      {
         double halfYaw = 0.5 * yawPitchRollOriginal.getYaw();
         double cYaw = Math.cos(halfYaw);
         double sYaw = Math.sin(halfYaw);

         double halfPitch = 0.5 * yawPitchRollOriginal.getPitch();
         double cPitch = Math.cos(halfPitch);
         double sPitch = Math.sin(halfPitch);

         double halfRoll = 0.5 * yawPitchRollOriginal.getRoll();
         double cRoll = Math.cos(halfRoll);
         double sRoll = Math.sin(halfRoll);

         qs = cYaw * cPitch * cRoll + sYaw * sPitch * sRoll;
         qx = cYaw * cPitch * sRoll - sYaw * sPitch * cRoll;
         qy = sYaw * cPitch * sRoll + cYaw * sPitch * cRoll;
         qz = sYaw * cPitch * cRoll - cYaw * sPitch * sRoll;
      }

      double halfRoll = 0.5 * roll;
      double cRoll = Math.cos(halfRoll);
      double sRoll = Math.sin(halfRoll);

      double x = cRoll * qx + sRoll * qs;
      double y = cRoll * qy - sRoll * qz;
      double z = cRoll * qz + sRoll * qy;
      double s = cRoll * qs - sRoll * qx;
      yawPitchRollToPack.setQuaternion(x, y, z, s);
   }

   /**
    * Append a rotation about the x-axis to {@code yawPitchRollOriginal} and stores the result in
    * {@code yawPitchRollToPack}.
    * <p>
    * All the yaw-pitch-rolls can be the same object.
    * </p>
    *
    * @param yawPitchRollOriginal the yaw-pitch-roll on which the yaw rotation is appended. Not
    *                             modified.
    * @param roll                 the angle to rotate about the x-axis.
    * @param yawPitchRollToPack   the yaw-pitch-roll in which the result is stored. Modified.
    */
   public static void appendRollRotation(YawPitchRollReadOnly yawPitchRollOriginal, double roll, YawPitchRollBasics yawPitchRollToPack)
   {
      double yaw = yawPitchRollOriginal.getYaw();
      double pitch = yawPitchRollOriginal.getPitch();
      roll = EuclidCoreTools.trimAngleMinusPiToPi(roll + yawPitchRollOriginal.getRoll());
      yawPitchRollToPack.set(yaw, pitch, roll);
   }
}
