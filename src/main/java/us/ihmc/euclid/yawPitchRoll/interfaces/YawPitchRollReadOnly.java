package us.ihmc.euclid.yawPitchRoll.interfaces;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.matrix.interfaces.CommonMatrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.rotationConversion.RotationMatrixConversion;
import us.ihmc.euclid.rotationConversion.RotationVectorConversion;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.YawPitchRollTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

/**
 * Read-only interface for a yaw-pitch-roll object.
 * <p>
 * A yaw-pitch-roll is used to represent a 3D orientation by three successive rotations: rotation
 * around the z-axis (yaw), then around the y-axis (pitch), and then around the x-axis (roll). The
 * three components yaw, pitch, and roll represents the angle for rotation expressed in radians.
 * </p>
 * <p>
 * In general, yaw-pitch-roll representation is considered one of the most intuitive way of
 * interpreting an orientation and is thus commonly used as an interface between human and machine.
 * However, there is no algebra directly accessible for manipulating orientations represented as
 * yaw-pitch-roll making it highly computationally expensive when compared to rotation matrices or
 * quaternions. In addition, yaw-pitch-roll representation is sensitive to gimbal lock which happens
 * when the pitch angle is in the neighborhood of either <i>pi/2</i> or -<i>pi/2</i>. When close to
 * such configuration, converting orientation to yaw-pitch-roll becomes inaccurate and can sometimes
 * lead to unexpected results.
 * </p>
 * <p>
 * Equivalent representation of yaw-pitch-roll as 3-by-3 rotation matrix:
 *
 * <pre>
 *     / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
 * R = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
 *     \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
 * </pre>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface YawPitchRollReadOnly extends Orientation3DReadOnly
{
   /**
    * Returns the yaw component of this yaw-pitch-roll orientation.
    *
    * @return the yaw angle.
    */
   @Override
   double getYaw();

   /**
    * Returns the yaw component of this yaw-pitch-roll orientation.
    *
    * @return the yaw angle.
    */
   default float getYaw32()
   {
      return (float) getYaw();
   }

   /**
    * Returns the pitch component of this yaw-pitch-roll orientation.
    *
    * @return the pitch angle.
    */
   @Override
   double getPitch();

   /**
    * Returns the pitch component of this yaw-pitch-roll orientation.
    *
    * @return the pitch angle.
    */
   default float getPitch32()
   {
      return (float) getPitch();
   }

   /**
    * Returns the roll component of this yaw-pitch-roll orientation.
    *
    * @return the roll angle.
    */
   @Override
   double getRoll();

   /**
    * Returns the roll component of this yaw-pitch-roll orientation.
    *
    * @return the roll angle.
    */
   default float getRoll32()
   {
      return (float) getRoll();
   }

   /**
    * Tests if this yaw-pitch-roll contains a {@link Double#NaN}.
    *
    * @return {@code true} if this yaw-pitch-roll contains a {@link Double#NaN}, {@code false}
    *         otherwise.
    */
   @Override
   default boolean containsNaN()
   {
      return EuclidCoreTools.containsNaN(getYaw(), getPitch(), getRoll());
   }

   /**
    * Tests whether the three angles of this yaw-pitch-roll are equal to zero.
    *
    * @param epsilon the tolerance to use for the comparison.
    * @return {@code true} if the three angles are equal to zero, {@code false} otherwise.
    */
   default boolean isZero(double epsilon)
   {
      return isZeroOrientation(epsilon);
   }

   /**
    * {@inheritDoc}
    * <p>
    * A yaw-pitch-roll is a zero orientation when all the three angles are equal to zero.
    * </p>
    */
   @Override
   default boolean isZeroOrientation(double epsilon)
   {
      return YawPitchRollTools.isZero(getYaw(), getPitch(), getRoll(), epsilon);
   }

   /**
    * {@inheritDoc}
    * <p>
    * A yaw-pitch-roll is an orientation 2D if both the pitch and roll angles are equal to zero given
    * the tolerance {@code espilon}.
    * </p>
    */
   @Override
   default boolean isOrientation2D(double epsilon)
   {
      return YawPitchRollTools.isOrientation2D(getYaw(), getPitch(), getRoll(), epsilon);
   }

   /** {@inheritDoc} */
   @Override
   default double distance(Orientation3DReadOnly other, boolean limitToPi)
   {
      return YawPitchRollTools.distance(this, other, limitToPi);
   }

   /** {@inheritDoc} */
   @Override
   default double angle(boolean limitToPi)
   {
      return YawPitchRollTools.angle(this, limitToPi);
   }

   /** {@inheritDoc} */
   @Override
   default void get(CommonMatrix3DBasics rotationMatrixToPack)
   {
      RotationMatrixConversion.convertYawPitchRollToMatrix(this, rotationMatrixToPack);
   }

   /** {@inheritDoc} */
   @Override
   default void get(AxisAngleBasics axisAngleToPack)
   {
      axisAngleToPack.setYawPitchRoll(getYaw(), getPitch(), getRoll());
   }

   /** {@inheritDoc} */
   @Override
   default void get(QuaternionBasics quaternionToPack)
   {
      quaternionToPack.setYawPitchRoll(getYaw(), getPitch(), getRoll());
   }

   /** {@inheritDoc} */
   @Override
   default void get(YawPitchRollBasics yawPitchRollToPack)
   {
      yawPitchRollToPack.set(getYaw(), getPitch(), getRoll());
   }

   /** {@inheritDoc} */
   @Override
   default void getRotationVector(Vector3DBasics rotationVectorToPack)
   {
      RotationVectorConversion.convertYawPitchRollToRotationVector(getYaw(), getPitch(), getRoll(), rotationVectorToPack);
   }

   /** {@inheritDoc} */
   @Override
   default void getEuler(Tuple3DBasics eulerAnglesToPack)
   {
      eulerAnglesToPack.set(getRoll(), getPitch(), getYaw());
   }

   /**
    * Packs the components of this yaw-pitch-roll in an array starting from its first index. The
    * components are packed in the following order: yaw, pitch, and roll.
    *
    * @param yawPitchRollArrayToPack the array in which this yaw-pitch-roll is stored. Modified.
    */
   default void get(double[] yawPitchRollArrayToPack)
   {
      get(0, yawPitchRollArrayToPack);
   }

   /**
    * Packs the components of this yaw-pitch-roll in an array starting from {@code startIndex}. The
    * components are packed in the following order: yaw, pitch, and roll.
    *
    * @param startIndex              the index in the array where the first component is stored.
    * @param yawPitchRollArrayToPack the array in which this yaw-pitch-roll is stored. Modified.
    */
   default void get(int startIndex, double[] yawPitchRollArrayToPack)
   {
      yawPitchRollArrayToPack[startIndex++] = getYaw();
      yawPitchRollArrayToPack[startIndex++] = getPitch();
      yawPitchRollArrayToPack[startIndex] = getRoll();
   }

   /**
    * Packs the components of this yaw-pitch-roll in an array starting from its first index. The
    * components are packed in the following order: yaw, pitch, and roll.
    *
    * @param yawPitchRollArrayToPack the array in which this yaw-pitch-roll is stored. Modified.
    */
   default void get(float[] yawPitchRollArrayToPack)
   {
      get(0, yawPitchRollArrayToPack);
   }

   /**
    * Packs the components of this yaw-pitch-roll in an array starting from {@code startIndex}. The
    * components are packed in the following order: yaw, pitch, and roll.
    *
    * @param startIndex              the index in the array where the first component is stored.
    * @param yawPitchRollArrayToPack the array in which this yaw-pitch-roll is stored. Modified.
    */
   default void get(int startIndex, float[] yawPitchRollArrayToPack)
   {
      yawPitchRollArrayToPack[startIndex++] = getYaw32();
      yawPitchRollArrayToPack[startIndex++] = getPitch32();
      yawPitchRollArrayToPack[startIndex] = getRoll32();
   }

   /**
    * Selects a component of this yaw-pitch-roll based on {@code index} and returns its value.
    * <p>
    * For {@code index} values of 0, 1, and 2, the corresponding components are yaw, pitch, and roll,
    * respectively.
    * </p>
    *
    * @param index the index of the component to get.
    * @return the value of the component.
    * @throws IndexOutOfBoundsException if {@code index} &notin; [0, 2].
    */
   default double getElement(int index)
   {
      switch (index)
      {
         case 0:
            return getYaw();
         case 1:
            return getPitch();
         case 2:
            return getRoll();
         default:
            throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   /**
    * Selects a component of this yaw-pitch-roll based on {@code index} and returns its value.
    * <p>
    * For {@code index} values of 0, 1, and 2, the corresponding components are yaw, pitch, and roll,
    * respectively.
    * </p>
    *
    * @param index the index of the component to get.
    * @return the value of the component.
    * @throws IndexOutOfBoundsException if {@code index} &notin; [0, 2].
    */
   default float getElement32(int index)
   {
      switch (index)
      {
         case 0:
            return getYaw32();
         case 1:
            return getPitch32();
         case 2:
            return getRoll32();
         default:
            throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      YawPitchRollTools.transform(this, tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfOrientation2D)
   {
      YawPitchRollTools.transform(this, tupleOriginal, tupleTransformed, checkIfOrientation2D);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      YawPitchRollTools.transform(this, matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      YawPitchRollTools.transform(this, vectorOriginal, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      YawPitchRollTools.inverseTransform(this, tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfOrientation2D)
   {
      YawPitchRollTools.inverseTransform(this, tupleOriginal, tupleTransformed, checkIfOrientation2D);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      YawPitchRollTools.inverseTransform(this, vectorOriginal, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      YawPitchRollTools.inverseTransform(this, matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default boolean equals(EuclidGeometry geometry)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof YawPitchRollReadOnly))
         return false;
      YawPitchRollReadOnly other = (YawPitchRollReadOnly) geometry;
      if (!EuclidCoreTools.equals(getYaw(), other.getYaw()))
         return false;
      if (!EuclidCoreTools.equals(getPitch(), other.getPitch()))
         return false;
      if (!EuclidCoreTools.equals(getRoll(), other.getRoll()))
         return false;

      return true;
   }

   /** {@inheritDoc} */
   @Override
   default boolean epsilonEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof YawPitchRollReadOnly))
         return false;

      YawPitchRollReadOnly other = (YawPitchRollReadOnly) geometry;
      if (!EuclidCoreTools.epsilonEquals(getYaw(), other.getYaw(), epsilon))
         return false;
      if (!EuclidCoreTools.epsilonEquals(getPitch(), other.getPitch(), epsilon))
         return false;
      if (!EuclidCoreTools.epsilonEquals(getRoll(), other.getRoll(), epsilon))
         return false;

      return true;
   }

   /**
    * Gets a representative {@code String} of this yaw-pitch-roll given a specific format to use.
    * <p>
    * Using the default format {@link EuclidCoreIOTools#DEFAULT_FORMAT}, this provides a {@code String}
    * as follows:
    *
    * <pre>
    * yaw-pitch-roll: ( 0.674,  0.455,  0.582 )
    * </pre>
    * </p>
    */
   @Override
   default String toString(String format)
   {
      return EuclidCoreIOTools.getYawPitchRollString(format, this);
   }
}
