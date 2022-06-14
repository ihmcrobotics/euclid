package us.ihmc.euclid.axisAngle.interfaces;

import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.matrix.interfaces.CommonMatrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.rotationConversion.RotationMatrixConversion;
import us.ihmc.euclid.rotationConversion.RotationVectorConversion;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.tools.AxisAngleTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollBasics;

/**
 * Read-only interface for an axis-angle object.
 * <p>
 * An axis-angle is used to represent a 3D orientation by a unitary axis of components (x, y, z) and
 * an angle of rotation usually expressed in radians.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface AxisAngleReadOnly extends Orientation3DReadOnly
{
   /**
    * {@inheritDoc}
    * <p>
    * An axis-angle is a zero orientation when its angle is equal to zero.
    * </p>
    */
   @Override
   default boolean isZeroOrientation(double epsilon)
   {
      return Math.abs(getAngle()) <= epsilon;
   }

   /**
    * Gets the reference to the axis part of this axis-angle.
    *
    * @return the reference to the axis vector.
    */
   UnitVector3DReadOnly getAxis();

   /**
    * Returns the angle of this axis-angle, usually expressed in radians.
    *
    * @return the angle.
    */
   double getAngle();

   /**
    * Returns the angle of this axis-angle, usually expressed in radians.
    *
    * @return the angle.
    */
   default float getAngle32()
   {
      return (float) getAngle();
   }

   /**
    * Returns the x-component of the unitary axis of this axis-angle.
    *
    * @return the x-component of the unitary axis.
    */
   default double getX()
   {
      return getAxis().getX();
   }

   /**
    * Returns the x-component of the unitary axis of this axis-angle.
    *
    * @return the x-component of the unitary axis.
    */
   default float getX32()
   {
      return (float) getX();
   }

   /**
    * Returns the y-component of the unitary axis of this axis-angle.
    *
    * @return the y-component of the unitary axis.
    */
   default double getY()
   {
      return getAxis().getY();
   }

   /**
    * Returns the y-component of the unitary axis of this axis-angle.
    *
    * @return the y-component of the unitary axis.
    */
   default float getY32()
   {
      return (float) getY();
   }

   /**
    * Returns the z-component of the unitary axis of this axis-angle.
    *
    * @return the z-component of the unitary axis.
    */
   default double getZ()
   {
      return getAxis().getZ();
   }

   /**
    * Returns the z-component of the unitary axis of this axis-angle.
    *
    * @return the z-component of the unitary axis.
    */
   default float getZ32()
   {
      return (float) getZ();
   }

   /**
    * Tests if this axis-angle contains a {@link Double#NaN}.
    *
    * @return {@code true} if this axis-angle contains a {@link Double#NaN}, {@code false} otherwise.
    */
   @Override
   default boolean containsNaN()
   {
      return EuclidCoreTools.containsNaN(getX(), getY(), getZ(), getAngle());
   }

   /**
    * Calculates and returns the norm of the axis of this axis-angle.
    * <p>
    * norm = &radic;(x<sup>2</sup> + y<sup>2</sup> + z<sup>2</sup>)
    * </p>
    *
    * @return the norm's value of the axis.
    */
   default double axisNorm()
   {
      return getAxis().length();
   }

   /**
    * Tests if the axis of this axis-angle is of unit-length.
    *
    * @param epsilon tolerance to use in this test.
    * @return {@code true} if the axis is unitary, {@code false} otherwise.
    * @deprecated Unneeded since {@link UnitVector3D} is used to implement the axis.
    * @since 0.13.0
    */
   @Deprecated
   default boolean isAxisUnitary(double epsilon)
   {
      return Math.abs(1.0 - axisNorm()) < epsilon;
   }

   /**
    * {@inheritDoc}
    * <p>
    * An axis-angle is an orientation 2D if either:
    * <ul>
    * <li>the absolute value of the angle is less that {@code epsilon}.
    * <li>the absolute value of the x and y components of the axis are both less than {@code epsilon}.
    * </ul>
    * </p>
    */
   @Override
   default boolean isOrientation2D(double epsilon)
   {
      return Math.abs(getAngle()) < epsilon || Math.abs(getX()) < epsilon && Math.abs(getY()) < epsilon;
   }

   /** {@inheritDoc} */
   @Override
   default double distance(Orientation3DReadOnly other, boolean limitToPi)
   {
      return AxisAngleTools.distance(this, other, limitToPi);
   }

   /**
    * Computes and returns the angular distance from origin.
    *
    * @return the the angular distance from origin.
    */
   @Override
   default double angle()
   {
      return getAngle();
   }

   /** {@inheritDoc} */
   @Override
   default double angle(boolean limitToPi)
   {
      return Math.abs(EuclidCoreTools.trimAngleMinusPiToPi(angle()));
   }

   /** {@inheritDoc} */
   @Override
   default void get(CommonMatrix3DBasics rotationMatrixToPack)
   {
      RotationMatrixConversion.convertAxisAngleToMatrix(this, rotationMatrixToPack);
   }

   /** {@inheritDoc} */
   @Override
   default void get(AxisAngleBasics axisAngleToPack)
   {
      axisAngleToPack.setAxisAngle(getX(), getY(), getZ(), getAngle());
   }

   /** {@inheritDoc} */
   @Override
   default void get(QuaternionBasics quaternionToPack)
   {
      quaternionToPack.setAxisAngle(getX(), getY(), getZ(), getAngle());
   }

   /** {@inheritDoc} */
   @Override
   default void get(YawPitchRollBasics yawPitchRollToPack)
   {
      yawPitchRollToPack.setAxisAngle(getX(), getY(), getZ(), getAngle());
   }

   /** {@inheritDoc} */
   @Override
   default void getRotationVector(Vector3DBasics rotationVectorToPack)
   {
      RotationVectorConversion.convertAxisAngleToRotationVector(this, rotationVectorToPack);
   }

   /** {@inheritDoc} */
   @Override
   default void getEuler(Tuple3DBasics eulerAnglesToPack)
   {
      YawPitchRollConversion.convertAxisAngleToYawPitchRoll(this, eulerAnglesToPack);
   }

   /** {@inheritDoc} */
   @Override
   default double getYaw()
   {
      return YawPitchRollConversion.computeYaw(this);
   }

   /** {@inheritDoc} */
   @Override
   default double getPitch()
   {
      return YawPitchRollConversion.computePitch(this);
   }

   /** {@inheritDoc} */
   @Override
   default double getRoll()
   {
      return YawPitchRollConversion.computeRoll(this);
   }

   /**
    * Packs the components of this axis-angle in an array starting from its first index. The components
    * are packed in the following order: x, y, z, and angle.
    *
    * @param axisAngleArrayToPack the array in which this axis-angle is stored. Modified.
    */
   default void get(double[] axisAngleArrayToPack)
   {
      get(0, axisAngleArrayToPack);
   }

   /**
    * Packs the components of this axis-angle in an array starting from {@code startIndex}. The
    * components are packed in the following order: x, y, z, and angle.
    *
    * @param startIndex           the index in the array where the first component is stored.
    * @param axisAngleArrayToPack the array in which this axis-angle is stored. Modified.
    */
   default void get(int startIndex, double[] axisAngleArrayToPack)
   {
      axisAngleArrayToPack[startIndex++] = getX();
      axisAngleArrayToPack[startIndex++] = getY();
      axisAngleArrayToPack[startIndex++] = getZ();
      axisAngleArrayToPack[startIndex] = getAngle();
   }

   /**
    * Packs the components of this axis-angle in an array starting from its first index. The components
    * are packed in the following order: x, y, z, and angle.
    *
    * @param axisAngleArrayToPack the array in which this axis-angle is stored. Modified.
    */
   default void get(float[] axisAngleArrayToPack)
   {
      get(0, axisAngleArrayToPack);
   }

   /**
    * Packs the components of this axis-angle in an array starting from {@code startIndex}. The
    * components are packed in the following order: x, y, z, and angle.
    *
    * @param startIndex           the index in the array where the first component is stored.
    * @param axisAngleArrayToPack the array in which this axis-angle is stored. Modified.
    */
   default void get(int startIndex, float[] axisAngleArrayToPack)
   {
      axisAngleArrayToPack[startIndex++] = getX32();
      axisAngleArrayToPack[startIndex++] = getY32();
      axisAngleArrayToPack[startIndex++] = getZ32();
      axisAngleArrayToPack[startIndex] = getAngle32();
   }

   /**
    * Selects a component of this axis-angle based on {@code index} and returns its value.
    * <p>
    * For {@code index} values of 0, 1, and 2, the corresponding components are x, y, and z,
    * respectively, while 3 corresponds to the angle.
    * </p>
    *
    * @param index the index of the component to get.
    * @return the value of the component.
    * @throws IndexOutOfBoundsException if {@code index} &notin; [0, 3].
    */
   default double getElement(int index)
   {
      switch (index)
      {
         case 0:
            return getX();
         case 1:
            return getY();
         case 2:
            return getZ();
         case 3:
            return getAngle();
         default:
            throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   /**
    * Selects a component of this axis-angle based on {@code index} and returns its value.
    * <p>
    * For {@code index} values of 0, 1, and 2, the corresponding components are x, y, and z,
    * respectively, while 3 corresponds to the angle.
    * </p>
    *
    * @param index the index of the component to get.
    * @return the value of the component.
    * @throws IndexOutOfBoundsException if {@code index} &notin; [0, 3].
    */
   default float getElement32(int index)
   {
      switch (index)
      {
         case 0:
            return getX32();
         case 1:
            return getY32();
         case 2:
            return getZ32();
         case 3:
            return getAngle32();
         default:
            throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      AxisAngleTools.transform(this, tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfOrientation2D)
   {
      AxisAngleTools.transform(this, tupleOriginal, tupleTransformed, checkIfOrientation2D);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      AxisAngleTools.transform(this, matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      AxisAngleTools.transform(this, vectorOriginal, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      AxisAngleTools.inverseTransform(this, tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfOrientation2D)
   {
      AxisAngleTools.inverseTransform(this, tupleOriginal, tupleTransformed, checkIfOrientation2D);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      AxisAngleTools.inverseTransform(this, vectorOriginal, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      AxisAngleTools.inverseTransform(this, matrixOriginal, matrixTransformed);
   }

   /**
    * Tests on a per component basis, if this axis-angle is exactly equal to {@code other}. A failing
    * test does not necessarily mean that the two axis-angles represent two different orientations.
    *
    * @param geometry the EuclidGeometry to compare against this. Not modified.
    * @return {@code true} if the two axis-angles are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   @Override
   default boolean equals(EuclidGeometry geometry)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof AxisAngleReadOnly))
         return false;
      AxisAngleReadOnly other = (AxisAngleReadOnly) geometry;
      return getX() == other.getX() && getY() == other.getY() && getZ() == other.getZ() && getAngle() == other.getAngle();
   }

   /**
    * Tests on a per component basis, if this axis-angle is equal to {@code other} to an
    * {@code epsilon}. A failing test does not necessarily mean that the two axis-angles represent two
    * different orientations.
    *
    * @param geometry the object to compare against this. Not modified.
    * @param epsilon  tolerance to use when comparing each component.
    * @return {@code true} if the two axis-angle are equal component-wise, {@code false} otherwise.
    */
   @Override
   default boolean epsilonEquals(EuclidGeometry geometry, double epsilon)
   {
      if (!(geometry instanceof AxisAngleReadOnly))
         return false;

      AxisAngleReadOnly other = (AxisAngleReadOnly) geometry;
      if (!EuclidCoreTools.epsilonEquals(getX(), other.getX(), epsilon) || !EuclidCoreTools.epsilonEquals(getY(), other.getY(), epsilon)
            || !EuclidCoreTools.epsilonEquals(getZ(), other.getZ(), epsilon) || !EuclidCoreTools.epsilonEquals(getAngle(), other.getAngle(), epsilon))
         return false;

      return true;
   }

   /**
    * Provides a {@code String} representation of this axis-angle as follows: (x, y, z, angle).
    *
    * @param format the format to use for each number.
    * @return the {@code String} representing this axis-angle.
    */
   @Override
   default String toString(String format)
   {
      return EuclidCoreIOTools.getAxisAngleString(format, this);
   }

}