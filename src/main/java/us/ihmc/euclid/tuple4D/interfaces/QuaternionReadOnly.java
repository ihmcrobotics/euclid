package us.ihmc.euclid.tuple4D.interfaces;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.matrix.interfaces.CommonMatrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.rotationConversion.RotationMatrixConversion;
import us.ihmc.euclid.rotationConversion.RotationVectorConversion;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollBasics;

/**
 * Read-only interface for unit-quaternion used to represent 3D orientations.
 * <p>
 * When describing a 4D tuple, its 4 components are often gathered in two groups: the scalar part
 * {@code s} and the vector part ({@code x}, {@code y}, {@code z}).
 * </p>
 * <p>
 * Note on the difference between applying a 3D transform on a quaternion and a 4D vector:
 * <ul>
 * <li>When transformed by a homogeneous transformation matrix, a quaternion is only pre-multiplied
 * by the rotation part of the transform, resulting in concatenating the orientations of the
 * transform and the quaternion.
 * <li>When transformed by a homogeneous transformation matrix, a 4D vector scalar part {@code s}
 * remains unchanged. The vector part ({@code x}, {@code y}, {@code z}) is scaled and rotated, and
 * translated by {@code s} times the translation part of the transform. Note that for {@code s = 0},
 * a 4D vector behaves as a 3D vector, and for {@code s = 1} it behaves as a 3D point.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface QuaternionReadOnly extends Tuple4DReadOnly, Orientation3DReadOnly
{
   /** Default tolerance used to verify that this quaternion is a unit-quaternion. */
   public static final double EPS_UNITARY = 1.0e-7;

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return Tuple4DReadOnly.super.containsNaN();
   }

   /**
    * {@inheritDoc}
    * <p>
    * A quaternion is a zero orientation when it is equal to the neutral quaternion, i.e.
    * {@code (x=0, y=0, z=0, s=1)}.
    * </p>
    */
   @Override
   default boolean isZeroOrientation(double epsilon)
   {
      return QuaternionTools.isNeutralQuaternion(this, epsilon);
   }

   /**
    * Tests if this quaternion has a norm equal to 1+/-{@code epsilon}.
    *
    * @param epsilon the tolerance to use.
    * @return {@code true} if this quaternion is a proper unit-quaternion, {@code false} otherwise.
    */
   default boolean isUnitary(double epsilon)
   {
      return Math.abs(norm() - 1.0) < epsilon;
   }

   /**
    * {@inheritDoc}
    * <p>
    * A quaternion is an orientation 2D if:
    * <ul>
    * <li>the absolute value of the x component is less than {@code epsilon}.
    * <li>the absolute value of the y component is less than {@code epsilon}.
    * </ul>
    * </p>
    */
   @Override
   default boolean isOrientation2D(double epsilon)
   {
      return Math.abs(getX()) < epsilon && Math.abs(getY()) < epsilon;
   }

   /**
    * Asserts that this quaternion has a norm equal to 1+/-{@value #EPS_UNITARY}.
    *
    * @param epsilon the tolerance to use.
    * @throws RuntimeException if this quaternion is not a proper unit-quaternion.
    */
   default void checkIfUnitary()
   {
      checkIfUnitary(EPS_UNITARY);
   }

   /**
    * Asserts that this quaternion has a norm equal to 1+/-{@code epsilon}.
    *
    * @param epsilon the tolerance to use.
    * @throws RuntimeException if this quaternion is not a proper unit-quaternion.
    */
   default void checkIfUnitary(double epsilon)
   {
      if (!isUnitary(epsilon))
         throw new RuntimeException("This quaternion is not a unit-quaternion.");
   }

   /**
    * Efficiently compute the norm of this quaternion.
    */
   @Override
   default double norm()
   {
      return EuclidCoreTools.fastSquareRoot(normSquared());
   }

   /** {@inheritDoc} */
   @Override
   default double distance(Orientation3DReadOnly other, boolean limitToPi)
   {
      return QuaternionTools.distance(this, other, limitToPi);
   }

   /** {@inheritDoc} */
   @Override
   default double angle(boolean limitToPi)
   {
      return QuaternionTools.angle(this, limitToPi);
   }

   /**
    * Calculates and returns the angle of the rotation this quaternion represents.
    * 
    * @deprecated Use {@link #angle()} instead.
    * @return the angle &in; [-2<i>pi</i>;2<i>pi</i>].
    */
   @Deprecated
   default double getAngle()
   {
      return QuaternionTools.angle(this);
   }

   /** {@inheritDoc} */
   @Override
   default void get(CommonMatrix3DBasics rotationMatrixToPack)
   {
      RotationMatrixConversion.convertQuaternionToMatrix(this, rotationMatrixToPack);
   }

   /** {@inheritDoc} */
   @Override
   default void get(AxisAngleBasics axisAngleToPack)
   {
      axisAngleToPack.setQuaternion(getX(), getY(), getZ(), getS());
   }

   /** {@inheritDoc} */
   @Override
   default void get(QuaternionBasics quaternionToPack)
   {
      quaternionToPack.setQuaternion(getX(), getY(), getZ(), getS());
   }

   /** {@inheritDoc} */
   @Override
   default void get(YawPitchRollBasics yawPitchRollToPack)
   {
      yawPitchRollToPack.setQuaternion(getX(), getY(), getZ(), getS());
   }

   /** {@inheritDoc} */
   @Override
   default void getRotationVector(Vector3DBasics rotationVectorToPack)
   {
      RotationVectorConversion.convertQuaternionToRotationVector(this, rotationVectorToPack);
   }

   /** {@inheritDoc} */
   @Override
   default void getEuler(Tuple3DBasics eulerAnglesToPack)
   {
      YawPitchRollConversion.convertQuaternionToYawPitchRoll(this, eulerAnglesToPack);
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

   /** {@inheritDoc} */
   @Override
   default void transform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      QuaternionTools.transform(this, tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void addTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      QuaternionTools.addTransform(this, tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfOrientation2D)
   {
      QuaternionTools.transform(this, tupleOriginal, tupleTransformed, checkIfOrientation2D);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      QuaternionTools.transform(this, matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      QuaternionTools.transform(this, vectorOriginal, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      QuaternionTools.inverseTransform(this, tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfOrientation2D)
   {
      QuaternionTools.inverseTransform(this, tupleOriginal, tupleTransformed, checkIfOrientation2D);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      QuaternionTools.inverseTransform(this, vectorOriginal, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      QuaternionTools.inverseTransform(this, matrixOriginal, matrixTransformed);
   }
   
   /**
    * Tests on a per component basis if this quaternion is equal to the given {@code other} to an
    * {@code epsilon}.
    *
    * @param other   the other quaternion to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two tuples are equal, {@code false} otherwise.
    */
   default boolean epsilonEquals(Object other, double epsilon)
   {
      if( !(other instanceof QuaternionReadOnly))
      {
         return false;
      }
      
      return TupleTools.epsilonEquals(this,(QuaternionReadOnly) other, epsilon);
   }
   
   /**
    * Provides a {@code String} representation of this quaternion as follows: (x, y, z, s).
    *
    * @param format the format to use for each number.
    * @return the {@code String} representing this quaternion.
    */
   default String toString(String format)
   {
      return EuclidCoreIOTools.getTuple4DString(format, this);
   }
   
}