package us.ihmc.euclid.matrix.interfaces;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.rotationConversion.RotationVectorConversion;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.tools.Matrix3DTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollBasics;

/**
 * Read-only interface used for 3-by-3 rotation matrices.
 * <p>
 * A rotation matrix is used to represent a 3D orientation through its 9 coefficients. A rotation
 * matrix has to comply to several constraints:
 * <ul>
 * <li>each column of the matrix represents a unitary vector,
 * <li>each row of the matrix represents a unitary vector,
 * <li>every pair of columns of the matrix represents two orthogonal vectors,
 * <li>every pair of rows of the matrix represents two orthogonal vectors,
 * <li>the matrix determinant is equal to {@code 1}.
 * </ul>
 * A rotation matrix has the nice property <i>R<sup>T</sup> = R<sup>-1</sup></i>.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface RotationMatrixReadOnly extends Matrix3DReadOnly, Orientation3DReadOnly
{
   /**
    * Returns the state of this matrix dirty flag.
    * <p>
    * This matrix is marked as dirty to indicate its values have been updated and not yet propagated to
    * this matrix features.
    * </p>
    *
    * @return the current value of the dirty flag.
    */
   boolean isDirty();

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return Matrix3DReadOnly.super.containsNaN();
   }

   /**
    * {@inheritDoc}
    * <p>
    * A rotation matrix is a zero orientation when it is equal to the identity matrix.
    * </p>
    */
   @Override
   default boolean isZeroOrientation()
   {
      return isIdentity();
   }

   /**
    * {@inheritDoc}
    * <p>
    * A rotation matrix is a zero orientation when it is equal to the identity matrix.
    * </p>
    */
   @Override
   default boolean isZeroOrientation(double epsilon)
   {
      return isIdentity(epsilon);
   }

   /**
    * {@inheritDoc}
    * <p>
    * This rotation matrix is considered to be an orientation 2D if:
    * <ul>
    * <li>the last diagonal coefficient m22 is equal to 1.0 +/- {@code epsilon},
    * <li>the coefficients {@code m20}, {@code m02}, {@code m21}, and {@code m12} are equal to 0.0 +/-
    * {@code epsilon}.
    * </ul>
    * </p>
    */
   @Override
   default boolean isOrientation2D(double epsilon)
   {
      return isIdentity() || isMatrix2D(epsilon);
   }

   /**
    * {@inheritDoc}
    *
    * @param other the other rotation matrix to compute the distance. Not modified.
    * @return the angle representing the distance between the two rotation matrices. It is contained in
    *         [0, <i>pi</i>].
    */
   @Override
   default double distance(Orientation3DReadOnly other)
   {
      return RotationMatrixTools.distance(this, other);
   }

   /**
    * {@inheritDoc}
    *
    * @param other     the other rotation matrix to compute the distance. Not modified.
    * @param limitToPi Does nothing here. Result is always capped to [0,<i>pi</i>].
    * @return the angle representing the distance between the two rotation matrices. It is contained in
    *         [0, <i>pi</i>].
    */
   @Override
   default double distance(Orientation3DReadOnly other, boolean limitToPi)
   {
      return distance(other);
   }

   /**
    * {@inheritDoc}
    * 
    * @param limitToPi Does nothing here. Result is always capped to [0,<i>pi</i>].
    * @return the angular distance from origin. It is contained in [0, <i>pi</i>].
    */
   @Override
   default double angle(boolean limitToPi)
   {
      return RotationMatrixTools.angle(this);
   }

   /** {@inheritDoc} */
   @Override
   default void get(CommonMatrix3DBasics rotationMatrixToPack)
   {
      rotationMatrixToPack.set(this);
   }

   /** {@inheritDoc} */
   @Override
   default void get(AxisAngleBasics axisAngleToPack)
   {
      axisAngleToPack.setRotationMatrix(getM00(), getM01(), getM02(), getM10(), getM11(), getM12(), getM20(), getM21(), getM22());
   }

   /** {@inheritDoc} */
   @Override
   default void get(QuaternionBasics quaternionToPack)
   {
      quaternionToPack.setRotationMatrix(getM00(), getM01(), getM02(), getM10(), getM11(), getM12(), getM20(), getM21(), getM22());
   }

   /** {@inheritDoc} */
   @Override
   default void get(YawPitchRollBasics yawPitchRollToPack)
   {
      yawPitchRollToPack.setRotationMatrix(getM00(), getM01(), getM02(), getM10(), getM11(), getM12(), getM20(), getM21(), getM22());
   }

   /** {@inheritDoc} */
   @Override
   default void getRotationVector(Vector3DBasics rotationVectorToPack)
   {
      RotationVectorConversion.convertMatrixToRotationVector(this, rotationVectorToPack);
   }

   /** {@inheritDoc} */
   @Override
   default void getEuler(Tuple3DBasics eulerAnglesToPack)
   {
      YawPitchRollConversion.convertMatrixToYawPitchRoll(this, eulerAnglesToPack);
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
   default void addTransform(Tuple3DBasics tupleToTransform)
   {
      addTransform(tupleToTransform, tupleToTransform);
   }

   /** {@inheritDoc} */
   @Override
   default void addTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      if (isIdentity())
         tupleTransformed.add(tupleOriginal);
      else
         Matrix3DReadOnly.super.addTransform(tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple by this matrix and subtracts the result to the tuple.
    * <p>
    * tupleToTransform = tupleToTransform - this * tupleToTransform
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    */
   @Override
   default void subTransform(Tuple3DBasics tupleToTransform)
   {
      subTransform(tupleToTransform, tupleToTransform);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this matrix and subtracts the result to
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = tupleTransformed - this * tupleOriginal
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to add the result to. Modified.
    */
   @Override
   default void subTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      if (isIdentity())
         tupleTransformed.sub(tupleOriginal);
      else
         Matrix3DTools.subTransform(this, tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Tuple3DBasics tupleToTransform)
   {
      transform(tupleToTransform, tupleToTransform);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      if (isIdentity() && tupleOriginal != tupleTransformed)
         tupleTransformed.set(tupleOriginal);
      else
         Matrix3DReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Tuple2DBasics tupleToTransform)
   {
      transform(tupleToTransform, true);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      transform(tupleOriginal, tupleTransformed, true);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Tuple2DBasics tupleToTransform, boolean checkIfOrientation2D)
   {
      transform(tupleToTransform, tupleToTransform, checkIfOrientation2D);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfOrientation2D)
   {
      if (isIdentity() && tupleOriginal != tupleTransformed)
         tupleTransformed.set(tupleOriginal);
      else
         Matrix3DReadOnly.super.transform(tupleOriginal, tupleTransformed, checkIfOrientation2D);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Vector4DBasics vectorToTransform)
   {
      transform(vectorToTransform, vectorToTransform);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      if (isIdentity() && vectorOriginal != vectorTransformed)
         vectorTransformed.set(vectorOriginal);
      else
         Matrix3DReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Matrix3DBasics matrixToTransform)
   {
      transform(matrixToTransform, matrixToTransform);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      if (isIdentity() && matrixOriginal != matrixTransformed)
      {
         matrixTransformed.set(matrixOriginal);
      }
      else
      {
         Matrix3DTools.multiply(this, matrixOriginal, matrixTransformed);
         Matrix3DTools.multiplyTransposeRight(matrixTransformed, this, matrixTransformed);
      }
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Tuple3DBasics tupleToTransform)
   {
      inverseTransform(tupleToTransform, tupleToTransform);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      if (isIdentity() && tupleOriginal != tupleTransformed)
      {
         tupleTransformed.set(tupleOriginal);
      }
      else
      {
         double x = getM00() * tupleOriginal.getX() + getM10() * tupleOriginal.getY() + getM20() * tupleOriginal.getZ();
         double y = getM01() * tupleOriginal.getX() + getM11() * tupleOriginal.getY() + getM21() * tupleOriginal.getZ();
         double z = getM02() * tupleOriginal.getX() + getM12() * tupleOriginal.getY() + getM22() * tupleOriginal.getZ();
         tupleTransformed.set(x, y, z);
      }
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Tuple2DBasics tupleToTransform)
   {
      inverseTransform(tupleToTransform, true);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      inverseTransform(tupleOriginal, tupleTransformed, true);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Tuple2DBasics tupleToTransform, boolean checkIfOrientation2D)
   {
      inverseTransform(tupleToTransform, tupleToTransform, checkIfOrientation2D);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfOrientation2D)
   {
      if (isIdentity() && tupleOriginal != tupleTransformed)
      {
         tupleTransformed.set(tupleOriginal);
      }
      else
      {
         if (checkIfOrientation2D)
            checkIfOrientation2D();

         double x = getM00() * tupleOriginal.getX() + getM10() * tupleOriginal.getY();
         double y = getM01() * tupleOriginal.getX() + getM11() * tupleOriginal.getY();
         tupleTransformed.set(x, y);
      }
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Vector4DBasics vectorToTransform)
   {
      inverseTransform(vectorToTransform, vectorToTransform);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      if (isIdentity() && vectorOriginal != vectorTransformed)
      {
         vectorTransformed.set(vectorOriginal);
      }
      else
      {
         double x = getM00() * vectorOriginal.getX() + getM10() * vectorOriginal.getY() + getM20() * vectorOriginal.getZ();
         double y = getM01() * vectorOriginal.getX() + getM11() * vectorOriginal.getY() + getM21() * vectorOriginal.getZ();
         double z = getM02() * vectorOriginal.getX() + getM12() * vectorOriginal.getY() + getM22() * vectorOriginal.getZ();
         vectorTransformed.set(x, y, z, vectorOriginal.getS());
      }
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Matrix3DBasics matrixToTransform)
   {
      inverseTransform(matrixToTransform, matrixToTransform);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      if (isIdentity() && matrixOriginal != matrixTransformed)
      {
         matrixTransformed.set(matrixOriginal);
      }
      else
      {
         Matrix3DTools.multiplyTransposeLeft(this, matrixOriginal, matrixTransformed);
         Matrix3DTools.multiply(matrixTransformed, this, matrixTransformed);
      }
   }

   /**
    * Provides a {@code String} representation of this matrix as follows:
    *
    * <pre>
    * /-0.576, -0.784,  0.949 \
    * | 0.649, -0.542, -0.941 |
    * \-0.486, -0.502, -0.619 /
    * </pre>
    *
    * @param format the format to be used.
    * @return the {@code String} representing this matrix.
    */
   @Override
   default String toString(String format)
   {
      return Matrix3DReadOnly.super.toString(format);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same orientation to an {@code epsilon}.
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * that the 2 orientations are of the same type nor that they are equal on a per-component bases.
    * </p>
    *
    * @param geometry  the object to compare against this.
    * @param epsilon the maximum angle for the two orientations to be considered equal.
    * @return {@code true} if the two orientations represent the same geometry, {@code false}
    *         otherwise.
    */
   @Override
   default boolean geometricallyEquals(EuclidGeometry geometry, double epsilon)
   {
      return Orientation3DReadOnly.super.geometricallyEquals(geometry, epsilon);
   }
}
