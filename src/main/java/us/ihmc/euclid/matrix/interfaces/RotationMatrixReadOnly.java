package us.ihmc.euclid.matrix.interfaces;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.matrix.RotationMatrix;
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
 * Read interface used for 3-by-3 rotation matrices.
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
    * Computes and returns the distance between this rotation matrix and the {@code other}.
    *
    * @param other the other rotation matrix to compute the distance. Not modified.
    * @return the angle representing the distance between the two rotation matrices. It is contained in
    *         [0, <i>pi</i>].
    */
   default double distance(RotationMatrixReadOnly other)
   {
      return RotationMatrixTools.distance(this, other);
   }

   /** {@inheritDoc} */
   @Override
   default void get(RotationMatrix rotationMatrixToPack)
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
   default void getEuler(Tuple3DBasics eulerAnglesToPack)
   {
      YawPitchRollConversion.convertMatrixToYawPitchRoll(this, eulerAnglesToPack);
   }

   /** {@inheritDoc} */
   @Deprecated
   @Override
   default void getYawPitchRoll(double[] yawPitchRollToPack)
   {
      YawPitchRollConversion.convertMatrixToYawPitchRoll(this, yawPitchRollToPack);
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
      Matrix3DReadOnly.super.addTransform(tupleToTransform);
   }

   /** {@inheritDoc} */
   @Override
   default void addTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      Matrix3DReadOnly.super.addTransform(tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Tuple3DBasics tupleToTransform)
   {
      Matrix3DReadOnly.super.transform(tupleToTransform);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      Matrix3DReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Tuple2DBasics tupleToTransform)
   {
      Matrix3DReadOnly.super.transform(tupleToTransform);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      Matrix3DReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Tuple2DBasics tupleToTransform, boolean checkIfOrientation2D)
   {
      Matrix3DReadOnly.super.transform(tupleToTransform, checkIfOrientation2D);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfOrientation2D)
   {
      Matrix3DReadOnly.super.transform(tupleOriginal, tupleTransformed, checkIfOrientation2D);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Vector4DBasics vectorToTransform)
   {
      Matrix3DReadOnly.super.transform(vectorToTransform);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      Matrix3DReadOnly.super.transform(vectorOriginal, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Matrix3DBasics matrixToTransform)
   {
      Matrix3DReadOnly.super.transform(matrixToTransform);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      Matrix3DTools.multiply(this, matrixOriginal, matrixTransformed);
      Matrix3DTools.multiplyTransposeRight(matrixTransformed, this, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Tuple3DBasics tupleToTransform)
   {
      Matrix3DReadOnly.super.inverseTransform(tupleToTransform);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      double x = getM00() * tupleOriginal.getX() + getM10() * tupleOriginal.getY() + getM20() * tupleOriginal.getZ();
      double y = getM01() * tupleOriginal.getX() + getM11() * tupleOriginal.getY() + getM21() * tupleOriginal.getZ();
      double z = getM02() * tupleOriginal.getX() + getM12() * tupleOriginal.getY() + getM22() * tupleOriginal.getZ();
      tupleTransformed.set(x, y, z);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Tuple2DBasics tupleToTransform)
   {
      Matrix3DReadOnly.super.inverseTransform(tupleToTransform);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      Matrix3DReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Tuple2DBasics tupleToTransform, boolean checkIfOrientation2D)
   {
      Matrix3DReadOnly.super.inverseTransform(tupleToTransform, checkIfOrientation2D);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfOrientation2D)
   {
      if (checkIfOrientation2D)
         checkIfOrientation2D();

      double x = getM00() * tupleOriginal.getX() + getM10() * tupleOriginal.getY();
      double y = getM01() * tupleOriginal.getX() + getM11() * tupleOriginal.getY();
      tupleTransformed.set(x, y);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Vector4DBasics vectorToTransform)
   {
      Matrix3DReadOnly.super.inverseTransform(vectorToTransform);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      double x = getM00() * vectorOriginal.getX() + getM10() * vectorOriginal.getY() + getM20() * vectorOriginal.getZ();
      double y = getM01() * vectorOriginal.getX() + getM11() * vectorOriginal.getY() + getM21() * vectorOriginal.getZ();
      double z = getM02() * vectorOriginal.getX() + getM12() * vectorOriginal.getY() + getM22() * vectorOriginal.getZ();
      vectorTransformed.set(x, y, z, vectorOriginal.getS());
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Matrix3DBasics matrixToTransform)
   {
      Matrix3DReadOnly.super.inverseTransform(matrixToTransform);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      Matrix3DTools.multiplyTransposeLeft(this, matrixOriginal, matrixTransformed);
      Matrix3DTools.multiply(matrixTransformed, this, matrixTransformed);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same orientation to an {@code epsilon}.
    * <p>
    * Two rotation matrices are considered geometrically equal if the magnitude of their difference is
    * less than or equal to {@code epsilon}.
    * </p>
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * {@code this.epsilonEquals(other, epsilon)} and vice versa.
    * </p>
    *
    * @param other the other rotation matrix to compare against this. Not modified.
    * @param epsilon the maximum angle between the two rotation matrices to be considered equal.
    * @return {@code true} if the two rotation matrices represent the same geometry, {@code false}
    *         otherwise.
    */
   default boolean geometricallyEquals(RotationMatrixReadOnly other, double epsilon)
   {
      return distance(other) <= epsilon;
   }
}
