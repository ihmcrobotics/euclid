package us.ihmc.euclid.transform;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.GeometricallyComparable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.CommonMatrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tools.Matrix3DTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;

/**
 * A {@code RigidBodyTransform} represents a 4-by-4 transformation matrix that can rotate and
 * translate.
 * <p>
 * For efficiency and readability, the transform is never stored in a 4-by-4 matrix.
 * </p>
 * <p>
 * The {@code RigidBodyTransform} is composed of {@link RotationMatrix} to rotate, and a
 * {@link Vector3D} to translate.
 * </p>
 * <p>
 * A few special cases to keep in mind:
 * <ul>
 * <li>when transforming a {@link QuaternionBasics}, the rotation part of this transform is prepend
 * to the quaternion, such that the output remains a proper unit-quaternion that still only
 * describes a rotation.
 * <li>when transforming a {@link RotationMatrix}, the rotation part of this transform is prepend to
 * the rotation matrix, such that the output remains a proper rotation matrix.
 * <li>when applying this transform on a {@link Point3DBasics} or {@link Point2DBasics}, this object
 * is, in order, rotated and then translated.
 * <li>when applying this transform on a {@link Vector3DBasics} or {@link Vector2DBasics}, this
 * object is only rotated. It is NOT translated.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class RigidBodyTransform
      implements RigidBodyTransformBasics, EpsilonComparable<RigidBodyTransform>, GeometricallyComparable<RigidBodyTransform>, Settable<RigidBodyTransform>
{
   /** The rotation part of this transform. */
   private final RotationMatrix rotationMatrix = new RotationMatrix();
   /** The translation part of this transform. */
   private final Vector3D translationVector = new Vector3D();

   /**
    * Creates a new rigid-body transform set to identity.
    * <p>
    * When set to identity, this transform has no effect when transforming a geometry object.
    * </p>
    */
   public RigidBodyTransform()
   {
   }

   /**
    * Creates a new rigid-body transform and sets it to {@code other}.
    *
    * @param other the other rigid-body transform to copy. Not modified.
    */
   public RigidBodyTransform(RigidBodyTransformReadOnly other)
   {
      set(other);
   }

   /**
    * Creates a new rigid-body transform and sets its raw components from the given {@code matrix}.
    * <p>
    * The rotation part R is set as follows:
    *
    * <pre>
    *     / matrix.get(0, 0) matrix.get(0, 1) matrix.get(0, 2) \
    * R = | matrix.get(1, 0) matrix.get(1, 1) matrix.get(1, 2) |
    *     \ matrix.get(2, 0) matrix.get(2, 1) matrix.get(2, 2) /
    * </pre>
    *
    * The translation part T is set as follows:
    *
    * <pre>
    *     / matrix.get(0, 3) \
    * T = | matrix.get(1, 3) |
    *     \ matrix.get(2, 3) /
    * </pre>
    * </p>
    *
    * @param matrix the matrix to get this transform's components from. Not modified.
    * @throws NotARotationMatrixException if the resulting matrix for the rotation part of this
    *                                     transform is not a rotation matrix.
    */
   public RigidBodyTransform(DenseMatrix64F matrix)
   {
      set(matrix);
   }

   /**
    * Sets the raw components of this rigid-body transform from the given {@code transformArray}.
    * <p>
    * The rotation part R is set as follows:
    *
    * <pre>
    *     / transformArray[0] transformArray[1] transformArray[ 2] \
    * R = | transformArray[4] transformArray[5] transformArray[ 6] |
    *     \ transformArray[8] transformArray[9] transformArray[10] /
    * </pre>
    *
    * The translation part T is set as follows:
    *
    * <pre>
    *     / transformArray[ 3] \
    * T = | transformArray[ 7] |
    *     \ transformArray[11] /
    * </pre>
    * </p>
    *
    * @param transformArray the 1D row-major array to get this transform's components from. Not
    *                       modified.
    * @throws NotARotationMatrixException if the resulting matrix for the rotation part of this
    *                                     transform is not a rotation matrix.
    */
   public RigidBodyTransform(double[] transformArray)
   {
      set(transformArray);
   }

   /**
    * Creates a new rigid-body transform and sets it to the given {@code orientation} and
    * {@code translation}.
    *
    * @param orientation the orientation used to set this transform's rotation part. Not modified.
    * @param translation the tuple used to set this transform's translation part. Not modified.
    */
   public RigidBodyTransform(Orientation3DReadOnly orientation, Tuple3DReadOnly translation)
   {
      set(orientation, translation);
   }

   /**
    * Creates a new rigid-body transform and sets it from the given 12 coefficients.
    *
    * @param m00 the 1st row 1st column component of the rotation part of this transform.
    * @param m01 the 1st row 2nd column component of the rotation part of this transform.
    * @param m02 the 1st row 3rd column component of the rotation part of this transform.
    * @param m03 the x-component of the translation part of this transform.
    * @param m10 the 2nd row 1st column component of the rotation part of this transform.
    * @param m11 the 2nd row 2nd column component of the rotation part of this transform.
    * @param m12 the 2nd row 3rd column component of the rotation part of this transform.
    * @param m13 the y-component of the translation part of this transform.
    * @param m20 the 3rd row 1st column component of the rotation part of this transform.
    * @param m21 the 3rd row 2nd column component of the rotation part of this transform.
    * @param m22 the 3rd row 3rd column component of the rotation part of this transform.
    * @param m23 the z-component of the translation part of this transform.
    * @throws NotARotationMatrixException if the components for the rotation part do not represent a
    *                                     rotation matrix.
    */
   public RigidBodyTransform(double m00, double m01, double m02, double m03, double m10, double m11, double m12, double m13, double m20, double m21, double m22,
                             double m23)
   {
      set(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23);
   }

   /**
    * Resets this rigid-body transform to identity.
    * <p>
    * When set to identity, this transform has no effect when transforming a geometry object.
    * </p>
    */
   public void setIdentity()
   {
      RigidBodyTransformBasics.super.setToZero();
   }

   /**
    * Computes the determinant of the rotation part of this transform.
    *
    * @return the determinant's value.
    * @deprecated Use {@code this.getRotation().determinant()} instead.
    */
   @Deprecated
   public double determinantRotationPart()
   {
      return getRotation().determinant();
   }

   /**
    * Sets this rigid-body transform from the given 12 coefficients.
    *
    * @param m00 the 1st row 1st column component of the rotation part of this transform.
    * @param m01 the 1st row 2nd column component of the rotation part of this transform.
    * @param m02 the 1st row 3rd column component of the rotation part of this transform.
    * @param m03 the x-component of the translation part of this transform.
    * @param m10 the 2nd row 1st column component of the rotation part of this transform.
    * @param m11 the 2nd row 2nd column component of the rotation part of this transform.
    * @param m12 the 2nd row 3rd column component of the rotation part of this transform.
    * @param m13 the y-component of the translation part of this transform.
    * @param m20 the 3rd row 1st column component of the rotation part of this transform.
    * @param m21 the 3rd row 2nd column component of the rotation part of this transform.
    * @param m22 the 3rd row 3rd column component of the rotation part of this transform.
    * @param m23 the z-component of the translation part of this transform.
    * @throws NotARotationMatrixException if the components for the rotation part do not represent a
    *                                     rotation matrix.
    */
   public void set(double m00, double m01, double m02, double m03, double m10, double m11, double m12, double m13, double m20, double m21, double m22,
                   double m23)
   {
      getRotation().set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      getTranslation().set(m03, m13, m23);
   }

   /**
    * Sets this rigid-body transform from the given 12 coefficients.
    * <p>
    * Prefer using the method
    * {@link #set(double, double, double, double, double, double, double, double, double, double, double, double)}
    * as it asserts that the coefficients for the rotation part represent a rotation matrix.
    * </p>
    *
    * @param m00 the 1st row 1st column component of the rotation part of this transform.
    * @param m01 the 1st row 2nd column component of the rotation part of this transform.
    * @param m02 the 1st row 3rd column component of the rotation part of this transform.
    * @param m03 the x-component of the translation part of this transform.
    * @param m10 the 2nd row 1st column component of the rotation part of this transform.
    * @param m11 the 2nd row 2nd column component of the rotation part of this transform.
    * @param m12 the 2nd row 3rd column component of the rotation part of this transform.
    * @param m13 the y-component of the translation part of this transform.
    * @param m20 the 3rd row 1st column component of the rotation part of this transform.
    * @param m21 the 3rd row 2nd column component of the rotation part of this transform.
    * @param m22 the 3rd row 3rd column component of the rotation part of this transform.
    * @param m23 the z-component of the translation part of this transform.
    */
   public void setUnsafe(double m00, double m01, double m02, double m03, double m10, double m11, double m12, double m13, double m20, double m21, double m22,
                         double m23)
   {
      getRotation().setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      getTranslation().set(m03, m13, m23);
   }

   /**
    * Sets this rigid-body transform to {@code other}.
    *
    * @param other the other rigid-body transform to copy the values from. Not modified.
    */
   @Override
   public void set(RigidBodyTransform other)
   {
      set((RigidBodyTransformReadOnly) other);
   }

   /**
    * Sets the raw components of this rigid-body transform from the given {@code matrix}.
    * <p>
    * The rotation-scale part R is set as follows:
    *
    * <pre>
    *     / matrix.get(0, 0) matrix.get(0, 1) matrix.get(0, 2) \
    * R = | matrix.get(1, 0) matrix.get(1, 1) matrix.get(1, 2) |
    *     \ matrix.get(2, 0) matrix.get(2, 1) matrix.get(2, 2) /
    * </pre>
    *
    * The translation part T is set as follows:
    *
    * <pre>
    *     / matrix.get(0, 3) \
    * T = | matrix.get(1, 3) |
    *     \ matrix.get(2, 3) /
    * </pre>
    * </p>
    *
    * @param matrix the matrix to get this transform's components from. Not modified.
    * @throws NotARotationMatrixException if the resulting matrix for the rotation part of this
    *                                     transform is not a rotation matrix.
    */
   public void set(DenseMatrix64F matrix)
   {
      getRotation().set(matrix);
      getTranslation().set(0, 3, matrix);
   }

   /**
    * Sets the raw components of this rigid-body transform from the given {@code matrix}.
    * <p>
    * The rotation part R is set as follows:
    *
    * <pre>
    *     / matrix.get(startRow + 0, startColumn + 0) matrix.get(startRow + 0, startColumn + 1) matrix.get(startRow + 0, startColumn + 2) \
    * R = | matrix.get(startRow + 1, startColumn + 0) matrix.get(startRow + 1, startColumn + 1) matrix.get(startRow + 1, startColumn + 2) |
    *     \ matrix.get(startRow + 2, startColumn + 0) matrix.get(startRow + 2, startColumn + 1) matrix.get(startRow + 2, startColumn + 2) /
    * </pre>
    *
    * The translation part T is set as follows:
    *
    * <pre>
    *     / matrix.get(startRow + 0, startColumn + 3) \
    * T = | matrix.get(startRow + 1, startColumn + 3) |
    *     \ matrix.get(startRow + 2, startColumn + 3) /
    * </pre>
    * </p>
    *
    * @param matrix      the matrix to get this transform's components from. Not modified.
    * @param startRow    the row index of the first component to read.
    * @param startColumn the column index of the first component to read.
    * @throws NotARotationMatrixException if the resulting matrix for the rotation part of this
    *                                     transform is not a rotation matrix.
    */
   public void set(DenseMatrix64F matrix, int startRow, int startColumn)
   {
      getRotation().set(startRow, startColumn, matrix);
      getTranslation().set(startRow, startColumn + 3, matrix);
   }

   /**
    * Sets the raw components of this rigid-body transform from the given {@code transformArray}.
    * <p>
    * The rotation-scale part R is set as follows:
    *
    * <pre>
    *     / transformArray[0] transformArray[1] transformArray[ 2] \
    * R = | transformArray[4] transformArray[5] transformArray[ 6] |
    *     \ transformArray[8] transformArray[9] transformArray[10] /
    * </pre>
    *
    * The translation part T is set as follows:
    *
    * <pre>
    *     / transformArray[ 3] \
    * T = | transformArray[ 7] |
    *     \ transformArray[11] /
    * </pre>
    * </p>
    *
    * @param transformArray the 1D row-major array to get this transform's components from. Not
    *                       modified.
    * @throws NotARotationMatrixException if the resulting matrix for the rotation part of this
    *                                     transform is not a rotation matrix.
    */
   public void set(double[] transformArray)
   {
      double m00 = transformArray[0];
      double m01 = transformArray[1];
      double m02 = transformArray[2];
      double m03 = transformArray[3];
      double m10 = transformArray[4];
      double m11 = transformArray[5];
      double m12 = transformArray[6];
      double m13 = transformArray[7];
      double m20 = transformArray[8];
      double m21 = transformArray[9];
      double m22 = transformArray[10];
      double m23 = transformArray[11];

      getRotation().set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      getTranslation().set(m03, m13, m23);
   }

   /**
    * Sets the raw components of this rigid-body transform from the given {@code transformArray}.
    * <p>
    * The rotation-scale part R is set as follows:
    *
    * <pre>
    *     / transformArray[0] transformArray[4] transformArray[ 8] \
    * R = | transformArray[1] transformArray[5] transformArray[ 9] |
    *     \ transformArray[2] transformArray[6] transformArray[10] /
    * </pre>
    *
    * The translation part T is set as follows:
    *
    * <pre>
    *     / transformArray[12] \
    * T = | transformArray[13] |
    *     \ transformArray[14] /
    * </pre>
    * </p>
    *
    * @param transformArray the 1D column-major array to get this transform's components from. Not
    *                       modified.
    * @throws NotARotationMatrixException if the resulting matrix for the rotation part of this
    *                                     transform is not a rotation matrix.
    */
   public void setAsTranspose(double[] transformArray)
   {
      double m00 = transformArray[0];
      double m01 = transformArray[4];
      double m02 = transformArray[8];
      double m03 = transformArray[12];
      double m10 = transformArray[1];
      double m11 = transformArray[5];
      double m12 = transformArray[9];
      double m13 = transformArray[13];
      double m20 = transformArray[2];
      double m21 = transformArray[6];
      double m22 = transformArray[10];
      double m23 = transformArray[14];

      getRotation().set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      getTranslation().set(m03, m13, m23);
   }

   /**
    * Sets the raw components of this rigid-body transform from the given {@code transformArray}.
    * <p>
    * The rotation-scale part R is set as follows:
    *
    * <pre>
    *     / transformArray[0] transformArray[1] transformArray[ 2] \
    * R = | transformArray[4] transformArray[5] transformArray[ 6] |
    *     \ transformArray[8] transformArray[9] transformArray[10] /
    * </pre>
    *
    * The translation part T is set as follows:
    *
    * <pre>
    *     / transformArray[ 3] \
    * T = | transformArray[ 7] |
    *     \ transformArray[11] /
    * </pre>
    * </p>
    *
    * @param transformArray the 1D row-major array to get this transform's components from. Not
    *                       modified.
    * @throws NotARotationMatrixException if the resulting matrix for the rotation part of this
    *                                     transform is not a rotation matrix.
    */
   public void set(float[] transformArray)
   {
      double m00 = transformArray[0];
      double m01 = transformArray[1];
      double m02 = transformArray[2];
      double m03 = transformArray[3];
      double m10 = transformArray[4];
      double m11 = transformArray[5];
      double m12 = transformArray[6];
      double m13 = transformArray[7];
      double m20 = transformArray[8];
      double m21 = transformArray[9];
      double m22 = transformArray[10];
      double m23 = transformArray[11];

      getRotation().set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      getTranslation().set(m03, m13, m23);
   }

   /**
    * Sets the raw components of this rigid-body transform from the given {@code transformArray}.
    * <p>
    * The rotation-scale part R is set as follows:
    *
    * <pre>
    *     / transformArray[0] transformArray[4] transformArray[ 8] \
    * R = | transformArray[1] transformArray[5] transformArray[ 9] |
    *     \ transformArray[2] transformArray[6] transformArray[10] /
    * </pre>
    *
    * The translation part T is set as follows:
    *
    * <pre>
    *     / transformArray[12] \
    * T = | transformArray[13] |
    *     \ transformArray[14] /
    * </pre>
    * </p>
    *
    * @param transformArray the 1D column-major array to get this transform's components from. Not
    *                       modified.
    * @throws NotARotationMatrixException if the resulting matrix for the rotation part of this
    *                                     transform is not a rotation matrix.
    */
   public void setAsTranspose(float[] transformArray)
   {
      double m00 = transformArray[0];
      double m01 = transformArray[4];
      double m02 = transformArray[8];
      double m03 = transformArray[12];
      double m10 = transformArray[1];
      double m11 = transformArray[5];
      double m12 = transformArray[9];
      double m13 = transformArray[13];
      double m20 = transformArray[2];
      double m21 = transformArray[6];
      double m22 = transformArray[10];
      double m23 = transformArray[14];

      getRotation().set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      getTranslation().set(m03, m13, m23);
   }

   /**
    * Sets the rotation and translation parts of this transform separately.
    *
    * @param rotationMatrix the matrix used to set the rotation part of this transform. Not modified.
    * @param translation    the tuple used to set the translation part of this transform. Not modified.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation matrix.
    */
   public void set(Matrix3DReadOnly rotationMatrix, Tuple3DReadOnly translation)
   {
      getRotation().set(rotationMatrix);
      getTranslation().set(translation);
   }

   /**
    * Sets the rotation part of this transform from the given 9 coefficients.
    *
    * @param m00 the 1st row 1st column component of the rotation part of this transform.
    * @param m01 the 1st row 2nd column component of the rotation part of this transform.
    * @param m02 the 1st row 3rd column component of the rotation part of this transform.
    * @param m10 the 2nd row 1st column component of the rotation part of this transform.
    * @param m11 the 2nd row 2nd column component of the rotation part of this transform.
    * @param m12 the 2nd row 3rd column component of the rotation part of this transform.
    * @param m20 the 3rd row 1st column component of the rotation part of this transform.
    * @param m21 the 3rd row 2nd column component of the rotation part of this transform.
    * @param m22 the 3rd row 3rd column component of the rotation part of this transform.
    * @throws NotARotationMatrixException if the resulting matrix does not represent a rotation matrix.
    * @deprecated Use {@code this.getRotation().set(m00, m01, m02, m10, m11, m12, m20, m21, m22)}
    *             instead.
    */
   @Deprecated
   public void setRotation(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      getRotation().set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Sets the rotation part of this transform from the given 9 coefficients.
    * <p>
    * Prefer using the method
    * {@link #setRotation(double, double, double, double, double, double, double, double, double)} as
    * it asserts that the coefficients 0represent a rotation matrix.
    * </p>
    *
    * @param m00 the 1st row 1st column component of the rotation part of this transform.
    * @param m01 the 1st row 2nd column component of the rotation part of this transform.
    * @param m02 the 1st row 3rd column component of the rotation part of this transform.
    * @param m10 the 2nd row 1st column component of the rotation part of this transform.
    * @param m11 the 2nd row 2nd column component of the rotation part of this transform.
    * @param m12 the 2nd row 3rd column component of the rotation part of this transform.
    * @param m20 the 3rd row 1st column component of the rotation part of this transform.
    * @param m21 the 3rd row 2nd column component of the rotation part of this transform.
    * @param m22 the 3rd row 3rd column component of the rotation part of this transform.
    * @deprecated Use {@code this.getRotation().setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22)}
    *             instead.
    */
   @Deprecated
   public void setRotationUnsafe(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      getRotation().setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Sets the rotation part of this transform to the given matrix.
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param rotationMatrix the matrix used to set the rotation part of this transform. Not modified.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation matrix.
    * @deprecated Use {@code this.getRotation().set(rotationMatrix)} instead.
    */
   @Deprecated
   public void setRotation(DenseMatrix64F rotationMatrix)
   {
      getRotation().set(rotationMatrix);
   }

   /**
    * Sets the rotation part of this transform to the given matrix.
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param rotationMatrix the matrix used to set the rotation part of this transform. Not modified.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation matrix.
    * @deprecated Use {@code this.getRotation().set(rotationMatrix)} instead.
    */
   @Deprecated
   public void setRotation(Matrix3DReadOnly rotationMatrix)
   {
      getRotation().set(rotationMatrix);
   }

   /**
    * Sets the rotation part of this transform to the given matrix and sets the translation part to
    * zero.
    *
    * @param rotationMatrix the matrix used to set the rotation part of this transform. Not modified.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation matrix.
    */
   public void setRotationAndZeroTranslation(DenseMatrix64F rotationMatrix)
   {
      getRotation().set(rotationMatrix);
      setTranslationToZero();
   }

   /**
    * Sets the rotation part of this transform to the given matrix and sets the translation part to
    * zero.
    *
    * @param rotationMatrix the matrix used to set the rotation part of this transform. Not modified.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation matrix.
    */
   public void setRotationAndZeroTranslation(Matrix3DReadOnly rotationMatrix)
   {
      getRotation().set(rotationMatrix);
      setTranslationToZero();
   }

   /**
    * Performs a linear interpolation from {@code this} to {@code other} given the percentage
    * {@code alpha}.
    *
    * <pre>
    * this.translationVector = (1.0 - alpha) * this.translationVector + alpha * other.translationVector
    * this.rotationMatrix = (1.0 - alpha) * this.rotationMatrix + alpha * other.rotationMatrix
    * </pre>
    *
    * @param other the other transform used in the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not modifying
    *              {@code this}, while a value of 1 is equivalent to setting {@code this} to
    *              {@code other}.
    */
   public void interpolate(RigidBodyTransform other, double alpha)
   {
      interpolate(this, other, alpha);
   }

   /**
    * Performs a linear interpolation from {@code transform1} to {@code transform2} given the
    * percentage {@code alpha}.
    *
    * <pre>
    * this.translationVector = (1.0 - alpha) * transform1.translationVector + alpha *
    * transform2.translationVector
    * this.rotationMatrix = (1.0 - alpha) * transform1.rotationMatrix + alpha *
    * transform2.rotationMatrix
    * </pre>
    *
    * @param transform1 the first transform used in the interpolation. Not modified.
    * @param transform2 the second transform used in the interpolation. Not modified.
    * @param alpha      the percentage to use for the interpolation. A value of 0 will result in
    *                   setting {@code this} to {@code transform1}, while a value of 1 is equivalent to
    *                   setting {@code this} to {@code transform2}.
    */
   public void interpolate(RigidBodyTransform transform1, RigidBodyTransform transform2, double alpha)
   {
      getRotation().interpolate(transform1.getRotation(), transform2.getRotation(), alpha);
      getTranslation().interpolate(transform1.getTranslation(), transform2.getTranslation(), alpha);
   }

   /**
    * Packs this transform as a 4-by-4 matrix.
    *
    * <pre>
    *     / R(0, 0) R(0, 1) R(0, 2) Tx \
    * H = | R(1, 0) R(1, 1) R(1, 2) Ty |
    *     | R(2, 0) R(2, 1) R(2, 2) Tz |
    *     \    0       0       0     1 /
    * </pre>
    *
    * where R is the 3-by-3 rotation matrix and (Tx, Ty, Tz) is the translation part of this transform.
    *
    * @param matrixToPack the matrix in which this transform is stored. Modified.
    */
   public void get(DenseMatrix64F matrixToPack)
   {
      EuclidCoreTools.checkMatrixMinimumSize(4, 4, matrixToPack);
      getRotation().get(matrixToPack);
      getTranslation().get(0, 3, matrixToPack);
      matrixToPack.unsafe_set(3, 0, 0.0);
      matrixToPack.unsafe_set(3, 1, 0.0);
      matrixToPack.unsafe_set(3, 2, 0.0);
      matrixToPack.unsafe_set(3, 3, 1.0);
   }

   /**
    * Packs this transform as a 4-by-4 matrix.
    *
    * <pre>
    *     / R(0, 0) R(0, 1) R(0, 2) Tx \
    * H = | R(1, 0) R(1, 1) R(1, 2) Ty |
    *     | R(2, 0) R(2, 1) R(2, 2) Tz |
    *     \    0       0       0     1 /
    * </pre>
    *
    * where R is the 3-by-3 rotation matrix and (Tx, Ty, Tz) is the translation part of this transform.
    *
    * @param startRow     the first row index to start writing in {@code matrixToPack}.
    * @param startColumn  the first column index to start writing in {@code matrixToPack}.
    * @param matrixToPack the matrix in which this transform is stored. Modified.
    */
   public void get(int startRow, int startColumn, DenseMatrix64F matrixToPack)
   {
      EuclidCoreTools.checkMatrixMinimumSize(startRow + 4, startColumn + 4, matrixToPack);
      getRotation().get(startRow, startColumn, matrixToPack);
      getTranslation().get(startRow, startColumn + 3, matrixToPack);
      startRow += 3;
      matrixToPack.unsafe_set(startRow, startColumn++, 0.0);
      matrixToPack.unsafe_set(startRow, startColumn++, 0.0);
      matrixToPack.unsafe_set(startRow, startColumn++, 0.0);
      matrixToPack.unsafe_set(startRow, startColumn, 1.0);
   }

   /**
    * Packs this transform as a 4-by-4 matrix into a 1D row-major array.
    *
    * <pre>
    *     / R(0, 0) R(0, 1) R(0, 2) Tx \
    * H = | R(1, 0) R(1, 1) R(1, 2) Ty |
    *     | R(2, 0) R(2, 1) R(2, 2) Tz |
    *     \    0       0       0     1 /
    * </pre>
    *
    * where R is the 3-by-3 rotation matrix and (Tx, Ty, Tz) is the translation part of this transform.
    *
    * @param transformArrayToPack the array in which this transform is stored. Modified.
    */
   public void get(double[] transformArrayToPack)
   {
      transformArrayToPack[0] = getM00();
      transformArrayToPack[1] = getM01();
      transformArrayToPack[2] = getM02();
      transformArrayToPack[3] = getM03();
      transformArrayToPack[4] = getM10();
      transformArrayToPack[5] = getM11();
      transformArrayToPack[6] = getM12();
      transformArrayToPack[7] = getM13();
      transformArrayToPack[8] = getM20();
      transformArrayToPack[9] = getM21();
      transformArrayToPack[10] = getM22();
      transformArrayToPack[11] = getM23();
      transformArrayToPack[12] = getM30();
      transformArrayToPack[13] = getM31();
      transformArrayToPack[14] = getM32();
      transformArrayToPack[15] = getM33();
   }

   /**
    * Packs this transform as a 4-by-4 matrix into a 1D row-major array.
    *
    * <pre>
    *     / R(0, 0) R(0, 1) R(0, 2) Tx \
    * H = | R(1, 0) R(1, 1) R(1, 2) Ty |
    *     | R(2, 0) R(2, 1) R(2, 2) Tz |
    *     \    0       0       0     1 /
    * </pre>
    *
    * where R is the 3-by-3 rotation matrix and (Tx, Ty, Tz) is the translation part of this transform.
    *
    * @param transformArrayToPack the array in which this transform is stored. Modified.
    */
   public void get(float[] transformArrayToPack)
   {
      transformArrayToPack[0] = (float) getM00();
      transformArrayToPack[1] = (float) getM01();
      transformArrayToPack[2] = (float) getM02();
      transformArrayToPack[3] = (float) getM03();
      transformArrayToPack[4] = (float) getM10();
      transformArrayToPack[5] = (float) getM11();
      transformArrayToPack[6] = (float) getM12();
      transformArrayToPack[7] = (float) getM13();
      transformArrayToPack[8] = (float) getM20();
      transformArrayToPack[9] = (float) getM21();
      transformArrayToPack[10] = (float) getM22();
      transformArrayToPack[11] = (float) getM23();
      transformArrayToPack[12] = (float) getM30();
      transformArrayToPack[13] = (float) getM31();
      transformArrayToPack[14] = (float) getM32();
      transformArrayToPack[15] = (float) getM33();
   }

   /**
    * Packs the rotation matrix and translation vector of this rigid-body transform.
    *
    * @param rotationMarixToPack the matrix to set to the rotation of this transform. Modified.
    * @param translationToPack   the tuple to set to the translation of this transform. Modified.
    */
   public void get(CommonMatrix3DBasics rotationMarixToPack, Tuple3DBasics translationToPack)
   {
      rotationMarixToPack.set(getRotation());
      translationToPack.set(getTranslation());
   }

   @Override
   public RotationMatrixBasics getRotation()
   {
      return rotationMatrix;
   }

   /**
    * Packs the rotation part of this rigid-body transform.
    *
    * @param rotationMatrixToPack the matrix in which the rotation part of this transform is stored.
    *                             Modified.
    * @deprecated Use {@code rotationMatrixToPack.set(this.getRotation())} instead.
    */
   @Deprecated
   public void getRotation(CommonMatrix3DBasics rotationMatrixToPack)
   {
      rotationMatrixToPack.set(getRotation());
   }

   /**
    * Packs the rotation part of this rigid-body transform.
    *
    * @param rotationMatrixToPack the matrix in which the rotation part of this transform is stored.
    *                             Modified.
    * @deprecated Use {@code this.getRotation().get(rotationMatrixToPack)} instead.
    */
   @Deprecated
   public void getRotation(DenseMatrix64F rotationMatrixToPack)
   {
      getRotation().get(rotationMatrixToPack);
   }

   /**
    * Packs the rotation part of this rigid-body transform in 1D row-major array.
    *
    * @param rotationMatrixArrayToPack the array in which the rotation part of this transform is
    *                                  stored. Modified.
    * @deprecated Use {@code this.getRotation().get(rotationMatrixToPack)} instead.
    */
   @Deprecated
   public void getRotation(double[] rotationMatrixArrayToPack)
   {
      getRotation().get(rotationMatrixArrayToPack);
   }

   @Override
   public Vector3DBasics getTranslation()
   {
      return translationVector;
   }

   /**
    * Retrieves and returns a coefficient of this transform given its row and column indices.
    *
    * @param row    the row of the coefficient to return.
    * @param column the column of the coefficient to return.
    * @return the coefficient's value.
    * @throws ArrayIndexOutOfBoundsException if either {@code row} &notin; [0, 3] or {@code column}
    *                                        &notin; [0, 3].
    */
   public double getElement(int row, int column)
   {
      if (row < 3)
      {
         if (column < 3)
         {
            return getRotation().getElement(row, column);
         }
         else if (column < 4)
         {
            return getTranslation().getElement(row);
         }
         else
         {
            throw Matrix3DTools.columnOutOfBoundsException(3, column);
         }
      }
      else if (row < 4)
      {
         if (column < 3)
         {
            return 0.0;
         }
         else if (column < 4)
         {
            return 1.0;
         }
         else
         {
            throw Matrix3DTools.columnOutOfBoundsException(3, column);
         }
      }
      else
      {
         throw Matrix3DTools.rowOutOfBoundsException(3, row);
      }
   }

   /**
    * Gets the 1st row 1st column coefficient of this transform.
    *
    * @return the 1st row 1st column coefficient.
    */
   public double getM00()
   {
      return getRotation().getM00();
   }

   /**
    * Gets the 1st row 2nd column coefficient of this transform.
    *
    * @return the 1st row 2nd column coefficient.
    */
   public double getM01()
   {
      return getRotation().getM01();
   }

   /**
    * Gets the 1st row 3rd column coefficient of this transform.
    *
    * @return the 1st row 3rd column coefficient.
    */
   public double getM02()
   {
      return getRotation().getM02();
   }

   /**
    * Gets the 1st row 4th column coefficient of this transform.
    *
    * @return the 1st row 4th column coefficient.
    */
   public double getM03()
   {
      return getTranslation().getX();
   }

   /**
    * Gets the 2nd row 1st column coefficient of this transform.
    *
    * @return the 2nd row 1st column coefficient.
    */
   public double getM10()
   {
      return getRotation().getM10();
   }

   /**
    * Gets the 2nd row 2nd column coefficient of this transform.
    *
    * @return the 2nd row 2nd column coefficient.
    */
   public double getM11()
   {
      return getRotation().getM11();
   }

   /**
    * Gets the 2nd row 3rd column coefficient of this transform.
    *
    * @return the 2nd row 3rd column coefficient.
    */
   public double getM12()
   {
      return getRotation().getM12();
   }

   /**
    * Gets the 2nd row 4th column coefficient of this transform.
    *
    * @return the 2nd row 4th column coefficient.
    */
   public double getM13()
   {
      return getTranslation().getY();
   }

   /**
    * Gets the 3rd row 1st column coefficient of this transform.
    *
    * @return the 3rd row 1st column coefficient.
    */
   public double getM20()
   {
      return getRotation().getM20();
   }

   /**
    * Gets the 3rd row 2nd column coefficient of this transform.
    *
    * @return the 3rd row 2nd column coefficient.
    */
   public double getM21()
   {
      return getRotation().getM21();
   }

   /**
    * Gets the 3rd row 3rd column coefficient of this transform.
    *
    * @return the 3rd row 3rd column coefficient.
    */
   public double getM22()
   {
      return getRotation().getM22();
   }

   /**
    * Gets the 3rd row 4th column coefficient of this transform.
    *
    * @return the 3rd row 4th column coefficient.
    */
   public double getM23()
   {
      return getTranslation().getZ();
   }

   /**
    * Gets the 4th row 1st column coefficient of this transform.
    * <p>
    * Note: {@code m30 = 0.0}.
    * </p>
    *
    * @return the 4th row 1st column coefficient.
    */
   public double getM30()
   {
      return 0.0;
   }

   /**
    * Gets the 4th row 2nd column coefficient of this transform.
    * <p>
    * Note: {@code m31 = 0.0}.
    * </p>
    *
    * @return the 4th row 2nd column coefficient.
    */
   public double getM31()
   {
      return 0.0;
   }

   /**
    * Gets the 4th row 3rd column coefficient of this transform.
    * <p>
    * Note: {@code m32 = 0.0}.
    * </p>
    *
    * @return the 4th row 3rd column coefficient.
    */
   public double getM32()
   {
      return 0.0;
   }

   /**
    * Gets the 4th row 4th column coefficient of this transform.
    * <p>
    * Note: {@code m33 = 1.0}.
    * </p>
    *
    * @return the 4th row 4th column coefficient.
    */
   public double getM33()
   {
      return 1.0;
   }

   /**
    * Tests separately and on a per component basis if the rotation part and the translation part of
    * this transform and {@code other} are equal to an {@code epsilon}.
    *
    * @param other the other rigid-body transform to compare against this. Not modified.
    */
   @Override
   public boolean epsilonEquals(RigidBodyTransform other, double epsilon)
   {
      return getRotation().epsilonEquals(other.getRotation(), epsilon) && getTranslation().epsilonEquals(other.getTranslation(), epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(RigidBodyTransform)}, it returns {@code false} otherwise or if the {@code object}
    * is {@code null}.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof RigidBodyTransform)
         return equals((RigidBodyTransform) object);
      else
         return false;
   }

   /**
    * Tests separately and on a per component basis if the rotation part and the translation part of
    * this transform and {@code other} are exactly equal.
    * <p>
    * The method returns {@code false} if the given transform is {@code null}.
    * </p>
    *
    * @param other the other transform to compare against this. Not modified.
    * @return {@code true} if the two transforms are exactly equal, {@code false} otherwise.
    */
   public boolean equals(RigidBodyTransform other)
   {
      if (other == this)
         return true;
      else if (other == null)
         return false;
      else
         return getRotation().equals(other.getRotation()) && getTranslation().equals(other.getTranslation());
   }

   /**
    * Two rigid body transforms are considered geometrically equal if both the rotation matrices and
    * translation vectors are equal.
    *
    * @param other   the other rigid body transform to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two rigid body transforms are equal, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(RigidBodyTransform other, double epsilon)
   {
      return other.getRotation().geometricallyEquals(getRotation(), epsilon) && other.getTranslation().geometricallyEquals(getTranslation(), epsilon);
   }

   /**
    * Provides a {@code String} representation of this transform as follows: <br>
    * m00, m01, m02 | m03 <br>
    * m10, m11, m12 | m13 <br>
    * m20, m21, m22 | m23
    *
    * @return the {@code String} representing this transform.
    */
   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getRigidBodyTransformString(this);
   }

   @Override
   public int hashCode()
   {
      long bits = EuclidHashCodeTools.addToHashCode(getRotation().hashCode(), getTranslation().hashCode());
      return EuclidHashCodeTools.toIntHashCode(bits);
   }
}
