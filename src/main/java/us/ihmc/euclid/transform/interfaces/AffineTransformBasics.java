package us.ihmc.euclid.transform.interfaces;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.exceptions.NotARotationScaleMatrixException;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public interface AffineTransformBasics extends AffineTransformReadOnly, Clearable
{
   @Override
   LinearTransform3DBasics getLinearTransform();

   @Override
   Vector3DBasics getTranslation();

   /**
    * Resets this affine transform to identity.
    * <p>
    * When set to identity, this transform has no effect when transforming a geometry object.
    * </p>
    */
   default void setIdentity()
   {
      getLinearTransform().setIdentity();
      getTranslation().setToZero();
   }

   /**
    * Resets this affine transform to identity.
    * <p>
    * When set to identity, this transform has no effect when transforming a geometry object.
    * </p>
    */
   @Override
   default void setToZero()
   {
      setIdentity();
   }

   default void setLinearTransformToZero()
   {
      getLinearTransform().setIdentity();
   }

   /**
    * Sets all the scale factors to 1.0.
    */
   default void resetScale()
   {
      getLinearTransform().resetScale();
   }

   /**
    * Sets the translation part to zero.
    */
   default void setTranslationToZero()
   {
      getTranslation().setToZero();
   }

   /**
    * Sets all the components of this affine transform making it invalid.
    */
   @Override
   default void setToNaN()
   {
      getLinearTransform().setToNaN();
      getTranslation().setToNaN();
   }

   /**
    * Tests if at least one element of this transform is equal to {@linkplain Double#NaN}.
    *
    * @return {@code true} if at least one element of this transform is equal to
    *         {@linkplain Double#NaN}, {@code false} otherwise.
    */
   @Override
   default boolean containsNaN()
   {
      return getLinearTransform().containsNaN() || getTranslation().containsNaN();
   }

   default void invert()
   {
      if (hasLinearTransform())
      {
         getLinearTransform().invert();
         if (hasTranslation())
            getLinearTransform().transform(getTranslation());
      }
      getTranslation().negate();
   }

   /**
    * Sets this affine transform from the given 12 coefficients.
    *
    * @param m00 the 1st row 1st column component of the rotation-scale part of this transform.
    * @param m01 the 1st row 2nd column component of the rotation-scale part of this transform.
    * @param m02 the 1st row 3rd column component of the rotation-scale part of this transform.
    * @param m03 the x-component of the translation part of this transform.
    * @param m10 the 2nd row 1st column component of the rotation-scale part of this transform.
    * @param m11 the 2nd row 2nd column component of the rotation-scale part of this transform.
    * @param m12 the 2nd row 3rd column component of the rotation-scale part of this transform.
    * @param m13 the y-component of the translation part of this transform.
    * @param m20 the 3rd row 1st column component of the rotation-scale part of this transform.
    * @param m21 the 3rd row 2nd column component of the rotation-scale part of this transform.
    * @param m22 the 3rd row 3rd column component of the rotation-scale part of this transform.
    * @param m23 the z-component of the translation part of this transform.
    * @throws NotARotationScaleMatrixException if the components for the rotation-scale part do not
    *                                          represent a rotation-scale matrix.
    */
   default void set(double m00, double m01, double m02, double m03, double m10, double m11, double m12, double m13, double m20, double m21, double m22,
                    double m23)
   {
      getLinearTransform().set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      getTranslation().set(m03, m13, m23);
   }

   default void set(AffineTransformReadOnly other)
   {
      getLinearTransform().set(other.getLinearTransform());
      getTranslation().set(other.getTranslation());
   }

   /**
    * Sets this affine transform to the given rigid-body transform.
    * <p>
    * The scaling part of this transform will be reset.
    * </p>
    *
    * @param rigidBodyTransform the rigid-body transform to copy the values from. Not modified.
    */
   default void set(RigidBodyTransformReadOnly rigidBodyTransform)
   {
      getLinearTransform().set(rigidBodyTransform.getRotation());
      getTranslation().set(rigidBodyTransform.getTranslation());
   }

   /**
    * Sets the raw components of this affine transform from the given {@code matrix}.
    * <p>
    * The rotation-scale part M is set as follows:
    *
    * <pre>
    *     / matrix.get(0, 0) matrix.get(0, 1) matrix.get(0, 2) \
    * M = | matrix.get(1, 0) matrix.get(1, 1) matrix.get(1, 2) |
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
    * @throws NotARotationScaleMatrixException if the resulting matrix for the rotation-scale part of
    *                                          this transform is not a rotation-scale matrix.
    */
   default void set(DMatrix matrix)
   {
      getLinearTransform().set(matrix);
      getTranslation().set(0, 3, matrix);
   }

   /**
    * Sets the raw components of this affine transform from the given {@code matrix}.
    * <p>
    * The rotation-scale part M is set as follows:
    *
    * <pre>
    *     / matrix.get(startRow + 0, startColumn + 0) matrix.get(startRow + 0, startColumn + 1) matrix.get(startRow + 0, startColumn + 2) \
    * M = | matrix.get(startRow + 1, startColumn + 0) matrix.get(startRow + 1, startColumn + 1) matrix.get(startRow + 1, startColumn + 2) |
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
    * @throws NotARotationScaleMatrixException if the resulting matrix for the rotation-scale part of
    *                                          this transform is not a rotation-scale matrix.
    */
   default void set(DMatrix matrix, int startRow, int startColumn)
   {
      getLinearTransform().set(startRow, startColumn, matrix);
      getTranslation().set(startRow, startColumn + 3, matrix);
   }

   /**
    * Sets the raw components of this affine transform from the given {@code transformArray}.
    * <p>
    * The rotation-scale part M is set as follows:
    *
    * <pre>
    *     / transformArray[0] transformArray[1] transformArray[ 2] \
    * M = | transformArray[4] transformArray[5] transformArray[ 6] |
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
    * @throws NotARotationScaleMatrixException if the resulting matrix for the rotation-scale part of
    *                                          this transform is not a rotation-scale matrix.
    */
   default void set(double[] transformArray)
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

      getLinearTransform().set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      getTranslation().set(m03, m13, m23);
   }

   /**
    * Sets the rotation-scale and translation parts of this transform separately.
    *
    * @param linearTransform the matrix used to set the rotation-scale part of this transform. Not
    *                        modified.
    * @param translation     the tuple used to set the translation part of this transform. Not
    *                        modified.
    * @throws NotARotationScaleMatrixException if the given {@code rotationScaleMatrix} is not a
    *                                          rotation-scale matrix.
    */
   default void set(Matrix3DReadOnly linearTransform, Tuple3DReadOnly translation)
   {
      getLinearTransform().set(linearTransform);
      getTranslation().set(translation);
   }

   default void set(DMatrix linearTransform, Tuple3DReadOnly translation)
   {
      getLinearTransform().set(linearTransform);
      getTranslation().set(translation);
   }

   default void set(RotationMatrixReadOnly rotationMatrix, Tuple3DReadOnly translation)
   {
      getLinearTransform().set(rotationMatrix);
      getTranslation().set(translation);
   }

   default void set(Orientation3DReadOnly orientation, Tuple3DReadOnly translation)
   {
      getLinearTransform().set(orientation);
      getTranslation().set(translation);
   }

   /**
    * Sets the x-component of the translation part of this transform.
    * <p>
    * This method does not affect the rotation part nor the scale part of this transform.
    * </p>
    *
    * @param x the x-component of the translation part.
    */
   default void setTranslationX(double x)
   {
      getTranslation().setX(x);
   }

   /**
    * Sets the y-component of the translation part of this transform.
    * <p>
    * This method does not affect the rotation part nor the scale part of this transform.
    * </p>
    *
    * @param y the y-component of the translation part.
    */
   default void setTranslationY(double y)
   {
      getTranslation().setY(y);
   }

   /**
    * Sets the z-component of the translation part of this transform.
    * <p>
    * This method does not affect the rotation part nor the scale part of this transform.
    * </p>
    *
    * @param z the z-component of the translation part.
    */
   default void setTranslationZ(double z)
   {
      getTranslation().setZ(z);
   }

   /**
    * Sets the translation part of this transform.
    * <p>
    * This method does not affect the rotation part nor the scale part of this transform.
    * </p>
    *
    * @param x the x-component of the translation part.
    * @param y the y-component of the translation part.
    * @param z the z-component of the translation part.
    */
   default void setTranslation(double x, double y, double z)
   {
      getTranslation().set(x, y, z);
   }

   /**
    * Sets the translation part of this transform.
    * <p>
    * This method does not affect the rotation part nor the scale part of this transform.
    * </p>
    *
    * @param translation tuple used to set the translation part of this transform. Not modified.
    */
   default void setTranslation(Tuple3DReadOnly translation)
   {
      getTranslation().set(translation);
   }

   default void setLinearTransform(Matrix3DReadOnly linearTransform)
   {
      getLinearTransform().set(linearTransform);
   }

   default void setLinearTransform(DMatrix linearTransform)
   {
      getLinearTransform().set(linearTransform);
   }

   default void setLinearTransform(RotationMatrixReadOnly rotationMatrix)
   {
      getLinearTransform().set(rotationMatrix);
   }

   default void setLinearTransform(Orientation3DReadOnly orientation)
   {
      getLinearTransform().set(orientation);
   }

   /**
    * Performs the multiplication of this with the given {@code other}.
    * <p>
    * Note: the scale part of either affine transform is not used when performing the multiplication.
    * This operation does not affect the scale of this transform.
    * </p>
    * <p>
    * this = this * other
    * </p>
    *
    * @param other the other transform to multiply this with. Not modified.
    */
   default void multiply(AffineTransformReadOnly other)
   {
      if (other.hasTranslation())
         getLinearTransform().addTransform(other.getTranslation(), getTranslation());
      getLinearTransform().multiply(other.getLinearTransform());
   }

   /**
    * Performs the multiplication of this with the given {@code rigidBodyTransform}.
    * <p>
    * Note: the scale part of this affine transform is not used when performing the multiplication.
    * This operation does not affect the scale of this transform.
    * </p>
    * <p>
    * this = this * rigidBodyTransform
    * </p>
    *
    * @param rigidBodyTransform the rigid-body transform to multiply this with. Not modified.
    */
   default void multiply(RigidBodyTransformReadOnly rigidBodyTransform)
   {
      if (rigidBodyTransform.hasTranslation())
         getLinearTransform().addTransform(rigidBodyTransform.getTranslation(), getTranslation());
      getLinearTransform().appendRotation(rigidBodyTransform.getRotation());
   }

   /**
    * Performs the multiplication of the inverse of this with the given {@code other}.
    * <p>
    * Note: the scale part of the either affine transform is not used when performing the
    * multiplication. This operation does not affect the scale of this transform.
    * </p>
    * <p>
    * this = this<sup>-1</sup> * other
    * </p>
    *
    * @param other the other transform to multiply this with. Not modified.
    */
   default void multiplyInvertThis(AffineTransformReadOnly other)
   {
      getTranslation().sub(other.getTranslation(), getTranslation());

      getLinearTransform().invert();
      getLinearTransform().transform(getTranslation());
      getLinearTransform().multiply(other.getLinearTransform());
   }

   /**
    * Performs the multiplication of this with the inverse of the given {@code other}.
    * <p>
    * Note: the scale part of the either affine transform is not used when performing the
    * multiplication. This operation does not affect the scale of this transform.
    * </p>
    * <p>
    * this = this * other<sup>-1</sup>
    * </p>
    *
    * @param other the other transform to multiply this with. Not modified.
    */
   default void multiplyInvertOther(AffineTransformReadOnly other)
   {
      getLinearTransform().multiplyInvertOther(other.getLinearTransform());

      if (other.hasTranslation())
         getLinearTransform().subTransform(other.getTranslation(), getTranslation());
   }

   /**
    * Performs the multiplication of the inverse of this with the given {@code rigidBodyTransform}.
    * <p>
    * Note: the scale part of this affine transform is not used when performing the multiplication.
    * This operation does not affect the scale of this transform.
    * </p>
    * <p>
    * this = this<sup>-1</sup> * rigidBodyTransform
    * </p>
    *
    * @param rigidBodyTransform the rigid-body transform to multiply this with. Not modified.
    */
   default void multiplyInvertThis(RigidBodyTransformReadOnly rigidBodyTransform)
   {
      getTranslation().sub(rigidBodyTransform.getTranslation(), getTranslation());

      getLinearTransform().invert();
      getLinearTransform().transform(getTranslation());
      getLinearTransform().appendRotation(rigidBodyTransform.getRotation());
   }

   /**
    * Performs the multiplication of this with the inverse of the given {@code rigidBodyTransform}.
    * <p>
    * Note: the scale part of this affine transform is not used when performing the multiplication.
    * This operation does not affect the scale of this transform.
    * </p>
    * <p>
    * this = this * rigidBodyTransform<sup>-1</sup>
    * </p>
    *
    * @param rigidBodyTransform the rigid-body transform to multiply this with. Not modified.
    */
   default void multiplyInvertOther(RigidBodyTransformReadOnly rigidBodyTransform)
   {
      getLinearTransform().appendRotationInvertOther(rigidBodyTransform.getRotation());
      getLinearTransform().subTransform(rigidBodyTransform.getTranslation(), getTranslation());
   }

   /**
    * Append a translation transform to this transform.
    * <p>
    * Note: the scale part of this affine transform is not used when performing the multiplication.
    * </p>
    *
    * <pre>
    *               / 1 0 0 translation.x \
    * this = this * | 0 1 0 translation.y |
    *               | 0 0 1 translation.z |
    *               \ 0 0 0      1        /
    * </pre>
    * <p>
    * This method does not affect the rotation part nor the scale part of this transform.
    * </p>
    *
    * @param translation the translation to append to this transform. Not modified.
    */
   default void appendTranslation(Tuple3DReadOnly translation)
   {
      getLinearTransform().addTransform(translation, getTranslation());
   }

   /**
    * Append a translation transform to this transform.
    * <p>
    * Note: the scale part of this affine transform is not used when performing the multiplication.
    * </p>
    *
    * <pre>
    *               / 1 0 0 x \
    * this = this * | 0 1 0 y |
    *               | 0 0 1 z |
    *               \ 0 0 0 1 /
    * </pre>
    * <p>
    * This method does not affect the rotation part nor the scale part of this transform.
    * </p>
    *
    * @param x the translation along the x-axis to apply.
    * @param y the translation along the y-axis to apply.
    * @param z the translation along the z-axis to apply.
    */
   default void appendTranslation(double x, double y, double z)
   {
      double thisX = getTranslation().getX();
      double thisY = getTranslation().getY();
      double thisZ = getTranslation().getZ();

      getTranslation().set(x, y, z);
      getLinearTransform().transform(getTranslation());
      getTranslation().add(thisX, thisY, thisZ);
   }

   default void appendOrientation(Orientation3DReadOnly orientation)
   {
      getLinearTransform().appendRotation(orientation);
   }

   /**
    * Append a rotation about the z-axis to the rotation part of this transform.
    *
    * <pre>
    *         / cos(yaw) -sin(yaw) 0 \
    * R = R * | sin(yaw)  cos(yaw) 0 |
    *         \    0         0     1 /
    * </pre>
    * <p>
    * This method does not affect the scale part nor the translation part of this transform.
    * </p>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   default void appendYawRotation(double yaw)
   {
      getLinearTransform().appendYawRotation(yaw);
   }

   /**
    * Append a rotation about the y-axis to the rotation part of this transform.
    *
    * <pre>
    *         /  cos(pitch) 0 sin(pitch) \
    * R = R * |      0      1     0      |
    *         \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    * <p>
    * This method does not affect the scale part nor the translation part of this transform.
    * </p>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   default void appendPitchRotation(double pitch)
   {
      getLinearTransform().appendPitchRotation(pitch);
   }

   /**
    * Append a rotation about the x-axis to the rotation part of this transform.
    *
    * <pre>
    *         / 1     0          0     \
    * R = R * | 0 cos(roll) -sin(roll) |
    *         \ 0 sin(roll)  cos(roll) /
    * </pre>
    * <p>
    * This method does not affect the scale part nor the translation part of this transform.
    * </p>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   default void appendRollRotation(double roll)
   {
      getLinearTransform().appendRollRotation(roll);
   }

   default void appendScale(double scale)
   {
      appendScale(scale, scale, scale);
   }

   default void appendScale(Tuple3DReadOnly scale)
   {
      appendScale(scale.getX(), scale.getY(), scale.getZ());
   }

   default void appendScale(double x, double y, double z)
   {
      getLinearTransform().appendScale(x, y, z);
   }

   /**
    * Performs the multiplication of {@code other} with this transform.
    * <p>
    * Note: the scale part of either affine transform is not used when performing the multiplication.
    * This operation does not affect the scale of this transform.
    * </p>
    * <p>
    * this = other * this
    * </p>
    *
    * @param other the other transform to multiply this with. Not modified.
    */
   default void preMultiply(AffineTransformReadOnly other)
   {
      other.getLinearTransform().transform(getTranslation());
      getTranslation().add(other.getTranslation());
      getLinearTransform().preMultiply(other.getLinearTransform());
   }

   /**
    * Performs the multiplication of {@code rigidBodyTransform} with this transform.
    * <p>
    * Note: this operation does not affect the scale of this transform.
    * </p>
    * <p>
    * this = rigidBodyTransform * this
    * </p>
    *
    * @param rigidBodyTransform the other transform to multiply this with. Not modified.
    */
   default void preMultiply(RigidBodyTransformReadOnly rigidBodyTransform)
   {
      rigidBodyTransform.transform(getTranslation());
      getTranslation().add(rigidBodyTransform.getTranslation());
      getLinearTransform().prepend(rigidBodyTransform.getRotation());
   }

   /**
    * Performs the multiplication of {@code other} with the inverse of this transform.
    * <p>
    * Note: the scale part of either affine transform is not used when performing the multiplication.
    * This operation does not affect the scale of this transform.
    * </p>
    * <p>
    * this = other * this<sup>-1</sup>
    * </p>
    *
    * @param other the other transform to multiply this with. Not modified.
    */
   default void preMultiplyInvertThis(AffineTransformReadOnly other)
   {
      getLinearTransform().preMultiplyInvertThis(other.getLinearTransform());
      getLinearTransform().transform(getTranslation());
      getTranslation().sub(other.getTranslation(), getTranslation());
   }

   /**
    * Performs the multiplication of the inverse of {@code other} with this transform.
    * <p>
    * Note: the scale part of either affine transform is not used when performing the multiplication.
    * This operation does not affect the scale of this transform.
    * </p>
    * <p>
    * this = other<sup>-1</sup> * this
    * </p>
    *
    * @param other the other transform to multiply this with. Not modified.
    */
   default void preMultiplyInvertOther(AffineTransformReadOnly other)
   {
      getTranslation().sub(other.getTranslation());
      other.getLinearTransform().inverseTransform(getTranslation());
      getLinearTransform().preMultiplyInvertOther(other.getLinearTransform());
   }

   /**
    * Performs the multiplication of {@code rigidBodyTransform} with the inverse of this transform.
    * <p>
    * Note: the scale part of this affine transform is not used when performing the multiplication.
    * This operation does not affect the scale of this transform.
    * </p>
    * <p>
    * this = rigidBodyTransform * this<sup>-1</sup>
    * </p>
    *
    * @param rigidBodyTransform the rigid-body transform to multiply this with. Not modified.
    */
   default void preMultiplyInvertThis(RigidBodyTransformReadOnly rigidBodyTransform)
   {
      getLinearTransform().prependInvertThis(rigidBodyTransform.getRotation());
      getLinearTransform().transform(getTranslation());
      getTranslation().sub(rigidBodyTransform.getTranslation(), getTranslation());
   }

   /**
    * Performs the multiplication of the inverse of {@code rigidBodyTransform} with this transform.
    * <p>
    * Note: this operation does not affect the scale of this transform.
    * </p>
    * <p>
    * this = rigidBodyTransform<sup>-1</sup> * this
    * </p>
    *
    * @param rigidBodyTransform the rigid-body transform to multiply this with. Not modified.
    */
   default void preMultiplyInvertOther(RigidBodyTransformReadOnly rigidBodyTransform)
   {
      getTranslation().sub(rigidBodyTransform.getTranslation());
      rigidBodyTransform.getRotation().inverseTransform(getTranslation());
      getLinearTransform().prependInvertOther(rigidBodyTransform.getRotation());
   }

   /**
    * Prepend a translation transform to this transform.
    *
    * <pre>
    *        / 1 0 0 translation.x \
    * this = | 0 1 0 translation.y | * this
    *        | 0 0 1 translation.z |
    *        \ 0 0 0      1        /
    * </pre>
    * <p>
    * This method does not affect the rotation part nor the scale part of this transform.
    * </p>
    *
    * @param translation the translation to prepend to this transform. Not modified.
    */
   default void prependTranslation(Tuple3DReadOnly translation)
   {
      getTranslation().add(translation);
   }

   /**
    * Prepend a translation transform to this transform.
    *
    * <pre>
    *        / 1 0 0 x \
    * this = | 0 1 0 y | * this
    *        | 0 0 1 z |
    *        \ 0 0 0 1 /
    * </pre>
    * <p>
    * This method does not affect the rotation part nor the scale part of this transform.
    * </p>
    *
    * @param x the translation along the x-axis to apply.
    * @param y the translation along the y-axis to apply.
    * @param z the translation along the z-axis to apply.
    */
   default void prependTranslation(double x, double y, double z)
   {
      getTranslation().add(x, y, z);
   }

   /**
    * Prepend a rotation about the z-axis to the rotation part of this transform.
    * <p>
    * Note that the scale part of this transform is not used for this operation.
    * </p>
    * <p>
    * This method first rotates the translation part and then prepend the yaw-rotation to the rotation
    * part of this transform.
    * </p>
    *
    * <pre>
    *        / cos(yaw) -sin(yaw)  0   0 \
    * this = | sin(yaw)  cos(yaw)  0   0 | * this
    *        |    0         0      1   0 |
    *        \    0         0      0   1 /
    * </pre>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   default void prependYawRotation(double yaw)
   {
      RotationMatrixTools.applyYawRotation(yaw, getTranslation(), getTranslation());
      getLinearTransform().prependYawRotation(yaw);
   }

   /**
    * Prepend a rotation about the y-axis to this transform.
    * <p>
    * Note that the scale part of this transform is not used for this operation.
    * </p>
    * <p>
    * This method first rotates the translation part and then prepend the pitch-rotation to the
    * rotation part of this transform.
    * </p>
    *
    * <pre>
    *        /  cos(pitch) 0 sin(pitch)  0 \
    * this = |      0      1     0       0 | * this
    *        | -sin(pitch) 0 cos(pitch)  0 |
    *        \      0      0     0       1 /
    * </pre>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   default void prependPitchRotation(double pitch)
   {
      RotationMatrixTools.applyPitchRotation(pitch, getTranslation(), getTranslation());
      getLinearTransform().prependPitchRotation(pitch);
   }

   /**
    * Prepend a rotation about the x-axis to this transform.
    * <p>
    * Note that the scale part of this transform is not used for this operation.
    * </p>
    * <p>
    * This method first rotates the translation part and then prepend the roll-rotation to the rotation
    * part of this transform.
    * </p>
    *
    * <pre>
    *        / 1     0          0     0 \
    * this = | 0 cos(roll) -sin(roll) 0 | * this
    *        | 0 sin(roll)  cos(roll) 0 |
    *        \ 0     0          0     1 /
    * </pre>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   default void prependRollRotation(double roll)
   {
      RotationMatrixTools.applyRollRotation(roll, getTranslation(), getTranslation());
      getLinearTransform().prependRollRotation(roll);
   }

   default void prependScale(double scale)
   {
      prependScale(scale, scale, scale);
   }

   default void prependScale(Tuple3DReadOnly scale)
   {
      prependScale(scale.getX(), scale.getY(), scale.getZ());
   }

   default void prependScale(double x, double y, double z)
   {
      getTranslation().scale(x, y, z);
      getLinearTransform().prependScale(x, y, z);
   }
}
