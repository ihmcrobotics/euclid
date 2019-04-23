package us.ihmc.euclid.transform;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.GeometricallyComparable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

/**
 * A {@code QuaternionBasedTransform} represents a 4-by-4 transformation matrix that can rotate and
 * translate.
 * <p>
 * The {@code QuaternionBasedTransform} provides the same features as {@link RigidBodyTransform}.
 * However, it stores the rotation part of the transform as a quaternion.
 * </p>
 * <p>
 * For efficiency and readability, the transform is never stored in a 4-by-4 matrix.
 * </p>
 * <p>
 * A few special cases to keep in mind:
 * <ul>
 * <li>when applying this transform on a {@link Point3DBasics} or {@link Point2DBasics}, this object
 * is rotated, then translated.
 * <li>when applying this transform on a {@link Vector3DBasics} or {@link Vector2DBasics}, this
 * object is only rotated. It is NOT translated.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class QuaternionBasedTransform implements RigidBodyTransformBasics, EpsilonComparable<QuaternionBasedTransform>,
      GeometricallyComparable<QuaternionBasedTransform>, Settable<QuaternionBasedTransform>
{
   /** The rotation part of this transform. */
   private final Quaternion quaternion = new Quaternion();
   /** The translation part of this transform. */
   private final Vector3D translationVector = new Vector3D();

   /**
    * Creates a new quaternion-based transform set to identity.
    * <p>
    * When set to identity, this transform has no effect when transforming a geometry object.
    * </p>
    */
   public QuaternionBasedTransform()
   {
      setIdentity();
   }

   /**
    * Creates a new quaternion-based transform and initializes to the given rigid-body transform.
    *
    * @param rigidBodyTransform the rigid-body transform to copy. Not modified.
    */
   public QuaternionBasedTransform(RigidBodyTransformReadOnly rigidBodyTransform)
   {
      set(rigidBodyTransform);
   }

   /**
    * Creates a new quaternion-based transform given the value of its 7 components (4 components for
    * quaternion and 3 for the translation).
    * <p>
    * The quaternion q is set as follows:
    *
    * <pre>
    *     / qx = matrix.get(0, 0) \
    * q = | qy = matrix.get(1, 0) |
    *     | qz = matrix.get(2, 0) |
    *     \ qs = matrix.get(3, 0) /
    * </pre>
    *
    * The translation part T is set as follows:
    *
    * <pre>
    *     / matrix.get(4, 0) \
    * T = | matrix.get(5, 0) |
    *     \ matrix.get(6, 0) /
    * </pre>
    * </p>
    *
    * @param matrix the column vector containing the values of the 7 components of this transform. Not
    *           modified.
    */
   public QuaternionBasedTransform(DenseMatrix64F matrix)
   {
      set(matrix);
   }

   /**
    * Creates a new quaternion-based transform given the value of its 7 components (4 components for
    * quaternion and 3 for the translation).
    * <p>
    * The quaternion q is set as follows:
    *
    * <pre>
    *     / qx = array[0] \
    * q = | qy = array[1] |
    *     | qz = array[2] |
    *     \ qs = array[3] /
    * </pre>
    *
    * The translation part T is set as follows:
    *
    * <pre>
    *     / array[4] \
    * T = | array[5] |
    *     \ array[6] /
    * </pre>
    * </p>
    *
    * @param array the array containing the values of the 7 components of this transform. Not modified.
    */
   public QuaternionBasedTransform(double[] array)
   {
      set(array);
   }

   /**
    * Creates a new quaternion-based transform and initializes it to the given rotation matrix and
    * translation.
    *
    * @param rotationMatrix the rotation matrix used to initialize the quaternion of this transform.
    *           Not modified.
    * @param translation the tuple used to initialize the translation part of this transform. Not
    *           modified.
    */
   public QuaternionBasedTransform(RotationMatrixReadOnly rotationMatrix, Tuple3DReadOnly translation)
   {
      set(rotationMatrix, translation);
   }

   /**
    * Creates a new quaternion-based transform and initializes it to the given orientation and
    * translation.
    *
    * @param orientation the orientation used to initialize the quaternion of this transform. Not
    *           modified.
    * @param translation the tuple used to initialize the translation part of this transform. Not
    *           modified.
    */
   public QuaternionBasedTransform(Orientation3DReadOnly orientation, Tuple3DReadOnly translation)
   {
      set(orientation, translation);
   }

   /**
    * Resets this quaternion-based transform to represent a zero rotation and zero translation.
    * <p>
    * When set to zero, this transform has no effect when transforming a geometry object.
    * </p>
    */
   public void setIdentity()
   {
      quaternion.setToZero();
      translationVector.setToZero();
   }

   /**
    * Sets the 7 components of this transform.
    * <p>
    * The quaternion part will be normalized.
    * </p>
    *
    * @param qx the x-component of the quaternion's vector part.
    * @param qy the y-component of the quaternion's vector part.
    * @param qz the z-component of the quaternion's vector part.
    * @param qs the scalar component of the quaternion.
    * @param x the x-component of the translation.
    * @param y the y-component of the translation.
    * @param z the z-component of the translation.
    */
   public void set(double qx, double qy, double qz, double qs, double x, double y, double z)
   {
      quaternion.set(qx, qy, qz, qs);
      translationVector.set(x, y, z);
   }

   /**
    * Sets the 7 components of this transform.
    * <p>
    * Prefer using the method {@link #set(double, double, double, double, double, double, double)} as
    * it normalizes the quaternion part.
    * </p>
    *
    * @param qx the x-component of the quaternion's vector part.
    * @param qy the y-component of the quaternion's vector part.
    * @param qz the z-component of the quaternion's vector part.
    * @param qs the scalar component of the quaternion.
    * @param x the x-component of the translation.
    * @param y the y-component of the translation.
    * @param z the z-component of the translation.
    */
   public void setUnsafe(double qx, double qy, double qz, double qs, double x, double y, double z)
   {
      quaternion.setUnsafe(qx, qy, qz, qs);
      translationVector.set(x, y, z);
   }

   /**
    * Sets this quaternion-based transform to the given {@code other}.
    *
    * @param other the other quaternion-based transform. Not modified.
    */
   @Override
   public void set(QuaternionBasedTransform other)
   {
      set((RigidBodyTransformReadOnly) other);
   }

   /**
    * Sets the value of this transform's 7 components.
    * <p>
    * The quaternion q is set as follows:
    *
    * <pre>
    *     / qx = matrix.get(0, 0) \
    * q = | qy = matrix.get(1, 0) |
    *     | qz = matrix.get(2, 0) |
    *     \ qs = matrix.get(3, 0) /
    * </pre>
    *
    * The translation part T is set as follows:
    *
    * <pre>
    *     / matrix.get(4, 0) \
    * T = | matrix.get(5, 0) |
    *     \ matrix.get(6, 0) /
    * </pre>
    * </p>
    *
    * @param matrix the column vector containing the values of the 7 components of this transform. Not
    *           modified.
    */
   public void set(DenseMatrix64F matrix)
   {
      quaternion.set(matrix);
      translationVector.set(4, matrix);
   }

   /**
    * Sets the value of this transform's 7 components.
    * <p>
    * The quaternion q is set as follows:
    *
    * <pre>
    *     / qx = array[0] \
    * q = | qy = array[1] |
    *     | qz = array[2] |
    *     \ qs = array[3] /
    * </pre>
    *
    * The translation part T is set as follows:
    *
    * <pre>
    *     / array[4] \
    * T = | array[5] |
    *     \ array[6] /
    * </pre>
    * </p>
    *
    * @param array the array containing the values of the 7 components of this transform. Not modified.
    */
   public void set(double[] array)
   {
      quaternion.set(array);
      translationVector.set(4, array);
   }

   /**
    * Performs a linear interpolation from this transform to {@code other} given the percentage
    * {@code alpha}.
    * <p>
    * The interpolation is done on the rotation part and translation part separately.
    * </p>
    * <p>
    * this = (1.0 - alpha) * this + alpha * other
    * </p>
    *
    * @param other the other transform used for the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not modifying
    *           this transform, while a value of 1 is equivalent to setting this transform to
    *           {@code other}.
    */
   public void interpolate(QuaternionBasedTransform other, double alpha)
   {
      interpolate(this, other, alpha);
   }

   /**
    * Performs a linear interpolation from {@code firstTransform} to {@code secondTransform} given the
    * percentage {@code alpha}.
    * <p>
    * The interpolation is done on the rotation part and translation part separately.
    * </p>
    * <p>
    * this = (1.0 - alpha) * firstTransform + alpha * secondTransform
    * </p>
    *
    * @param firstTransform the first transform used in the interpolation. Not modified.
    * @param secondTransform the second transform used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           this transform to {@code tuple1}, while a value of 1 is equivalent to setting this
    *           transform to {@code tuple2}.
    */
   public void interpolate(QuaternionBasedTransform firstTransform, QuaternionBasedTransform secondTransform, double alpha)
   {
      quaternion.interpolate(firstTransform.getRotation(), secondTransform.getRotation(), alpha);
      translationVector.interpolate(firstTransform.getTranslation(), secondTransform.getTranslation(), alpha);
   }

   /**
    * Packs this quaternion-based transform in a column vector.
    *
    * <pre>
    *     / qx \
    *     | qy |
    *     | qz |
    * H = | qs |
    *     | tx |
    *     | ty |
    *     \ tz /
    * </pre>
    *
    * where (qx, qy, qz, qs) is the quaternion and (tx, ty, tz) the translation of this transform.
    *
    * @param matrixToPack the column vector in which this transform is stored. Modified.
    */
   public void get(DenseMatrix64F matrixToPack)
   {
      quaternion.get(matrixToPack);
      translationVector.get(4, matrixToPack);
   }

   /**
    * Packs this quaternion-based transform in a column vector.
    *
    * <pre>
    *     / qx \
    *     | qy |
    *     | qz |
    * H = | qs |
    *     | tx |
    *     | ty |
    *     \ tz /
    * </pre>
    *
    * where (qx, qy, qz, qs) is the quaternion and (tx, ty, tz) the translation of this transform.
    *
    * @param startRow the first row index to start writing in {@code matrixToPack}.
    * @param column the column index to write in {@code matrixToPack}.
    * @param matrixToPack the column vector in which this transform is stored. Modified.
    */
   public void get(int startRow, int column, DenseMatrix64F matrixToPack)
   {
      quaternion.get(startRow, column, matrixToPack);
      translationVector.get(startRow + 4, column, matrixToPack);
   }

   /**
    * Packs this quaternion-based transform in an array.
    *
    * <pre>
    *     / qx \
    *     | qy |
    *     | qz |
    * H = | qs |
    *     | tx |
    *     | ty |
    *     \ tz /
    * </pre>
    *
    * where (qx, qy, qz, qs) is the quaternion and (tx, ty, tz) the translation of this transform.
    * 
    * 
    * @param transformArrayToPack the array in which this transform is packed. Modified.
    */
   public void get(double[] transformArrayToPack)
   {
      quaternion.get(transformArrayToPack);
      translationVector.get(4, transformArrayToPack);
   }

   /**
    * Get the read-only reference to the quaternion of this transform.
    *
    * @return the quaternion of this transform.
    * @deprecated Use {@link #getRotation()} instead.
    */
   public QuaternionReadOnly getQuaternion()
   {
      return getRotation();
   }

   @Override
   public QuaternionBasics getRotation()
   {
      return quaternion;
   }

   /**
    * Gets the read-only reference of the translation part of this affine transform.
    *
    * @return the translation part of this transform.
    * @deprecated Use {@link #getTranslation()} instead.
    */
   public Vector3DReadOnly getTranslationVector()
   {
      return getTranslation();
   }

   @Override
   public Vector3DBasics getTranslation()
   {
      return translationVector;
   }

   /**
    * Tests separately and on a per component basis if the rotation part and the translation part of
    * this transform and {@code other} are equal to an {@code epsilon}.
    *
    * @param other the other quaternion-based transform to compare against this. Not modified.
    */
   @Override
   public boolean epsilonEquals(QuaternionBasedTransform other, double epsilon)
   {
      return quaternion.epsilonEquals(other.quaternion, epsilon) && translationVector.epsilonEquals(other.translationVector, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(QuaternionBasedTransform)}, it returns {@code false} otherwise or if the
    * {@code object} is {@code null}.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof QuaternionBasedTransform)
         return equals((QuaternionBasedTransform) object);
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
   public boolean equals(QuaternionBasedTransform other)
   {
      if (other == this)
         return true;
      else if (other == null)
         return false;
      else
         return quaternion.equals(other.quaternion) && translationVector.equals(other.translationVector);
   }

   /**
    * Two quaternion based transforms are considered geometrically equal if both the rotation-scale
    * matrices and translation vectors are equal.
    *
    * @param other the other quaternion based transform to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two quaternion based transforms are equal, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(QuaternionBasedTransform other, double epsilon)
   {
      return other.quaternion.geometricallyEquals(quaternion, epsilon) && other.translationVector.geometricallyEquals(translationVector, epsilon);
   }

   /**
    * Provides a {@code String} representation of this transform as follows: <br>
    * Quaternion: (qx, qy, qz, qs) <br>
    * Translation: ( x, y, z)
    *
    * @return the {@code String} representing this transform.
    */
   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getQuaternionBasedTransformString(this);
   }

   @Override
   public int hashCode()
   {
      long bits = EuclidHashCodeTools.addToHashCode(quaternion.hashCode(), translationVector.hashCode());
      return EuclidHashCodeTools.toIntHashCode(bits);
   }
}
