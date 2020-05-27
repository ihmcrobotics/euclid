package us.ihmc.euclid.matrix;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tools.Matrix3DTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * A {@code RotationMatrix} is a 3-by-3 matrix used to represent 3d orientations.
 * <p>
 * A rotation matrix has to comply to several constraints:
 * <ul>
 * <li>each column of the matrix represents a unitary vector,
 * <li>each row of the matrix represents a unitary vector,
 * <li>every pair of columns of the matrix represents two orthogonal vectors,
 * <li>every pair of rows of the matrix represents two orthogonal vectors,
 * <li>the matrix determinant is equal to {@code 1}.
 * </ul>
 * A rotation matrix has the nice property <i>R<sup>T</sup> = R<sup>-1</sup></i>.
 * </p>
 * <p>
 * A best effort has been put in the interface of {@code RotationMatrix} to maximize the use of the
 * inherent properties of a rotation matrix and to minimize manipulation errors resulting in an
 * improper rotation matrix.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class RotationMatrix implements RotationMatrixBasics, GeometryObject<RotationMatrix>
{
   /** The 1st row 1st column coefficient of this matrix. */
   private double m00;
   /** The 1st row 2nd column coefficient of this matrix. */
   private double m01;
   /** The 1st row 3rd column coefficient of this matrix. */
   private double m02;
   /** The 2nd row 1st column coefficient of this matrix. */
   private double m10;
   /** The 2nd row 2nd column coefficient of this matrix. */
   private double m11;
   /** The 2nd row 3rd column coefficient of this matrix. */
   private double m12;
   /** The 3rd row 1st column coefficient of this matrix. */
   private double m20;
   /** The 3rd row 2nd column coefficient of this matrix. */
   private double m21;
   /** The 3rd row 3rd column coefficient of this matrix. */
   private double m22;

   private boolean dirty = false;
   private boolean isIdentity = true;

   /**
    * Create a new rotation matrix initialized to identity.
    */
   public RotationMatrix()
   {
      setIdentity();
   }

   /**
    * Creates a new rotation matrix and initializes it from the given 9 coefficients.
    *
    * @param m00 the 1st row 1st column coefficient for this matrix.
    * @param m01 the 1st row 2nd column coefficient for this matrix.
    * @param m02 the 1st row 3rd column coefficient for this matrix.
    * @param m10 the 2nd row 1st column coefficient for this matrix.
    * @param m11 the 2nd row 2nd column coefficient for this matrix.
    * @param m12 the 2nd row 3rd column coefficient for this matrix.
    * @param m20 the 3rd row 1st column coefficient for this matrix.
    * @param m21 the 3rd row 2nd column coefficient for this matrix.
    * @param m22 the 3rd row 3rd column coefficient for this matrix.
    * @throws NotARotationMatrixException if the resulting matrix is not a rotation matrix.
    */
   public RotationMatrix(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Creates a new rotation matrix and initializes it from the given array.
    *
    * <pre>
    *        / rotationMatrixArray[0]  rotationMatrixArray[1]  rotationMatrixArray[2] \
    * this = | rotationMatrixArray[3]  rotationMatrixArray[4]  rotationMatrixArray[5] |
    *        \ rotationMatrixArray[6]  rotationMatrixArray[7]  rotationMatrixArray[8] /
    * </pre>
    *
    * @param rotationMatrixArray the array containing the values for this matrix. Not modified.
    * @throws NotARotationMatrixException if the resulting matrix is not a rotation matrix.
    */
   public RotationMatrix(double[] rotationMatrixArray)
   {
      set(rotationMatrixArray);
   }

   /**
    * Creates a new rotation matrix that is the same as {@code rotationMatrix}.
    *
    * @param rotationMatrix the other 3D matrix to copy the values from. Not modified.
    * @throws NotARotationMatrixException if the resulting matrix is not a rotation matrix.
    */
   public RotationMatrix(DMatrix rotationMatrix)
   {
      set(rotationMatrix);
   }

   /**
    * Creates a new rotation matrix that is the same as {@code rotationMatrix}.
    *
    * @param rotationMatrix the other 3D matrix to copy the values from. Not modified.
    * @throws NotARotationMatrixException if the resulting matrix is not a rotation matrix.
    */
   public RotationMatrix(Matrix3DReadOnly rotationMatrix)
   {
      set(rotationMatrix);
   }

   /**
    * Creates a new rotation matrix that is the same as {@code other}.
    *
    * @param other the other 3D matrix to copy the values from. Not modified.
    */
   public RotationMatrix(RotationMatrixReadOnly other)
   {
      set(other);
   }

   /**
    * Creates a new rotation matrix that represents the same orientation as the given one.
    *
    * @param orientation the orientation used to initialize this rotation matrix. Not modified.
    */
   public RotationMatrix(Orientation3DReadOnly orientation)
   {
      set(orientation);
   }

   /**
    * Creates a new rotation matrix representing the same orientation as the given rotation vector
    * {@code rotationVector}.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param rotationVector the rotation vector used to initialize this rotation matrix. Not modified.
    */
   public RotationMatrix(Vector3DReadOnly rotationVector)
   {
      setRotationVector(rotationVector);
   }

   /**
    * Creates a new rotation matrix and initializes such that it represents the same orientation as the
    * given yaw-pitch-roll {@code yaw}, {@code pitch}, and {@code roll}.
    *
    * @param yaw   the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll  the angle to rotate about the x-axis.
    */
   public RotationMatrix(double yaw, double pitch, double roll)
   {
      setYawPitchRoll(yaw, pitch, roll);
   }

   @Override
   public void setIdentity()
   {
      setUnsafe(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
      isIdentity = true;
      dirty = false;
   }

   /**
    * {@inheritDoc}
    * <p>
    * Note that this rotation matrix becomes invalid and has to be updated before being usable again.
    * </p>
    */
   @Override
   public void setToNaN()
   {
      setUnsafe(Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN);
      isIdentity = false;
      dirty = false;
   }

   /**
    * Orthonormalization of the rotation matrix using the
    * <a href="https://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process"> Gram-Schmidt method</a>.
    *
    * @throws NotARotationMatrixException if the orthonormalization failed.
    */
   @Override
   public void normalize()
   {
      if (isIdentity())
         setIdentity();
      else
         Matrix3DTools.normalize(this);
   }

   /**
    * {@inheritDoc}
    * <p>
    * The state of this rotation matrix is saved for performance improvement. It updated only when this
    * matrix is marked as dirty which can be set by calling {@link #markAsDirty()}. The matrix is
    * marked as dirty whenever its elements are updated.
    * </p>
    */
   @Override
   public boolean isIdentity()
   {
      if (dirty)
         isIdentity = RotationMatrixBasics.super.isIdentity();
      return isIdentity;
   }

   /**
    * Marks this rotation matrix as dirty.
    * <p>
    * When a rotation matrix is marked as dirty, {@link #isIdentity()} will perform a thorough test to
    * update the state of this matrix.
    * </p>
    */
   public void markAsDirty()
   {
      dirty = true;
   }

   /**
    * Transposes this matrix: m = m<sup>T</sup>.
    */
   @Override
   public void transpose()
   {
      double temp;

      temp = m01;
      m01 = m10;
      m10 = temp;

      temp = m02;
      m02 = m20;
      m20 = temp;

      temp = m12;
      m12 = m21;
      m21 = temp;
   }

   /**
    * Sets the 9 coefficients of this rotation matrix without performing any checks on the data
    * provided.
    * <p>
    * This method is meant for internal usage. Prefer using
    * {@link #set(double, double, double, double, double, double, double, double, double)} or
    * {@link #setAndNormalize(double, double, double, double, double, double, double, double, double)}.
    * </p>
    *
    * @param m00 the new 1st row 1st column coefficient for this matrix.
    * @param m01 the new 1st row 2nd column coefficient for this matrix.
    * @param m02 the new 1st row 3rd column coefficient for this matrix.
    * @param m10 the new 2nd row 1st column coefficient for this matrix.
    * @param m11 the new 2nd row 2nd column coefficient for this matrix.
    * @param m12 the new 2nd row 3rd column coefficient for this matrix.
    * @param m20 the new 3rd row 1st column coefficient for this matrix.
    * @param m21 the new 3rd row 2nd column coefficient for this matrix.
    * @param m22 the new 3rd row 3rd column coefficient for this matrix.
    */
   @Override
   public void setUnsafe(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      this.m00 = m00;
      this.m01 = m01;
      this.m02 = m02;

      this.m10 = m10;
      this.m11 = m11;
      this.m12 = m12;

      this.m20 = m20;
      this.m21 = m21;
      this.m22 = m22;

      markAsDirty();
   }

   /**
    * Sets this rotation matrix to equal the given one {@code other}.
    *
    * @param other the other rotation matrix to copy the values from. Not modified.
    */
   @Override
   public void set(RotationMatrix other)
   {
      set((RotationMatrixReadOnly) other);
   }

   /**
    * Sets this rotation matrix to equal the given one {@code other}.
    *
    * @param other the other rotation matrix to copy the values from. Not modified.
    */
   @Override
   public void set(RotationMatrixReadOnly other)
   {
      m00 = other.getM00();
      m01 = other.getM01();
      m02 = other.getM02();
      m10 = other.getM10();
      m11 = other.getM11();
      m12 = other.getM12();
      m20 = other.getM20();
      m21 = other.getM21();
      m22 = other.getM22();

      if (other.isDirty())
      {
         markAsDirty();
      }
      else
      {
         dirty = false;
         isIdentity = other.isIdentity();
      }
   }

   @Override
   public boolean isDirty()
   {
      return dirty;
   }

   /** {@inheritDoc} */
   @Override
   public double getM00()
   {
      return m00;
   }

   /** {@inheritDoc} */
   @Override
   public double getM01()
   {
      return m01;
   }

   /** {@inheritDoc} */
   @Override
   public double getM02()
   {
      return m02;
   }

   /** {@inheritDoc} */
   @Override
   public double getM10()
   {
      return m10;
   }

   /** {@inheritDoc} */
   @Override
   public double getM11()
   {
      return m11;
   }

   /** {@inheritDoc} */
   @Override
   public double getM12()
   {
      return m12;
   }

   /** {@inheritDoc} */
   @Override
   public double getM20()
   {
      return m20;
   }

   /** {@inheritDoc} */
   @Override
   public double getM21()
   {
      return m21;
   }

   /** {@inheritDoc} */
   @Override
   public double getM22()
   {
      return m22;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(Matrix3DReadOnly)}, it returns {@code false} otherwise or if the {@code object} is
    * {@code null}.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Matrix3DReadOnly)
         return equals((Matrix3DReadOnly) object);
      else
         return false;
   }

   /**
    * Tests on a per coefficient basis if this matrix is equal to the given {@code other} to an
    * {@code epsilon}.
    *
    * @param other   the other matrix to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two matrices are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(RotationMatrix other, double epsilon)
   {
      return RotationMatrixBasics.super.epsilonEquals(other, epsilon);
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
    * @param other   the other rotation matrix to compare against this. Not modified.
    * @param epsilon the maximum angle between the two rotation matrices to be considered equal.
    * @return {@code true} if the two rotation matrices represent the same geometry, {@code false}
    *         otherwise.
    */
   @Override
   public boolean geometricallyEquals(RotationMatrix other, double epsilon)
   {
      return RotationMatrixBasics.super.geometricallyEquals(other, epsilon);
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
    * @return the {@code String} representing this matrix.
    */
   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getMatrix3DString(this);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this matrix.
    *
    * @return the hash code value for this matrix.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }
}
