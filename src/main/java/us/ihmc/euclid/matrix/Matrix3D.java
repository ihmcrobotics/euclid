package us.ihmc.euclid.matrix;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;

/**
 * A {@code Matrix3D} is a 3-by-3 matrix used for general linear applications.
 * <p>
 * This version of 3D matrix uses double precision fields to save the value of each component. It is
 * meant for garbage free usage.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class Matrix3D implements Matrix3DBasics
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

   /**
    * Creates a new 3D matrix with all its coefficients set to zero.
    */
   public Matrix3D()
   {
   }

   /**
    * Creates a new 3D matrix and initializes it from the given array.
    *
    * <pre>
    *        / matrixArray[0]  matrixArray[1]  matrixArray[2] \
    * this = | matrixArray[3]  matrixArray[4]  matrixArray[5] |
    *        \ matrixArray[6]  matrixArray[7]  matrixArray[8] /
    * </pre>
    *
    * @param matrixArray the array containing the values for this matrix. Not modified.
    */
   public Matrix3D(double[] matrixArray)
   {
      set(matrixArray);
   }

   /**
    * Create a new 3D matrix and initializes it from the given matrix.
    *
    * @param matrix the matrix containing the values for this matrix. Not modified.
    */
   public Matrix3D(DMatrix matrix)
   {
      set(matrix);
   }

   /**
    * Creates a new 3D matrix and initializes it from the given 9 coefficients.
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
    */
   public Matrix3D(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Creates a new 3D matrix that is the same as {@code other}.
    *
    * @param other the other 3D matrix to copy the values from. Not modified.
    */
   public Matrix3D(Matrix3DReadOnly other)
   {
      set(other);
   }

   @Override
   public void set(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
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
   }

   /** {@inheritDoc} */
   @Override
   public void setM00(double m00)
   {
      this.m00 = m00;
   }

   /** {@inheritDoc} */
   @Override
   public void setM01(double m01)
   {
      this.m01 = m01;
   }

   /** {@inheritDoc} */
   @Override
   public void setM02(double m02)
   {
      this.m02 = m02;
   }

   /** {@inheritDoc} */
   @Override
   public void setM10(double m10)
   {
      this.m10 = m10;
   }

   /** {@inheritDoc} */
   @Override
   public void setM11(double m11)
   {
      this.m11 = m11;
   }

   /** {@inheritDoc} */
   @Override
   public void setM12(double m12)
   {
      this.m12 = m12;
   }

   /** {@inheritDoc} */
   @Override
   public void setM20(double m20)
   {
      this.m20 = m20;
   }

   /** {@inheritDoc} */
   @Override
   public void setM21(double m21)
   {
      this.m21 = m21;
   }

   /** {@inheritDoc} */
   @Override
   public void setM22(double m22)
   {
      this.m22 = m22;
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
    * Provides a {@code String} representation of this matrix as follows:
    *
    * <pre>
    * /m00, m01, m02 \
    * |m10, m11, m12 |
    * \m20, m21, m22 /
    * </pre>
    *
    * @return the {@code String} representing this matrix.
    */
   @Override
   public String toString()
   {
      return Matrix3DBasics.super.toString(EuclidCoreIOTools.DEFAULT_FORMAT);
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

   /**
    * Two 3D matrices are considered geometrically equal if they are epsilon equal.
    * <p>
    * This method is equivalent to {@link #epsilonEquals(Matrix3DReadOnly, double)}.
    * </p>
    *
    * @param object  the object to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two matrices are equal, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(Object object, double epsilon)
   {
      return Matrix3DBasics.super.epsilonEquals(object, epsilon);
   }
}
