package us.ihmc.euclid.referenceFrame;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameMatrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRotationMatrixBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * A {@code RotationMatrix} is a 3-by-3 matrix expressed in a given reference frame.
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
public class FrameRotationMatrix implements FrameRotationMatrixBasics
{
   /** The reference frame is which this rotation matrix is currently expressed. */
   private ReferenceFrame referenceFrame;
   /** The rotation matrix holding the current components of this frame rotation matrix. */
   private final RotationMatrix rotationMatrix = new RotationMatrix();

   /**
    * Create a new rotation matrix and initializes to identity and its reference frame to
    * {@link ReferenceFrame#getWorldFrame()}.
    */
   public FrameRotationMatrix()
   {
      setToZero(ReferenceFrame.getWorldFrame());
   }

   /**
    * Create a new rotation matrix and initializes to identity and its reference frame to
    * {@code referenceFrame}.
    *
    * @param referenceFrame the initial frame for this frame rotation matrix.
    */
   public FrameRotationMatrix(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   /**
    * Creates a new rotation matrix and initializes it from the given 9 coefficients and reference
    * frame.
    *
    * @param referenceFrame the initial frame for this frame rotation matrix.
    * @param m00            the 1st row 1st column coefficient for this matrix.
    * @param m01            the 1st row 2nd column coefficient for this matrix.
    * @param m02            the 1st row 3rd column coefficient for this matrix.
    * @param m10            the 2nd row 1st column coefficient for this matrix.
    * @param m11            the 2nd row 2nd column coefficient for this matrix.
    * @param m12            the 2nd row 3rd column coefficient for this matrix.
    * @param m20            the 3rd row 1st column coefficient for this matrix.
    * @param m21            the 3rd row 2nd column coefficient for this matrix.
    * @param m22            the 3rd row 3rd column coefficient for this matrix.
    * @throws NotARotationMatrixException if the resulting matrix is not a rotation matrix.
    */
   public FrameRotationMatrix(ReferenceFrame referenceFrame,
                              double m00,
                              double m01,
                              double m02,
                              double m10,
                              double m11,
                              double m12,
                              double m20,
                              double m21,
                              double m22)
   {
      setIncludingFrame(referenceFrame, m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Creates a new rotation matrix and initializes it from the given array and reference frame.
    *
    * <pre>
    *        / rotationMatrixArray[0]  rotationMatrixArray[1]  rotationMatrixArray[2] \
    * this = | rotationMatrixArray[3]  rotationMatrixArray[4]  rotationMatrixArray[5] |
    *        \ rotationMatrixArray[6]  rotationMatrixArray[7]  rotationMatrixArray[8] /
    * </pre>
    *
    * @param referenceFrame      the initial frame for this frame rotation matrix.
    * @param rotationMatrixArray the array containing the values for this matrix. Not modified.
    * @throws NotARotationMatrixException if the resulting matrix is not a rotation matrix.
    */
   public FrameRotationMatrix(ReferenceFrame referenceFrame, double[] rotationMatrixArray)
   {
      setIncludingFrame(referenceFrame, rotationMatrixArray);
   }

   /**
    * Creates a new rotation matrix that is the same as {@code rotationMatrix} and initializes its
    * reference frame.
    *
    * @param referenceFrame the initial frame for this frame rotation matrix.
    * @param rotationMatrix the other 3D matrix to copy the values from. Not modified.
    * @throws NotARotationMatrixException if the resulting matrix is not a rotation matrix.
    */
   public FrameRotationMatrix(ReferenceFrame referenceFrame, DMatrix rotationMatrix)
   {
      setIncludingFrame(referenceFrame, rotationMatrix);
   }

   /**
    * Creates a new rotation matrix that is the same as {@code rotationMatrix} and initializes its
    * reference frame.
    *
    * @param referenceFrame the initial frame for this frame rotation matrix.
    * @param rotationMatrix the other 3D matrix to copy the values from. Not modified.
    * @throws NotARotationMatrixException if the resulting matrix is not a rotation matrix.
    */
   public FrameRotationMatrix(ReferenceFrame referenceFrame, Matrix3DReadOnly rotationMatrix)
   {
      setIncludingFrame(referenceFrame, rotationMatrix);
   }

   /**
    * Creates a new rotation matrix that is the same as {@code rotationMatrixReadOnly} and initializes
    * its reference frame.
    *
    * @param referenceFrame         the initial frame for this frame rotation matrix.
    * @param rotationMatrixReadOnly the other 3D matrix to copy the values from. Not modified.
    */
   public FrameRotationMatrix(ReferenceFrame referenceFrame, RotationMatrixReadOnly rotationMatrixReadOnly)
   {
      setIncludingFrame(referenceFrame, rotationMatrixReadOnly);
   }

   /**
    * Creates a new rotation matrix that represents the same orientation as the given one and
    * initializes its reference frame.
    *
    * @param referenceFrame the initial frame for this frame rotation matrix.
    * @param orientation    the orientation used to initialize this rotation matrix. Not modified.
    */
   public FrameRotationMatrix(ReferenceFrame referenceFrame, Orientation3DReadOnly orientation)
   {
      setIncludingFrame(referenceFrame, orientation);
   }

   /**
    * Creates a new rotation matrix representing the same orientation as the given rotation vector
    * {@code rotationVector} and initializes its reference frame.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param referenceFrame the initial frame for this frame rotation matrix.
    * @param rotationVector the rotation vector used to initialize this rotation matrix. Not modified.
    */
   public FrameRotationMatrix(ReferenceFrame referenceFrame, Vector3DReadOnly rotationVector)
   {
      setRotationVectorIncludingFrame(referenceFrame, rotationVector);
   }

   /**
    * Creates a new rotation matrix and initializes such that it represents the same orientation as the
    * given yaw-pitch-roll {@code yaw}, {@code pitch}, and {@code roll}.
    *
    * @param referenceFrame the initial frame for this frame rotation matrix.
    * @param yaw            the angle to rotate about the z-axis.
    * @param pitch          the angle to rotate about the y-axis.
    * @param roll           the angle to rotate about the x-axis.
    */
   public FrameRotationMatrix(ReferenceFrame referenceFrame, double yaw, double pitch, double roll)
   {
      setYawPitchRollIncludingFrame(referenceFrame, yaw, pitch, roll);
   }

   /**
    * Creates a new rotation matrix that is the same as {@code rotationMatrix} and initializes its
    * reference frame.
    *
    * @param rotationMatrix the other 3D matrix to copy the values from. Not modified.
    * @throws NotARotationMatrixException if the resulting matrix is not a rotation matrix.
    */
   public FrameRotationMatrix(FrameMatrix3DReadOnly rotationMatrix)
   {
      setIncludingFrame(rotationMatrix);
   }

   /**
    * Creates a new rotation matrix that is the same as {@code other}.
    *
    * @param other the other 3D matrix to copy the values from. Not modified.
    */
   public FrameRotationMatrix(FrameRotationMatrixReadOnly other)
   {
      setIncludingFrame(other);
   }

   /**
    * Creates a new rotation matrix that represents the same orientation as the given one.
    *
    * @param orientation the orientation used to initialize this rotation matrix. Not modified.
    */
   public FrameRotationMatrix(FrameOrientation3DReadOnly orientation)
   {
      setIncludingFrame(orientation);
   }

   /**
    * Creates a new rotation matrix representing the same orientation as the given rotation vector
    * {@code rotationVector} and initializes its reference frame.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param rotationVector the rotation vector used to initialize this rotation matrix. Not modified.
    */
   public FrameRotationMatrix(FrameVector3DReadOnly rotationVector)
   {
      setRotationVectorIncludingFrame(rotationVector);
   }

   /** {@inheritDoc} */
   @Override
   public void set(RotationMatrixReadOnly other)
   {
      rotationMatrix.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public void setUnsafe(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /** {@inheritDoc} */
   @Override
   public void setIdentity()
   {
      rotationMatrix.setIdentity();
   }

   /** {@inheritDoc} */
   @Override
   public void setToNaN()
   {
      rotationMatrix.setToNaN();
   }

   /** {@inheritDoc} */
   @Override
   public void normalize()
   {
      rotationMatrix.normalize();
   }

   /** {@inheritDoc} */
   @Override
   public boolean isIdentity()
   {
      return rotationMatrix.isIdentity();
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
      rotationMatrix.markAsDirty();
   }

   /** {@inheritDoc} */
   @Override
   public void transpose()
   {
      rotationMatrix.transpose();
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public boolean isDirty()
   {
      return rotationMatrix.isDirty();
   }

   /** {@inheritDoc} */
   @Override
   public double getM00()
   {
      return rotationMatrix.getM00();
   }

   /** {@inheritDoc} */
   @Override
   public double getM01()
   {
      return rotationMatrix.getM01();
   }

   /** {@inheritDoc} */
   @Override
   public double getM02()
   {
      return rotationMatrix.getM02();
   }

   /** {@inheritDoc} */
   @Override
   public double getM10()
   {
      return rotationMatrix.getM10();
   }

   /** {@inheritDoc} */
   @Override
   public double getM11()
   {
      return rotationMatrix.getM11();
   }

   /** {@inheritDoc} */
   @Override
   public double getM12()
   {
      return rotationMatrix.getM12();
   }

   /** {@inheritDoc} */
   @Override
   public double getM20()
   {
      return rotationMatrix.getM20();
   }

   /** {@inheritDoc} */
   @Override
   public double getM21()
   {
      return rotationMatrix.getM21();
   }

   /** {@inheritDoc} */
   @Override
   public double getM22()
   {
      return rotationMatrix.getM22();
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(FrameMatrix3DReadOnly)}, it returns {@code false} otherwise or if the
    * {@code object} is {@code null}.
    * <p>
    * If the two matrices have different frames, this method returns {@code false}.
    * </p>
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameMatrix3DReadOnly)
         return FrameRotationMatrixBasics.super.equals((FrameMatrix3DReadOnly) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this matrix as follows:
    *
    * <pre>
    * /-0.576, -0.784,  0.949 \
    * | 0.649, -0.542, -0.941 |
    * \-0.486, -0.502, -0.619 /
    * worldFrame
    * </pre>
    *
    * @return the {@code String} representing this matrix.
    */
   @Override
   public String toString()
   {
      return EuclidFrameIOTools.getFrameMatrix3DString(this);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this matrix.
    *
    * @return the hash code value for this matrix.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(rotationMatrix, referenceFrame);
   }
}
