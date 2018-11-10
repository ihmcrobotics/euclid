package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameMatrix3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameMatrix3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;

/**
 * {@code FrameMatrix3D} is a 3D matrix expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Matrix3DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FrameMatrix3D}. This allows, for instance, to enforce, at runtime, that operations on
 * points occur in the same coordinate system. Also, via the method
 * {@link #changeFrame(ReferenceFrame)}, one can easily calculates the value of a point in different
 * reference frames.
 * </p>
 * <p>
 * Because a {@code FrameMatrix3D} extends {@code Matrix3DBasics}, it is compatible with methods
 * only requiring {@code Matrix3DBasics}. However, these methods do NOT assert that the operation
 * occur in the proper coordinate system. Use this feature carefully and always prefer using methods
 * requiring {@code FrameMatrix3D}.
 * </p>
 */
public class FrameMatrix3D implements FrameMatrix3DBasics, GeometryObject<FrameMatrix3D>
{
   /** The reference frame is which this point is currently expressed. */
   private ReferenceFrame referenceFrame;
   /** The matrix holding the current value of the coefficients. */
   private final Matrix3D matrix3D = new Matrix3D();
   /** Transform used to reduce the number of operations when changing frame. */
   private final RigidBodyTransform transformToDesiredFrame = new RigidBodyTransform();

   /**
    * Creates a new 3D matrix with all its coefficients set to zero and its reference frame set to
    * {@link ReferenceFrame#getWorldFrame()}.
    */
   public FrameMatrix3D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new frame matrix and initializes its coefficients to zero and its reference frame to
    * {@code referenceFrame}.
    * 
    * @param referenceFrame the initial reference frame for this frame matrix.
    */
   public FrameMatrix3D(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   /**
    * Creates a new frame matrix that is the same as {@code other}.
    * 
    * @param other the other matrix to copy the values and reference frame from. Not modified.
    */
   public FrameMatrix3D(FrameMatrix3DReadOnly other)
   {
      setIncludingFrame(other);
   }

   /**
    * Creates a new frame matrix and initializes it to the given matrix and sets its reference frame
    * to {@code referenceFrame}.
    * 
    * @param referenceFrame the initial reference frame for this frame matrix.
    * @param matrix3DReadOnly the matrix to copy the coefficients from. Not modified.
    */
   public FrameMatrix3D(ReferenceFrame referenceFrame, Matrix3DReadOnly matrix3DReadOnly)
   {
      setIncludingFrame(referenceFrame, matrix3DReadOnly);
   }

   /**
    * Sets this frame matrix to {@code other}.
    *
    * @param other the other frame matrix to set this to. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   @Override
   public void set(FrameMatrix3D other)
   {
      FrameMatrix3DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void changeFrame(ReferenceFrame desiredFrame)
   {
      if (desiredFrame == referenceFrame)
         return;

      referenceFrame.getTransformToDesiredFrame(transformToDesiredFrame, desiredFrame);
      applyTransform(transformToDesiredFrame);
      setReferenceFrame(desiredFrame);
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public void setM00(double m00)
   {
      matrix3D.setM00(m00);
   }

   /** {@inheritDoc} */
   @Override
   public void setM01(double m01)
   {
      matrix3D.setM01(m01);
   }

   /** {@inheritDoc} */
   @Override
   public void setM02(double m02)
   {
      matrix3D.setM02(m02);
   }

   /** {@inheritDoc} */
   @Override
   public void setM10(double m10)
   {
      matrix3D.setM10(m10);
   }

   /** {@inheritDoc} */
   @Override
   public void setM11(double m11)
   {
      matrix3D.setM11(m11);
   }

   /** {@inheritDoc} */
   @Override
   public void setM12(double m12)
   {
      matrix3D.setM12(m12);
   }

   /** {@inheritDoc} */
   @Override
   public void setM20(double m20)
   {
      matrix3D.setM20(m20);
   }

   /** {@inheritDoc} */
   @Override
   public void setM21(double m21)
   {
      matrix3D.setM21(m21);
   }

   /** {@inheritDoc} */
   @Override
   public void setM22(double m22)
   {
      matrix3D.setM22(m22);
   }

   /**
    * Gets the reference frame in which this matrix is currently expressed.
    */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public double getM00()
   {
      return matrix3D.getM00();
   }

   /** {@inheritDoc} */
   @Override
   public double getM01()
   {
      return matrix3D.getM01();
   }

   /** {@inheritDoc} */
   @Override
   public double getM02()
   {
      return matrix3D.getM02();
   }

   /** {@inheritDoc} */
   @Override
   public double getM10()
   {
      return matrix3D.getM10();
   }

   /** {@inheritDoc} */
   @Override
   public double getM11()
   {
      return matrix3D.getM11();
   }

   /** {@inheritDoc} */
   @Override
   public double getM12()
   {
      return matrix3D.getM12();
   }

   /** {@inheritDoc} */
   @Override
   public double getM20()
   {
      return matrix3D.getM20();
   }

   /** {@inheritDoc} */
   @Override
   public double getM21()
   {
      return matrix3D.getM21();
   }

   /** {@inheritDoc} */
   @Override
   public double getM22()
   {
      return matrix3D.getM22();
   }

   /**
    * Two 3D matrices are considered geometrically equal if they are epsilon equal.
    * <p>
    * This method is equivalent to {@link #epsilonEquals(FrameMatrix3D, double)}.
    * </p>
    *
    * @param other the other matrix to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two matrices are equal, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(FrameMatrix3D other, double epsilon)
   {
      return epsilonEquals(other, epsilon);
   }

   /**
    * Tests on a per coefficient basis if this matrix is equal to the given {@code other} to an
    * {@code epsilon}.
    * <p>
    * If the two matrices have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the other matrix to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two matrices are equal and are expressed in the same reference
    *         frame, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(FrameMatrix3D other, double epsilon)
   {
      return FrameMatrix3DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method
    * returns {@link #equals(FrameMatrix3DReadOnly)}, it returns {@code false} otherwise.
    * <p>
    * If the two points have different frames, this method returns {@code false}.
    * </p>
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if the two points are exactly equal component-wise and are expressed in
    *         the same reference frame, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameMatrix3DReadOnly)
         return FrameMatrix3DBasics.super.equals((FrameMatrix3DReadOnly) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this matrix as follows: <br>
    * m00, m01, m02 <br>
    * m10, m11, m12 <br>
    * m20, m21, m22 <br>
    * worldFrame
    *
    * @return the {@code String} representing this frame matrix.
    */
   @Override
   public String toString()
   {
      return matrix3D.toString() + "\n" + referenceFrame;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this matrix.
    *
    * @return the hash code value for this matrix.
    */
   @Override
   public int hashCode()
   {
      return matrix3D.hashCode();
   }
}
