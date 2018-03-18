package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple4DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector4DBasics;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;

/**
 * {@code FrameVector4D} is a 4D vector expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Vector4DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FrameVector4D}. This allows, for instance, to enforce, at runtime, that operations on
 * vectors occur in the same coordinate system. Also, via the method
 * {@link #changeFrame(ReferenceFrame)}, one can easily calculates the value of a vector in
 * different reference frames.
 * </p>
 * <p>
 * Because a {@code FrameVector4D} extends {@code Vector4DBasics}, it is compatible with methods
 * only requiring {@code Vector4DBasics}. However, these methods do NOT assert that the operation
 * occur in the proper coordinate system. Use this feature carefully and always prefer using methods
 * requiring {@code FrameVector4D}.
 * </p>
 */
public class FrameVector4D implements FrameVector4DBasics, GeometryObject<FrameVector4D>
{
   /** The reference frame is which this vector is currently expressed. */
   private ReferenceFrame referenceFrame;
   /** The vector holding the current components of this frame vector. */
   private final Vector4D vector = new Vector4D();

   /**
    * Creates a new frame vector and initializes it components to zero and its reference frame to
    * {@link ReferenceFrame#getWorldFrame()}.
    */
   public FrameVector4D()
   {
      setToZero(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new frame vector and initializes it components to zero and its reference frame to the
    * {@code referenceFrame}.
    *
    * @param referenceFrame the initial frame for this frame vector.
    */
   public FrameVector4D(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   /**
    * Creates a new frame vector and initializes it with the given components and the given reference
    * frame.
    *
    * @param referenceFrame the initial frame for this frame vector.
    * @param x the x-component.
    * @param y the y-component.
    * @param z the z-component.
    * @param s the s-component.
    */
   public FrameVector4D(ReferenceFrame referenceFrame, double x, double y, double z, double s)
   {
      setIncludingFrame(referenceFrame, x, y, z, s);
   }

   /**
    * Creates a new frame vector and initializes its component {@code x}, {@code y}, {@code z},
    * {@code s} in order from the given array and initializes its reference frame.
    *
    * @param referenceFrame the initial frame for this frame vector.
    * @param vectorArray the array containing this vector's components. Not modified.
    */
   public FrameVector4D(ReferenceFrame referenceFrame, double[] vectorArray)
   {
      setIncludingFrame(referenceFrame, vectorArray);
   }

   /**
    * Creates a new frame vector and initializes it to {@code tuple4DReadOnly} and to the given
    * reference frame.
    *
    * @param referenceFrame the initial frame for this frame vector.
    * @param tuple4DReadOnly the tuple to copy the components from. Not modified.
    */
   public FrameVector4D(ReferenceFrame referenceFrame, Tuple4DReadOnly tuple4DReadOnly)
   {
      setIncludingFrame(referenceFrame, tuple4DReadOnly);
   }

   /**
    * Creates a new frame vector and initializes it to {@code other}.
    *
    * @param other the tuple to copy the components and reference frame from. Not modified.
    */
   public FrameVector4D(FrameTuple4DReadOnly other)
   {
      setIncludingFrame(other);
   }

   /**
    * Sets this frame vector to {@code other}.
    *
    * @param other the other frame vector to set this to. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   @Override
   public void set(FrameVector4D other)
   {
      FrameVector4DBasics.super.set(other);
   }

   /**
    * Sets the reference frame of this vector without updating or modifying its x, y, z, and s
    *
    * @param referenceFrame the new reference frame for this frame vector.
    */
   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   /**
    * Sets the x-component of this vector.
    *
    * @param x the x-component.
    */
   @Override
   public void setX(double x)
   {
      vector.setX(x);
   }

   /**
    * Sets the y-component of this vector.
    *
    * @param y the y-component.
    */
   @Override
   public void setY(double y)
   {
      vector.setY(y);
   }

   /**
    * Sets the z-component of this vector.
    *
    * @param z the z-component.
    */
   @Override
   public void setZ(double z)
   {
      vector.setZ(z);
   }

   /**
    * Sets the s-component of this vector.
    *
    * @param s the s-component.
    */
   @Override
   public void setS(double s)
   {
      vector.setS(s);
   }

   /**
    * Gets the reference frame in which this vector is currently expressed.
    */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   /**
    * Returns the value of the x-component of this vector.
    *
    * @return the x-component's value.
    */
   @Override
   public double getX()
   {
      return vector.getX();
   }

   /**
    * Returns the value of the y-component of this vector.
    *
    * @return the y-component's value.
    */
   @Override
   public double getY()
   {
      return vector.getY();
   }

   /**
    * Returns the value of the z-component of this vector.
    *
    * @return the z-component's value.
    */
   @Override
   public double getZ()
   {
      return vector.getZ();
   }

   /**
    * Returns the value of the s-component of this vector.
    *
    * @return the s-component's value.
    */
   @Override
   public double getS()
   {
      return vector.getS();
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(FrameTuple4DReadOnly)}, it returns {@code false} otherwise.
    * <p>
    * If the two vectors have different frames, this method returns {@code false}.
    * </p>
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if the two vectors are exactly equal component-wise and are expressed in the
    *         same reference frame, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((FrameTuple4DReadOnly) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Tests on a per component basis if this vector is equal to the given {@code other} to an
    * {@code epsilon}.
    * <p>
    * If the two vectors have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the other vector to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two vectors are equal and are expressed in the same reference frame,
    *         {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(FrameVector4D other, double epsilon)
   {
      return FrameVector4DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same vector 4D to an {@code epsilon}.
    * <p>
    * Two vectors are considered geometrically equal if they are at a distance of less than or equal to
    * {@code epsilon}.
    * </p>
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * {@code this.epsilonEquals(other, epsilon)} and vice versa.
    * </p>
    *
    * @param other the other vector 4D to compare against this. Not modified.
    * @param epsilon the maximum distance that the two vectors can be spaced and still considered
    *           equal.
    * @return {@code true} if the two vectors represent the same geometry, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   @Override
   public boolean geometricallyEquals(FrameVector4D other, double epsilon)
   {
      return FrameVector4DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Provides a {@code String} representation of this frame vector 4D as follows: (x, y, z,
    * s)-worldFrame.
    *
    * @return the {@code String} representing this frame vector 4D.
    */
   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getTuple4DString(this) + "-" + referenceFrame;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this frame vector
    * 4D.
    *
    * @return the hash code value for this frame vector 4D.
    */
   @Override
   public int hashCode()
   {
      return vector.hashCode();
   }
}
