package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * {@code FramePoint3D} is a 3D point expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Point3DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FramePoint3D}. This allows, for instance, to enforce, at runtime, that operations on
 * points occur in the same coordinate system. Also, via the method
 * {@link #changeFrame(ReferenceFrame)}, one can easily calculates the value of a point in different
 * reference frames.
 * </p>
 * <p>
 * Because a {@code FramePoint3D} extends {@code Point3DBasics}, it is compatible with methods only
 * requiring {@code Point3DBasics}. However, these methods do NOT assert that the operation occur in
 * the proper coordinate system. Use this feature carefully and always prefer using methods
 * requiring {@code FramePoint3D}.
 * </p>
 */
public class FramePoint3D implements FramePoint3DBasics, GeometryObject<FramePoint3D>
{
   /** The reference frame is which this point is currently expressed. */
   private ReferenceFrame referenceFrame;
   /** The point holding the current coordinates of this frame point. */
   private final Point3D point = new Point3D();

   /**
    * Creates a new frame point and initializes it coordinates to zero and its reference frame to
    * {@link ReferenceFrame#getWorldFrame()}.
    */
   public FramePoint3D()
   {
      setToZero(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new frame point and initializes it coordinates to zero and its reference frame to the
    * {@code referenceFrame}.
    *
    * @param referenceFrame the initial frame for this frame point.
    */
   public FramePoint3D(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   /**
    * Creates a new frame point and initializes it with the given coordinates and the given reference
    * frame.
    *
    * @param referenceFrame the initial frame for this frame point.
    * @param x the x-coordinate.
    * @param y the y-coordinate.
    * @param z the z-coordinate.
    */
   public FramePoint3D(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      setIncludingFrame(referenceFrame, x, y, z);
   }

   /**
    * Creates a new frame point and initializes its coordinates {@code x}, {@code y}, {@code z} in
    * order from the given array and initializes its reference frame.
    *
    * @param referenceFrame the initial frame for this frame point.
    * @param pointArray the array containing this point's coordinates. Not modified.
    */
   public FramePoint3D(ReferenceFrame referenceFrame, double[] pointArray)
   {
      setIncludingFrame(referenceFrame, pointArray);
   }

   /**
    * Creates a new frame point and initializes it to {@code tuple3DReadOnly} and to the given
    * reference frame.
    *
    * @param referenceFrame the initial frame for this frame point.
    * @param tuple3DReadOnly the tuple to copy the coordinates from. Not modified.
    */
   public FramePoint3D(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      setIncludingFrame(referenceFrame, tuple3DReadOnly);
   }

   /**
    * Creates a new frame point and initializes its x and y coordinate to {@code tuple2DReadOnly} and
    * to the given reference frame.
    *
    * @param referenceFrame the initial frame for this frame point.
    * @param tuple2DReadOnly the tuple to copy the coordinates from. Not modified.
    */
   public FramePoint3D(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
   {
      setIncludingFrame(referenceFrame, tuple2DReadOnly, 0.0);
   }

   /**
    * Creates a new frame point and initializes its reference frame x and y coordinates from
    * {@code frameTuple2DReadOnly}.
    *
    * @param frameTuple2DReadOnly the tuple to copy the coordinates and reference frame from. Not
    *           modified.
    */
   public FramePoint3D(FrameTuple2DReadOnly frameTuple2DReadOnly)
   {
      setIncludingFrame(frameTuple2DReadOnly, 0.0);
   }

   /**
    * Creates a new frame point and initializes it to {@code other}.
    *
    * @param other the tuple to copy the coordinates and reference frame from. Not modified.
    */
   public FramePoint3D(FrameTuple3DReadOnly other)
   {
      setIncludingFrame(other);
   }

   /**
    * Sets this frame point to {@code other}.
    *
    * @param other the other frame point to set this to. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   @Override
   public void set(FramePoint3D other)
   {
      FramePoint3DBasics.super.set(other);
   }

   /**
    * Sets the reference frame of this point without updating or modifying its x, y, and z components.
    *
    * @param referenceFrame the new reference frame for this frame point.
    */
   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   /**
    * Sets the x-coordinate of this point.
    *
    * @param x the x-coordinate.
    */
   @Override
   public void setX(double x)
   {
      point.setX(x);
   }

   /**
    * Sets the y-coordinate of this point.
    *
    * @param y the y-coordinate.
    */
   @Override
   public void setY(double y)
   {
      point.setY(y);
   }

   /**
    * Sets the z-coordinate of this point.
    *
    * @param z the z-coordinate.
    */
   @Override
   public void setZ(double z)
   {
      point.setZ(z);
   }

   /**
    * Gets the reference frame in which this point is currently expressed.
    */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   /**
    * Returns the value of the x-coordinate of this point.
    *
    * @return the x-coordinate's value.
    */
   @Override
   public double getX()
   {
      return point.getX();
   }

   /**
    * Returns the value of the y-coordinate of this point.
    *
    * @return the y-coordinate's value.
    */
   @Override
   public double getY()
   {
      return point.getY();
   }

   /**
    * Returns the value of the z-coordinate of this point.
    *
    * @return the z-coordinate's value.
    */
   @Override
   public double getZ()
   {
      return point.getZ();
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(FrameTuple3DReadOnly)}, it returns {@code false} otherwise.
    * <p>
    * If the two points have different frames, this method returns {@code false}.
    * </p>
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if the two points are exactly equal component-wise and are expressed in the
    *         same reference frame, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((FrameTuple3DReadOnly) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Tests on a per component basis if this point is equal to the given {@code other} to an
    * {@code epsilon}.
    * <p>
    * If the two points have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the other point to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two points are equal and are expressed in the same reference frame,
    *         {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(FramePoint3D other, double epsilon)
   {
      return FramePoint3DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same point 3D to an {@code epsilon}.
    * <p>
    * Two points are considered geometrically equal if they are at a distance of less than or equal to
    * {@code epsilon}.
    * </p>
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * {@code this.epsilonEquals(other, epsilon)} and vice versa.
    * </p>
    *
    * @param other the other point 3D to compare against this. Not modified.
    * @param epsilon the maximum distance that the two points can be spaced and still considered equal.
    * @return {@code true} if the two points represent the same geometry, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   @Override
   public boolean geometricallyEquals(FramePoint3D other, double epsilon)
   {
      return FramePoint3DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Provides a {@code String} representation of this frame point 3D as follows: (x, y, z)-worldFrame.
    *
    * @return the {@code String} representing this frame point 3D.
    */
   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getTuple3DString(this) + "-" + referenceFrame;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this frame point 3D.
    *
    * @return the hash code value for this frame point 3D.
    */
   @Override
   public int hashCode()
   {
      return point.hashCode();
   }
}
