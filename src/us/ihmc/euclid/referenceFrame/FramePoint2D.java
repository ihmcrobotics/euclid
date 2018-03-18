package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * {@code FramePoint2D} is a 2D point expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Point2DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FramePoint2D}. This allows, for instance, to enforce, at runtime, that operations on
 * points occur in the same coordinate system. Also, via the method
 * {@link #changeFrame(ReferenceFrame)}, one can easily calculates the value of a point in different
 * reference frames.
 * </p>
 * <p>
 * Because a {@code FramePoint2D} extends {@code Point2DBasics}, it is compatible with methods only
 * requiring {@code Point2DBasics}. However, these methods do NOT assert that the operation occur in
 * the proper coordinate system. Use this feature carefully and always prefer using methods
 * requiring {@code FramePoint2D}.
 * </p>
 */
public class FramePoint2D implements FramePoint2DBasics, GeometryObject<FramePoint2D>
{
   /** The reference frame is which this point is currently expressed. */
   private ReferenceFrame referenceFrame;
   /** The point holding the current coordinates of this frame point. */
   private final Point2D point = new Point2D();
   /** Rigid-body transform used to perform garbage-free operations. */
   private final RigidBodyTransform transformToDesiredFrame = new RigidBodyTransform();

   /**
    * Creates a new frame point and initializes it coordinates to zero and its reference frame to
    * {@link ReferenceFrame#getWorldFrame()}.
    */
   public FramePoint2D()
   {
      setToZero(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new frame point and initializes it coordinates to zero and its reference frame to the
    * {@code referenceFrame}.
    *
    * @param referenceFrame the initial frame for this frame point.
    */
   public FramePoint2D(ReferenceFrame referenceFrame)
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
    */
   public FramePoint2D(ReferenceFrame referenceFrame, double x, double y)
   {
      setIncludingFrame(referenceFrame, x, y);
   }

   /**
    * Creates a new frame point and initializes its coordinates {@code x}, {@code y} in order from the
    * given array and initializes its reference frame.
    *
    * @param referenceFrame the initial frame for this frame point.
    * @param pointArray the array containing this point's coordinates. Not modified.
    */
   public FramePoint2D(ReferenceFrame referenceFrame, double[] pointArray)
   {
      setIncludingFrame(referenceFrame, pointArray);
   }

   /**
    * Creates a new frame point and initializes it to {@code tuple2DReadOnly} and to the given
    * reference frame.
    *
    * @param referenceFrame the initial frame for this frame point.
    * @param tuple2DReadOnly the tuple to copy the coordinates from. Not modified.
    */
   public FramePoint2D(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
   {
      setIncludingFrame(referenceFrame, tuple2DReadOnly);
   }

   /**
    * Creates a new frame point and initializes it to the x and y coordinates of
    * {@code tuple3DReadOnly} and to the given reference frame.
    *
    * @param referenceFrame the initial frame for this frame point.
    * @param tuple3DReadOnly the tuple to copy the coordinates from. Not modified.
    */
   public FramePoint2D(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      setIncludingFrame(referenceFrame, tuple3DReadOnly);
   }

   /**
    * Creates a new frame point and initializes it to {@code other}.
    *
    * @param other the tuple to copy the components and reference frame from. Not modified.
    */
   public FramePoint2D(FrameTuple2DReadOnly other)
   {
      setIncludingFrame(other);
   }

   /**
    * Creates a new frame point and initializes it to the x and y coordinates of
    * {@code frameTuple3DReadOnly}.
    *
    * @param frameTuple3DReadOnly the tuple to copy the coordinates and reference frame from. Not
    *           modified.
    */
   public FramePoint2D(FrameTuple3DReadOnly frameTuple3DReadOnly)
   {
      setIncludingFrame(frameTuple3DReadOnly);
   }

   /**
    * Sets this frame point to {@code other}.
    *
    * @param other the other frame point to set this to. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   @Override
   public void set(FramePoint2D other)
   {
      FramePoint2DBasics.super.set(other);
   }

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

   /** {@inheritDoc} */
   @Override
   public void changeFrame(ReferenceFrame desiredFrame)
   {
      // Check for the trivial case: the geometry is already expressed in the desired frame.
      if (desiredFrame == referenceFrame)
         return;

      /*
       * By overriding changeFrame, on the transformToDesiredFrame is being checked instead of checking
       * both referenceFrame.transformToRoot and desiredFrame.transformToRoot.
       */
      referenceFrame.getTransformToDesiredFrame(transformToDesiredFrame, desiredFrame);
      applyTransform(transformToDesiredFrame);
      referenceFrame = desiredFrame;
   }

   /** {@inheritDoc} */
   @Override
   public final void changeFrameAndProjectToXYPlane(ReferenceFrame desiredFrame)
   {
      // Check for the trivial case: the geometry is already expressed in the desired frame.
      if (desiredFrame == referenceFrame)
         return;

      referenceFrame.getTransformToDesiredFrame(transformToDesiredFrame, desiredFrame);
      applyTransform(transformToDesiredFrame, false);
      referenceFrame = desiredFrame;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(FrameTuple2DReadOnly)}, it returns {@code false} otherwise.
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
         return equals((FrameTuple2DReadOnly) object);
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
   public boolean epsilonEquals(FramePoint2D other, double epsilon)
   {
      return FramePoint2DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same point 2D to an {@code epsilon}.
    * <p>
    * Two points are considered geometrically equal if they are at a distance of less than or equal to
    * {@code epsilon}.
    * </p>
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * {@code this.epsilonEquals(other, epsilon)} and vice versa.
    * </p>
    *
    * @param other the other point 2D to compare against this. Not modified.
    * @param epsilon the maximum distance that the two points can be spaced and still considered equal.
    * @return {@code true} if the two points represent the same geometry, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   @Override
   public boolean geometricallyEquals(FramePoint2D other, double epsilon)
   {
      return FramePoint2DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Provides a {@code String} representation of this frame point 2D as follows: (x, y)-worldFrame.
    *
    * @return the {@code String} representing this frame point 2D.
    */
   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getTuple2DString(this) + "-" + referenceFrame;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this frame point 2D.
    *
    * @return the hash code value for this frame point 2D.
    */
   @Override
   public int hashCode()
   {
      return point.hashCode();
   }
}
