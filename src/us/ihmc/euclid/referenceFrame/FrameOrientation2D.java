package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.Orientation2D;
import us.ihmc.euclid.geometry.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;

/**
 * A {@code FrameOrientation2D} represents an orientation in the XY-plane expressed in a given
 * reference frame.
 * 
 * @author Sylvain Bertrand
 *
 */
public class FrameOrientation2D implements FrameOrientation2DBasics, GeometryObject<FrameOrientation2D>
{
   /** The reference frame is which this orientation is currently expressed. */
   private ReferenceFrame referenceFrame;
   /** The orientation holding the current coordinates of this frame orientation. */
   private final Orientation2D orientation = new Orientation2D();

   /**
    * Creates a new orientation 2D initialized with its yaw angle to zero and its reference frame to
    * {@link ReferenceFrame#getWorldFrame()}.
    */
   public FrameOrientation2D()
   {
      setToZero(ReferenceFrame.getWorldFrame());
   }

   /**
    * Create a new orientation 2D initialized with its yaw angle to zero in a given reference frame.
    * 
    * @param referenceFrame the initial reference frame for this orientation 2D.
    */
   public FrameOrientation2D(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   /**
    * Creates a new orientation 2D and initializes its yaw angle to the given one, and its reference
    * frame to {@link ReferenceFrame#getWorldFrame()}.
    * 
    * @param yaw the initial yaw angle for this orientation 2D.
    */
   public FrameOrientation2D(double yaw)
   {
      setIncludingFrame(ReferenceFrame.getWorldFrame(), yaw);
   }

   /**
    * Creates a new orientation 2D and initializes its yaw angle and reference frame.
    * 
    * @param referenceFrame the initial reference frame for this orientation 2D.
    * @param yaw the initial yaw angle for this orientation 2D.
    */
   public FrameOrientation2D(ReferenceFrame referenceFrame, double yaw)
   {
      setIncludingFrame(referenceFrame, yaw);
   }

   /**
    * Creates a new frame vector and initializes its orientation to the given {@link Orientation2D} and
    * its reference frame to {@link ReferenceFrame#getWorldFrame()}.
    *
    * @param orientation2DReadOnly the orientation this frame orientation will represent. Modified.
    */
   public FrameOrientation2D(Orientation2DReadOnly orientation2DReadOnly)
   {
      this(ReferenceFrame.getWorldFrame(), orientation2DReadOnly);
   }

   /**
    * Creates a new frame vector and initializes its orientation components to the given
    * {@link Orientation2DReadOnly} and its reference frame to the given {@link ReferenceFrame}.
    *
    * @param referenceFrame the initial frame for this orientation.
    * @param orientation2DReadOnly the orientation this frame orientation will represent. Not modified.
    */
   public FrameOrientation2D(ReferenceFrame referenceFrame, Orientation2DReadOnly orientation2DReadOnly)
   {
      setIncludingFrame(referenceFrame, orientation2DReadOnly);
   }

   /**
    * Creates a new frame orientation 2D and initializes it to {@code other}.
    *
    * @param other the tuple to copy the orientation and reference frame from. Not modified.
    */
   public FrameOrientation2D(FrameOrientation2DReadOnly other)
   {
      setIncludingFrame(other);
   }

   /**
    * Creates a new frame orientation 2D and initializes it using a frame quaternion.
    * 
    * @param frameQuaternionReadOnly the frame quaternion to get the yaw angle and reference frame
    *           from. Not modified.
    */
   public FrameOrientation2D(FrameQuaternionReadOnly frameQuaternionReadOnly)
   {
      setIncludingFrame(frameQuaternionReadOnly);
   }

   /** {@inheritDoc} */
   @Override
   public void set(FrameOrientation2D other)
   {
      FrameOrientation2DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public void setYaw(double yaw)
   {
      orientation.setYaw(yaw);
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public double getYaw()
   {
      return orientation.getYaw();
   }

   /** {@inheritDoc} */
   @Override
   public void applyTransform(Transform transform)
   {
      orientation.applyTransform(transform);
   }

   /** {@inheritDoc} */
   @Override
   public void applyInverseTransform(Transform transform)
   {
      orientation.applyInverseTransform(transform);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(FrameOrientation2DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((FrameOrientation2DReadOnly) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Tests if the yaw angle of this orientation is equal to an {@code epsilon} to the yaw of
    * {@code other}.
    * <p>
    * Note that this method performs number comparison and not an angle comparison, such that:
    * -<i>pi</i> &ne; <i>pi</i>.
    * </p>
    *
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two orientations are equal, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   @Override
   public boolean epsilonEquals(FrameOrientation2D other, double epsilon)
   {
      return FrameOrientation2DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two orientations are geometrically
    * similar, i.e. the difference in yaw of {@code this} and {@code other} is less than or equal to
    * {@code epsilon}.
    *
    * @param other the orientation to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two orientations represent the same geometry, {@code false}
    *         otherwise.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   @Override
   public boolean geometricallyEquals(FrameOrientation2D other, double epsilon)
   {
      return FrameOrientation2DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Provides a {@code String} representation of this frame orientation 2D as follows:<br>
    * (0.123 )-worldFrame
    *
    * @return the {@code String} representing this frame orientation 2D.
    */
   @Override
   public String toString()
   {
      return orientation.toString() + "-" + referenceFrame;
   }

   /**
    * Calculates and returns a hash code value from the value of the angle of this frame orientation
    * 2D.
    *
    * @return the hash code value for this frame orientation 2D.
    */
   @Override
   public int hashCode()
   {
      return orientation.hashCode();
   }
}
