package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.orientation.Orientation2D;
import us.ihmc.euclid.orientation.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.EuclidFrameGeometry;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.Transform;

/**
 * A {@code FrameOrientation2D} represents an orientation in the XY-plane expressed in a given
 * reference frame.
 *
 * @author Sylvain Bertrand
 */
public class FrameOrientation2D implements FrameOrientation2DBasics, Settable<FrameOrientation2D>
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
    * @param yaw            the initial yaw angle for this orientation 2D.
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
    * @param referenceFrame        the initial frame for this orientation.
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
    *                                from. Not modified.
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
    * {@link #equals(EuclidFrameGeometry)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameOrientation2DReadOnly)
         return equals((EuclidFrameGeometry) object);
      else
         return false;
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
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
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
      return EuclidHashCodeTools.toIntHashCode(orientation, referenceFrame);
   }
}
