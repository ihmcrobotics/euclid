package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.Orientation2D;
import us.ihmc.euclid.geometry.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation2DReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;

public class FrameOrientation2D implements FrameOrientation2DBasics
{
   /** The reference frame is which this orientation is currently expressed. */
   private ReferenceFrame referenceFrame;
   /** The orientation holding the current coordinates of this frame orientation. */
   private final Orientation2D orientation = new Orientation2D();

   /**
    * Creates a new frame vector and initializes its orientation to the given {@link Orientation2D}
    * and its reference frame to {@link ReferenceFrame#getWorldFrame()}.
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
    * @param orientation2DReadOnly the orientation this frame orientation will represent. Not
    *           modified.
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
      setIncludingFrame(referenceFrame, other);
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
