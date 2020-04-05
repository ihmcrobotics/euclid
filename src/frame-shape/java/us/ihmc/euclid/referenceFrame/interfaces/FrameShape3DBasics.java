package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;

/**
 * @author Sylvain Bertrand
 */
public interface FrameShape3DBasics extends FixedFrameShape3DBasics, FrameChangeable
{
   /**
    * Sets all the components of this shape to {@link Double#NaN} and sets the current reference frame
    * to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame to be associated with this shape.
    */
   default void setToNaN(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToNaN();
   }

   /**
    * Sets all the components of this frame shape to zero and sets the current reference frame to
    * {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame to be associated with this shape.
    */
   default void setToZero(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToZero();
   }

   /** {@inheritDoc} */
   @Override
   FrameShape3DBasics copy();
}
