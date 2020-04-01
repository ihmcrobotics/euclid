package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;

/**
 * Read and write interface for a point shape 3D expressed in a changeable reference frame, i.e. the
 * reference frame in which this line is expressed can be changed.
 * <p>
 * A point shape 3D is represented by its position.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FramePointShape3DBasics extends FixedFramePointShape3DBasics, FrameShape3DBasics, FramePoint3DBasics
{
   /** {@inheritDoc} */
   @Override
   default void setToZero(ReferenceFrame referenceFrame)
   {
      FrameShape3DBasics.super.setToZero(referenceFrame);
   }

   /** {@inheritDoc} */
   @Override
   default void setToNaN(ReferenceFrame referenceFrame)
   {
      FrameShape3DBasics.super.setToNaN(referenceFrame);
   }

   /** {@inheritDoc} */
   @Override
   FramePointShape3DBasics copy();
}
