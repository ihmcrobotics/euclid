package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

/**
 * Interface used for any transformable object that this associated with a reference frame.
 *
 * @author Sylvain Bertrand
 */
public interface FrameChangeable extends ReferenceFrameHolder, Transformable
{
   /**
    * Sets the reference frame of this object.
    * <p>
    * Note that the typical implementation for this method is to only set reference frame without
    * performing any transformation on the geometry.
    * </p>
    *
    * @param referenceFrame the new reference frame.
    */
   void setReferenceFrame(ReferenceFrame referenceFrame);

   /**
    * Transforms this geometry such that the transformation represents the same geometry but from the
    * perspective of another reference frame: {@code desiredFrame}.
    * <p>
    * Once the geometry is transformed, the reference frame is updated to {@code desiredFrame}. In the
    * case, {@code this.referenceFrame == desiredFrame}, this method does nothing.
    * </p>
    *
    * @param desiredFrame the reference frame in which this geometry is to be expressed.
    */
   default void changeFrame(ReferenceFrame desiredFrame)
   {
      getReferenceFrame().transformFromThisToDesiredFrame(desiredFrame, this);
      setReferenceFrame(desiredFrame);
   }
}
