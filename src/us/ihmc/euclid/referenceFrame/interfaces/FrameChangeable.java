package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;

public interface FrameChangeable extends ReferenceFrameHolder, Transformable
{
   /**
    * Sets the reference frame of this object.
    * 
    * @param referenceFrame the new reference frame.
    */
   void setReferenceFrame(ReferenceFrame reference);

   /**
    * Transforms this geometry such that the transformation represents the same geometry but from
    * the perspective of another reference frame: {@code desiredFrame}.
    * <p>
    * Once the geometry is transformed, the reference frame is updated to {@code desiredFrame}. In
    * the case, {@code this.referenceFrame == desiredFrame}, this method does nothing.
    * </p>
    * 
    * @param desiredFrame the reference frame in which this geometry is to be expressed.
    */
   default void changeFrame(ReferenceFrame desiredFrame)
   {
      // Check for the trivial case: the geometry is already expressed in the desired frame.
      if (desiredFrame == getReferenceFrame())
         return;

      getReferenceFrame().verifySameRoots(desiredFrame);

      RigidBodyTransform referenceFrameTransformToRoot, desiredFrameTransformToRoot;

      if ((referenceFrameTransformToRoot = getReferenceFrame().getTransformToRoot()) != null)
      {
         applyTransform(referenceFrameTransformToRoot);
      }

      if ((desiredFrameTransformToRoot = desiredFrame.getTransformToRoot()) != null)
      {
         applyInverseTransform(desiredFrameTransformToRoot);
      }

      setReferenceFrame(desiredFrame);
   }
}
