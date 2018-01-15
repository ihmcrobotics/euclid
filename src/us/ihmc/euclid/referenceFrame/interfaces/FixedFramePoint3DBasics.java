package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

public interface FixedFramePoint3DBasics extends FramePoint3DReadOnly, FixedFrameTuple3DBasics, Point3DBasics
{
   /**
    * Sets this point coordinate to the given {@code referenceFrame}'s origin coordinate in this
    * frame tuple current frame.
    *
    * @param referenceFrame the reference frame of interest.
    */
   default void setFromReferenceFrame(ReferenceFrame referenceFrame)
   {
      ReferenceFrame thisReferenceFrame = getReferenceFrame();
      setToZero();

      // Check for the trivial case: the geometry is already expressed in the desired frame.
      if (referenceFrame == thisReferenceFrame)
         return;

      thisReferenceFrame.verifySameRoots(referenceFrame);

      RigidBodyTransform referenceFrameTransformToRoot, desiredFrameTransformToRoot;

      if ((referenceFrameTransformToRoot = thisReferenceFrame.getTransformToRoot()) != null)
      { // Equivalent to using applyTransform
         set(referenceFrameTransformToRoot.getTranslationVector());
      }

      if ((desiredFrameTransformToRoot = referenceFrame.getTransformToRoot()) != null)
      {
         applyInverseTransform(desiredFrameTransformToRoot);
      }
   }
}
