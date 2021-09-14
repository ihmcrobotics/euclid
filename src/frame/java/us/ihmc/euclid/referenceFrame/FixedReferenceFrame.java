package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

public class FixedReferenceFrame extends ReferenceFrame
{
   public FixedReferenceFrame(String name, ReferenceFrame parentFrame, RigidBodyTransformReadOnly transformToParent)
   {
      super(name, parentFrame, transformToParent, parentFrame.isAStationaryFrame(), parentFrame.isZupFrame() && transformToParent.isRotation2D(), true);
   }

   @Override
   public void update()
   {
      // Since it is fixed, the transform does not change.
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
   }
}
