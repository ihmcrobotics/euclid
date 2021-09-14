package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class FixedReferenceFrame extends ReferenceFrame
{
   public FixedReferenceFrame(String name, ReferenceFrame parentFrame, Tuple3DReadOnly translationOffsetFromParent)
   {
      this(name, parentFrame, new RigidBodyTransform(EuclidCoreTools.neutralQuaternion, translationOffsetFromParent));
   }

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
