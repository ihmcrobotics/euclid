package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

public class RootFrame extends AbstractFrameBase implements FrameBase
{
   private final static RigidBodyTransformReadOnly identity = EuclidCoreTools.identityTransform;

   public RootFrame(String name)
   {
      super(name);
   }

   @Override
   public String getNameId()
   {
      return name;
   }

   @Override
   public boolean isRoot()
   {
      return true;
   }

   @Override
   public boolean isInertialFrame()
   {
      return true;
   }

   @Override
   public boolean isFixedInParent()
   {
      return true;
   }

   @Override
   public FrameBase getParent()
   {
      return null;
   }

   @Override
   public FrameBase getRoot()
   {
      return this;
   }

   @Override
   public RigidBodyTransformReadOnly getTransformToParent()
   {
      return null;
   }

   @Override
   public RigidBodyTransformReadOnly getTransformToRoot()
   {
      return identity;
   }

   @Override
   public void getTransformTo(FrameBase toFrame, RigidBodyTransformBasics transformToPack)
   {
      checkSameRoots(toFrame);
      transformToPack.setAndInvert(toFrame.getTransformToRoot());
   }

   @Override
   public void transformTo(FrameBase toFrame, Transformable objectToTransform)
   {
      checkSameRoots(toFrame);
      objectToTransform.applyInverseTransform(toFrame.getTransformToRoot());
   }
}
