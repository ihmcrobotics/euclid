package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

public class RTFrame extends AbstractFrameBase
{
   private String nameId;
   private FrameBase parent;
   private FrameBase root;
   private final boolean isInertialFrame;
   private final boolean isFixedInParent;
   private RigidBodyTransform transformToParent, transformToRoot;

   private boolean isDirty = true;

   public RTFrame(String name)
   {
      this(name, null);
   }

   public RTFrame(String name, FrameBase parent)
   {
      this(name, parent, false, false);
   }

   public RTFrame(String name, FrameBase parent, boolean isInertialFrame, boolean isFixedInParent)
   {
      super(name);
      this.parent = parent;
      this.isInertialFrame = isInertialFrame;
      this.isFixedInParent = isFixedInParent;
      transformToParent = new RigidBodyTransform();
      transformToRoot = new RigidBodyTransform();
   }

   @Override
   public String getNameId()
   {
      if (nameId == null)
      {
         if (parent != null)
            nameId = parent.getNameId() + SEPARATOR + name;
         else
            nameId = name;
      }
      return nameId;
   }

   @Override
   public boolean isRoot()
   {
      return parent == null;
   }

   @Override
   public boolean isInertialFrame()
   {
      return isInertialFrame;
   }

   @Override
   public boolean isFixedInParent()
   {
      return isFixedInParent;
   }

   @Override
   public FrameBase getParent()
   {
      return parent;
   }

   @Override
   public FrameBase getRoot()
   {
      if (root == null)
         root = parent == null ? this : parent.getRoot();
      return root;
   }

   @Override
   public RigidBodyTransform getTransformToParent()
   {
      return transformToParent;
   }

   @Override
   public RigidBodyTransformReadOnly getTransformToRoot()
   {
      return transformToRoot; // TODO update
   }
}
