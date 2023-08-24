package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

public interface FrameBase
{
   /** A string used to separate frame names in the {@link #nameId} of the reference frame */
   static final String SEPARATOR = ":";

   String getName();

   String getNameId();

   boolean isRoot();

   boolean isInertialFrame();

   boolean isFixedInParent();

   FrameBase getParent();

   FrameBase getRoot();

   RigidBodyTransformReadOnly getTransformToParent();

   RigidBodyTransformReadOnly getTransformToRoot();

   void getTransformTo(FrameBase toFrame, RigidBodyTransformBasics transformToPack);

   default void transformTo(FrameBase target, Transformable objectToTransform)
   {
      if (this == target)
      { // Check for trivial case
         return;
      }

      checkSameRoots(target);

      // The general approach is:
      // objectToTransform = (desired.transformToRoot)^-1 * this.transformToRoot * objectToTransform
      // Or code-wise:
      // 1- objectToTransform.applyTransform(transformToRoot);
      // 2- objectToTransform.applyInverseTransform(desiredFrame.transformToRoot);
      // As this requires 2 transformations, we first check for simpler cases:
      if (isRoot())
      {
         /*
          * If this is the root frame, desiredFrame cannot be the root frame, i.e. it would have triggered
          * the previous condition as there can be only one root per frame tree. Thus: this.transformToRoot
          * is the identity, only 1 transformation here.
          */
         objectToTransform.applyInverseTransform(target.getTransformToRoot());
         return;
      }

      if (target.isRoot())
      {
         /*
          * If desiredFrame is the root frame, this cannot be the root frame, i.e. it would have triggered
          * the previous condition as there can be only one root per frame tree. Thus:
          * desiredFrame.transformToRoot is the identity, only 1 transformation here.
          */
         objectToTransform.applyTransform(getTransformToRoot());
         return;
      }

      FrameBase thisParent = getParent();

      if (target == thisParent)
      { // Test direct connection between the frames:
         objectToTransform.applyTransform(getTransformToParent());
         return;
      }

      FrameBase targetParent = target.getParent();

      if (this == targetParent)
      { // Test direct connection between the frames:
         objectToTransform.applyInverseTransform(target.getTransformToParent());
         return;
      }
      else if (thisParent == targetParent)
      {
         /*
          * Common parentFrame. Here 2 transformations are needed but the transforms involved will often be
          * simple (rotation only or translation only) whereas the transformToRoot of most frame is a complex
          * transform.
          */
         objectToTransform.applyTransform(getTransformToParent());
         objectToTransform.applyInverseTransform(target.getTransformToParent());
         return;
      }

      if (thisParent.getParent() == target)
      { // Look at a distance of 2, which involves 2 transformations with transforms that will often be simple (rotation only or translation only).
         objectToTransform.applyTransform(getTransformToParent());
         objectToTransform.applyTransform(thisParent.getTransformToParent());
         return;
      }

      if (this == targetParent.getParent())
      { // Look at a distance of 2, which involves 2 transformations with transforms that will often be simple (rotation only or translation only).
         objectToTransform.applyInverseTransform(targetParent.getTransformToParent());
         objectToTransform.applyInverseTransform(target.getTransformToParent());
         return;
      }

      // This is the general scenario:
      objectToTransform.applyTransform(getTransformToRoot());
      objectToTransform.applyInverseTransform(target.getTransformToRoot());
   }

   default boolean isParent(FrameBase query)
   {
      return query == getParent();
   }

   default boolean isChild(FrameBase query)
   {
      return query.isParent(this);
   }

   default boolean isAncestor(FrameBase query)
   {
      FrameBase ancestor = getParent();

      while (ancestor != null)
      {
         if (ancestor == query)
            return true;
         ancestor = getParent();
      }

      return false;
   }

   default void checkSameRoots(FrameBase other)
   {
      if (getRoot() != other.getRoot())
         throw new RuntimeException("Frames do not have same roots. this = " + this + ", other = " + other);
   }

   default void checkIsAncestor(FrameBase other)
   {
      if (!isAncestor(other))
         throw new RuntimeException(other.getNameId() + " is not an ancestor of " + getNameId());
   }

   default void checkFrameMatch(FrameHolder frameHolder) throws ReferenceFrameMismatchException
   {
      checkFrameMatch(frameHolder.getFrame());
   }

   default void checkFrameMatch(FrameBase other) throws ReferenceFrameMismatchException
   {
      if (this != other)
      {
         throw new ReferenceFrameMismatchException("Other: " + other + " does not match this: " + this);
      }
   }

   int getNumberOfChildren();

   FrameBase getChild(int index);

   void addListener(FrameChangedListener listener);

   void removeListeners();

   boolean removeListener(FrameChangedListener listener);
}
