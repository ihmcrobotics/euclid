package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.referenceFrame.FrameNameRestrictionLevel;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

public interface FrameBase
{
   /** A string used to separate frame names in the {@link #nameId} of the reference frame */
   static final String SEPARATOR = ":";

   static FrameNameRestrictionLevel DEFAULT_RESTRICTION_LEVEL = FrameNameRestrictionLevel.loadFromEnvironment("euclid.referenceFrame.restrictionLevel",
                                                                                                              "FrameNameRestrictionLevel",
                                                                                                              FrameNameRestrictionLevel.NONE);

   String getName();

   String getNameId();

   boolean isRoot();

   boolean isInertialFrame();

   boolean isFixedInParent();

   FrameBase getParent();

   FrameBase getRoot();

   void setNameRestrictionLevel(FrameNameRestrictionLevel nameRestrictionLevel);

   FrameNameRestrictionLevel getNameRestrictionLevel();

   RigidBodyTransformReadOnly getTransformToParent();

   RigidBodyTransformReadOnly getTransformToRoot();

   default void getTransformTo(FrameBase toFrame, RigidBodyTransformBasics transformToPack)
   {
      try
      {
         if (this == toFrame)
         { // Check for trivial case
            transformToPack.setToZero();
            return;
         }

         checkSameRoots(toFrame);

         // The general approach is:
         // transformToPack = (desiredFrame.transformToRoot)^-1 * this.transformToRoot
         // As this requires a transform multiplication, we first check for simpler cases:
         if (isRoot())
         {
            /*
             * If this is the root frame, desiredFrame cannot be the root frame, i.e. it would have triggered
             * the previous condition as there can be only one root per frame tree. Thus: this.transformToRoot
             * is the identity, no need for a multiplication here.
             */
            transformToPack.setAndInvert(toFrame.getTransformToRoot());
            return;
         }

         if (toFrame.isRoot())
         {
            /*
             * If desiredFrame is the root frame, this cannot be the root frame, i.e. it would have triggered
             * the previous condition as there can be only one root per frame tree. Thus:
             * desiredFrame.transformToRoot is the identity, no need for a multiplication here.
             */
            transformToPack.set(getTransformToRoot());
            return;
         }

         FrameBase thisParent = getParent();

         if (thisParent == toFrame)
         { // Test direct connection between the frames:
            transformToPack.set(getTransformToParent());
            return;
         }

         FrameBase otherParent = toFrame.getParent();

         if (this == otherParent)
         { // Test direct connection between the frames:
            transformToPack.setAndInvert(toFrame.getTransformToParent());
            return;
         }

         if (thisParent == otherParent)
         {
            /*
             * Common parentFrame. Here the multiplication is needed but the transforms involved will often be
             * simple (rotation only or translation only) whereas the transformToRoot of most frame is a complex
             * transform.
             */
            transformToPack.setAndInvert(toFrame.getTransformToParent());
            transformToPack.multiply(getTransformToParent());
            return;
         }

         FrameBase thisGrandparent = thisParent.getParent();

         if (thisGrandparent == toFrame)
         { // Look at a distance of 2, which would involve the multiplication of 2 transforms that will often be simple (rotation only or translation only).
            transformToPack.set(getTransformToParent());
            transformToPack.preMultiply(thisParent.getTransformToParent());
            return;
         }

         FrameBase otherGrandparent = otherParent.getParent();

         if (this == otherGrandparent)
         { // Look at a distance of 2, which would involve the multiplication of 2 transforms that will often be simple (rotation only or translation only).
            transformToPack.setAndInvert(toFrame.getTransformToParent());
            transformToPack.multiplyInvertOther(otherParent.getTransformToParent());
            return;
         }

         // This is the general scenario:
         transformToPack.setAndInvert(toFrame.getTransformToRoot());
         transformToPack.multiply(getTransformToRoot());
      }
      catch (NotARotationMatrixException e)
      {
         throw new NotARotationMatrixException("Caught exception, this frame: " + getName() + ", other frame: " + toFrame.getName() + ", exception:\n"
                                               + e.getMessage());
      }
   }

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

   default boolean hasUniqueNameInSubtree(FrameBase query)
   {
      if (query != this && areNamesEqual(getName(), query.getName()))
         return false;

      for (int i = 0; i < getNumberOfChildren(); i++)
      {
         if (!getChild(i).hasUniqueNameInSubtree(query))
            return false;
      }
      return true;
   }

   default boolean doChildrenHaveUniqueNames()
   {
      for (int i = 0; i < getNumberOfChildren(); i++)
      {
         FrameBase childA = getChild(i);

         if (childA == null)
            continue;

         for (int j = i + 1; j < getNumberOfChildren(); j++)
         {
            FrameBase childB = getChild(j);

            if (childB != null && areNamesEqual(childA.getName(), childB.getName()))
               return false;
         }
      }
      return true;
   }

   static boolean areNamesEqual(String nameA, String nameB)
   {
      if (nameA == nameB)
         return true;
      if (nameA.hashCode() != nameB.hashCode())
         return false;
      return nameA.equals(nameB);
   }
}
