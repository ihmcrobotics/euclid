package us.ihmc.euclid.referenceFrame.interfaces;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.FrameNameRestrictionLevel;
import us.ihmc.euclid.referenceFrame.interfaces.FrameChangedListener.Change;

public abstract class AbstractFrameBase implements FrameBase
{
   protected final String name;
   protected final WeakList<FrameBase> children = new WeakList<>();
   private List<FrameChangedListener> changedListeners;

   private FrameNameRestrictionLevel nameRestrictionLevel;

   protected enum ChangeType
   {
      FRAME_ADDED, FRAME_REMOVED, FRAME_GCED
   }

   public AbstractFrameBase(String name)
   {
      this.name = name;
      children.setGCListener(itemCollectedIndex -> notifyListeners(ChangeType.FRAME_GCED, null, this));
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public int getNumberOfChildren()
   {
      return children.size();
   }

   @Override
   public FrameBase getChild(int index)
   {
      return children.get(index);
   }

   @Override
   public void setNameRestrictionLevel(FrameNameRestrictionLevel nameRestrictionLevel)
   {
      if (this.nameRestrictionLevel == nameRestrictionLevel)
         return;

      if (nameRestrictionLevel.ordinal() < this.nameRestrictionLevel.ordinal())
      {
         if (isRoot() && children.isEmpty())
         { // allow decreased restriction for root frames without children
            this.nameRestrictionLevel = nameRestrictionLevel;
         }
         else
         { // otherwise decreasing restriction level is not allowed
            throw new IllegalArgumentException("Cannot reduce name restriction level. Current mode: " + this.nameRestrictionLevel + ", tried to set to: "
                                               + nameRestrictionLevel);
         }
      }

      this.nameRestrictionLevel = nameRestrictionLevel;

      if (nameRestrictionLevel == FrameNameRestrictionLevel.NAME_ID)
      {
         if (!doChildrenHaveUniqueNames())
         {
            children.cleanupReferences();
            throw new RuntimeException("Duplicate ReferenceFrame nameId's detected: " + children.stream().map(FrameBase::getName).toList());
         }
      }
      else if (nameRestrictionLevel == FrameNameRestrictionLevel.FRAME_NAME)
      {
         // This operation is not efficient, but since it should not be frequently done it doesn't matter.
         // Find the closest frame to the root that has the FRAME_NAME restriction.
         FrameBase ancestor = this;
         while (!ancestor.isRoot() && ancestor.getParent().getNameRestrictionLevel() == FrameNameRestrictionLevel.FRAME_NAME)
            ancestor = ancestor.getParent();

         // We then check that this frame is unique in the subtree of the ancestor.
         if (!ancestor.hasUniqueNameInSubtree(this))
            throw new RuntimeException("Duplicate reference frame names detected: " + getName());
      }

      for (int i = 0; i < children.size(); i++)
      {
         children.get(i).setNameRestrictionLevel(nameRestrictionLevel);
      }
   }

   @Override
   public FrameNameRestrictionLevel getNameRestrictionLevel()
   {
      return nameRestrictionLevel;
   }

   @Override
   public void addListener(FrameChangedListener listener)
   {
      if (changedListeners == null)
         changedListeners = new ArrayList<>();
      changedListeners.add(listener);
   }

   public void removeListeners()
   {
      changedListeners = null;
   }

   @Override
   public boolean removeListener(FrameChangedListener listener)
   {
      if (changedListeners == null)
         return false;
      return changedListeners.remove(listener);
   }

   protected void notifyListeners(ChangeType type, FrameBase target, FrameBase targetParent)
   {
      if (changedListeners == null)
         return;

      FrameChange change = new FrameChange(type, target, targetParent);
      for (int i = 0; i < changedListeners.size(); i++)
         changedListeners.get(i).changed(change);
   }

   private class FrameChange implements Change
   {
      private final ChangeType type;
      private final FrameBase target, targetParent;

      public FrameChange(ChangeType type, FrameBase target, FrameBase targetParent)
      {
         this.type = type;
         this.target = target;
         this.targetParent = targetParent;
      }

      @Override
      public boolean wasAdded()
      {
         return type == ChangeType.FRAME_ADDED;
      }

      @Override
      public boolean wasRemoved()
      {
         return type == ChangeType.FRAME_REMOVED;
      }

      @Override
      public boolean wasGarbageCollected()
      {
         return type == ChangeType.FRAME_GCED;
      }

      @Override
      public FrameBase getSource()
      {
         return AbstractFrameBase.this;
      }

      @Override
      public FrameBase getTarget()
      {
         return target;
      }

      @Override
      public FrameBase getTargetParent()
      {
         return targetParent;
      }
   }
}