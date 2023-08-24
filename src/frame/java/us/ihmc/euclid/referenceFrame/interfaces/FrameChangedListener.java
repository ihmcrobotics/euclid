package us.ihmc.euclid.referenceFrame.interfaces;

public interface FrameChangedListener
{
   void changed(Change change);

   public static interface Change
   {
      boolean wasAdded();

      boolean wasRemoved();

      boolean wasGarbageCollected();

      FrameBase getSource();

      FrameBase getTarget();

      FrameBase getTargetParent();
   }
}
