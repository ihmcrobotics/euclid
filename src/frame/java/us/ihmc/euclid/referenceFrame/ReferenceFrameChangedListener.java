package us.ihmc.euclid.referenceFrame;

/**
 * Interface that receives notifications of changes to a {@link ReferenceFrame}.
 */
public interface ReferenceFrameChangedListener
{
   /**
    * Called after a change has been made to a {@link ReferenceFrame} or one of its descendants.
    *
    * @param change an object representing the change that was done.
    * @see Change
    */
   void changed(Change change);

   /**
    * Represents a report of a single change done to a {@link ReferenceFrame}.
    */
   public static interface Change
   {
      /**
       * Indicates that a reference frame was added:
       * <ul>
       * <li>The frame that triggered the event can be accessed via {@link #getSource()}.
       * <li>The frame that was added can be accessed via {@link #getTarget()}.
       * <li>The parent of the new frame can be accessed via {@link #getTargetParent()}.
       * </ul>
       * 
       * @return {@code true} if the change was the addition of a new reference frame.
       */
      boolean wasAdded();

      /**
       * Indicates that a reference frame was removed:
       * <ul>
       * <li>The frame that triggered the event can be accessed via {@link #getSource()}.
       * <li>The frame that was removed can be accessed via {@link #getTarget()}.
       * <li>The parent of the frame before it was removed can be accessed via {@link #getTargetParent()}.
       * </ul>
       * 
       * @return {@code true} if the change was the removal of a reference frame.
       */
      boolean wasRemoved();

      /**
       * Indicates that a reference frame was collected by the garbage collector:
       * <ul>
       * <li>The frame that triggered the event can be accessed via {@link #getSource()}.
       * <li>The frame that was removed cannot be accessed, {@link #getTarget()} returns {@code null}.
       * <li>The parent of the frame before it was removed can be accessed via {@link #getTargetParent()}.
       * </ul>
       * 
       * @return {@code true} if the change was a reference frame being garbage collected.
       */
      boolean wasGarbageCollected();

      /**
       * The reference frame that fired the change.
       * 
       * @return the source of this change.
       */
      ReferenceFrame getSource();

      /**
       * The reference frame that was added, removed, or {@code null} when the target was garbage
       * collected.
       * 
       * @return the added/removed frame, or {@code null} if garbage collected.
       */
      ReferenceFrame getTarget();

      /**
       * The parent of a frame that was added or removed.
       * 
       * @return the parent of the target.
       */
      ReferenceFrame getTargetParent();
   }
}
