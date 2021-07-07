package us.ihmc.euclid.referenceFrame.tools;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;

import us.ihmc.euclid.referenceFrame.FixedReferenceFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * This class provides generic tools as well factories for reference frames.
 */
public class ReferenceFrameTools
{
   /**
    * {@code worldFrame} is a root reference frame and is most of time the only root reference frame.
    * <p>
    * It is commonly assumed that its axes are aligned as follows:
    * <ul>
    * <li>The x-axis is usually referred to as the forward axis.
    * <li>With the x-axis referring forward, the y-axis points to the left.
    * <li>The z-axis points upward and usually points to the opposite direction to the gravitational
    * acceleration.
    * </ul>
    * </p>
    */
   private static final ReferenceFrame worldFrame = constructARootFrame("World");

   private ReferenceFrameTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Construct a new z-up root reference frame.
    * <p>
    * Most of the time, {@link #worldFrame} is the only root frame from which children reference frames
    * are added.
    * </p>
    * <p>
    * Note that frames added as children of this root frame belongs to a different reference frame tree
    * than the tree starting off of {@link #worldFrame}. Transformation across two different trees of
    * reference frames is forbidden as the transformation between them is undefined.
    * </p>
    * <p>
    * The parent frame and transforms of a root frame are all {@code null}.
    * </p>
    *
    * @param frameName the name of the new world frame.
    * @return the new non-moving z-up root reference frame.
    */
   public static ReferenceFrame constructARootFrame(String frameName)
   {
      ReferenceFrame ret = new ReferenceFrame(frameName)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
         }
      };

      return ret;
   }

   /**
    * Return the world reference frame that is a root reference frame.
    * <p>
    * The world reference frame can be used to create child reference frames. It is usually assumed
    * that the z-axis represents the up/down direction (parallel to gravity), the x-axis represents the
    * forward/backward direction and the y-axis represents the transversal direction.
    * </p>
    *
    * @return the world reference frame.
    */
   public static ReferenceFrame getWorldFrame()
   {
      return worldFrame;
   }

   /**
    * Creates a reference frame with an immutable transform from its parent.
    * <p>
    * The {@code transformFromParent} should describe the pose of the parent frame expressed in this
    * new frame.
    * </p>
    *
    * @param frameName           the name of the new frame.
    * @param parentFrame         the parent frame of the new reference frame.
    * @param transformFromParent the transform that can be used to transform a geometry object from the
    *                            parent frame to this frame. Not modified.
    * @return the new reference frame.
    */
   public static ReferenceFrame constructFrameWithUnchangingTransformFromParent(String frameName,
                                                                                ReferenceFrame parentFrame,
                                                                                RigidBodyTransformReadOnly transformFromParent)
   {
      RigidBodyTransform transformToParent = new RigidBodyTransform(transformFromParent);
      transformToParent.invert();
      return new FixedReferenceFrame(frameName, parentFrame, transformToParent);
   }

   /**
    * Creates a reference frame with an immutable translation offset from its parent.
    * <p>
    * The new reference frame has the same orientation as its parent frame.
    * </p>
    *
    * @param frameName                   the name of the new frame.
    * @param parentFrame                 the parent frame of the new reference frame.
    * @param translationOffsetFromParent describes the position of the new reference frame's origin
    *                                    expressed in the parent frame. Not modified.
    * @return the new reference frame.
    */
   public static ReferenceFrame constructFrameWithUnchangingTranslationFromParent(String frameName,
                                                                                  ReferenceFrame parentFrame,
                                                                                  Tuple3DReadOnly translationOffsetFromParent)
   {
      return new FixedReferenceFrame(frameName, parentFrame, translationOffsetFromParent);
   }

   /**
    * Creates a reference frame with an immutable transform to its parent.
    * <p>
    * The {@code transformToParent} should describe the pose of the new frame expressed in its parent
    * frame.
    * </p>
    *
    * @param frameName         the name of the new frame.
    * @param parentFrame       the parent frame of the new reference frame.
    * @param transformToParent the transform that can be used to transform a geometry object the new
    *                          frame to its parent frame. Not modified.
    * @return the new reference frame.
    */
   public static ReferenceFrame constructFrameWithUnchangingTransformToParent(String frameName,
                                                                              ReferenceFrame parentFrame,
                                                                              RigidBodyTransformReadOnly transformToParent)
   {
      return new FixedReferenceFrame(frameName, parentFrame, transformToParent);
   }

   /**
    * Creates a reference frame with the transform from it's parent maintained by the user.
    * <p>
    *     * The {@code transformFromParent} should describe the pose of the parent frame expressed in this
    *     * new frame.
    *     * </p>
    * <p>
    *    Note: It <strong>is</strong> necessary to call update on this reference frame.
    * </p>
    *
    *
    * @param frameName           the name of the new frame.
    * @param parentFrame         the parent frame of the new reference frame.
    * @param transformFromParent the transform that can be used to transform a geometry object the new
    *                            frame from the parent frame to this frame. Not modified.
    * @return the new reference frame.
    */
   public static ReferenceFrame constructFrameWithChangingTransformFromParent(String frameName,
                                                                              ReferenceFrame parentFrame,
                                                                              RigidBodyTransformReadOnly transformFromParent)
   {
      RigidBodyTransform transformToParent = new RigidBodyTransform(transformFromParent);
      transformToParent.invert();
      boolean isZupFrame = parentFrame.isZupFrame() && transformToParent.isRotation2D();
      boolean isAStationaryFrame = parentFrame.isAStationaryFrame();

      return new ReferenceFrame(frameName, parentFrame, transformToParent, isAStationaryFrame, isZupFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParentToUpdate)
         {
            transformToParentToUpdate.set(transformFromParent);
            transformToParentToUpdate.invert();
         }
      };
   }

   /**
    * Creates a reference frame with the transform to it's parent maintained by the user.
    * <p>
    * The {@code transformToParent} should describe the pose of the new frame expressed in its parent
    * frame.
    * </p>
    * <p>
    *    Note: It <strong>is</strong> necessary to call update on this reference frame.
    * </p>
    *
    *
    * @param frameName         the name of the new frame.
    * @param parentFrame       the parent frame of the new reference frame.
    * @param transformToParent the transform that can be used to transform a geometry object the new
    *                          frame to its parent frame. Not modified.
    * @return the new reference frame.
    */
   public static ReferenceFrame constructFrameWithChangingTransformToParent(String frameName,
                                                                            ReferenceFrame parentFrame,
                                                                            RigidBodyTransformReadOnly transformToParent)
   {
      boolean isZupFrame = parentFrame.isZupFrame() && transformToParent.isRotation2D();
      boolean isAStationaryFrame = parentFrame.isAStationaryFrame();

      return new ReferenceFrame(frameName, parentFrame, transformToParent, isAStationaryFrame, isZupFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParentToUpdate)
         {
            transformToParentToUpdate.set(transformToParent);
         }
      };
   }

   /**
    * Creates an array containing all the reference frames starting from the root and ending at the
    * given {@code referenceFrame}.
    *
    * @param referenceFrame the reference frame to which the path ends. Not modified.
    * @return the path from root to {@code referenceFrame}.
    */
   public static ReferenceFrame[] createPathFromRoot(ReferenceFrame referenceFrame)
   {
      ReferenceFrame parentFrame = referenceFrame.getParent();
      if (parentFrame == null)
      {
         return new ReferenceFrame[] {referenceFrame};
      }

      int newLength = parentFrame.getFramesStartingWithRootEndingWithThis().length + 1;
      ReferenceFrame[] ret = Arrays.copyOf(parentFrame.getFramesStartingWithRootEndingWithThis(), newLength);
      ret[newLength - 1] = referenceFrame;
      return ret;
   }

   /**
    * Will clear the entire frame tree that this frame is part of leaving only the root frame enabled.
    * All other frames in the tree will be removed and disabled.
    * <p>
    * Note that reference frames are automatically disposed of by the GC when no external reference
    * exists.
    * </p>
    *
    * @param frame in the frame tree that will be cleared.
    */
   public static void clearFrameTree(ReferenceFrame frame)
   {
      frame.getRootFrame().clearChildren();
   }

   /**
    * Will clear the entire frame tree of the {@link ReferenceFrameTools#worldFrame} tree.
    * <p>
    * Note that reference frames are automatically disposed of by the GC when no external reference
    * exists.
    * </p>
    */
   public static void clearWorldFrameTree()
   {
      worldFrame.clearChildren();
   }

   /**
    * Will create a collection of all reference frames in the frame tree that the provided frame is
    * part of.
    *
    * @param frame in the reference frame tree of interest.
    * @return all frames in the reference frame tree.
    */
   public static Collection<ReferenceFrame> getAllFramesInTree(ReferenceFrame frame)
   {
      Collection<ReferenceFrame> frames = new ArrayList<>();
      frames.add(frame.getRootFrame());
      collectAllDescendants(frame.getRootFrame(), frames);
      return frames;
   }

   /**
    * Gets a list that contains all the reference frames, including {@code start}, of the subtree that
    * starts at {@code start}.
    * <p>
    * This method generates garbage.
    * </p>
    *
    * @param start ancestor of the subtree frames to collect.
    * @return the new subtree list.
    */
   public static List<ReferenceFrame> collectFramesInSubtree(ReferenceFrame start)
   {
      List<ReferenceFrame> frames = new ArrayList<>();
      frames.add(start);
      collectAllDescendants(start, frames);
      return frames;
   }

   private static void collectAllDescendants(ReferenceFrame frame, Collection<ReferenceFrame> collectionToPack)
   {
      for (int i = 0; i < frame.getNumberOfChildren(); i++)
      {
         ReferenceFrame child = frame.getChild(i);
         if (child != null)
         {
            collectionToPack.add(child);
            collectAllDescendants(child, collectionToPack);
         }
      }
   }
}
