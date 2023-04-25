package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.referenceFrame.ReferenceFrameChangedListener.Change;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

import java.lang.ref.WeakReference;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.function.Predicate;

/**
 * {@code ReferenceFrame} represents a reference coordinate frame.
 * <p>
 * {@code ReferenceFrame}s are organized as a tree structure. A root reference frame such
 * {@link #getWorldFrame()} represents a global coordinate system with not parent. From this root
 * frame, children frame, with constant or variable transforms to the root, can be added recursively
 * to form the tree structure. This structure allows to define each reference frame with a
 * unmodifiable parent frame and their transform, i.e, a {@code RigidBodyTransform} providing both
 * position and orientation, with respect to that same parent frame. The transform to the root
 * reference frame is then internally computed by computing the path from each reference frame to
 * the root.
 * </p>
 * <p>
 * The most common ways to create a new {@code ReferenceFrame} are:
 * <ul>
 * <li>Using one of the constructors to create either a root reference frame or a child reference
 * frame. In both cases, the method {@link #updateTransformToParent(RigidBodyTransform)} has to be
 * overrided. When creating a root reference frame, the method should be empty, while creating a
 * child reference frame the implementation of the method is used to update the transform of the
 * child frame with respect to its parent.
 * <li>Using one of the static methods to create a reference frame with an unchanging transform with
 * respect to its parent.
 * <li>Look for extensions of {@code ReferenceFrame}.
 * </ul>
 * </p>
 * <p>
 * Using {@code ReferenceFrame}, a geometry, such as a point, a line, or a convex polygon, can
 * easily be expressed in its local coordinate system and then transformed to know its coordinates
 * from another coordinate system perspective. For instance, the location of the toes of a bipedal
 * robot can be easily determined in the local foot reference frame, then can be easily transformed
 * to the world frame or pelvis frame to know its location from the perspective of world or the
 * pelvis.
 * </p>
 * <p>
 * {@code ReferenceFrame} is only the base class of the reference frame framework. Several classes
 * allows for avoiding the burden of tracking to what frame a geometry is attached and how to
 * express a geometry in a different frame.
 * </p>
 */
public abstract class ReferenceFrame
{
   /** A string used to separate frame names in the {@link #nameId} of the reference frame */
   public static final String SEPARATOR = ":";

   public static FrameNameRestrictionLevel DEFAULT_RESTRICTION_LEVEL = FrameNameRestrictionLevel.loadFromEnvironment("euclid.referenceFrame.restrictionLevel",
                                                                                                                     "FrameNameRestrictionLevel",
                                                                                                                     FrameNameRestrictionLevel.NONE);

   /** The name of this reference frame. The name should preferably be unique. */
   private final String frameName;

   /**
    * A string that can be used to identify the {@link ReferenceFrame}. It contains the name of the
    * frame itself and all parents up to the root frame and is used for the {@link #hashCode()} and
    * {@link #equals()} methods.
    * <p>
    * In contrast to the {@link #frameIndex} this is a name based identifier that is only dependent on
    * names of frames. Note, that this means that two frames with the same name will be considered
    * equal even though they might be in different locations.
    * </p>
    */
   private final String nameId;

   /**
    * An ID that can be used to identify a frame inside a tree of reference frames. Each frame inside a
    * tree will have a frame ID that is different from all other frames inside the tree.
    * <p>
    * It is more reliably then a hash code since there will be no collisions within a single frame
    * tree. The disadvantage is that it is dependent on the order of construction of frames such that
    * this index can change from run to run even if the name of the frame remains unchanged.
    * </p>
    */
   private final long frameIndex;

   /**
    * A counter for the number of frames in the reference frame tree that starts at this frame. Note,
    * that this counter does not account for frames that might be removed from the frame tree. It is
    * meant to account all frames that were ever added to this frame tree to provide a unique number
    * Identifier for each frame. The counter is rest to zero if the root frame is cleared.
    */
   private long framesAddedToTree = 0L;

   /**
    * Additional custom hash code representing this frame.
    * <p>
    * Somewhat of a hack that allows to enforce two frames that are physically the same but with
    * different names to have the same hash code or to enforce a common frame to have a specific hash
    * code that can be known without holding on its actual instance.
    * </p>
    */
   private long additionalNameBasedHashCode;

   /**
    * The reference to which this frame is attached to.
    * <p>
    * The {@link #transformToParent} of this describes the pose of this reference frame with respect to
    * {@link #parentFrame}.
    * </p>
    */
   private final ReferenceFrame parentFrame;

   /**
    * A collection of all children of this reference frame. The use of {@code WeakReference} allows the
    * garbage collector to dispose of the children that are not referenced outside this class.
    */
   private final List<WeakReference<ReferenceFrame>> children = new ArrayList<>();

   /**
    * Set containing the {@link #frameName} of this frame's {@link #children}.
    * This is only used when {@link #nameRestrictionLevel} is set to {@link FrameNameRestrictionLevel#NAME_ID}.
    */
   private final HashSet<String> childrenNames = new HashSet<>();

   /**
    * If this frame has restriction level {@link FrameNameRestrictionLevel#FRAME_NAME}, this object stores the highest
    * frame up the tree with this same restriction level.
    */
   private ReferenceFrame topFrameNameRestriction = null;

   /**
    * Set containing the {@link #frameName} of this and all subtree frames. Only populated if this frame has restriction level
    * {@link FrameNameRestrictionLevel#FRAME_NAME} and {@link this#parentFrame} has a lesser restriction.
    */
   private final HashSet<String> subtreeFrameNames = new HashSet<>();

   /**
    * Indicated if a frame is deactivated. This happens if the frame is removed from the frame tree. In
    * this case all references to the frame should be dropped so it can be garbage collected.
    */
   private boolean hasBeenRemoved = false;

   /**
    * Entire from the root frame to this used to efficiently compute the pose of this reference frame
    * with respect to the root frame.
    */
   private final ReferenceFrame[] framesStartingWithRootEndingWithThis;

   /**
    * The pose of this transform with respect to its parent.
    * <p>
    * Notes:
    * <ul>
    * The root frame has no parent such that {@code transformToParent == null}.
    * <ul>
    * The transform can be constant over time or can change depending on the final implementation of
    * {@code ReferenceFrame}.
    * </p>
    */
   private final RigidBodyTransform transformToParent;

   // These need to be longs instead of integers or they'll role over too soon. With longs, you get at least 100 years of runtime.
   static long nextTransformToRootID = 1;

   long transformToRootID = Long.MIN_VALUE;

   /**
    * The current transform from this reference frame to the root frame.
    * <p>
    * For instance, one can calculate the coordinates in the root frame P<sub>root</sub> of a point P
    * expressed in this frame as follows:<br>
    * {@code transformToRoot.transform}(P, P<sub>root</sub>)
    * </p>
    */
   private final RigidBodyTransform transformToRoot;
   private boolean accessingTransformToRoot = false;
   /** Condition for the root frame only. */
   private Predicate<ReferenceFrame> treeUpdateCondition = null;

   /**
    * Field initialized at construction time that specifies if this reference frame represents a
    * stationary frame, i.e. a non-moving frame, with respect to the root reference frame.
    */
   private final boolean isAStationaryFrame; // TODO when isAStationaryFrame == true, transformToParent should be immutable.

   /**
    * Field initialized at construction time that specifies if at all time the z-axis of this reference
    * frame remains aligned with the z-axis of the root frame.
    */
   private final boolean isZupFrame;
   /**
    * Whether this frame is rigid attached to its parent such that its transform is immutable.
    */
   private final boolean isFixedInParent;

   /**
    * List of active listeners currently attached to this frame. Only instantiated when adding the
    * first listener.
    */
   private List<ReferenceFrameChangedListener> changedListeners = null;

   /**
    * Restriction level for the reference frame's name.
    */
   private FrameNameRestrictionLevel nameRestrictionLevel;

   /**
    * Creates a new root reference frame.
    * <p>
    * Please use the method {@link ReferenceFrameTools#constructARootFrame(String)} instead. This is to
    * use only when extending this class.
    * </p>
    * <p>
    * Most of the time, {@link #getWorldFrame()} is the only root frame from which children reference
    * frames are added.
    * </p>
    * <p>
    * Note that frames added as children of this root frame belongs to a different reference frame tree
    * than the tree starting off of {@link #getWorldFrame()}. Transformation across two different trees
    * of reference frames is forbidden as the transformation between them is undefined.
    * </p>
    * <p>
    * The parent frame and transforms of a root frame are all {@code null}.
    * </p>
    *
    * @param frameName the name of the new world frame.
    */
   public ReferenceFrame(String frameName)
   {
      this(frameName, null, null, true, true);
   }

   /**
    * Creates a new reference frame defined as being a child of the given {@code parentFrame}.
    * <p>
    * This new reference frame defined in the {@code parentFrame} and moves with it.
    * </p>
    * <p>
    * Its pose with respect to the {@code parentFrame} can be modified at runtime by changing the
    * transform in the method {@link #updateTransformToParent(RigidBodyTransform)} when overriding it.
    * </p>
    * <p>
    * This new reference frame is not a stationary frame, i.e. it is assumed to be potentially moving
    * with respect to the root frame. It is also not expected to have its z-axis aligned at all time
    * with the z-axis of the root frame.
    * </p>
    *
    * @param frameName   the name of the new frame.
    * @param parentFrame the parent frame of the new reference frame.
    */
   public ReferenceFrame(String frameName, ReferenceFrame parentFrame)
   {
      this(frameName, parentFrame, null, false, false);
   }

   /**
    * Creates a new reference frame defined as being a child of the given {@code parentFrame}.
    * <p>
    * This new reference frame defined in the {@code parentFrame} and moves with it.
    * </p>
    * <p>
    * Its pose with respect to the {@code parentFrame} can be modified at runtime by changing the
    * transform in the method {@link #updateTransformToParent(RigidBodyTransform)} when overriding it.
    * </p>
    *
    * @param frameName          the name of the new frame.
    * @param parentFrame        the parent frame of the new reference frame.
    * @param isAStationaryFrame refers to whether this new frame is stationary with respect to the root
    *                           frame or moving. If {@code true}, the {@code parentFrame} has to also
    *                           be stationary.
    * @param isZupFrame         refers to whether this new frame has its z-axis aligned with the root
    *                           frame at all time or not.
    * @throws IllegalArgumentException if {@code isAStationaryFrame} is {@code true} and the
    *                                  {@code parentFrame} is not a stationary frame.
    */
   public ReferenceFrame(String frameName, ReferenceFrame parentFrame, boolean isAStationaryFrame, boolean isZupFrame)
   {
      this(frameName, parentFrame, null, isAStationaryFrame, isZupFrame);
   }

   /**
    * Creates a new reference frame defined as being a child of the given {@code parentFrame} and
    * initializes the transform to its parent.
    * <p>
    * The {@code transformFromParent} should describe the pose of the new frame expressed in its parent
    * frame.
    * </p>
    * <p>
    * This new reference frame defined in the {@code parentFrame} and moves with it.
    * </p>
    * <p>
    * Its pose with respect to the {@code parentFrame} can be modified at runtime by changing the
    * transform in the method {@link #updateTransformToParent(RigidBodyTransform)} when overriding it.
    * </p>
    * <p>
    * This new reference frame is not a stationary frame, i.e. it is assumed to be potentially moving
    * with respect to the root frame. It is also not expected to have its z-axis aligned at all time
    * with the z-axis of the root frame.
    * </p>
    *
    * @param frameName         the name of the new frame.
    * @param parentFrame       the parent frame of the new reference frame.
    * @param transformToParent the transform that can be used to transform a geometry object the new
    *                          frame to its parent frame. Not modified.
    */
   public ReferenceFrame(String frameName, ReferenceFrame parentFrame, RigidBodyTransformReadOnly transformToParent)
   {
      this(frameName, parentFrame, transformToParent, false, false);
   }

   /**
    * Creates a new reference frame defined as being a child of the given {@code parentFrame} and
    * initializes the transform to its parent.
    * <p>
    * The {@code transformFromParent} should describe the pose of the new frame expressed in its parent
    * frame.
    * </p>
    * <p>
    * This new reference frame defined in the {@code parentFrame} and moves with it.
    * </p>
    * <p>
    * Its pose with respect to the {@code parentFrame} can be modified at runtime by changing the
    * transform in the method {@link #updateTransformToParent(RigidBodyTransform)} when overriding it.
    * </p>
    *
    * @param frameName          the name of the new frame.
    * @param parentFrame        the parent frame of the new reference frame.
    * @param transformToParent  the transform that can be used to transform a geometry object the new
    *                           frame to its parent frame. Not modified.
    * @param isAStationaryFrame refers to whether this new frame is stationary with respect to the root
    *                           frame or moving. If {@code true}, the {@code parentFrame} has to also
    *                           be stationary.
    * @param isZupFrame         refers to whether this new frame has its z-axis aligned with the root
    *                           frame at all time or not.
    * @throws IllegalArgumentException if {@code isAStationaryFrame} is {@code true} and the
    *                                  {@code parentFrame} is not a stationary frame.
    */
   public ReferenceFrame(String frameName,
                         ReferenceFrame parentFrame,
                         RigidBodyTransformReadOnly transformToParent,
                         boolean isAStationaryFrame,
                         boolean isZupFrame)
   {
      this(frameName, parentFrame, transformToParent, isAStationaryFrame, isZupFrame, false);
   }

   /**
    * Creates a new reference frame defined as being a child of the given {@code parentFrame} and
    * initializes the transform to its parent.
    * <p>
    * The {@code transformFromParent} should describe the pose of the new frame expressed in its parent
    * frame.
    * </p>
    * <p>
    * This new reference frame defined in the {@code parentFrame} and moves with it.
    * </p>
    * <p>
    * Its pose with respect to the {@code parentFrame} can be modified at runtime by changing the
    * transform in the method {@link #updateTransformToParent(RigidBodyTransform)} when overriding it.
    * </p>
    *
    * @param frameName          the name of the new frame.
    * @param parentFrame        the parent frame of the new reference frame.
    * @param transformToParent  the transform that can be used to transform a geometry object the new
    *                           frame to its parent frame. Not modified.
    * @param isAStationaryFrame refers to whether this new frame is stationary with respect to the root
    *                           frame or moving. If {@code true}, the {@code parentFrame} has to also
    *                           be stationary.
    * @param isZupFrame         refers to whether this new frame has its z-axis aligned with the root
    *                           frame at all time or not.
    * @param isFixedInParent    refers to whether this new frame should be considered as rigidly
    *                           attached to its parent, its pose is constant. This class does not use
    *                           this flag internally, it can be later accessed via
    *                           {@link #isFixedInParent()}.
    * @throws IllegalArgumentException if {@code isAStationaryFrame} is {@code true} and the
    *                                  {@code parentFrame} is not a stationary frame.
    */
   public ReferenceFrame(String frameName,
                         ReferenceFrame parentFrame,
                         RigidBodyTransformReadOnly transformToParent,
                         boolean isAStationaryFrame,
                         boolean isZupFrame,
                         boolean isFixedInParent)
   {
      if (frameName.contains(SEPARATOR))
      {
         throw new RuntimeException("A reference frame name can not contain '" + SEPARATOR + "'. Tried to construct a frame with name " + frameName + ".");
      }

      this.frameName = frameName;
      this.parentFrame = parentFrame;
      framesStartingWithRootEndingWithThis = ReferenceFrameTools.createPathFromRoot(this);

      if (parentFrame == null)
      { // Setting up this ReferenceFrame as a root frame.
         transformToRootID = 0;
         nameId = frameName;
         frameIndex = 0L;

         transformToRoot = null;
         this.transformToParent = null;

         this.isAStationaryFrame = true;
         this.isZupFrame = true;
         this.isFixedInParent = true;
         this.nameRestrictionLevel = DEFAULT_RESTRICTION_LEVEL;

         if (nameRestrictionLevel == FrameNameRestrictionLevel.FRAME_NAME)
         {
            topFrameNameRestriction = this;
            subtreeFrameNames.add(frameName);
         }
      }
      else
      {
         parentFrame.checkIfRemoved();
         nameId = parentFrame.nameId + SEPARATOR + frameName;
         nameRestrictionLevel = parentFrame.nameRestrictionLevel;

         if (nameRestrictionLevel == FrameNameRestrictionLevel.FRAME_NAME)
            topFrameNameRestriction = parentFrame.topFrameNameRestriction;

         checkAndProcessNewFrameName();

         frameIndex = parentFrame.incrementFramesAdded();
         parentFrame.children.add(new WeakReference<>(this));

         transformToRoot = new RigidBodyTransform();
         this.transformToParent = new RigidBodyTransform();

         if (transformToParent != null)
         {
            this.transformToParent.set(transformToParent);
            this.transformToParent.normalizeRotationPart();
         }

         if (isAStationaryFrame && !parentFrame.isAStationaryFrame)
            throw new IllegalArgumentException("The child of a non-stationary frame cannot be stationary.");

         this.isAStationaryFrame = isAStationaryFrame;
         this.isZupFrame = isZupFrame;
         this.isFixedInParent = isFixedInParent;

         notifyListeners(ChangeType.FRAME_ADDED, this, parentFrame);
      }
   }

   /**
    * <p>
    * Sets the level of naming restriction for the sub-tree of reference frames at and below this frame.
    * </p>
    * <p>
    * Three levels of restriction are possible. Restriction is only allowed to increase.
    * </p>
    * <ul>
    *    <li>NONE: no restrictions on reference frame names are imposed</li>
    *    <li>NAME_ID: each fully qualified nameId must be unique, e.g. "World:elevator:pelvis" </li>
    *    <li>FRAME_NAME: each frame name must be unique, e.g. "afterKneePitchFrame" </li>
    * </ul>
    */
   public void setNameRestrictionLevel(FrameNameRestrictionLevel nameRestrictionLevel)
   {
      if (this.nameRestrictionLevel == nameRestrictionLevel)
         return;

      if (nameRestrictionLevel.ordinal() < this.nameRestrictionLevel.ordinal())
      {
         if (parentFrame == null && children.isEmpty())
         { // allow decreased restriction for root frames without children
            topFrameNameRestriction = null;
            this.nameRestrictionLevel = nameRestrictionLevel;
            subtreeFrameNames.clear();
         }
         else
         { // otherwise decreasing restriction level is not allowed
            throw new IllegalArgumentException("Cannot reduce name restriction level. Current mode: " + this.nameRestrictionLevel + ", tried to set to: " + nameRestrictionLevel);
         }
      }

      updateChildren();
      setAndCheckRestrictionLevelRecursively(nameRestrictionLevel, this);
   }

   private void setAndCheckRestrictionLevelRecursively(FrameNameRestrictionLevel nameRestrictionLevel, ReferenceFrame frameWithNewRestrictionLevel)
   {
      this.nameRestrictionLevel = nameRestrictionLevel;

      if (nameRestrictionLevel == FrameNameRestrictionLevel.NAME_ID)
      {
         for (int i = 0; i < children.size(); i++)
         {
            ReferenceFrame childFrame = children.get(i).get();
            if (childFrame == null)
               continue;
            if (!childrenNames.add(childFrame.frameName))
               throw new RuntimeException("Duplicate ReferenceFrame nameId's detected: " + nameId);
         }
      }
      else if (nameRestrictionLevel == FrameNameRestrictionLevel.FRAME_NAME)
      {
         topFrameNameRestriction = frameWithNewRestrictionLevel;
         if (!topFrameNameRestriction.subtreeFrameNames.add(frameName))
            throw new RuntimeException("Duplicate ReferenceFrame names detected: " + frameName);
      }

      for (int i = 0; i < children.size(); i++)
      {
         ReferenceFrame childFrame = children.get(i).get();
         if (childFrame != null)
            childFrame.setAndCheckRestrictionLevelRecursively(nameRestrictionLevel, frameWithNewRestrictionLevel);
      }
   }

   /**
    * When a new reference frame is created, this checks the validity of the frame's name given the {@link #nameRestrictionLevel}.
    */
   private void checkAndProcessNewFrameName()
   {
      if (nameRestrictionLevel == FrameNameRestrictionLevel.NAME_ID)
      {
         if (!parentFrame.childrenNames.add(frameName))
            throw new RuntimeException("Duplicate reference frame: " + nameId);
      }
      else if (nameRestrictionLevel == FrameNameRestrictionLevel.FRAME_NAME)
      {
         if (topFrameNameRestriction.subtreeFrameNames.contains(frameName))
            throw new RuntimeException("Frame " + frameName + " already exists: " + nameId);

         topFrameNameRestriction.subtreeFrameNames.add(frameName);
      }
   }

   public FrameNameRestrictionLevel getNameRestrictionLevel()
   {
      checkIfRemoved();
      return nameRestrictionLevel;
   }

   private long incrementFramesAdded()
   {
      framesAddedToTree++;

      if (parentFrame == null)
      {
         return framesAddedToTree;
      }
      else
      {
         return parentFrame.incrementFramesAdded();
      }
   }

   /**
    * Tests if this reference frame is {@link #getWorldFrame()}.
    *
    * @return {@code true} if this is {@link #getWorldFrame()}, {@code false} otherwise.
    */
   public boolean isWorldFrame()
   {
      checkIfRemoved();
      return this == ReferenceFrameTools.getWorldFrame();
   }

   /**
    * Tests if this reference frame is the root, i.e. no parent frame, of its reference frame tree.
    *
    * @return {@code true} if this is a root frame, {@code false} otherwise.
    */
   public boolean isRootFrame()
   {
      checkIfRemoved();
      return parentFrame == null;
   }

   /**
    * Tests if this reference frame is to be considered as stationary frame, i.e. not moving with
    * respect to its root frame.
    *
    * @return {@code true} if this is a stationary frame, {@code false} other.
    */
   public boolean isAStationaryFrame()
   {
      checkIfRemoved();
      return isAStationaryFrame;
   }

   /**
    * Tests if this reference frame is considered to have its z-axis aligned with the root frame.
    *
    * @return {@code true} if this is a z-up frame, {@code false} otherwise.
    */
   public boolean isZupFrame()
   {
      checkIfRemoved();
      return isZupFrame;
   }

   /**
    * Returns whether this reference frame should be considered as rigidly attached to its parent, i.e.
    * its transform relative to its parent can be considered as constant.
    * 
    * @return {@code true} if this frame can be considered as fixed in parent.
    */
   public boolean isFixedInParent()
   {
      checkIfRemoved();
      return isFixedInParent;
   }

   /**
    * The user must call update each tick. It will then call
    * {@link #updateTransformToParent(RigidBodyTransform)} which should be overridden to indicate how
    * the transform to each frame's parent should be updated.
    * <p>
    * Note that it is not necessary to call update on reference frames with an unchanging transform to
    * parent, even if the parent frame is moving.
    * </p>
    */
   public void update()
   {
      checkIfRemoved();

      if (parentFrame == null)
      {
         return;
      }

      updateTransformToParent(transformToParent);
      transformToRootID = Long.MIN_VALUE;
   }

   /**
    * Override this method to define how this reference frame should be located with respect to its
    * parent frame over time by setting the argument {@code transformToParent}.
    * <p>
    * The {@code transformFromParent} should describe the pose of this frame expressed in its parent
    * frame.
    * </p>
    *
    * @param transformToParent the transform to updated according to how this reference frame should
    *                          now positioned with respect to its parent frame. Modified.
    */
   protected abstract void updateTransformToParent(RigidBodyTransform transformToParent);

   /**
    * Returns the parent frame of this reference frame.
    * <p>
    * Note that a root frame has no parent frame, such that this method returns {@code null} if this is
    * a root frame.
    * </p>
    *
    * @return the parent frame of this reference frame.
    */
   public ReferenceFrame getParent()
   {
      checkIfRemoved();
      return parentFrame;
   }

   /**
    * Retrieves the root frame of the tree of reference frame that this frame belongs to.
    *
    * @return the root frame.
    */
   public ReferenceFrame getRootFrame()
   {
      checkIfRemoved();
      return framesStartingWithRootEndingWithThis[0];
   }

   /**
    * Returns a copy of this reference frame's transform to parent.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * This transform can be applied to a vector defined in this frame in order to obtain the equivalent
    * vector in the parent frame.
    * </p>
    *
    * @return a copy of the transform to the parent frame.
    */
   public RigidBodyTransform getTransformToParent()
   {
      RigidBodyTransform transformToReturn = new RigidBodyTransform();
      getTransformToParent(transformToReturn);
      return transformToReturn;
   }

   /**
    * packs this reference frame's transform to parent into the given transform
    * {@code transformToPack}.
    * <p>
    * This transform can be applied to a vector defined in this frame in order to obtain the equivalent
    * vector in the parent frame.
    * </p>
    *
    * @param transformToPack the transform in which this frame's transform to its parent frame is
    *                        stored. Modified.
    */
   // TODO For backward compatibility. Remove me in future release.
   public void getTransformToParent(RigidBodyTransform transformToPack)
   {
      checkIfRemoved();
      transformToPack.set(transformToParent);
   }

   /**
    * packs this reference frame's transform to parent into the given transform
    * {@code transformToPack}.
    * <p>
    * This transform can be applied to a vector defined in this frame in order to obtain the equivalent
    * vector in the parent frame.
    * </p>
    *
    * @param transformToPack the transform in which this frame's transform to its parent frame is
    *                        stored. Modified.
    */
   public void getTransformToParent(RigidBodyTransformBasics transformToPack)
   {
      checkIfRemoved();
      transformToPack.set(transformToParent);
   }

   /**
    * Gets the name of this reference frame.
    * <p>
    * Reference frames usually have a unique name among the reference frames in the same tree but this
    * is not guaranteed.
    * </p>
    *
    * @return this frame's name.
    */
   public String getName()
   {
      checkIfRemoved();
      return frameName;
   }

   /**
    * A string that can be used to identify this reference frame.
    * <p>
    * It contains the name of the frame itself and all parents up to the root frame and is used for the
    * {@link #hashCode()} and {@link #equals(Object)} methods.
    * </p>
    * <p>
    * In contrast to the {@link #frameIndex} this is a name based identifier that is only dependent on
    * names of frames. Note, that this means that two frames with the same name will be considered
    * equal even though they might be in different locations.
    * </p>
    * 
    * @return the name identifier for this reference frame.
    */
   public String getNameId()
   {
      checkIfRemoved();
      return nameId;
   }

   /**
    * Returns the transform that can be used to transform a geometry object defined in this frame to
    * obtain its equivalent expressed in the {@code desiredFrame}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param desiredFrame the goal frame.
    * @return the transform from this frame to the {@code desiredFrame}.
    */
   public RigidBodyTransform getTransformToDesiredFrame(ReferenceFrame desiredFrame)
   {
      RigidBodyTransform ret = new RigidBodyTransform();
      getTransformToDesiredFrame(ret, desiredFrame);

      return ret;
   }

   /**
    * Returns the transform that can be used to transform a geometry object defined in this frame to
    * obtain its equivalent expressed in {@link #getWorldFrame()}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @return the transform from this frame to the {@link #getWorldFrame()}.
    */
   public RigidBodyTransform getTransformToWorldFrame()
   {
      RigidBodyTransform ret = new RigidBodyTransform();
      getTransformToDesiredFrame(ret, ReferenceFrameTools.getWorldFrame());
      return ret;
   }

   /**
    * Packs the transform that can be used to transform a geometry object defined in this frame to
    * obtain its equivalent expressed in the {@code desiredFrame} into {@code transformToPack}.
    *
    * @param transformToPack the transform in which this frame's transform to the {@code desiredFrame}
    *                        is stored. Modified.
    * @param desiredFrame    the goal frame.
    */
   // TODO For backward compatibility. Remove me in future release.
   public void getTransformToDesiredFrame(RigidBodyTransform transformToPack, ReferenceFrame desiredFrame)
   {
      getTransformToDesiredFrame((RigidBodyTransformBasics) transformToPack, desiredFrame);
   }

   /**
    * Packs the transform that can be used to transform a geometry object defined in this frame to
    * obtain its equivalent expressed in the {@code desiredFrame} into {@code transformToPack}.
    *
    * @param transformToPack the transform in which this frame's transform to the {@code desiredFrame}
    *                        is stored. Modified.
    * @param desiredFrame    the goal frame.
    */
   public void getTransformToDesiredFrame(RigidBodyTransformBasics transformToPack, ReferenceFrame desiredFrame)
   {
      checkIfRemoved();

      try
      {
         if (this == desiredFrame)
         { // Check for trivial case
            transformToPack.setToZero();
            return;
         }

         verifySameRoots(desiredFrame);

         // The general approach is:
         // transformToPack = (desiredFrame.transformToRoot)^-1 * this.transformToRoot
         // As this requires a transform multiplication, we first check for simpler cases:
         if (isRootFrame())
         {
            /*
             * If this is the root frame, desiredFrame cannot be the root frame, i.e. it would have triggered
             * the previous condition as there can be only one root per frame tree. Thus: this.transformToRoot
             * is the identity, no need for a multiplication here.
             */
            transformToPack.setAndInvert(desiredFrame.getTransformToRoot());
         }
         else if (desiredFrame.isRootFrame())
         {
            /*
             * If desiredFrame is the root frame, this cannot be the root frame, i.e. it would have triggered
             * the previous condition as there can be only one root per frame tree. Thus:
             * desiredFrame.transformToRoot is the identity, no need for a multiplication here.
             */
            transformToPack.set(getTransformToRoot());
         }
         else if (isParentFrame(desiredFrame))
         { // Test direct connection between the frames:
            transformToPack.set(transformToParent);
         }
         else if (desiredFrame.isParentFrame(this))
         { // Test direct connection between the frames:
            transformToPack.setAndInvert(desiredFrame.transformToParent);
         }
         else if (parentFrame == desiredFrame.parentFrame)
         {
            /*
             * Common parentFrame. Here the multiplication is needed but the transforms involved will often be
             * simple (rotation only or translation only) whereas the transformToRoot of most frame is a complex
             * transform.
             */
            transformToPack.setAndInvert(desiredFrame.transformToParent);
            transformToPack.multiply(transformToParent);
         }
         else if (parentFrame.parentFrame == desiredFrame)
         { // Look at a distance of 2, which would involve the multiplication of 2 transforms that will often be simple (rotation only or translation only).
            transformToPack.set(transformToParent);
            if (!parentFrame.isRootFrame()) // If it is the root, then parentFrame.transformToParent is identity.
               transformToPack.preMultiply(parentFrame.transformToParent);
         }
         else if (this == desiredFrame.parentFrame.parentFrame)
         { // Look at a distance of 2, which would involve the multiplication of 2 transforms that will often be simple (rotation only or translation only).
            transformToPack.setAndInvert(desiredFrame.transformToParent);
            if (!desiredFrame.parentFrame.isRootFrame()) // If it is the root, then desiredFrame.parentFrame.transformToParent is identity.
               transformToPack.multiplyInvertOther(desiredFrame.parentFrame.transformToParent);
         }
         else
         { // This is the general scenario:
            transformToPack.setAndInvert(desiredFrame.getTransformToRoot());
            transformToPack.multiply(getTransformToRoot());
         }
      }
      catch (NotARotationMatrixException e)
      {
         throw new NotARotationMatrixException("Caught exception, this frame: " + frameName + ", other frame: " + desiredFrame.getName() + ", exception:\n"
               + e.getMessage());
      }
   }

   /**
    * Test whether the given frame is the parent of this frame.
    *
    * @param frame the query.
    * @return {@code true} if the query is the parent of this frame, {@code false} otherwise.
    */
   public boolean isParentFrame(ReferenceFrame frame)
   {
      checkIfRemoved();
      return frame == parentFrame;
   }

   /**
    * Test whether the given frame is a child of this frame.
    *
    * @param frame the query.
    * @return {@code true} if the query is a child of this frame, {@code false} otherwise.
    */
   public boolean isChildFrame(ReferenceFrame frame)
   {
      checkIfRemoved();
      return frame.isParentFrame(this);
   }

   /**
    * Asserts that this frame and {@code referenceFrame} share the same root frame.
    *
    * @param referenceFrame the query.
    * @throws RuntimeException if this frame and the query do not share the same root frame.
    */
   public void verifySameRoots(ReferenceFrame referenceFrame)
   {
      if (getRootFrame() != referenceFrame.getRootFrame())
      {
         throw new RuntimeException("Frames do not have same roots. this = " + this + ", referenceFrame = " + referenceFrame);
      }
   }

   /**
    * Asserts that {@code referenceFrame} is an ancestor of this frame.
    *
    * @param referenceFrame the query.
    * @throws RuntimeException if this frame and the query do not share the same root frame.
    */
   public void verifyIsAncestor(ReferenceFrame referenceFrame)
   {
      if (isRootFrame() || !parentFrame.checkIsAncestorRecursively(referenceFrame))
      {
         throw new RuntimeException(referenceFrame.getNameId() + " is not an ancestor of " + getNameId());
      }
   }

   private boolean checkIsAncestorRecursively(ReferenceFrame referenceFrame)
   {
      if (this == referenceFrame)
      {
         return true;
      }
      else if (isRootFrame())
      {
         return false;
      }
      else
      {
         return parentFrame.checkIsAncestorRecursively(referenceFrame);
      }
   }

   /**
    * Transforms the given {@code objectToTransform} by the transform from this reference frame to the
    * given {@code desiredFrame}.
    * <p>
    * This method can be used to change the reference frame in which {@code objectToTransform} is
    * expressed from {@code this} to {@code desiredFrame}.
    * </p>
    * <p>
    * <b>The given implementation of the given {@code Transformable} should check for
    * {@link RigidBodyTransform#hasRotation()} and {@link RigidBodyTransform#hasTranslation()} to
    * perform the transformation efficiently.</b>
    * </p>
    *
    * @param desiredFrame      the target frame for the transformation.
    * @param objectToTransform the object to apply the transformation on. Modified.
    */
   public void transformFromThisToDesiredFrame(ReferenceFrame desiredFrame, Transformable objectToTransform)
   {
      checkIfRemoved();

      if (this == desiredFrame)
      { // Check for trivial case
         return;
      }

      verifySameRoots(desiredFrame);

      // The general approach is:
      // objectToTransform = (desired.transformToRoot)^-1 * this.transformToRoot * objectToTransform
      // Or code-wise:
      // 1- objectToTransform.applyTransform(transformToRoot);
      // 2- objectToTransform.applyInverseTransform(desiredFrame.transformToRoot);
      // As this requires 2 transformations, we first check for simpler cases:
      if (isRootFrame())
      {
         /*
          * If this is the root frame, desiredFrame cannot be the root frame, i.e. it would have triggered
          * the previous condition as there can be only one root per frame tree. Thus: this.transformToRoot
          * is the identity, only 1 transformation here.
          */
         objectToTransform.applyInverseTransform(desiredFrame.getTransformToRoot());
      }
      else if (desiredFrame.isRootFrame())
      {
         /*
          * If desiredFrame is the root frame, this cannot be the root frame, i.e. it would have triggered
          * the previous condition as there can be only one root per frame tree. Thus:
          * desiredFrame.transformToRoot is the identity, only 1 transformation here.
          */
         objectToTransform.applyTransform(getTransformToRoot());
      }
      else if (isParentFrame(desiredFrame))
      { // Test direct connection between the frames:
         objectToTransform.applyTransform(transformToParent);
      }
      else if (desiredFrame.isParentFrame(this))
      { // Test direct connection between the frames:
         objectToTransform.applyInverseTransform(desiredFrame.transformToParent);
      }
      else if (parentFrame == desiredFrame.parentFrame)
      {
         /*
          * Common parentFrame. Here 2 transformations are needed but the transforms involved will often be
          * simple (rotation only or translation only) whereas the transformToRoot of most frame is a complex
          * transform.
          */
         objectToTransform.applyTransform(transformToParent);
         objectToTransform.applyInverseTransform(desiredFrame.transformToParent);
      }
      else if (parentFrame.parentFrame == desiredFrame)
      { // Look at a distance of 2, which involves 2 transformations with transforms that will often be simple (rotation only or translation only).

         objectToTransform.applyTransform(transformToParent);
         if (!parentFrame.isRootFrame()) // If it is the root, then parentFrame.transformToParent is identity.
            objectToTransform.applyTransform(parentFrame.transformToParent);
      }
      else if (this == desiredFrame.parentFrame.parentFrame)
      { // Look at a distance of 2, which involves 2 transformations with transforms that will often be simple (rotation only or translation only).
         if (!desiredFrame.parentFrame.isRootFrame()) // If it is the root, then desiredFrame.parentFrame.transformToParent is identity.
            objectToTransform.applyInverseTransform(desiredFrame.parentFrame.transformToParent);
         objectToTransform.applyInverseTransform(desiredFrame.transformToParent);
      }
      else
      { // This is the general scenario:
         objectToTransform.applyTransform(getTransformToRoot());
         objectToTransform.applyInverseTransform(desiredFrame.getTransformToRoot());
      }
   }

   /**
    * Returns the internal reference to this frame's transform to the root frame.
    * <p>
    * The transform can be used to transform a geometry object defined in this frame to obtain its
    * equivalent expressed in the root frame.
    * </p>
    *
    * @return the internal reference to the transform from this frame to the root frame.
    */
   public RigidBodyTransform getTransformToRoot()
   {
      efficientComputeTransform();
      return transformToRoot;
   }

   private void efficientComputeTransform()
   {
      Predicate<ReferenceFrame> treeUpdateCondition = framesStartingWithRootEndingWithThis[0].treeUpdateCondition;

      if (treeUpdateCondition != null && !treeUpdateCondition.test(this))
         return;

      checkIfRemoved();

      int chainLength = framesStartingWithRootEndingWithThis.length;

      boolean updateFromHereOnOut = false;
      long previousUpdateId = 0;

      for (int i = 0; i < chainLength; i++)
      {
         ReferenceFrame referenceFrame = framesStartingWithRootEndingWithThis[i];

         if (!updateFromHereOnOut)
         {
            if (referenceFrame.transformToRootID < previousUpdateId)
            {
               updateFromHereOnOut = true;
               nextTransformToRootID++;
            }
         }

         if (updateFromHereOnOut)
         {
            if (referenceFrame.parentFrame != null)
            {
               if (referenceFrame.accessingTransformToRoot)
               { // We have concurrent access of the transform to root, let's abort the update.
                  return;
               }

               try
               {
                  referenceFrame.accessingTransformToRoot = true;

                  RigidBodyTransform parentsTransformToRoot = referenceFrame.parentFrame.transformToRoot;

                  if (parentsTransformToRoot != null)
                  {
                     if (referenceFrame.parentFrame.accessingTransformToRoot)
                     { // We have concurrent access of the transform to root, let's abort the update.
                        return;
                     }

                     referenceFrame.parentFrame.accessingTransformToRoot = true;
                     try
                     {
                        referenceFrame.transformToRoot.set(parentsTransformToRoot);
                     }
                     finally
                     {
                        referenceFrame.parentFrame.accessingTransformToRoot = false;
                     }
                     referenceFrame.transformToRoot.multiply(referenceFrame.transformToParent);
                     referenceFrame.transformToRoot.normalizeRotationPart();
                  }
                  else
                  {
                     referenceFrame.transformToRoot.set(referenceFrame.transformToParent);
                  }

                  referenceFrame.transformToRootID = nextTransformToRootID;
               }
               finally
               {
                  referenceFrame.accessingTransformToRoot = false;
               }
            }
         }

         previousUpdateId = referenceFrame.transformToRootID;
      }
   }

   /**
    * Overrides the {@link Object#toString()} method to print this reference frame's name.
    *
    * @return this frame's name.
    */
   @Override
   public String toString()
   {
      checkIfRemoved();
      return frameName; // + "\nTransform to Parent = " + this.transformToParent;
   }

   /**
    * Checks if the query holds onto this reference frame.
    * <p>
    * This is usually used from verifying that a geometry is expressed in a specific frame.
    * </p>
    *
    * @param referenceFrameHolder the query holding a reference frame.
    * @throws ReferenceFrameMismatchException if the query holds onto a different frame than this.
    */
   public void checkReferenceFrameMatch(ReferenceFrameHolder referenceFrameHolder) throws ReferenceFrameMismatchException
   {
      checkReferenceFrameMatch(referenceFrameHolder.getReferenceFrame());
   }

   /**
    * Check if this frame and the query are the same.
    *
    * @param referenceFrame the query.
    * @throws ReferenceFrameMismatchException if the query and this are two different frame.
    */
   public void checkReferenceFrameMatch(ReferenceFrame referenceFrame) throws ReferenceFrameMismatchException
   {
      checkIfRemoved();
      if (this != referenceFrame)
      {
         String msg = "Argument's frame " + referenceFrame + " does not match " + this;

         throw new ReferenceFrameMismatchException(msg);
      }
   }

   /**
    * Checks if this frame is equal to {@link #getWorldFrame()}.
    *
    * @throws RuntimeException if this is not {@link #getWorldFrame()}.
    */
   public void checkIsWorldFrame() throws RuntimeException
   {
      checkIfRemoved();
      if (!isWorldFrame())
      {
         throw new RuntimeException("Frame " + this + " is not world frame.");
      }
   }

   /**
    * Checks if this is a stationary frame, i.e. not moving with respect to the root frame.
    *
    * @throws RuntimeException if this is not a stationary frame.
    */
   public void checkIsAStationaryFrame() throws RuntimeException
   {
      checkIfRemoved();
      if (!isAStationaryFrame())
      {
         throw new RuntimeException("Frame " + this + " is not a stationary frame.");
      }
   }

   /**
    * Checks if this is a z-up frame, i.e. its z-axis is aligned with the root frame's z-axis.
    *
    * @throws RuntimeException if this is not a z-up frame.
    */
   public void checkIsAZUpFrame() throws RuntimeException
   {
      checkIfRemoved();
      if (!isZupFrame())
      {
         throw new RuntimeException("Frame " + this + " is not a z-up frame.");
      }
   }

   /**
    * The hash code of a reference frame is based on the {@link #nameId} of the frame. This means that
    * the hash code will be equal for two distinct frames that have the same name. To differentiate all
    * frames in a tree regardless of their name use the {@link #getFrameIndex()} method.
    *
    * @return the hash code of the {@link #nameId} of this frame.
    */
   @Override
   public int hashCode()
   {
      checkIfRemoved();
      return nameId.hashCode();
   }

   /**
    * A hash code computed from the {@link #frameName}, which is only guarunteed to be unique within subtrees that have
    * {@link FrameNameRestrictionLevel#FRAME_NAME}.
    *
    * @return the hash code of the {@link #frameName} of this frame.
    */
   public int getFrameNameHashCode()
   {
      checkIfRemoved();
      return frameName.hashCode();
   }

   /**
    * This method will return true if the provided object is a reference frame with the same
    * {@link #nameId} as this frame. This means that two distinct frames with the same name are
    * considered equal. To differentiate all frames in a tree regardless of their name use the
    * {@link #getFrameIndex()} method.
    *
    * @return {@code true} if the provided object is of type {@link ReferenceFrame} and its
    *         {@link #nameId} matches this.
    */
   @Override
   public boolean equals(Object other)
   {
      checkIfRemoved();
      if (other instanceof ReferenceFrame)
      {
         return ((ReferenceFrame) other).nameId.equals(nameId);
      }
      return false;
   }

   /**
    * Gets the {@link #frameIndex} of this reference frame. The frame index is a unique number that
    * identifies each reference frame within a reference frame tree. No two frames inside a frame tree
    * can have the same index.
    *
    * @return the frame index that is unique in the frame tree that this frame is part of.
    */
   public long getFrameIndex()
   {
      checkIfRemoved();
      return frameIndex;
   }

   /**
    * Gets the value of this frame's custom hash code.
    * <p>
    * Somewhat of a hack that allows to enforce two frames that are physically the same but with
    * different names to have the same hash code or to enforce a common frame to have a specific hash
    * code that can be known without holding on its actual instance.
    * </p>
    *
    * @return the name based hash code for this reference frame.
    */
   public long getAdditionalNameBasedHashCode()
   {
      checkIfRemoved();
      return additionalNameBasedHashCode;
   }

   /**
    * Sets this frame's custom hash code's value.
    * <p>
    * Somewhat of a hack that allows to enforce two frames that are physically the same but with
    * different names to have the same hash code or to enforce a common frame to have a specific hash
    * code that can be known without holding on its actual instance.
    * </p>
    *
    * @param additionalNameBasedHashCode the new value of this frame's custom hash code.
    */
   public void setAdditionalNameBasedHashCode(long additionalNameBasedHashCode)
   {
      checkIfRemoved();
      this.additionalNameBasedHashCode = additionalNameBasedHashCode;
   }

   protected void checkIfRemoved()
   {
      if (hasBeenRemoved)
      {
         throw new RuntimeException("Can not use frame that was removed from the frame tree.");
      }
   }

   /**
    * Will remove this frame from the frame tree.
    * <p>
    * This recursively disables all children of this frame also.
    * </p>
    * <p>
    * Note that reference frames are automatically disposed of by the GC when no external reference
    * exists.
    * </p>
    */
   public void remove()
   {
      if (!hasBeenRemoved && parentFrame != null)
      {
         parentFrame.updateChildren();

         for (int i = 0; i < parentFrame.children.size(); i++)
         {
            if (parentFrame.children.get(i).get() == this)
            {
               parentFrame.children.remove(i);
               parentFrame.childrenNames.remove(frameName);
               break;
            }
         }

         if (topFrameNameRestriction != null)
            topFrameNameRestriction.subtreeFrameNames.remove(frameName);

         notifyListeners(ChangeType.FRAME_REMOVED, this, parentFrame);
         disableRecursivly();
      }
   }

   private void updateChildren()
   {
      boolean hasChildBeenGCed = false;

      for (int i = children.size() - 1; i >= 0; i--)
      {
         if (children.get(i).get() == null)
         {
            children.remove(i);
            hasChildBeenGCed = true;
         }
      }

      if (hasChildBeenGCed)
         notifyListeners(ChangeType.FRAME_GCED, null, this);
   }

   /**
    * Removes and disables all the children of {@code this}.
    * <p>
    * Note that reference frames are automatically disposed of by the GC when no external reference
    * exists.
    * </p>
    *
    * @see #remove()
    */
   public void clearChildren()
   {
      checkIfRemoved();
      children.stream().map(WeakReference::get).filter(child -> child != null).forEach(child -> child.disableRecursivly());
      children.clear();
      childrenNames.clear();
      if (topFrameNameRestriction != null)
         topFrameNameRestriction.subtreeFrameNames.remove(frameName);

      if (isRootFrame())
         framesAddedToTree = 0L;
   }

   private void disableRecursivly()
   {
      hasBeenRemoved = true;
      children.stream().map(WeakReference::get).filter(child -> child != null).forEach(child -> child.disableRecursivly());
      changedListeners = null;

      if (topFrameNameRestriction != null)
         topFrameNameRestriction.subtreeFrameNames.remove(frameName);
   }

   /**
    * Gets the number of children attached to this frame.
    *
    * @return the number of children.
    */
   public int getNumberOfChildren()
   {
      checkIfRemoved();
      updateChildren();
      return children.size();
   }

   /**
    * Gets the <tt>index</tt><sup>th</sup> child attached to this frame.
    * <p>
    * Although very unlikely, note that it is possible that the return frame is {@code null}.
    * </p>
    *
    * @param index the index of the frame to retrieve.
    * @return the child frame attached to {@code this}.
    * @see #getNumberOfChildren()
    */
   public ReferenceFrame getChild(int index)
   {
      checkIfRemoved();
      return children.get(index).get();
   }

   /**
    * Getter for the read only view of all reference frames starting with the root frame of this frame
    * tree all the way to this frame.
    *
    * @return the list of frames from the root frame to this.
    */
   public ReferenceFrame[] getFramesStartingWithRootEndingWithThis()
   {
      checkIfRemoved();
      return framesStartingWithRootEndingWithThis;
   }

   /**
    * @return the root frame of the world reference frame tree.
    * @see ReferenceFrameTools#getWorldFrame()
    */
   public static ReferenceFrame getWorldFrame()
   {
      return ReferenceFrameTools.getWorldFrame();
   }

   /**
    * Sets a predicate that restricts when the transform to root can be updated.
    * <p>
    * The condition applies to all reference frame that belongs the same tree as this reference frame.
    * </p>
    * <p>
    * This predicate can be used to restrict the update to be done only by one thread for instance.
    * </p>
    *
    * @param treeUpdateCondition the filtering condition for the update of the transform to root.
    */
   public void setTreeUpdateCondition(Predicate<ReferenceFrame> treeUpdateCondition)
   {
      checkIfRemoved();
      getRootFrame().treeUpdateCondition = treeUpdateCondition;
   }

   /**
    * Adds a listener to this reference frame.
    *
    * @param listener the listener for listening to changes done on this frame and its descendants.
    */
   public void addListener(ReferenceFrameChangedListener listener)
   {
      checkIfRemoved();
      if (changedListeners == null)
         changedListeners = new ArrayList<>();
      changedListeners.add(listener);
   }

   /**
    * Removes all listeners previously added to this reference frame.
    */
   public void removeListeners()
   {
      checkIfRemoved();
      changedListeners = null;
   }

   /**
    * Tries to remove a listener from this reference frame. If the listener could not be found and
    * removed, nothing happens.
    *
    * @param listener the listener to remove.
    * @return {@code true} if the listener was removed, {@code false} if the listener was not found and
    *         nothing happened.
    */
   public boolean removeListener(ReferenceFrameChangedListener listener)
   {
      checkIfRemoved();
      if (changedListeners == null)
         return false;
      return changedListeners.remove(listener);
   }

   private void notifyListeners(ChangeType type, ReferenceFrame target, ReferenceFrame targetParent)
   {
      if (changedListeners != null)
      {
         FrameChange change = new FrameChange(type, target, targetParent);
         for (int i = 0; i < changedListeners.size(); i++)
            changedListeners.get(i).changed(change);
      }

      if (parentFrame != null)
         parentFrame.notifyListeners(type, target, targetParent);
   }

   private enum ChangeType
   {
      FRAME_ADDED, FRAME_REMOVED, FRAME_GCED
   }

   private class FrameChange implements Change
   {
      private final ChangeType type;
      private final ReferenceFrame target, targetParent;

      public FrameChange(ChangeType type, ReferenceFrame target, ReferenceFrame targetParent)
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
      public ReferenceFrame getSource()
      {
         return ReferenceFrame.this;
      }

      @Override
      public ReferenceFrame getTarget()
      {
         return target;
      }

      @Override
      public ReferenceFrame getTargetParent()
      {
         return targetParent;
      }
   }
}
