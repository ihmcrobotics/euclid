package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.interfaces.Pose2DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameOrientation2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

/**
 * {@code FramePose2D} is a 2D pose expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Pose2DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FramePose2D}. This allows, for instance, to enforce, at runtime, that operations on poses
 * occur in the same coordinate system. Also, via the method {@link #changeFrame(ReferenceFrame)},
 * one can easily calculates the value of a point in different reference frames.
 * </p>
 * <p>
 * Because a {@code FramePose2D} extends {@code Pose2DBasics}, it is compatible with methods only
 * requiring {@code Pose2DBasics}. However, these methods do NOT assert that the operation occur in
 * the proper coordinate system. Use this feature carefully and always prefer using methods
 * requiring {@code FramePose2D}.
 * </p>
 */
public class FramePose2D implements FramePose2DBasics
{
   /** The reference frame is which this point is currently expressed. */
   private ReferenceFrame referenceFrame;
   private final FixedFramePoint2DBasics position = EuclidFrameFactories.newFixedFramePoint2DBasics(this);
   private final FixedFrameOrientation2DBasics orientation = EuclidFrameFactories.newFixedFrameOrientation2DBasics(this);

   /**
    * Creates a new pose 2D initialized with its position at (0, 0) and orientation at 0 and its
    * reference frame to {@code ReferenceFrame#getWorldFrame()}.
    */
   public FramePose2D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new pose 2D initialized with its position at (0, 0) and orientation at 0 in the given
    * {@code referenceFrame}.
    *
    * @param referenceFrame the reference frame used to initialize this frame pose.
    */
   public FramePose2D(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   /**
    * Creates a new frame pose and initializes its current reference frame to
    * {@link ReferenceFrame#getWorldFrame()} and pose to {@code pose2dReadOnly}.
    *
    * @param pose2DReadOnly the pose used to initialize the this frame pose. Not modified.
    */
   public FramePose2D(Pose2DReadOnly pose2DReadOnly)
   {
      setIncludingFrame(ReferenceFrame.getWorldFrame(), pose2DReadOnly);
   }

   /**
    * Creates a new frame pose and initializes its current reference frame and pose.
    * <p>
    * The given {@code pose}'s reference is saved internally for performing all the future operations
    * with this {@code FramePose3D}.
    * </p>
    *
    * @param referenceFrame the initial reference frame in which the given pose is expressed in.
    * @param pose           the pose that is to be used internally. Copied. Not modified.
    */
   public FramePose2D(ReferenceFrame referenceFrame, Pose2DReadOnly pose)
   {
      setIncludingFrame(referenceFrame, pose);
   }

   /**
    * Creates a new frame pose 2D and initializes it from the given {@code position} and {@code yaw}
    * angle.
    *
    * @param referenceFrame the initial reference frame for this frame pose 2D.
    * @param position       the tuple used to initialize the position of this frame pose 2D. Not
    *                       modified.
    * @param yaw            the angle used to initialize the orientation of this frame pose 2D. Not
    *                       modified.
    */
   public FramePose2D(ReferenceFrame referenceFrame, Tuple2DReadOnly position, double yaw)
   {
      setIncludingFrame(referenceFrame, position, yaw);
   }

   /**
    * Creates a new pose 2D initialize it from the given position and orientation.
    *
    * @param position    the position used to initialize this frame pose. Not modified.
    * @param orientation the orientation used to initialize this frame pose. Not modified.
    * @throws ReferenceFrameMismatchException if {@code position} and {@code orientation} are not
    *                                         expressed in the same reference frame.
    */
   public FramePose2D(FrameTuple2DReadOnly position, FrameOrientation2DReadOnly orientation)
   {
      setIncludingFrame(position, orientation);
   }

   /**
    * Creates a new frame pose and initializes it to {@code other}.
    *
    * @param other the other pose used to initialize this frame pose. Not modified.
    */
   public FramePose2D(FramePose2DReadOnly other)
   {
      setIncludingFrame(other);
   }

   /**
    * Creates a new frame pose 2D and initializes it from the x, y, and yaw components of the given
    * {@code framePose3DReadOnly}.
    *
    * @param framePose3DReadOnly the frame pose 3D used to initialize this frame pose 2D. Not modified.
    */
   public FramePose2D(FramePose3DReadOnly framePose3DReadOnly)
   {
      setIncludingFrame(framePose3DReadOnly);
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFramePoint2DBasics getPosition()
   {
      return position;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameOrientation2DBasics getOrientation()
   {
      return orientation;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(FramePose2DReadOnly)}, it returns {@code false} otherwise.
    * <p>
    * If the two poses have different frames, this method returns {@code false}.
    * </p>
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FramePose2DReadOnly)
         return equals((FramePose2DReadOnly) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this pose 2D as follows:<br>
    * Pose 2D: position = (x, y), orientation = (yaw)-worldFrame
    *
    * @return the {@code String} representing this pose 2D.
    */
   @Override
   public String toString()
   {
      return FramePose2DBasics.super.toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this pose 2D.
    *
    * @return the hash code value for this pose 2D.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(position, orientation);
   }
}
