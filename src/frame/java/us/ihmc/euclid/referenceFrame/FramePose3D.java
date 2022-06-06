package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * {@code FramePose3D} is a 3D pose expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Pose3DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FramePose3D}. This allows, for instance, to enforce, at runtime, that operations on poses
 * occur in the same coordinate system. Also, via the method {@link #changeFrame(ReferenceFrame)},
 * one can easily calculates the value of a point in different reference frames.
 * </p>
 * <p>
 * Because a {@code FramePose3D} extends {@code Pose3DBasics}, it is compatible with methods only
 * requiring {@code Pose3DBasics}. However, these methods do NOT assert that the operation occur in
 * the proper coordinate system. Use this feature carefully and always prefer using methods
 * requiring {@code FramePose3D}.
 * </p>
 */
public class FramePose3D implements FramePose3DBasics
{
   /** The reference frame is which this point is currently expressed. */
   private ReferenceFrame referenceFrame;
   private final FixedFramePoint3DBasics position = EuclidFrameFactories.newFixedFramePoint3DBasics(this);
   private final FixedFrameQuaternionBasics orientation = EuclidFrameFactories.newFixedFrameQuaternionBasics(this);

   /**
    * Creates a new pose 3D initialized with its position and orientation set to zero and its reference
    * frame to {@code ReferenceFrame#getWorldFrame()}.
    */
   public FramePose3D()
   {
      setToZero(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new pose 3D initialized with its position and orientation set to zero in the given
    * {@code referenceFrame}.
    *
    * @param referenceFrame the reference frame used to initialize this frame pose.
    */
   public FramePose3D(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   /**
    * Creates a new frame pose and initializes its current reference frame to
    * {@link ReferenceFrame#getWorldFrame()} and pose to {@code pose}.
    *
    * @param pose3DReadOnly the pose used to initialize the this frame pose. Not modified.
    */
   public FramePose3D(Pose3DReadOnly pose3DReadOnly)
   {
      setIncludingFrame(ReferenceFrame.getWorldFrame(), pose3DReadOnly);
   }

   /**
    * Creates a new frame pose and initializes its current reference frame and pose.
    *
    * @param referenceFrame the initial reference frame in which the given pose is expressed in.
    * @param pose3DReadOnly the pose used to initialize the this frame pose. Not modified.
    */
   public FramePose3D(ReferenceFrame referenceFrame, Pose3DReadOnly pose3DReadOnly)
   {
      setIncludingFrame(referenceFrame, pose3DReadOnly);
   }

   /**
    * Creates a new frame pose and initializes its current reference frame and pose.
    *
    * @param referenceFrame     the initial reference frame in which the given pose is expressed in.
    * @param rigidBodyTransform the rigid body transform used to initialize the this frame pose. Not
    *                           modified.
    */
   public FramePose3D(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly rigidBodyTransform)
   {
      setIncludingFrame(referenceFrame, rigidBodyTransform);
   }

   /**
    * Creates a new frame pose and initializes its current reference frame and pose.
    *
    * @param referenceFrame the initial reference frame in which the given pose is expressed in.
    * @param position       the tuple used to initialize the position. Not modified.
    * @param orientation    the initial orientation. Not modified.
    */
   public FramePose3D(ReferenceFrame referenceFrame, Tuple3DReadOnly position, Orientation3DReadOnly orientation)
   {
      setIncludingFrame(referenceFrame, position, orientation);
   }

   /**
    * Creates a new pose 3D initialize it from the given position and orientation.
    *
    * @param position    the position used to initialize this frame pose. Not modified.
    * @param orientation the orientation used to initialize this frame pose. Not modified.
    * @throws ReferenceFrameMismatchException if {@code position} and {@code orientation} are not
    *                                         expressed in the same reference frame.
    */
   public FramePose3D(FrameTuple3DReadOnly position, FrameOrientation3DReadOnly orientation)
   {
      setIncludingFrame(position, orientation);
   }

   /**
    * Creates a new frame pose and initializes it to {@code other}.
    *
    * @param other the other pose used to initialize this frame pose. Not modified.
    */
   public FramePose3D(FramePose3DReadOnly other)
   {
      setIncludingFrame(other);
   }

   /**
    * Sets this frame pose 3D to the {@code other} frame pose 3D.
    *
    * @param other the other frame pose 3D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *                                         the same reference frame.
    */
   @Override
   public void set(FramePose3D other)
   {
      FramePose3DBasics.super.set(other);
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
   public FixedFramePoint3DBasics getPosition()
   {
      return position;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameQuaternionBasics getOrientation()
   {
      return orientation;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(FramePose3DReadOnly)}, it returns {@code false} otherwise.
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
      if (object instanceof FramePose3DReadOnly)
         return equals((FramePose3DReadOnly) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this pose 3D as follows:<br>
    * Pose 3D: position = (x, y, z), orientation = (x, y, z, s)-worldFrame
    *
    * @return the {@code String} representing this pose 3D.
    */
   @Override
   public String toString()
   {
      return EuclidFrameIOTools.getFramePose3DString(this);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this pose 3D.
    *
    * @return the hash code value for this pose 3D.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(position, orientation);
   }


}
