package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.interfaces.Pose2DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameOrientation2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
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
public class FramePose2D implements FramePose2DBasics, GeometryObject<FramePose2D>
{
   /** The reference frame is which this point is currently expressed. */
   private ReferenceFrame referenceFrame;
   /** Pose used to perform the operations. */
   private final Pose2D pose = new Pose2D();

   private final FixedFramePoint2DBasics positionPart = new FixedFramePoint2DBasics()
   {
      @Override
      public void setX(double x)
      {
         pose.setX(x);
      }

      @Override
      public void setY(double y)
      {
         pose.setY(y);
      }

      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return referenceFrame;
      }

      @Override
      public double getX()
      {
         return pose.getX();
      }

      @Override
      public double getY()
      {
         return pose.getY();
      }
   };

   private final FixedFrameOrientation2DBasics orientationPart = new FixedFrameOrientation2DBasics()
   {
      @Override
      public void setYaw(double yaw)
      {
         pose.setYaw(yaw);
      }

      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return referenceFrame;
      }

      @Override
      public double getYaw()
      {
         return pose.getYaw();
      }

      @Override
      public void applyTransform(Transform transform)
      {
         pose.getOrientation().applyTransform(transform);
      }

      @Override
      public void applyInverseTransform(Transform transform)
      {
         pose.getOrientation().applyInverseTransform(transform);
      }
   };

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
    * @param pose the pose that is to be used internally. Copied. Not modified.
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
    * @param position the tuple used to initialize the position of this frame pose 2D. Not modified.
    * @param yaw the angle used to initialize the orientation of this frame pose 2D. Not modified.
    */
   public FramePose2D(ReferenceFrame referenceFrame, Tuple2DReadOnly position, double yaw)
   {
      setIncludingFrame(referenceFrame, position, yaw);
   }

   /**
    * Creates a new pose 2D initialize it from the given position and orientation.
    *
    * @param position the position used to initialize this frame pose. Not modified.
    * @param orientation the orientation used to initialize this frame pose. Not modified.
    * @throws ReferenceFrameMismatchException if {@code position} and {@code orientation} are not
    *            expressed in the same reference frame.
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
   public void set(FramePose2D other)
   {
      FramePose2DBasics.super.set(other);
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
      return positionPart;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameOrientation2DBasics getOrientation()
   {
      return orientationPart;
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
      try
      {
         return equals((FramePose2DReadOnly) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Tests on a per component basis if this pose is equal to the given {@code other} to an
    * {@code epsilon}.
    * <p>
    * If the two poses have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the other pose to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two poses are equal and are expressed in the same reference frame,
    *         {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(FramePose2D other, double epsilon)
   {
      return FramePose2DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two poses are geometrically similar.
    * <p>
    * Two poses are geometrically equal if both their position and orientation are geometrically equal.
    * </p>
    *
    * @param other the pose to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two poses represent the same geometry, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   @Override
   public boolean geometricallyEquals(FramePose2D other, double epsilon)
   {
      return FramePose2DBasics.super.geometricallyEquals(other, epsilon);
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
      return pose.toString() + "-" + referenceFrame;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this pose 2D.
    *
    * @return the hash code value for this pose 2D.
    */
   @Override
   public int hashCode()
   {
      return pose.hashCode();
   }
}
