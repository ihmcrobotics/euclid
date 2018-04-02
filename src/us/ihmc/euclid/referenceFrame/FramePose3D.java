package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

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
public class FramePose3D implements FramePose3DBasics, GeometryObject<FramePose3D>
{
   /** The reference frame is which this point is currently expressed. */
   private ReferenceFrame referenceFrame;
   /** Pose used to perform the operations. */
   private final Pose3D pose = new Pose3D();

   private final FixedFramePoint3DBasics positionPart = new FixedFramePoint3DBasics()
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
      public void setZ(double z)
      {
         pose.setZ(z);
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

      @Override
      public double getZ()
      {
         return pose.getZ();
      }
   };

   private final FixedFrameQuaternionBasics orientationPart = new FixedFrameQuaternionBasics()
   {

      @Override
      public void setUnsafe(double qx, double qy, double qz, double qs)
      {
         pose.getOrientation().setUnsafe(qx, qy, qz, qs);
      }

      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return referenceFrame;
      }

      @Override
      public double getX()
      {
         return pose.getOrientation().getX();
      }

      @Override
      public double getY()
      {
         return pose.getOrientation().getY();
      }

      @Override
      public double getZ()
      {
         return pose.getOrientation().getZ();
      }

      @Override
      public double getS()
      {
         return pose.getOrientation().getS();
      }
   };

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
    * @param referenceFrame the initial reference frame in which the given pose is expressed in.
    * @param rigidBodyTransform the rigid body transform used to initialize the this frame pose. Not
    *           modified.
    */
   public FramePose3D(ReferenceFrame referenceFrame, RigidBodyTransform rigidBodyTransform)
   {
      setIncludingFrame(referenceFrame, rigidBodyTransform);
   }

   /**
    * Creates a new frame pose and initializes its current reference frame and pose.
    *
    * @param referenceFrame the initial reference frame in which the given pose is expressed in.
    * @param position the tuple used to initialize the position. Not modified.
    * @param orientation the quaternion used to initialize the orientation. Not modified.
    */
   public FramePose3D(ReferenceFrame referenceFrame, Tuple3DReadOnly position, QuaternionReadOnly orientation)
   {
      setIncludingFrame(referenceFrame, position, orientation);
   }

   /**
    * Creates a new frame pose and initializes its current reference frame and pose.
    *
    * @param referenceFrame the initial reference frame in which the given pose is expressed in.
    * @param position the tuple used to initialize the position. Not modified.
    * @param orientation the axis-angle used to initialize the orientation. Not modified.
    */
   public FramePose3D(ReferenceFrame referenceFrame, Tuple3DReadOnly position, AxisAngleReadOnly orientation)
   {
      setIncludingFrame(referenceFrame, position, orientation);
   }

   /**
    * Creates a new pose 3D initialize it from the given position and orientation.
    *
    * @param position the position used to initialize this frame pose. Not modified.
    * @param orientation the orientation used to initialize this frame pose. Not modified.
    * @throws ReferenceFrameMismatchException if {@code position} and {@code orientation} are not
    *            expressed in the same reference frame.
    */
   public FramePose3D(FrameTuple3DReadOnly position, FrameQuaternionReadOnly orientation)
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
    *            the same reference frame.
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
      return positionPart;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameQuaternionBasics getOrientation()
   {
      return orientationPart;
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
      try
      {
         return super.equals(object);
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
   public boolean epsilonEquals(FramePose3D other, double epsilon)
   {
      return FramePose3DBasics.super.epsilonEquals(other, epsilon);
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
   public boolean geometricallyEquals(FramePose3D other, double epsilon)
   {
      return FramePose3DBasics.super.geometricallyEquals(other, epsilon);
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
      return pose.toString() + "-" + referenceFrame;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this pose 3D.
    *
    * @return the hash code value for this pose 3D.
    */
   @Override
   public int hashCode()
   {
      return pose.hashCode();
   }
}
