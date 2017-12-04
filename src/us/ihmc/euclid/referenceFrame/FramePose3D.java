package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;

public class FramePose3D extends FrameGeometryObject<FramePose3D, Pose3D> implements FramePose3DReadOnly, Pose3DBasics
{
   /** Pose used to perform the operations. */
   protected final Pose3D pose;

   /**
    * Creates a new frame pose and initializes its current reference frame to
    * {@link ReferenceFrame#getWorldFrame()} and pose to {@code pose}.
    * <p>
    * The given {@code pose}'s reference is saved internally for performing all the future
    * operations with this {@code FramePose3D}.
    * </p>
    *
    * @param pose the pose that is to be used internally. Copied. Not modified.
    */
   public FramePose3D(Pose3DReadOnly pose)
   {
      this(ReferenceFrame.getWorldFrame(), pose);
   }
   
   /**
    * Creates a new frame pose and initializes its current reference frame and pose.
    * <p>
    * The given {@code pose}'s reference is saved internally for performing all the future
    * operations with this {@code FramePose3D}.
    * </p>
    *
    * @param referenceFrame the initial reference frame in which the given pose is expressed in.
    * @param pose the pose that is to be used internally. Copied. Not modified.
    */
   public FramePose3D(ReferenceFrame referenceFrame, Pose3DReadOnly pose)
   {
      super(referenceFrame, new Pose3D(pose));
      this.pose = getGeometryObject();
   }

   /** {@inheritDoc} */
   @Override
   public Point3DBasics getPosition()
   {
      return pose.getPosition();
   }

   /** {@inheritDoc} */
   @Override
   public QuaternionBasics getOrientation()
   {
      return pose.getOrientation();
   }

   /** {@inheritDoc} */
   @Override
   public double getX()
   {
      return pose.getX();
   }

   /** {@inheritDoc} */
   @Override
   public double getY()
   {
      return pose.getY();
   }

   /** {@inheritDoc} */
   @Override
   public double getZ()
   {
      return pose.getZ();
   }

   /** {@inheritDoc} */
   @Override
   public double getYaw()
   {
      return pose.getYaw();
   }

   /** {@inheritDoc} */
   @Override
   public double getPitch()
   {
      return pose.getPitch();
   }

   /** {@inheritDoc} */
   @Override
   public double getRoll()
   {
      return pose.getRoll();
   }
   
   /**
    * Sets this frame pose 3D to the {@code other} frame pose 3D.
    *
    * @param other the other frame pose 3D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not
    *          expressed in the same reference frame.
    */
   public void set(FramePose3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Pose3DBasics.super.set(other);
   }
   
   /**
    * Sets both position and orientation.
    *
    * @param position the tuple with the new position coordinates. Not modified.
    * @param orientation the axis-angle with the new orientation. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code position} are not
    *          expressed in the same reference frame.
    */
   public void set(FrameTuple3DReadOnly position, AxisAngleReadOnly orientation)
   {
      checkReferenceFrameMatch(position);
      Pose3DBasics.super.set(position, orientation);
   }

   /**
    * Sets both position and orientation.
    *
    * @param position the tuple with the new position coordinates. Not modified.
    * @param orientation the quaternion with the new orientation. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code position} are not
    *          expressed in the same reference frame.
    */
   public void set(FrameTuple3DReadOnly position, QuaternionReadOnly orientation)
   {
      checkReferenceFrameMatch(position);
      Pose3DBasics.super.set(position, orientation);
   }
   
   /**
    * Sets both position and orientation.
    *
    * @param position the tuple with the new position coordinates. Not modified.
    * @param orientation the quaternion with the new orientation. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code orientation} are not
    *          expressed in the same reference frame.
    */
   public void set(Tuple3DReadOnly position, FrameQuaternionReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      Pose3DBasics.super.set(position, orientation);
   }

   /**
    * Sets both position and orientation.
    *
    * @param position the tuple with the new position coordinates. Not modified.
    * @param orientation the quaternion with the new orientation. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code position}, 
    *          and {@code orientation} are not expressed in the same reference frame.
    */
   public void set(FrameTuple3DReadOnly position, FrameQuaternionReadOnly orientation)
   {
      checkReferenceFrameMatch(position);
      checkReferenceFrameMatch(orientation);
      Pose3DBasics.super.set(position, orientation);
   }

   /**
    * Sets the position to the given tuple.
    *
    * @param position the tuple with the new position coordinates. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code position} are not
    *          expressed in the same reference frame.
    */
   public void setPosition(FrameTuple3DReadOnly position)
   {
      checkReferenceFrameMatch(position);
      Pose3DBasics.super.setPosition(position);
   }

   /**
    * Sets the position from the given tuple 2D and z coordinate.
    *
    * @param position2D the tuple with the new x and y coordinates. Not modified.
    * @param z the new z value.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code position2D} are not
    *          expressed in the same reference frame.
    */
   public void setPosition(FrameTuple2DReadOnly position2D, double z)
   {
      checkReferenceFrameMatch(position2D);
      Pose3DBasics.super.setPosition(position2D, z);
   }

   /**
    * Sets the x and y coordinates from the given tuple 2D, the z coordinate remains unchanged.
    *
    * @param position2D the tuple with the new x and y coordinates. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code position2D} are not
    *          expressed in the same reference frame.
    */
   public void setPosition(FrameTuple2DReadOnly position2D)
   {
      checkReferenceFrameMatch(position2D);
      Pose3DBasics.super.setPosition(position2D);
   }
   
   /**
    * Sets the orientation part of this pose 3D with the given quaternion.
    *
    * @param orientation the quaternion used to set this pose's orientation. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code orientation} are not
    *          expressed in the same reference frame.
    */
   public void setOrientation(FrameQuaternionReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      Pose3DBasics.super.setOrientation(orientation);
   }

   /**
    * Adds the given {@code translation} to this pose 3D assuming it is expressed in the coordinates
    * in which this pose is expressed.
    * <p>
    * If the {@code translation} is expressed in the local coordinates described by this pose 3D,
    * use {@link #appendTranslation(FrameTuple3DReadOnly)}.
    * </p>
    *
    * @param translation tuple containing the translation to apply to this pose 3D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code translation} are not
    *          expressed in the same reference frame.
    */
   public void prependTranslation(FrameTuple3DReadOnly translation)
   {
      checkReferenceFrameMatch(translation);
      Pose3DBasics.super.prependTranslation(translation);
   }

   /**
    * Rotates the position part of this pose 3D by the given {@code rotation} and prepends it to the
    * orientation part.
    *
    * @param rotation the rotation to prepend to this pose 3D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code rotation} are not
    *          expressed in the same reference frame.
    */
   public void prependRotation(FrameQuaternionReadOnly rotation)
   {
      checkReferenceFrameMatch(rotation);
      Pose3DBasics.super.prependRotation(rotation);
   }

   /**
    * Rotates, then adds the given {@code translation} to this pose 3D.
    * <p>
    * Use this method if the {@code translation} is expressed in the local coordinates described by
    * this pose 3D. Otherwise, use {@link #prependTranslation(FrameTuple3DReadOnly)}.
    * </p>
    *
    * @param translation tuple containing the translation to apply to this pose 3D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code translation} are not
    *          expressed in the same reference frame.
    */
   public void appendTranslation(FrameTuple3DReadOnly translation)
   {
      checkReferenceFrameMatch(translation);
      Pose3DBasics.super.appendTranslation(translation);
   }

   /**
    * Appends the given rotation to this pose 3D.
    * <p>
    * More precisely, the position part is unchanged while the orientation part is updated as
    * follows:<br>
    * {@code this.orientation = this.orientation * rotation}
    * </p>
    *
    * @param rotation the rotation to append to this pose 3D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code rotation} are not
    *          expressed in the same reference frame.
    */
   public void appendRotation(FrameQuaternionReadOnly rotation)
   {
      checkReferenceFrameMatch(rotation);
      Pose3DBasics.super.appendRotation(rotation);
   }
   
   /**
    * Performs a linear interpolation from {@code this} to {@code other} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * this.position + alpha * other.position<br>
    * this.orientation = (1.0 - alpha) * this.orientation + alpha * other.orientation
    * </p>
    *
    * @param other the other pose 3D used for the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not
    *           modifying {@code this}, while a value of 1 is equivalent to setting {@code this} to
    *           {@code other}.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not
    *          expressed in the same reference frame.
    */
   public void interpolate(FramePose3D other, double alpha)
   {
      checkReferenceFrameMatch(other);
      Pose3DBasics.super.interpolate(other.pose, alpha);
   }

   /**
    * Performs a linear interpolation from {@code pose1} to {@code pose2} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * pose1.position + alpha * pose2.position<br>
    * this.orientation = (1.0 - alpha) * pose1.orientation + alpha * pose2.orientation
    * </p>
    *
    * @param pose1 the first pose 3D used in the interpolation. Not modified.
    * @param pose2 the second pose 3D used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           {@code this} to {@code pose1}, while a value of 1 is equivalent to setting
    *           {@code this} to {@code pose2}.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pose1} are not
    *          expressed in the same reference frame.
    */
   public void interpolate(FramePose3D pose1, Pose3D pose2, double alpha)
   {
      checkReferenceFrameMatch(pose1);
      Pose3DBasics.super.interpolate(pose1.pose, pose2, alpha);
   }

   /**
    * Performs a linear interpolation from {@code pose1} to {@code pose2} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * pose1.position + alpha * pose2.position<br>
    * this.orientation = (1.0 - alpha) * pose1.orientation + alpha * pose2.orientation
    * </p>
    *
    * @param pose1 the first pose 3D used in the interpolation. Not modified.
    * @param pose2 the second pose 3D used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           {@code this} to {@code pose1}, while a value of 1 is equivalent to setting
    *           {@code this} to {@code pose2}.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pose2} are not
    *          expressed in the same reference frame.
    */
   public void interpolate(Pose3D pose1, FramePose3D pose2, double alpha)
   {
      checkReferenceFrameMatch(pose2);
      Pose3DBasics.super.interpolate(pose1, pose2.pose, alpha);
   }

   /**
    * Performs a linear interpolation from {@code pose1} to {@code pose2} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * pose1.position + alpha * pose2.position<br>
    * this.orientation = (1.0 - alpha) * pose1.orientation + alpha * pose2.orientation
    * </p>
    *
    * @param pose1 the first pose 3D used in the interpolation. Not modified.
    * @param pose2 the second pose 3D used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           {@code this} to {@code pose1}, while a value of 1 is equivalent to setting
    *           {@code this} to {@code pose2}.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code pose1}, and {@code pose2}
    *          are not expressed in the same reference frame.
    */
   public void interpolate(FramePose3D pose1, FramePose3D pose2, double alpha)
   {
      checkReferenceFrameMatch(pose1);
      checkReferenceFrameMatch(pose2);
      Pose3DBasics.super.interpolate(pose1.pose, pose2.pose, alpha);
   }
}
