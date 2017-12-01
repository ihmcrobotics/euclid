package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.interfaces.Orientation2DBasics;
import us.ihmc.euclid.geometry.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose2DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;

public class FramePose2D extends FrameGeometryObject<FramePose2D, Pose2D> implements FramePose2DReadOnly, Pose2DBasics
{
   /** Pose used to perform the operations. */
   protected final Pose2D pose;

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
   public FramePose2D(Pose2DReadOnly pose)
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
   public FramePose2D(ReferenceFrame referenceFrame, Pose2DReadOnly pose)
   {
      super(referenceFrame, new Pose2D(pose));
      this.pose = getGeometryObject();
   }

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
   public void setYaw(double yaw)
   {
      pose.setYaw(yaw);
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
   public double getYaw()
   {
      return pose.getYaw();
   }

   @Override
   public Point2DBasics getPosition()
   {
      return pose.getPosition();
   }

   @Override
   public Orientation2DBasics getOrientation()
   {
      return pose.getOrientation();
   }

   public void set(FramePose2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Pose2DBasics.super.set(other);
   }

   public void set(FrameTuple2DReadOnly position, Orientation2DReadOnly orientation)
   {
      checkReferenceFrameMatch(position);
      Pose2DBasics.super.set(position, orientation);
   }

   public void setPosition(FrameTuple2DReadOnly position)
   {
      checkReferenceFrameMatch(position);
      Pose2DBasics.super.setPosition(position);
   }

   /**
    * Adds the given {@code translation} to this pose 2D assuming it is expressed in the coordinates
    * in which this pose is expressed.
    * <p>
    * If the {@code translation} is expressed in the local coordinates described by this pose 2D,
    * use {@link #appendTranslation(FrameTuple2DReadOnly)}.
    * </p>
    *
    * @param translation tuple containing the translation to apply to this pose 2D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code translation} are not
    *          expressed in the same reference frame.
    */
   public void prependTranslation(FrameTuple2DReadOnly translation)
   {
      checkReferenceFrameMatch(translation);
      Pose2DBasics.super.prependTranslation(translation);
   }

   /**
    * Rotates, then adds the given {@code translation} to this pose 2D.
    * <p>
    * Use this method if the {@code translation} is expressed in the local coordinates described by
    * this pose 2D. Otherwise, use {@link #prependTranslation(FrameTuple2DReadOnly)}.
    * </p>
    *
    * @param translation tuple containing the translation to apply to this pose 2D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code translation} are not
    *          expressed in the same reference frame.
    */
   public void appendTranslation(FrameTuple2DReadOnly translation)
   {
      checkReferenceFrameMatch(translation);
      Pose2DBasics.super.appendTranslation(translation);
   }

   /**
    * Performs a linear interpolation from {@code this} to {@code other} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * this.position + alpha * other.position<br>
    * this.orientation = (1.0 - alpha) * this.orientation + alpha * other.orientation
    * </p>
    *
    * @param other the other pose 2D used for the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not
    *           modifying {@code this}, while a value of 1 is equivalent to setting {@code this} to
    *           {@code other}.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not
    *          expressed in the same reference frame.
    */
   public void interpolate(FramePose2D other, double alpha)
   {
      checkReferenceFrameMatch(other);
      Pose2DBasics.super.interpolate(other.pose, alpha);
   }

   /**
    * Performs a linear interpolation from {@code pose1} to {@code pose2} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * pose1.position + alpha * pose2.position<br>
    * this.orientation = (1.0 - alpha) * pose1.orientation + alpha * pose2.orientation
    * </p>
    *
    * @param pose1 the first pose 2D used in the interpolation. Not modified.
    * @param pose2 the second pose 2D used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           {@code this} to {@code pose1}, while a value of 1 is equivalent to setting
    *           {@code this} to {@code pose2}.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pose1} are not
    *          expressed in the same reference frame.
    */
   public void interpolate(FramePose2D pose1, Pose2D pose2, double alpha)
   {
      checkReferenceFrameMatch(pose1);
      Pose2DBasics.super.interpolate(pose1.pose, pose2, alpha);
   }

   /**
    * Performs a linear interpolation from {@code pose1} to {@code pose2} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * pose1.position + alpha * pose2.position<br>
    * this.orientation = (1.0 - alpha) * pose1.orientation + alpha * pose2.orientation
    * </p>
    *
    * @param pose1 the first pose 2D used in the interpolation. Not modified.
    * @param pose2 the second pose 2D used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           {@code this} to {@code pose1}, while a value of 1 is equivalent to setting
    *           {@code this} to {@code pose2}.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pose2} are not
    *          expressed in the same reference frame.
    */
   public void interpolate(Pose2D pose1, FramePose2D pose2, double alpha)
   {
      checkReferenceFrameMatch(pose2);
      Pose2DBasics.super.interpolate(pose1, pose2.pose, alpha);
   }

   /**
    * Performs a linear interpolation from {@code pose1} to {@code pose2} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * pose1.position + alpha * pose2.position<br>
    * this.orientation = (1.0 - alpha) * pose1.orientation + alpha * pose2.orientation
    * </p>
    *
    * @param pose1 the first pose 2D used in the interpolation. Not modified.
    * @param pose2 the second pose 2D used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           {@code this} to {@code pose1}, while a value of 1 is equivalent to setting
    *           {@code this} to {@code pose2}.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code pose1} and
    *          {@code pose2} are not expressed in the same reference frame.
    */
   public void interpolate(FramePose2D pose1, FramePose2D pose2, double alpha)
   {
      checkReferenceFrameMatch(pose1);
      checkReferenceFrameMatch(pose2);
      Pose2DBasics.super.interpolate(pose1.pose, pose2.pose, alpha);
   }
}
