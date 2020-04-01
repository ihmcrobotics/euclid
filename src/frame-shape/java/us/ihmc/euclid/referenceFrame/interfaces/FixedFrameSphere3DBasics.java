package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Read-only interface for a sphere 3D expressed in a constant reference frame, i.e. the reference
 * frame of this object cannot be changed via this interface.
 * <p>
 * A sphere 3D is represented by its position and radius.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FixedFrameSphere3DBasics extends Sphere3DBasics, FrameSphere3DReadOnly, FixedFrameShape3DBasics
{
   /** {@inheritDoc} */
   @Override
   FixedFramePoint3DBasics getPosition();

   /**
    * Copies the {@code other} sphere data into {@code this}.
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param other          the other sphere to copy. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, Sphere3DReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(other);
   }

   /**
    * Copies the {@code other} sphere data into {@code this}.
    *
    * @param other the other sphere to copy. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(FrameSphere3DReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }

   /**
    * Sets this sphere properties.
    *
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param centerX        the x-coordinate of the center.
    * @param centerY        the y-coordinate of the center.
    * @param centerZ        the z-coordinate of the center.
    * @param radius         the radius for this sphere.
    * @throws IllegalArgumentException        if {@code radius < 0.0}.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, double centerX, double centerY, double centerZ, double radius)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(centerX, centerY, centerZ, radius);
   }

   /**
    * Sets this sphere properties.
    *
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param center         the position of this sphere center. Not modified.
    * @param radius         the radius for this sphere.
    * @throws IllegalArgumentException        if {@code radius < 0.0}.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, Point3DReadOnly center, double radius)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(center, radius);
   }

   /**
    * Sets this sphere properties.
    *
    * @param center the position of this sphere center. Not modified.
    * @param radius the radius for this sphere.
    * @throws IllegalArgumentException        if {@code radius < 0.0}.
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void set(FramePoint3DReadOnly center, double radius)
   {
      set(center.getReferenceFrame(), center, radius);
   }

   /**
    * Copies the {@code other} sphere data into {@code this}.
    * <p>
    * If {@code other} is expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(ReferenceFrame, Sphere3DReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param other          the other sphere to copy. Not modified.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, Sphere3DReadOnly other)
   {
      Sphere3DBasics.super.set(other);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Copies the {@code other} sphere data into {@code this}.
    * <p>
    * If {@code other} is expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(FrameSphere3DReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param other the other sphere to copy. Not modified.
    */
   default void setMatchingFrame(FrameSphere3DReadOnly other)
   {
      setMatchingFrame(other.getReferenceFrame(), other);
   }

   /**
    * Sets this sphere properties.
    * <p>
    * If {@code other} is expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(ReferenceFrame, double, double, double, double)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param centerX        the x-coordinate of the center.
    * @param centerY        the y-coordinate of the center.
    * @param centerZ        the z-coordinate of the center.
    * @param radius         the radius for this sphere.
    * @throws IllegalArgumentException if {@code radius < 0.0}.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, double centerX, double centerY, double centerZ, double radius)
   {
      set(centerX, centerY, centerZ, radius);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this sphere properties.
    * <p>
    * If {@code other} is expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(ReferenceFrame, Point3DReadOnly, double)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param center         the position of this sphere center. Not modified.
    * @param radius         the radius for this sphere.
    * @throws IllegalArgumentException if {@code radius < 0.0}.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, Point3DReadOnly center, double radius)
   {
      set(center, radius);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this sphere properties.
    * <p>
    * If {@code other} is expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(FramePoint3DReadOnly, double)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param center the position of this sphere center. Not modified.
    * @param radius the radius for this sphere.
    * @throws IllegalArgumentException if {@code radius < 0.0}.
    */
   default void setMatchingFrame(FramePoint3DReadOnly center, double radius)
   {
      setMatchingFrame(center.getReferenceFrame(), center, radius);
   }

   /**
    * Returns {@code null} as this shape is not defined by a pose.
    */
   @Override
   default FixedFrameShape3DPoseBasics getPose()
   {
      return null;
   }
}
