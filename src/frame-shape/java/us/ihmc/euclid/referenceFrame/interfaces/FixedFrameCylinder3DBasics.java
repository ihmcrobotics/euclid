package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read and write interface for a cylinder 3D expressed in a constant reference frame, i.e. the
 * reference frame of this object cannot be changed via this interface.
 * <p>
 * A cylinder 3D is represented by its length, its radius, the position of its center, and its axis
 * of revolution.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FixedFrameCylinder3DBasics extends Cylinder3DBasics, FrameCylinder3DReadOnly, FixedFrameShape3DBasics
{
   /** {@inheritDoc} */
   @Override
   FixedFramePoint3DBasics getPosition();

   /** {@inheritDoc} */
   @Override
   FixedFrameUnitVector3DBasics getAxis();

   /**
    * Copies the {@code other} cylinder data into {@code this}.
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param other          the other cylinder to copy. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, Cylinder3DReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      Cylinder3DBasics.super.set(other);
   }

   /**
    * Copies the {@code other} cylinder data into {@code this}.
    *
    * @param other the other cylinder to copy. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(FrameCylinder3DReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }

   /**
    * Sets this cylinder properties.
    *
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param position       the position of this cylinder center. Not modified.
    * @param axis           the axis of revolution of this cylinder. Not modified.
    * @param length         the new length.
    * @param radius         the new radius.
    * @throws IllegalArgumentException        if {@code length} or {@code radius} is negative.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, Point3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
   {
      checkReferenceFrameMatch(referenceFrame);
      Cylinder3DBasics.super.set(position, axis, length, radius);
   }

   /**
    * Sets this cylinder properties.
    *
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param position       the position of this cylinder center. Not modified.
    * @param axis           the axis of revolution of this cylinder. Not modified.
    * @param length         the new length.
    * @param radius         the new radius.
    * @throws IllegalArgumentException        if {@code length} or {@code radius} is negative.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(FramePoint3DReadOnly position, FrameVector3DReadOnly axis, double length, double radius)
   {
      position.checkReferenceFrameMatch(axis);
      set(position.getReferenceFrame(), position, axis, length, radius);
   }

   /**
    * Copies the {@code other} cylinder data into {@code this}.
    * <p>
    * If {@code other} is expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(ReferenceFrame, Cylinder3DReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param other          the other cylinder to copy. Not modified.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, Cylinder3DReadOnly other)
   {
      Cylinder3DBasics.super.set(other);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Copies the {@code other} cylinder data into {@code this}.
    * <p>
    * If {@code other} is expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(ReferenceFrame, Cylinder3DReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param other the other cylinder to copy. Not modified.
    */
   default void setMatchingFrame(FrameCylinder3DReadOnly other)
   {
      setMatchingFrame(other.getReferenceFrame(), other);
   }

   /**
    * Sets this cylinder properties.
    * <p>
    * If the arguments are expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(ReferenceFrame, Point3DReadOnly, Vector3DReadOnly, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param position       the position of this cylinder center. Not modified.
    * @param axis           the axis of revolution of this cylinder. Not modified.
    * @param length         the new length.
    * @param radius         the new radius.
    * @throws IllegalArgumentException if {@code length} or {@code radius} is negative.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, Point3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
   {
      Cylinder3DBasics.super.set(position, axis, length, radius);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this cylinder properties.
    * <p>
    * If the arguments are expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(FramePoint3DReadOnly, FrameVector3DReadOnly, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param position the position of this cylinder center. Not modified.
    * @param axis     the axis of revolution of this cylinder. Not modified.
    * @param length   the new length.
    * @param radius   the new radius.
    * @throws IllegalArgumentException        if {@code length} or {@code radius} is negative.
    * @throws ReferenceFrameMismatchException if {@code position} and {@code axis} are not expressed in
    *                                         the same reference frame.
    */
   default void setMatchingFrame(FramePoint3DReadOnly position, FrameVector3DReadOnly axis, double length, double radius)
   {
      position.checkReferenceFrameMatch(axis);
      setMatchingFrame(position.getReferenceFrame(), position, axis, length, radius);
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
