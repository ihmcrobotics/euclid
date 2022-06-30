package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.shape.primitives.interfaces.Torus3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Torus3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read and write interface for a torus 3D expressed in a constant reference frame, i.e. the
 * reference frame of this object cannot be changed via this interface.
 * <p>
 * A torus is represented by its position, its axis of revolution, the radius of its tube, and the
 * radius from the torus axis to the tube center.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FixedFrameTorus3DBasics extends Torus3DBasics, FrameTorus3DReadOnly, FixedFrameShape3DBasics
{
   /** {@inheritDoc} */
   @Override
   FixedFramePoint3DBasics getPosition();

   /** {@inheritDoc} */
   @Override
   FixedFrameUnitVector3DBasics getAxis();

   /**
    * Copies the {@code other} torus data into {@code this}.
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param other          the other torus to copy. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, Torus3DReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      Torus3DBasics.super.set(other);
   }

   /**
    * Copies the {@code other} torus data into {@code this}.
    *
    * @param other the other torus to copy. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(FrameTorus3DReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }

   /**
    * Sets this torus properties.
    *
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param position       the position of this torus center. Not modified.
    * @param axis           the axis of revolution of this torus. Not modified.
    * @param radius         radius from the torus center to the tube center.
    * @param tubeRadius     radius of the torus' tube.
    * @throws IllegalArgumentException        if either {@code radius < 0.0} or
    *                                         {@code tubeRadius < 0.0}.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, Point3DReadOnly position, Vector3DReadOnly axis, double radius, double tubeRadius)
   {
      checkReferenceFrameMatch(referenceFrame);
      Torus3DBasics.super.set(position, axis, radius, tubeRadius);
   }

   /**
    * Sets this torus properties.
    *
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param position       the position of this torus center. Not modified.
    * @param axis           the axis of revolution of this torus. Not modified.
    * @param radius         radius from the torus center to the tube center.
    * @param tubeRadius     radius of the torus' tube.
    * @throws IllegalArgumentException        if either {@code radius < 0.0} or
    *                                         {@code tubeRadius < 0.0}.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(FramePoint3DReadOnly position, FrameVector3DReadOnly axis, double radius, double tubeRadius)
   {
      position.checkReferenceFrameMatch(axis);
      set(position.getReferenceFrame(), position, axis, radius, tubeRadius);
   }

   /**
    * Copies the {@code other} torus data into {@code this}.
    * <p>
    * If {@code other} is expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(ReferenceFrame, Torus3DReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param other          the other torus to copy. Not modified.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, Torus3DReadOnly other)
   {
      Torus3DBasics.super.set(other);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Copies the {@code other} torus data into {@code this}.
    * <p>
    * If {@code other} is expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(ReferenceFrame, Torus3DReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param other the other torus to copy. Not modified.
    */
   default void setMatchingFrame(FrameTorus3DReadOnly other)
   {
      setMatchingFrame(other.getReferenceFrame(), other);
   }

   /**
    * Sets this torus properties.
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
    * @param position       the position of this torus center. Not modified.
    * @param axis           the axis of revolution of this torus. Not modified.
    * @param radius         radius from the torus center to the tube center.
    * @param tubeRadius     radius of the torus' tube.
    * @throws IllegalArgumentException if either {@code radius < 0.0} or {@code tubeRadius < 0.0}.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, Point3DReadOnly position, Vector3DReadOnly axis, double radius, double tubeRadius)
   {
      Torus3DBasics.super.set(position, axis, radius, tubeRadius);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this torus properties.
    * <p>
    * If the arguments are expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(FramePoint3DReadOnly, FrameVector3DReadOnly, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param position   the position of this torus center. Not modified.
    * @param axis       the axis of revolution of this torus. Not modified.
    * @param radius     radius from the torus center to the tube center.
    * @param tubeRadius radius of the torus' tube.
    * @throws IllegalArgumentException        if either {@code radius < 0.0} or
    *                                         {@code tubeRadius < 0.0}.
    * @throws ReferenceFrameMismatchException if {@code position} and {@code axis} are not expressed in
    *                                         the same reference frame.
    */
   default void setMatchingFrame(FramePoint3DReadOnly position, FrameVector3DReadOnly axis, double radius, double tubeRadius)
   {
      position.checkReferenceFrameMatch(axis);
      setMatchingFrame(position.getReferenceFrame(), position, axis, radius, tubeRadius);
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
