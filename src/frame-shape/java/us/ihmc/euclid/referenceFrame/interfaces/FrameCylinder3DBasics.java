package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read and write interface for a cylinder 3D expressed in a changeable reference frame, i.e. the
 * reference frame in which this line is expressed can be changed.
 * <p>
 * A cylinder 3D is represented by its length, its radius, the position of its center, and its axis
 * of revolution.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FrameCylinder3DBasics extends FixedFrameCylinder3DBasics, FrameShape3DBasics
{
   /**
    * Copies the {@code other} cylinder data into {@code this} and updates its reference frame.
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param other          the other cylinder to copy. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Cylinder3DReadOnly other)
   {
      setReferenceFrame(referenceFrame);
      set(other);
   }

   /**
    * Copies the {@code other} cylinder data into {@code this} and updates its reference frame.
    *
    * @param other the other cylinder to copy. Not modified.
    */
   default void setIncludingFrame(FrameCylinder3DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }

   /**
    * Sets this cylinder properties and its reference frame.
    *
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param position       the position of this cylinder center. Not modified.
    * @param axis           the axis of revolution of this cylinder. Not modified.
    * @param length         the new length.
    * @param radius         the new radius.
    * @throws IllegalArgumentException if {@code length} or {@code radius} is negative.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
   {
      setReferenceFrame(referenceFrame);
      set(position, axis, length, radius);
   }

   /**
    * Sets this cylinder properties and its reference frame.
    *
    * @param position the position of this cylinder center. Not modified.
    * @param axis     the axis of revolution of this cylinder. Not modified.
    * @param length   the new length.
    * @param radius   the new radius.
    * @throws IllegalArgumentException if {@code length} or {@code radius} is negative.
    */
   default void setIncludingFrame(FramePoint3DReadOnly position, FrameVector3DReadOnly axis, double length, double radius)
   {
      position.checkReferenceFrameMatch(axis);
      setIncludingFrame(position.getReferenceFrame(), position, axis, length, radius);
   }

   /** {@inheritDoc} */
   @Override
   FrameCylinder3DBasics copy();
}