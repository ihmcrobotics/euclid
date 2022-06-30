package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.shape.primitives.interfaces.Torus3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read and write interface for a torus 3D expressed in a changeable reference frame, i.e. the
 * reference frame in which this line is expressed can be changed.
 * <p>
 * A torus is represented by its position, its axis of revolution, the radius of its tube, and the
 * radius from the torus axis to the tube center.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FrameTorus3DBasics extends FixedFrameTorus3DBasics, FrameShape3DBasics
{
   /**
    * Copies the {@code other} torus data into {@code this} and updates its reference frame.
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param other          the other torus to copy. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Torus3DReadOnly other)
   {
      setReferenceFrame(referenceFrame);
      set(other);
   }

   /**
    * Copies the {@code other} torus data into {@code this} and updates its reference frame.
    *
    * @param other the other torus to copy. Not modified.
    */
   default void setIncludingFrame(FrameTorus3DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }

   /**
    * Sets this torus properties and its reference frame.
    *
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param position       the position of this torus center. Not modified.
    * @param axis           the axis of revolution of this torus. Not modified.
    * @param radius         radius from the torus center to the tube center.
    * @param tubeRadius     radius of the torus' tube.
    * @throws IllegalArgumentException if either {@code radius < 0.0} or {@code tubeRadius < 0.0}.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly position, Vector3DReadOnly axis, double radius, double tubeRadius)
   {
      setReferenceFrame(referenceFrame);
      set(position, axis, radius, tubeRadius);
   }

   /**
    * Sets this torus properties and its reference frame.
    *
    * @param position   the position of this torus center. Not modified.
    * @param axis       the axis of revolution of this torus. Not modified.
    * @param radius     radius from the torus center to the tube center.
    * @param tubeRadius radius of the torus' tube.
    * @throws IllegalArgumentException        if either {@code radius < 0.0} or
    *                                         {@code tubeRadius < 0.0}.
    * @throws ReferenceFrameMismatchException if the frame argument are not expressed in the same
    *                                         reference frame.
    */
   default void setIncludingFrame(FramePoint3DReadOnly position, FrameVector3DReadOnly axis, double radius, double tubeRadius)
   {
      position.checkReferenceFrameMatch(axis);
      setIncludingFrame(position.getReferenceFrame(), position, axis, radius, tubeRadius);
   }

   /** {@inheritDoc} */
   @Override
   FrameTorus3DBasics copy();
}
