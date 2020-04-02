package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Read-only interface for a sphere 3D expressed in a changeable reference frame, i.e. the reference
 * frame in which this line is expressed can be changed.
 * <p>
 * A sphere 3D is represented by its position and radius.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FrameSphere3DBasics extends FixedFrameSphere3DBasics, FrameShape3DBasics
{
   /**
    * Copies the {@code other} sphere data into {@code this} and updates its reference frame.
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param other          the other sphere to copy. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Sphere3DReadOnly other)
   {
      setReferenceFrame(referenceFrame);
      set(other);
   }

   /**
    * Copies the {@code other} sphere data into {@code this} and updates its reference frame.
    *
    * @param other the other sphere to copy. Not modified.
    */
   default void setIncludingFrame(FrameSphere3DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }

   /**
    * Sets this sphere properties and its reference frame.
    *
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param centerX        the x-coordinate of the center.
    * @param centerY        the y-coordinate of the center.
    * @param centerZ        the z-coordinate of the center.
    * @param radius         the radius for this sphere.
    * @throws IllegalArgumentException if {@code radius < 0.0}.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, double centerX, double centerY, double centerZ, double radius)
   {
      setReferenceFrame(referenceFrame);
      set(centerX, centerY, centerZ, radius);
   }

   /**
    * Sets this sphere properties and its reference frame.
    *
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param center         the position of this sphere center. Not modified.
    * @param radius         the radius for this sphere.
    * @throws IllegalArgumentException if {@code radius < 0.0}.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly center, double radius)
   {
      setReferenceFrame(referenceFrame);
      set(center, radius);
   }

   /**
    * Sets this sphere properties and its reference frame.
    *
    * @param center the position of this sphere center. Not modified.
    * @param radius the radius for this sphere.
    * @throws IllegalArgumentException if {@code radius < 0.0}.
    */
   default void setIncludingFrame(FramePoint3DReadOnly center, double radius)
   {
      setIncludingFrame(center.getReferenceFrame(), center, radius);
   }

   /** {@inheritDoc} */
   @Override
   FrameSphere3DBasics copy();
}
