package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

/**
 * Write and read interface for a point shape 3D.
 * <p>
 * A point shape 3D is represented by its position.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface PointShape3DBasics extends PointShape3DReadOnly, Shape3DBasics, Point3DBasics
{
   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return PointShape3DReadOnly.super.containsNaN();
   }

   @Override
   default Shape3DPoseBasics getPose()
   {
      return null;
   }
}
