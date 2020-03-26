package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;

/**
 * Write and read interface for representing a 3D shape.
 *
 * @author Sylvain Bertrand
 */
public interface Shape3DBasics extends Shape3DReadOnly, Clearable, Transformable
{
   @Override
   Shape3DPoseBasics getPose();

   @Override
   Shape3DBasics copy();
}
