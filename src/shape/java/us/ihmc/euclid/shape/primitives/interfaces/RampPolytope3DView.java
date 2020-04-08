package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Provides a {@link ConvexPolytope3DReadOnly} view backed by a {@link Ramp3DReadOnly}.
 * 
 * @author Sylvain Bertrand
 */
public interface RampPolytope3DView extends ConvexPolytope3DReadOnly
{
   Face3DReadOnly getRampFace();

   Face3DReadOnly getXMaxFace();

   Face3DReadOnly getYMaxFace();

   Face3DReadOnly getYMinFace();
   
   Face3DReadOnly getZMinFace();
   
   default Face3DReadOnly getMaxFace(Axis3D axis)
   {
      switch (axis)
      {
         case X:
            return getXMaxFace();
         case Y:
            return getYMaxFace();
         case Z:
            return getRampFace();
         default:
            throw new IllegalStateException();
      }
   }

   default Face3DReadOnly getMinFace(Axis3D axis)
   {
      switch (axis)
      {
         case X:
            return getRampFace();
         case Y:
            return getYMinFace();
         case Z:
            return getZMinFace();
         default:
            throw new IllegalStateException();
      }
   }

   Ramp3DReadOnly getOwner();

   @Override
   default Ramp3DBasics copy()
   {
      return getOwner().copy();
   }

   @Override
   default BoundingBox3DReadOnly getBoundingBox()
   {
      return getOwner().getBoundingBox();
   }

   @Override
   default Point3DReadOnly getCentroid()
   {
      return getOwner().getCentroid();
   }

   @Override
   default double getVolume()
   {
      return getOwner().getVolume();
   }

   @Override
   default double getConstructionEpsilon()
   {
      return 0;
   }
}
