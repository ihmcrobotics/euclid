package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Provides a {@link ConvexPolytope3DReadOnly} view backed by a {@link Box3DReadOnly}.
 * 
 * @author Sylvain Bertrand
 */
public interface BoxPolytope3DView extends ConvexPolytope3DReadOnly
{
   Face3DReadOnly getXMaxFace();

   Face3DReadOnly getYMaxFace();

   Face3DReadOnly getZMaxFace();

   default Face3DReadOnly getMaxFace(Axis3D axis)
   {
      switch (axis)
      {
         case X:
            return getXMaxFace();
         case Y:
            return getYMaxFace();
         case Z:
            return getZMaxFace();
         default:
            throw new IllegalStateException();
      }
   }

   Face3DReadOnly getXMinFace();

   Face3DReadOnly getYMinFace();

   Face3DReadOnly getZMinFace();

   default Face3DReadOnly getMinFace(Axis3D axis)
   {
      switch (axis)
      {
         case X:
            return getXMinFace();
         case Y:
            return getYMinFace();
         case Z:
            return getZMinFace();
         default:
            throw new IllegalStateException();
      }
   }

   Box3DReadOnly getOwner();

   @Override
   default Shape3DBasics copy()
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
