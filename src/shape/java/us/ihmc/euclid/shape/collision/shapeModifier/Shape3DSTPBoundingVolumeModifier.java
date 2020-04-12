package us.ihmc.euclid.shape.collision.shapeModifier;

import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.shape.collision.shapeModifier.interfaces.GJKShape3DModifier;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;

public class Shape3DSTPBoundingVolumeModifier implements GJKShape3DModifier
{
   private final Box3DSTPBoundingVolume box3DSTPBV = new Box3DSTPBoundingVolume();
   private final Capsule3DSTPBoundingVolume capsule3DSTPBV = new Capsule3DSTPBoundingVolume();
   private final ConvexPolytope3DSTPBoundingVolume convexPolytope3DSTPBV = new ConvexPolytope3DSTPBoundingVolume();
   private final Cylinder3DSTPBoundingVolume cylinder3DSTPBV = new Cylinder3DSTPBoundingVolume();
   private final Ramp3DSTPBoundingVolume ramp3DSTPBV = new Ramp3DSTPBoundingVolume();

   public Shape3DSTPBoundingVolumeModifier()
   {
   }

   public void setMargins(double minimumMargin, double maximumMargin)
   {
      box3DSTPBV.setMargins(minimumMargin, maximumMargin);
      capsule3DSTPBV.setMargins(minimumMargin, maximumMargin);
      convexPolytope3DSTPBV.setMargins(minimumMargin, maximumMargin);
      cylinder3DSTPBV.setMargins(minimumMargin, maximumMargin);
      ramp3DSTPBV.setMargins(minimumMargin, maximumMargin);
   }

   @Override
   public SupportingVertexHolder toSupportingVertexHolder(Shape3DReadOnly shape3D)
   {
      if (shape3D instanceof Box3DReadOnly)
      {
         box3DSTPBV.setShape3D((Box3DReadOnly) shape3D);
         return box3DSTPBV;
      }

      if (shape3D instanceof Capsule3DReadOnly)
      {
         capsule3DSTPBV.setShape3D((Capsule3DReadOnly) shape3D);
         return capsule3DSTPBV;
      }

      if (shape3D instanceof ConvexPolytope3DReadOnly)
      {
         convexPolytope3DSTPBV.setShape3D((ConvexPolytope3DReadOnly) shape3D);
         return convexPolytope3DSTPBV;
      }

      if (shape3D instanceof Cylinder3DReadOnly)
      {
         cylinder3DSTPBV.setShape3D((Cylinder3DReadOnly) shape3D);
         return cylinder3DSTPBV;
      }

      if (shape3D instanceof Ramp3DReadOnly)
      {
         ramp3DSTPBV.setShape3D((Ramp3DReadOnly) shape3D);
         return ramp3DSTPBV;
      }

      return null;
   }
}
