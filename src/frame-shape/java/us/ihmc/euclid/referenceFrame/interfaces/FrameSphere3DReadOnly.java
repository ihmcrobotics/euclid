package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeTools;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;

public interface FrameSphere3DReadOnly extends Sphere3DReadOnly, FrameShape3DReadOnly
{
   @Override
   FramePoint3DReadOnly getPosition();

   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      FrameShape3DReadOnly.super.getBoundingBox(boundingBoxToPack);
   }

   @Override
   default void getBoundingBox(ReferenceFrame destinationFrame, BoundingBox3DBasics boundingBoxToPack)
   {
      EuclidFrameShapeTools.boundingBoxSphere3D(this, destinationFrame, boundingBoxToPack);
   }

   default boolean epsilonEquals(FrameSphere3DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return Sphere3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   default boolean geometricallyEquals(FrameSphere3DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return Sphere3DReadOnly.super.geometricallyEquals(other, epsilon);
   }

   default boolean equals(FrameSphere3DReadOnly other)
   {
      if (other == this)
         return true;
      else if (other == null || getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return getPosition().equals(other.getPosition()) && getRadius() == other.getRadius();
   }
}
