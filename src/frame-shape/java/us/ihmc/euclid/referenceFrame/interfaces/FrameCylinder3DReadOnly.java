package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeTools;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface FrameCylinder3DReadOnly extends Cylinder3DReadOnly, FrameShape3DReadOnly
{
   @Override
   FramePoint3DReadOnly getPosition();

   @Override
   FrameVector3DReadOnly getAxis();

   @Override
   default FramePoint3DReadOnly getTopCenter()
   {
      FramePoint3D topCenter = new FramePoint3D(getReferenceFrame());
      topCenter.scaleAdd(getHalfLength(), getAxis(), getPosition());
      return topCenter;
   }

   default Point3DReadOnly getBottomCenter()
   {
      FramePoint3D bottomCenter = new FramePoint3D(getReferenceFrame());
      bottomCenter.scaleAdd(-getHalfLength(), getAxis(), getPosition());
      return bottomCenter;
   }

   default boolean epsilonEquals(FrameCylinder3DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return Cylinder3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   default boolean geometricallyEquals(FrameCylinder3DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return Cylinder3DReadOnly.super.geometricallyEquals(other, epsilon);
   }

   default boolean equals(FrameCylinder3DReadOnly other)
   {
      if (other == this)
      {
         return true;
      }
      else if (other == null)
      {
         return false;
      }
      else
      {
         if (getReferenceFrame() != other.getReferenceFrame())
            return false;
         if (getLength() != other.getLength())
            return false;
         if (getRadius() != other.getRadius())
            return false;
         if (!getPosition().equals(other.getPosition()))
            return false;
         if (!getAxis().equals(other.getAxis()))
            return false;
         return true;
      }
   }

   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      FrameShape3DReadOnly.super.getBoundingBox(boundingBoxToPack);
   }

   @Override
   default void getBoundingBox(ReferenceFrame destinationFrame, BoundingBox3DBasics boundingBoxToPack)
   {
      EuclidFrameShapeTools.boundingBoxCylinder3D(this, destinationFrame, boundingBoxToPack);
   }
}
