package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DReadOnly;

public interface FramePointShape3DReadOnly extends PointShape3DReadOnly, FrameShape3DReadOnly, FramePoint3DReadOnly
{
   @Override
   default double distance(FramePoint3DReadOnly point)
   {
      return FramePoint3DReadOnly.super.distance(point);
   }

   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      FrameShape3DReadOnly.super.getBoundingBox(boundingBoxToPack);
   }

   @Override
   default void getBoundingBox(ReferenceFrame destinationFrame, BoundingBox3DBasics boundingBoxToPack)
   {
      boundingBoxToPack.getMinPoint().set(this);
      getReferenceFrame().transformFromThisToDesiredFrame(destinationFrame, boundingBoxToPack.getMinPoint());
      boundingBoxToPack.getMaxPoint().set(boundingBoxToPack.getMinPoint());
   }

   @Override
   FramePointShape3DReadOnly copy();
}
