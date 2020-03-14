package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface SupportingFrameVertexHolder extends SupportingVertexHolder, ReferenceFrameHolder
{
   @Override
   default FramePoint3DReadOnly getSupportingVertex(Vector3DReadOnly supportDirection)
   {
      FramePoint3D supportingVertex = new FramePoint3D();
      boolean success = getSupportingVertex(supportDirection, supportingVertex);
      if (success)
         return supportingVertex;
      else
         return null;
   }

   default FramePoint3DReadOnly getSupportingVertex(FrameVector3DReadOnly supportDirection)
   {
      checkReferenceFrameMatch(supportDirection);
      return getSupportingVertex((Vector3DReadOnly) supportDirection);
   }

   default boolean getSupportingVertex(Vector3DReadOnly supportDirection, FixedFramePoint3DBasics supportingVertexToPack)
   {
      checkReferenceFrameMatch(supportingVertexToPack);
      return getSupportingVertex(supportDirection, (Point3DBasics) supportingVertexToPack);
   }

   default boolean getSupportingVertex(Vector3DReadOnly supportDirection, FramePoint3DBasics supportingVertexToPack)
   {
      boolean success = getSupportingVertex(supportDirection, (Point3DBasics) supportingVertexToPack);
      if (success)
         supportingVertexToPack.setReferenceFrame(getReferenceFrame());
      return success;
   }

   default boolean getSupportingVertex(FrameVector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      checkReferenceFrameMatch(supportDirection);
      return getSupportingVertex((Vector3DReadOnly) supportDirection, supportingVertexToPack);
   }

   default boolean getSupportingVertex(FrameVector3DReadOnly supportDirection, FixedFramePoint3DBasics supportingVertexToPack)
   {
      checkReferenceFrameMatch(supportDirection);
      return getSupportingVertex((Vector3DReadOnly) supportDirection, supportingVertexToPack);
   }

   default boolean getSupportingVertex(FrameVector3DReadOnly supportDirection, FramePoint3DBasics supportingVertexToPack)
   {
      checkReferenceFrameMatch(supportDirection);
      return getSupportingVertex((Vector3DReadOnly) supportDirection, supportingVertexToPack);
   }
}
