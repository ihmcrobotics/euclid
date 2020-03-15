package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.FrameBoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public interface FrameShape3DReadOnly extends Shape3DReadOnly, SupportingFrameVertexHolder
{
   default boolean evaluatePoint3DCollision(Point3DReadOnly pointToCheck, FixedFramePoint3DBasics closestPointOnSurfaceToPack,
                                            FixedFrameVector3DBasics normalAtClosestPointToPack)
   {
      checkReferenceFrameMatch(closestPointOnSurfaceToPack);
      checkReferenceFrameMatch(normalAtClosestPointToPack);
      return evaluatePoint3DCollision(pointToCheck, (Point3DBasics) closestPointOnSurfaceToPack, (Vector3DBasics) normalAtClosestPointToPack);
   }

   default boolean evaluatePoint3DCollision(Point3DReadOnly pointToCheck, FramePoint3DBasics closestPointOnSurfaceToPack,
                                            FrameVector3DBasics normalAtClosestPointToPack)
   {
      closestPointOnSurfaceToPack.setReferenceFrame(getReferenceFrame());
      normalAtClosestPointToPack.setReferenceFrame(getReferenceFrame());
      return evaluatePoint3DCollision(pointToCheck, (Point3DBasics) closestPointOnSurfaceToPack, (Vector3DBasics) normalAtClosestPointToPack);
   }

   default boolean evaluatePoint3DCollision(FramePoint3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack,
                                            Vector3DBasics normalAtClosestPointToPack)
   {
      checkReferenceFrameMatch(pointToCheck);
      return evaluatePoint3DCollision((Point3DReadOnly) pointToCheck, closestPointOnSurfaceToPack, normalAtClosestPointToPack);
   }

   default boolean evaluatePoint3DCollision(FramePoint3DReadOnly pointToCheck, FixedFramePoint3DBasics closestPointOnSurfaceToPack,
                                            FixedFrameVector3DBasics normalAtClosestPointToPack)
   {
      checkReferenceFrameMatch(pointToCheck);
      return evaluatePoint3DCollision((Point3DReadOnly) pointToCheck, closestPointOnSurfaceToPack, normalAtClosestPointToPack);
   }

   default boolean evaluatePoint3DCollision(FramePoint3DReadOnly pointToCheck, FramePoint3DBasics closestPointOnSurfaceToPack,
                                            FrameVector3DBasics normalAtClosestPointToPack)
   {
      checkReferenceFrameMatch(pointToCheck);
      return evaluatePoint3DCollision((Point3DReadOnly) pointToCheck, closestPointOnSurfaceToPack, normalAtClosestPointToPack);
   }

   default double distance(FramePoint3DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return distance((Point3DReadOnly) point);
   }

   default double signedDistance(FramePoint3DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return signedDistance((Point3DReadOnly) point);
   }

   default boolean isPointInside(FramePoint3DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return isPointInside((Point3DReadOnly) query);
   }

   default boolean isPointInside(FramePoint3DReadOnly query, double epsilon)
   {
      checkReferenceFrameMatch(query);
      return isPointInside((Point3DReadOnly) query, epsilon);
   }

   @Override
   default FramePoint3DBasics orthogonalProjectionCopy(Point3DReadOnly pointToProject)
   {
      FramePoint3D projection = new FramePoint3D();

      if (orthogonalProjection(pointToProject, projection))
         return projection;
      else
         return null;
   }

   default FramePoint3DBasics orthogonalProjectionCopy(FramePoint3DReadOnly pointToProject)
   {
      checkReferenceFrameMatch(pointToProject);
      return orthogonalProjectionCopy((Point3DReadOnly) pointToProject);
   }

   default boolean orthogonalProjection(FixedFramePoint3DBasics pointToProject)
   {
      return orthogonalProjection(pointToProject, pointToProject);
   }

   default boolean orthogonalProjection(FramePoint3DBasics pointToProject)
   {
      return orthogonalProjection(pointToProject, pointToProject);
   }

   default boolean orthogonalProjection(Point3DReadOnly pointToProject, FixedFramePoint3DBasics projectionToPack)
   {
      checkReferenceFrameMatch(projectionToPack);
      return orthogonalProjection(pointToProject, (Point3DBasics) projectionToPack);
   }

   default boolean orthogonalProjection(Point3DReadOnly pointToProject, FramePoint3DBasics projectionToPack)
   {
      projectionToPack.setReferenceFrame(getReferenceFrame());
      return orthogonalProjection(pointToProject, (Point3DBasics) projectionToPack);
   }

   default boolean orthogonalProjection(FramePoint3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      return orthogonalProjection((Point3DReadOnly) pointToProject, projectionToPack);
   }

   default boolean orthogonalProjection(FramePoint3DReadOnly pointToProject, FixedFramePoint3DBasics projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      return orthogonalProjection((Point3DReadOnly) pointToProject, projectionToPack);
   }

   default boolean orthogonalProjection(FramePoint3DReadOnly pointToProject, FramePoint3DBasics projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      return orthogonalProjection((Point3DReadOnly) pointToProject, projectionToPack);
   }

   @Override
   default FrameBoundingBox3DReadOnly getBoundingBox()
   {
      FrameBoundingBox3D boundingBox3D = new FrameBoundingBox3D();
      getBoundingBox(boundingBox3D);
      return boundingBox3D;
   }

   default FrameBoundingBox3DReadOnly getBoundingBox(ReferenceFrame destinationFrame)
   {
      FrameBoundingBox3D boundingBox3D = new FrameBoundingBox3D();
      getBoundingBox(destinationFrame, boundingBox3D);
      return boundingBox3D;
   }

   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      getBoundingBox(getReferenceFrame(), boundingBoxToPack);
   }

   default void getBoundingBox(FixedFrameBoundingBox3DBasics boundingBoxToPack)
   {
      checkReferenceFrameMatch(boundingBoxToPack);
      getBoundingBox(getReferenceFrame(), boundingBoxToPack);
   }

   default void getBoundingBox(FrameBoundingBox3DBasics boundingBoxToPack)
   {
      getBoundingBox(getReferenceFrame(), boundingBoxToPack);
   }

   void getBoundingBox(ReferenceFrame destinationFrame, BoundingBox3DBasics boundingBoxToPack);

   default void getBoundingBox(ReferenceFrame destinationFrame, FixedFrameBoundingBox3DBasics boundingBoxToPack)
   {
      destinationFrame.checkReferenceFrameMatch(boundingBoxToPack);
      getBoundingBox(destinationFrame, (BoundingBox3DBasics) boundingBoxToPack);
   }

   default void getBoundingBox(ReferenceFrame destinationFrame, FrameBoundingBox3DBasics boundingBoxToPack)
   {
      boundingBoxToPack.setReferenceFrame(destinationFrame);
      getBoundingBox(destinationFrame, (BoundingBox3DBasics) boundingBoxToPack);
   }
}
