package us.ihmc.euclid.referenceFrame.polytope.interfaces;

import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface FrameFace3DReadOnly extends Face3DReadOnly, SupportingFrameVertexHolder
{
   @Override
   FramePoint3DReadOnly getCentroid();

   @Override
   FrameVector3DReadOnly getNormal();

   @Override
   FrameBoundingBox3DReadOnly getBoundingBox();

   @Override
   List<? extends FrameHalfEdge3DReadOnly> getEdges();

   @Override
   default FrameHalfEdge3DReadOnly getEdge(int index)
   {
      return getEdges().get(index);
   }

   @Override
   default List<? extends FrameVertex3DReadOnly> getVertices()
   {
      return getEdges().stream().map(FrameHalfEdge3DReadOnly::getOrigin).collect(Collectors.toList());
   }

   @Override
   default FrameVertex3DReadOnly getVertex(int index)
   {
      return getEdge(index).getOrigin();
   }

   @SuppressWarnings("unchecked")
   @Override
   default List<? extends FrameHalfEdge3DReadOnly> lineOfSight(Point3DReadOnly observer)
   {
      return (List<? extends FrameHalfEdge3DReadOnly>) Face3DReadOnly.super.lineOfSight(observer);
   }

   default List<? extends FrameHalfEdge3DReadOnly> lineOfSight(FramePoint3DReadOnly observer)
   {
      checkReferenceFrameMatch(observer);
      return lineOfSight((Point3DReadOnly) observer);
   }

   @SuppressWarnings("unchecked")
   @Override
   default List<? extends FrameHalfEdge3DReadOnly> lineOfSight(Point3DReadOnly observer, double epsilon)
   {
      return (List<? extends FrameHalfEdge3DReadOnly>) Face3DReadOnly.super.lineOfSight(observer, epsilon);
   }

   default List<? extends FrameHalfEdge3DReadOnly> lineOfSight(FramePoint3DReadOnly observer, double epsilon)
   {
      checkReferenceFrameMatch(observer);
      return lineOfSight((Point3DReadOnly) observer, epsilon);
   }

   @Override
   default FrameHalfEdge3DReadOnly lineOfSightStart(Point3DReadOnly observer)
   {
      return (FrameHalfEdge3DReadOnly) Face3DReadOnly.super.lineOfSightStart(observer);
   }

   default FrameHalfEdge3DReadOnly lineOfSightStart(FramePoint3DReadOnly observer)
   {
      checkReferenceFrameMatch(observer);
      return lineOfSightStart((Point3DReadOnly) observer);
   }

   @Override
   default FrameHalfEdge3DReadOnly lineOfSightEnd(Point3DReadOnly observer)
   {
      return (FrameHalfEdge3DReadOnly) Face3DReadOnly.super.lineOfSightEnd(observer);
   }

   default FrameHalfEdge3DReadOnly lineOfSightEnd(FramePoint3DReadOnly observer)
   {
      checkReferenceFrameMatch(observer);
      return lineOfSightEnd((Point3DReadOnly) observer);
   }

   default boolean canObserverSeeEdge(FramePoint3DReadOnly observer, int index)
   {
      checkReferenceFrameMatch(observer);
      return Face3DReadOnly.super.canObserverSeeEdge(observer, index);
   }

   default boolean canObserverSeeEdge(FramePoint3DReadOnly observer, FrameHalfEdge3DReadOnly edge)
   {
      checkReferenceFrameMatch(observer, edge);
      return Face3DReadOnly.super.canObserverSeeEdge(observer, edge);
   }

   default boolean canObserverSeeFace(FramePoint3DReadOnly observer)
   {
      checkReferenceFrameMatch(observer);
      return Face3DReadOnly.super.canObserverSeeFace(observer);
   }

   default boolean canObserverSeeFace(FramePoint3DReadOnly observer, double epsilon)
   {
      checkReferenceFrameMatch(observer);
      return Face3DReadOnly.super.canObserverSeeFace(observer, epsilon);
   }

   default boolean isPointInFaceSupportPlane(FramePoint3DReadOnly query, double epsilon)
   {
      checkReferenceFrameMatch(query);
      return Face3DReadOnly.super.isPointInFaceSupportPlane(query, epsilon);
   }

   default boolean isPointInside(FramePoint3DReadOnly query, double epsilon)
   {
      checkReferenceFrameMatch(query);
      return Face3DReadOnly.super.isPointInside(query, epsilon);
   }

   default boolean isPointDirectlyAboveOrBelow(FramePoint3DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return Face3DReadOnly.super.isPointDirectlyAboveOrBelow(query);
   }

   default FrameFace3DReadOnly getNeighbor(int index)
   {
      return (FrameFace3DReadOnly) Face3DReadOnly.super.getNeighbor(index);
   }

   default FrameHalfEdge3DReadOnly getCommonEdgeWith(Face3DReadOnly neighbor)
   {
      return (FrameHalfEdge3DReadOnly) Face3DReadOnly.super.getCommonEdgeWith(neighbor);
   }

   default FrameHalfEdge3DReadOnly getCommonEdgeWith(FrameFace3DReadOnly neighbor)
   {
      checkReferenceFrameMatch(neighbor);
      return getCommonEdgeWith((Face3DReadOnly) neighbor);
   }

   default FrameHalfEdge3DReadOnly getClosestEdge(Point3DReadOnly query)
   {
      return (FrameHalfEdge3DReadOnly) Face3DReadOnly.super.getClosestEdge(query);
   }

   default FrameHalfEdge3DReadOnly getClosestEdge(FramePoint3DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return (FrameHalfEdge3DReadOnly) Face3DReadOnly.super.getClosestEdge(query);
   }

   @Override
   default FrameHalfEdge3DReadOnly getClosestVisibleEdge(Point3DReadOnly query)
   {
      return (FrameHalfEdge3DReadOnly) Face3DReadOnly.super.getClosestVisibleEdge(query);
   }

   default FrameHalfEdge3DReadOnly getClosestVisibleEdge(FramePoint3DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return getClosestVisibleEdge((Point3DReadOnly) query);
   }

   default double signedDistanceFromSupportPlane(FramePoint3DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return Face3DReadOnly.super.signedDistanceFromSupportPlane(query);
   }

   default double distance(FramePoint3DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return Face3DReadOnly.super.distance(query);
   }

   default double distanceFromSupportPlane(FramePoint3DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return Face3DReadOnly.super.distanceFromSupportPlane(query);
   }

   default FramePoint3DBasics orthogonalProjectionCopy(FramePoint3DReadOnly pointToProject)
   {
      FramePoint3D projection = new FramePoint3D();
      if (orthogonalProjection(pointToProject, projection))
         return projection;
      else
         return null;
   }

   default boolean orthogonalProjection(FramePoint3DReadOnly pointToProject, FramePoint3DBasics projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      projectionToPack.setReferenceFrame(getReferenceFrame());
      return Face3DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }

   default boolean orthogonalProjection(FramePoint3DReadOnly pointToProject, FixedFramePoint3DBasics projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject, projectionToPack);
      return Face3DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }

   @Override
   default FrameVertex3DReadOnly getSupportingVertex(Vector3DReadOnly supportDirection)
   {
      return (FrameVertex3DReadOnly) Face3DReadOnly.super.getSupportingVertex(supportDirection);
   }

   default FrameVertex3DReadOnly getSupportingVertex(FrameVector3DReadOnly supportDirection)
   {
      checkReferenceFrameMatch(supportDirection);
      return getSupportingVertex((Vector3DReadOnly) supportDirection);
   }

   default boolean epsilonEquals(FrameFace3DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return Face3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   default boolean geometricallyEquals(FrameFace3DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return Face3DReadOnly.super.geometricallyEquals(other, epsilon);
   }

   default boolean equals(FrameFace3DReadOnly other)
   {
      if (other == this)
         return true;
      if (other == null || getReferenceFrame() != other.getReferenceFrame())
         return false;
      if (getNumberOfEdges() != other.getNumberOfEdges())
         return false;

      for (int edgeIndex = 0; edgeIndex < getNumberOfEdges(); edgeIndex++)
      {
         if (!getEdge(edgeIndex).equals(other.getEdge(edgeIndex)))
            return false;
      }
      return true;
   }
}
