package us.ihmc.euclid.shape.convexPolytope.interfaces;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.collision.SupportingVertexHolder;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface Face3DReadOnly extends SupportingVertexHolder, Simplex3D
{
   /**
    * Gets the centroid of the face
    *
    * @return read only point object indicating the spatial location of the face centroid
    */
   Point3DReadOnly getCentroid();

   /**
    * Gets a vector normal to the face. If the face is part of a polytope, the vector will point away
    * from the centroid of the polytope. If the face is not part of a polytope, the direction of the
    * vector is arbitrary
    *
    * @return a read only vector normal to the face
    */
   Vector3DReadOnly getNormal();

   double getArea();

   BoundingBox3DReadOnly getBoundingBox();

   /**
    * Gets a list of all half edges that constitute the face
    *
    * @return a list of read only references to the half edges
    */
   List<? extends HalfEdge3DReadOnly> getEdges();

   /**
    * Gets a particular half edge that is part of the face
    *
    * @param index the index of the half edge that is required. Should be less than value returned by
    *           {@code getNumberOfEdges()}
    * @return
    */
   default HalfEdge3DReadOnly getEdge(int index)
   {
      return getEdges().get(index);
   }

   default List<? extends Vertex3DReadOnly> getVertices()
   {
      return getEdges().stream().map(HalfEdge3DReadOnly::getOrigin).collect(Collectors.toList());
   }

   default Vertex3DReadOnly getVertex(int index)
   {
      return getEdge(index).getOrigin();
   }

   /**
    * Gets the number of edges that the face is made up of
    *
    * @return
    */
   default int getNumberOfEdges()
   {
      return getEdges().size();
   }

   default boolean isEmpty()
   {
      return getEdges().isEmpty();
   }

   default boolean containsNaN()
   {
      if (isEmpty())
         return false;

      for (int edgeIndex = 0; edgeIndex < getNumberOfEdges(); edgeIndex++)
      {
         if (getEdge(edgeIndex).getOrigin().containsNaN())
            return true;
      }

      return false;
   }

   default List<? extends HalfEdge3DReadOnly> lineOfSight(Point3DReadOnly observer)
   {
      HalfEdge3DReadOnly currentEdge = lineOfSightStart(observer);

      if (currentEdge == null)
         return Collections.emptyList();

      List<HalfEdge3DReadOnly> lineOfSight = new ArrayList<>();

      for (int i = 0; i < getNumberOfEdges(); i++)
      {
         lineOfSight.add(currentEdge);
         currentEdge = currentEdge.getNext();

         if (!canObserverSeeEdge(observer, currentEdge))
            break;
      }

      return lineOfSight;
   }

   /**
    * Returns the first edge that is visible from the specified point.
    * <p>
    * A face edge is considered visible if its origin and destination can be connected by a straight
    * line to the specified point without crossing any other point First is defined in the counter
    * clockwise sense w.r.t to the face normal.
    * </p>
    *
    * @param observer the point in the plane of the face w.r.t. which the visible edge is to be
    *           computed
    * @return a read only reference to the first visible half edge
    */
   default HalfEdge3DReadOnly lineOfSightStart(Point3DReadOnly observer)
   {
      if (isEmpty())
         return null;
      if (getNumberOfEdges() <= 2)
         return getEdge(0);

      HalfEdge3DReadOnly startEdge = getEdge(0);
      HalfEdge3DReadOnly currentEdge = startEdge;
      boolean edgeVisible = canObserverSeeEdge(observer, currentEdge);

      if (edgeVisible)
      { // Search backward
         do
         {
            currentEdge = currentEdge.getPrevious();

            if (!canObserverSeeEdge(observer, currentEdge))
               return currentEdge.getNext();
         }
         while (currentEdge != startEdge);
      }
      else
      { // Search forward
         do
         {
            currentEdge = currentEdge.getNext();

            if (canObserverSeeEdge(observer, currentEdge))
               return currentEdge;
         }
         while (currentEdge != startEdge);
      }

      return null;
   }

   default HalfEdge3DReadOnly lineOfSightEnd(Point3DReadOnly observer)
   {
      if (isEmpty())
         return null;
      if (getNumberOfEdges() <= 2)
         return getEdge(0);

      HalfEdge3DReadOnly startEdge = getEdge(0);
      HalfEdge3DReadOnly currentEdge = startEdge;
      boolean edgeVisible = canObserverSeeEdge(observer, currentEdge);

      if (edgeVisible)
      { // Search forward
         do
         {
            currentEdge = currentEdge.getNext();

            if (!canObserverSeeEdge(observer, currentEdge))
               return currentEdge.getPrevious();
         }
         while (currentEdge != startEdge);
      }
      else
      { // Search backward
         do
         {
            currentEdge = currentEdge.getPrevious();

            if (canObserverSeeEdge(observer, currentEdge))
               return currentEdge;
         }
         while (currentEdge != startEdge);
      }

      return null;
   }

   /**
    * Checks if a point is strictly on the same side of the specified edge as the face centroid, i.e.
    * the left hand side w.r.t. to the edge
    *
    * @param point the point to be checked
    * @param index index of the edge w.r.t. which the point is to be evaluated
    * @return
    */
   default boolean canObserverSeeEdge(Point3DReadOnly observer, int index)
   {
      return canObserverSeeEdge(observer, getEdge(index));
   }

   default boolean canObserverSeeEdge(Point3DReadOnly query, HalfEdge3DReadOnly edge)
   {
      return EuclidPolytopeTools.isPoint3DOnLeftSideOfLine3D(query, edge.getOrigin(), edge.getDestination(), getNormal(), 0.0);
   }

   default boolean canObserverSeeEdge(Point3DReadOnly query, HalfEdge3DReadOnly edge, double epsilon)
   {
      if (EuclidPolytopeTools.isPoint3DOnLeftSideOfLine3D(query, edge.getOrigin(), edge.getDestination(), getNormal(), 0.0))
         return true;
      return edge.distanceFromSupportLine(query) <= epsilon;
   }

   /**
    * Check if the face is visible from the specified point
    *
    * @param point the point from which the visibility of the face is to be established
    * @param epsilon the precision of this check
    * @return {@code true} if the face is visible from the specified point, {@code false} otherwise
    */
   default boolean canObserverSeeFace(Point3DReadOnly observer)
   {
      // This covers the edge-case the observer is on the face.
      return !EuclidGeometryTools.isPoint3DBelowPlane3D(observer, getCentroid(), getNormal());
   }

   /**
    * Returns a dot product of the vector from a point on the face to the specified point and the face
    * normal A positive value indicates that the face is visible from the point specified
    *
    * @param point the point from which visibility of the face is to be tested
    * @return dot product result from the computation. Represents the distance of the face plane from
    *         the point
    */
   default double signedDistanceToPlane(Point3DReadOnly point)
   {
      return EuclidGeometryTools.signedDistanceFromPoint3DToPlane3D(point, getCentroid(), getNormal());
   }

   default double signedDistanceToEdge(Point3DReadOnly point, int edgeIndex)
   {
      return signedDistanceToEdge(point, getEdge(edgeIndex));
   }

   default double signedDistanceToEdge(Point3DReadOnly point, HalfEdge3DReadOnly edge)
   {
      return EuclidPolytopeTools.signedDistanceFromPoint3DToLine3D(point, edge.getOrigin(), edge.getDirection(false), getNormal());
   }

   /**
    * Checks if a particular point is in the same spatial plane as this face
    *
    * @param pointToCheck the point for which the check is to be performed
    * @param epsilon the error value that is acceptable (units should be distance)
    * @return {@code true} if the point is in the same plane as the face. {@code false} otherwise
    */
   default boolean isPointInFacePlane(Point3DReadOnly query, double epsilon)
   {
      if (getNumberOfEdges() < 3)
         return getEdge(0).distanceSquared(query) <= epsilon * epsilon;
      else
         return EuclidGeometryTools.distanceFromPoint3DToPlane3D(query, getCentroid(), getNormal()) < epsilon;
   }

   /**
    * Checks if a particular point is an interior point to the face. For a point to be an interior
    * point it must lie in the plane of the face and be strictly within the face edges
    *
    * @param vertexToCheck the point on which the check is to be performed
    * @param epsilon the precision level for this check operation
    * @return {@code true} if the point is an interior point. {@code false} otherwise
    */
   default boolean isPointInside(Point3DReadOnly query, double epsilon)
   {
      if (getNumberOfEdges() < 3)
         return getEdge(0).distanceSquared(query) <= epsilon * epsilon;
      else if (!getBoundingBox().isInsideEpsilon(query, epsilon))
         return false;
      else
         return isPointInFacePlane(query, epsilon) && isPointDirectlyAboveOrBelow(query);
   }

   default boolean isPointDirectlyAboveOrBelow(Point3DReadOnly query)
   {
      if (getNumberOfEdges() < 3)
         return false;

      HalfEdge3DReadOnly edge = getEdge(0);

      for (int i = 0; i < getNumberOfEdges(); i++)
      {
         if (canObserverSeeEdge(query, edge))
            return false;

         edge = edge.getNext();
      }

      return true;
   }

   /**
    * Return a reference to an adjacent face if this face is part of a polytope. If not can return a
    * {@code null} or throw a {@code NullPointerException}
    *
    * @param index the index of the adjacent face that is required. Should be less than
    *           {@code getNumberOfEdges()}
    * @return a read only reference to the adjacent face
    */
   default Face3DReadOnly getNeighbor(int index)
   {
      HalfEdge3DReadOnly twinEdge = getEdge(index).getTwin();
      if (twinEdge == null)
         return null;
      else
         return twinEdge.getFace();
   }

   default HalfEdge3DReadOnly getCommonEdgeWith(Face3DReadOnly neighbor)
   {
      for (int i = 0; i < getNumberOfEdges(); i++)
      {
         if (getNeighbor(i) == neighbor)
            return getEdge(i);
      }
      return null;
   }

   /**
    * Returns the edge closed to the point specified
    *
    * @param point the point to which the closed edge is required
    * @return read only reference to the half edge that is closed to the specified point
    */
   default HalfEdge3DReadOnly getClosestEdge(Point3DReadOnly point)
   {
      HalfEdge3DReadOnly startEdge = getEdge(0);
      HalfEdge3DReadOnly closestEdge = startEdge;
      double minDistanceSquared = startEdge.distanceSquared(point);
      HalfEdge3DReadOnly currentEdge = startEdge.getNext();

      while (currentEdge != startEdge)
      {
         double distanceSquared = currentEdge.distanceSquared(point);
         if (distanceSquared < minDistanceSquared)
         {
            closestEdge = currentEdge;
            minDistanceSquared = distanceSquared;
         }
         currentEdge = currentEdge.getNext();
      }

      return closestEdge;
   }

   default HalfEdge3DReadOnly getClosestVisibleEdge(Point3DReadOnly point)
   {
      if (isEmpty())
         return null;
      if (getNumberOfEdges() < 3)
         return getEdge(0);

      HalfEdge3DReadOnly closestVisibleEdge = null;
      double distanceSquaredToClosestVisibleEdge = Double.POSITIVE_INFINITY;

      HalfEdge3DReadOnly startEdge = getEdge(0);
      HalfEdge3DReadOnly currentEdge = startEdge;
      boolean edgeVisible = canObserverSeeEdge(point, currentEdge);

      if (edgeVisible)
      { // Search backward for the line-of-sight start while recording distance to closest edge.
         distanceSquaredToClosestVisibleEdge = startEdge.distanceSquared(point);
         closestVisibleEdge = startEdge;

         do
         {
            currentEdge = currentEdge.getPrevious();
            if (!canObserverSeeEdge(point, currentEdge))
               break;
            double distanceSquared = currentEdge.distanceSquared(point);
            if (distanceSquared < distanceSquaredToClosestVisibleEdge)
            {
               closestVisibleEdge = currentEdge;
               distanceSquaredToClosestVisibleEdge = distanceSquared;
            }
         }
         while (currentEdge != startEdge);

         // Go through the rest of the line-of-sight.
         currentEdge = startEdge;

         do
         {
            currentEdge = currentEdge.getNext();
            if (!canObserverSeeEdge(point, currentEdge))
               break;
            double distanceSquared = currentEdge.distanceSquared(point);
            if (distanceSquared < distanceSquaredToClosestVisibleEdge)
            {
               closestVisibleEdge = currentEdge;
               distanceSquaredToClosestVisibleEdge = distanceSquared;
            }
         }
         while (currentEdge != startEdge);
      }
      else
      { // Search forward
         do
         {
            currentEdge = currentEdge.getNext();

            if (canObserverSeeEdge(point, currentEdge))
            {
               double distanceSquared = currentEdge.distanceSquared(point);

               if (distanceSquared < distanceSquaredToClosestVisibleEdge)
               {
                  closestVisibleEdge = currentEdge;
                  distanceSquaredToClosestVisibleEdge = distanceSquared;
               }
            }
            else if (closestVisibleEdge != null)
            { // Early break as we know we went through the entire line-of-sight so no other edge can be closer.
               break;
            }
         }
         while (currentEdge != startEdge);
      }

      return closestVisibleEdge;
   }

   /**
    * Returns the shortest distance to the point specified
    *
    * @param point the point to which the distance is required
    * @return the shortest length from the specified point to the face
    */
   @Override
   default double distance(Point3DReadOnly point)
   {
      HalfEdge3DReadOnly closestVisibleEdge = getClosestVisibleEdge(point);

      if (closestVisibleEdge == null)
         return distanceToPlane(point);
      else
         return closestVisibleEdge.distance(point);
   }

   default double distanceToPlane(Point3DReadOnly point)
   {
      return EuclidGeometryTools.distanceFromPoint3DToPlane3D(point, getCentroid(), getNormal());
   }

   default Point3DBasics orthogonalProjectionCopy(Point3DReadOnly pointToProject)
   {
      Point3D projection = new Point3D();
      if (orthogonalProjection(pointToProject, projection))
         return projection;
      else
         return null;
   }

   default boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      HalfEdge3DReadOnly closestVisibleEdge = getClosestVisibleEdge(pointToProject);

      if (closestVisibleEdge == null)
         return EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, getCentroid(), getNormal(), projectionToPack);
      else
         return closestVisibleEdge.orthogonalProjection(pointToProject, projectionToPack);
   }

   @Override
   default Vertex3DReadOnly getSupportingVertex(Vector3DReadOnly supportDirection)
   {
      return getSupportingVertex(supportDirection.getX(), supportDirection.getY(), supportDirection.getZ());
   }

   default Vertex3DReadOnly getSupportingVertex(double supportDirectionX, double supportDirectionY, double supportDirectionZ)
   {
      if (isEmpty())
         return null;

      HalfEdge3DReadOnly startEdge = getEdge(0);
      Vertex3DReadOnly supportingVertex = startEdge.getOrigin();
      double maxDot = TupleTools.dot(supportDirectionX, supportDirectionY, supportDirectionZ, supportingVertex);
      HalfEdge3DReadOnly currentEdge = startEdge.getNext();

      while (currentEdge != startEdge)
      {
         Vertex3DReadOnly candidate = currentEdge.getOrigin();
         double currentDot = TupleTools.dot(supportDirectionX, supportDirectionY, supportDirectionZ, candidate);

         if (currentDot > maxDot)
         {
            maxDot = currentDot;
            supportingVertex = candidate;
         }

         currentEdge = currentEdge.getNext();
      }

      return supportingVertex;
   }

   @Override
   default boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      if (isEmpty())
         return false;
      supportingVertexToPack.set(getSupportingVertex(supportDirection));
      return true;
   }

   @Override
   default Simplex3D getSmallestSimplexMemberReference(Point3DReadOnly point)
   {
      HalfEdge3DReadOnly closestVisibleEdge = getClosestVisibleEdge(point);

      if (closestVisibleEdge == null)
         return this;
      else
         return closestVisibleEdge.getSmallestSimplexMemberReference(point);
   }

   @Override
   default boolean getSupportVectorDirectionTo(Point3DReadOnly point, Vector3DBasics supportVectorToPack)
   {
      if (isEmpty())
         return false;

      HalfEdge3DReadOnly closestVisibleEdge = getClosestVisibleEdge(point);

      if (closestVisibleEdge == null)
      {
         supportVectorToPack.set(getNormal());
         if (!EuclidGeometryTools.isPoint3DAbovePlane3D(point, getCentroid(), getNormal()))
            supportVectorToPack.negate();
         return true;
      }
      else
      {
         return closestVisibleEdge.getSupportVectorDirectionTo(point, supportVectorToPack);
      }
   }

   default boolean equals(Face3DReadOnly other)
   {
      if (other == null)
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

   default boolean epsilonEquals(Face3DReadOnly other, double epsilon)
   {
      if (getNumberOfEdges() != other.getNumberOfEdges())
         return false;

      for (int edgeIndex = 0; edgeIndex < getNumberOfEdges(); edgeIndex++)
      {
         if (!getEdge(edgeIndex).epsilonEquals(other.getEdge(edgeIndex), epsilon))
            return false;
      }
      return true;
   }

   /**
    * Geometrically evaluates if the face is within an epsilon vicinity of the specified face. This
    * check is actually performed on the vertices that the face is composed of. Both spatial and and
    * relations between vertices are checked
    *
    * @return {@code true} is the face is geometrically similar to the specified face, {@code false}
    *         otherwise
    */
   default boolean geometricallyEquals(Face3DReadOnly other, double epsilon)
   {
      if (getNumberOfEdges() != other.getNumberOfEdges())
         return false;

      HalfEdge3DReadOnly startEdge = null;
      HalfEdge3DReadOnly otherCurrentEdge = other.getEdge(0);
      HalfEdge3DReadOnly thisCurrentEdge = null;

      for (int edgeIndex = 0; edgeIndex < getNumberOfEdges(); edgeIndex++)
      {
         thisCurrentEdge = getEdge(edgeIndex);

         if (!thisCurrentEdge.geometricallyEquals(otherCurrentEdge, epsilon))
         {
            startEdge = thisCurrentEdge;
            break;
         }
      }

      if (startEdge == null)
         return false;

      do
      {
         otherCurrentEdge.getNext();
         thisCurrentEdge.getNext();
         if (!thisCurrentEdge.geometricallyEquals(otherCurrentEdge, epsilon))
            return false;
      }
      while (thisCurrentEdge != startEdge);

      return true;
   }
}