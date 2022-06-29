package us.ihmc.euclid.shape.convexPolytope.interfaces;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTools;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read-only interface for a face 3D that belongs to a convex polytope 3D.
 * <p>
 * This is part of a Doubly Connected Edge List data structure
 * <a href="https://en.wikipedia.org/wiki/Doubly_connected_edge_list"> link</a>.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface Face3DReadOnly extends SupportingVertexHolder
{
   /**
    * Gets the read-only reference to the centroid of the face.
    *
    * @return this face centroid location.
    */
   Point3DReadOnly getCentroid();

   /**
    * Gets a vector normal to the face.
    * <p>
    * A face's normal points away from the centroid of the convex polytope it belongs.
    * </p>
    *
    * @return this face normal vector.
    */
   Vector3DReadOnly getNormal();

   /**
    * Gets this face area.
    *
    * @return this face area.
    */
   double getArea();

   /**
    * Gets the tightest axis-aligned bounding box that contains this face.
    *
    * @return the bounding box.
    */
   BoundingBox3DReadOnly getBoundingBox();

   /**
    * Gets this face's half-edges.
    * <p>
    * The half-edges are clockwise ordered.
    * </p>
    *
    * @return this face's half-edges.
    */
   List<? extends HalfEdge3DReadOnly> getEdges();

   /**
    * Gets the read-only reference to the i<sup>th</sup> half-edge of this face.
    *
    * @param index the edge index &in; [0; {@link #getNumberOfEdges()}[.
    * @return the read-only reference to the edge.
    */
   default HalfEdge3DReadOnly getEdge(int index)
   {
      return getEdges().get(index);
   }

   /**
    * Gets this face's vertices.
    * <p>
    * WARNING: The default implementation of this method generates garbage.
    * </p>
    * <p>
    * The vertices are clockwise ordered.
    * </p>
    *
    * @return this face's vertices.
    */
   default List<? extends Vertex3DReadOnly> getVertices()
   {
      return getEdges().stream().map(HalfEdge3DReadOnly::getOrigin).collect(Collectors.toList());
   }

   /**
    * Gets the read-only reference to the i<sup>th</sup> vertex of this face.
    *
    * @param index the vertex index &in; [0; {@link #getNumberOfEdges()}[.
    * @return the read-only reference to the vertex.
    */
   default Vertex3DReadOnly getVertex(int index)
   {
      return getEdge(index).getOrigin();
   }

   /**
    * Gets the number of edges for this face.
    *
    * @return this face number of edges.
    */
   default int getNumberOfEdges()
   {
      return getEdges().size();
   }

   /**
    * Tests whether this face has at least one edge or not.
    *
    * @return {@code true} if this face has no edge, {@code false} otherwise.
    */
   default boolean isEmpty()
   {
      return getEdges().isEmpty();
   }

   /**
    * Tests whether this face contains at least one {@link Double#NaN} or not.
    *
    * @return {@code true} if this face contains {@link Double#NaN}, {@code false} otherwise.
    */
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

   /**
    * From the point of view of an observer located outside the face, only a continuous subset of the
    * face's edges can be seen defining a line-of-sight. This method finds the edges in order that are
    * in the line-of-sight.
    * <p>
    * WARNING: The default implementation of this method generates garbage.
    * </p>
    * <p>
    * In case the observer located directly above, below, or on the face, this method returns
    * {@code null}.
    * </p>
    *
    * @param observer the coordinates of the observer. Not modified.
    * @return the edges in order that are in the line-of-sight.
    */
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
    * Collects and returns the edges that are in the line-of-sight of the given observer in a similar
    * way as {@link #lineOfSight(Point3DReadOnly)}.
    * <p>
    * WARNING: The default implementation of this method generates garbage.
    * </p>
    * <p>
    * The line-of-sight is possibly extended to include the edge right before and right after if the
    * observer is within {@code epsilon} of the their support lines.
    * </p>
    * <p>
    * In case the observer located directly above, below, or on the face, this method returns
    * {@code null}.
    * </p>
    *
    * @param observer the coordinates of the observer. Not modified.
    * @param epsilon  tolerance to determine whether to extend the line-of-sight.
    * @return the edges in order that are in the line-of-sight.
    */
   default List<? extends HalfEdge3DReadOnly> lineOfSight(Point3DReadOnly observer, double epsilon)
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

      HalfEdge3DReadOnly firstVisibleEdge = lineOfSight.get(0);
      HalfEdge3DReadOnly lastVisibleEdge = lineOfSight.get(lineOfSight.size() - 1);

      HalfEdge3DReadOnly edgeBeforeLineOfSight = firstVisibleEdge.getPrevious();
      HalfEdge3DReadOnly edgeAfterLineOfSight = lastVisibleEdge.getNext();

      if (edgeBeforeLineOfSight.distanceFromSupportLine(observer) < epsilon)
      {
         firstVisibleEdge = edgeBeforeLineOfSight;
         lineOfSight.add(0, firstVisibleEdge);
      }
      else if (EuclidGeometryTools.distanceFromPoint3DToLine3D(edgeBeforeLineOfSight.getDestination(), observer, edgeBeforeLineOfSight.getOrigin()) < epsilon)
      { // Sometimes edgeBeforeLineOfSight is really small, in which case the previous test may not pass.
         firstVisibleEdge = edgeBeforeLineOfSight;
         lineOfSight.add(0, firstVisibleEdge);
      }

      if (edgeAfterLineOfSight.distanceFromSupportLine(observer) < epsilon)
      {
         lastVisibleEdge = edgeAfterLineOfSight;
         lineOfSight.add(lastVisibleEdge);
      }
      else if (EuclidGeometryTools.distanceFromPoint3DToLine3D(edgeAfterLineOfSight.getOrigin(), observer, edgeAfterLineOfSight.getDestination()) < epsilon)
      { // Sometimes edgeAfterLineOfSight is really small, in which case the previous test may not pass.
         lastVisibleEdge = edgeAfterLineOfSight;
         lineOfSight.add(lastVisibleEdge);
      }

      return lineOfSight;
   }

   /**
    * Finds and returns the first edge that is visible from the observer.
    * <p>
    * In case the observer located directly above, below, or on the face, this method returns
    * {@code null}.
    * </p>
    *
    * @param observer the coordinates of the observer. Not modified.
    * @return the first visible half edge.
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

   /**
    * Finds and returns the last edge that is visible from the observer.
    * <p>
    * In case the observer located directly above, below, or on the face, this method returns
    * {@code null}.
    * </p>
    *
    * @param observer the coordinates of the observer. Not modified.
    * @return the last visible half edge.
    */
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
    * Tests whether the i<sup>th</sup> edge of this face is visible from the observer.
    *
    * @param observer the coordinates of the observer. Not modified.
    * @param index    the index &in; [0; {@link #getNumberOfEdges()}[ of the edge to test.
    * @return {@code true} if the observer can see the edge, {@code false} otherwise.
    */
   default boolean canObserverSeeEdge(Point3DReadOnly observer, int index)
   {
      return canObserverSeeEdge(observer, getEdge(index));
   }

   /**
    * Tests whether the given edge is visible from the observer.
    * <p>
    * It is assumed that the given edge belongs to this face.
    * </p>
    *
    * @param observer the coordinates of the observer. Not modified.
    * @param edge     the edge to test. Not modified.
    * @return {@code true} if the observer can see the edge, {@code false} otherwise.
    */
   default boolean canObserverSeeEdge(Point3DReadOnly observer, HalfEdge3DReadOnly edge)
   {
      return EuclidPolytopeTools.isPoint3DOnLeftSideOfLine3D(observer, edge.getOrigin(), edge.getDestination(), getNormal());
   }

   /**
    * Tests whether this face is visible from the observer.
    * <p>
    * The face is visible from an observer if the face's normal is pointing towards it.
    * </p>
    *
    * @param observer the coordinates of the observer. Not modified.
    * @return {@code true} if the observer can see this face, {@code false} otherwise.
    */
   default boolean canObserverSeeFace(Point3DReadOnly observer)
   {
      return EuclidGeometryTools.isPoint3DAbovePlane3D(observer, getCentroid(), getNormal());
   }

   /**
    * Tests whether this face is visible from the observer.
    * <p>
    * The face is visible from an observer if the face's normal is pointing towards it and that the
    * observer is located at a distance greater than {@code epsilon} from this face support plane.
    * </p>
    *
    * @param observer the coordinates of the observer. Not modified.
    * @param epsilon  the minimum distance between the observer and this face support plane for the
    *                 face to be visible.
    * @return {@code true} if the observer can see this face, {@code false} otherwise.
    */
   default boolean canObserverSeeFace(Point3DReadOnly observer, double epsilon)
   {
      return signedDistanceFromSupportPlane(observer) > epsilon;
   }

   /**
    * Tests whether the query is within {@code epsilon} of this face's support plane.
    *
    * @param query   the coordinates of the query. Not modified.
    * @param epsilon the tolerance to use for this test.
    * @return {@code true} if the query is considered to be on this face's support plane, {@code false}
    *         otherwise.
    */
   default boolean isPointInFaceSupportPlane(Point3DReadOnly query, double epsilon)
   {
      if (getNumberOfEdges() < 3)
         return getEdge(0).distanceSquared(query) <= epsilon * epsilon;
      else
         return EuclidGeometryTools.distanceFromPoint3DToPlane3D(query, getCentroid(), getNormal()) < epsilon;
   }

   /**
    * Tests whether the query is located inside this face.
    *
    * @param query   the coordinates of the query. Not modified.
    * @param epsilon the tolerance to use for this test.
    * @return {@code true} if the point is an interior point, {@code false} otherwise.
    */
   default boolean isPointInside(Point3DReadOnly query, double epsilon)
   {
      if (getNumberOfEdges() < 3)
         return getEdge(0).distanceSquared(query) <= epsilon * epsilon;
      else if (!getBoundingBox().isInsideEpsilon(query, epsilon))
         return false;
      else
         return isPointInFaceSupportPlane(query, epsilon) && isPointDirectlyAboveOrBelow(query);
   }

   /**
    * Tests whether the query is located directly above or below this face, such its projection would
    * be located inside this face.
    *
    * @param query the coordinates of the query. Not modified.
    * @return {@code true} if the query is located either directly above or below this face,
    *         {@code false} otherwise.
    */
   default boolean isPointDirectlyAboveOrBelow(Point3DReadOnly query)
   {
      if (getNumberOfEdges() < 3)
         return false;

      HalfEdge3DReadOnly startEdge = getEdge(0);
      HalfEdge3DReadOnly edge = startEdge;

      do
      {
         if (canObserverSeeEdge(query, edge))
            return false;

         edge = edge.getNext();
      }
      while (edge != startEdge);

      return true;
   }

   /**
    * Gets the i<sup>th</sup> neighbor to this face.
    *
    * @param index the neighbor index &in; [0; {@link #getNumberOfEdges()}[.
    * @return the neighbor face.
    */
   default Face3DReadOnly getNeighbor(int index)
   {
      HalfEdge3DReadOnly twinEdge = getEdge(index).getTwin();
      if (twinEdge == null)
         return null;
      else
         return twinEdge.getFace();
   }

   /**
    * Finds, if it exists, the common edge between {@code this} and the given {@code neighbor}.
    * <p>
    * It is assumed that {@code neighbor} is part of the same convex polytope as this face.
    * </p>
    *
    * @param neighbor the face to find to common edge to. Not modified.
    * @return the edge common to both faces.
    */
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
    * Finds and returns the closest edge to the query.
    *
    * @param query the coordinates of the query. Not modified.
    * @return the closest edge to the query.
    */
   default HalfEdge3DReadOnly getClosestEdge(Point3DReadOnly query)
   {
      HalfEdge3DReadOnly startEdge = getEdge(0);
      HalfEdge3DReadOnly closestEdge = startEdge;
      double minDistanceSquared = startEdge.distanceSquared(query);
      HalfEdge3DReadOnly currentEdge = startEdge.getNext();

      while (currentEdge != startEdge)
      {
         double distanceSquared = currentEdge.distanceSquared(query);
         if (distanceSquared < minDistanceSquared)
         {
            closestEdge = currentEdge;
            minDistanceSquared = distanceSquared;
         }
         currentEdge = currentEdge.getNext();
      }

      return closestEdge;
   }

   /**
    * Finds and returns the closest visible edge to the query.
    * <p>
    * If the query is located directly above, below, or on the face, this method returns {@code null}.
    * </p>
    *
    * @param query the coordinates of the query. Not modified.
    * @return the closest visible edge to the query, or {@code null} if there is no visible edge from
    *         the query.
    */
   default HalfEdge3DReadOnly getClosestVisibleEdge(Point3DReadOnly query)
   {
      if (isEmpty())
         return null;
      if (getNumberOfEdges() < 3)
         return getEdge(0);

      HalfEdge3DReadOnly closestVisibleEdge = null;
      double distanceSquaredToClosestVisibleEdge = Double.POSITIVE_INFINITY;

      HalfEdge3DReadOnly startEdge = getEdge(0);
      HalfEdge3DReadOnly currentEdge = startEdge;
      boolean edgeVisible = canObserverSeeEdge(query, currentEdge);

      if (edgeVisible)
      { // Search backward for the line-of-sight start while recording distance to closest edge.
         distanceSquaredToClosestVisibleEdge = startEdge.distanceSquared(query);
         closestVisibleEdge = startEdge;

         do
         {
            currentEdge = currentEdge.getPrevious();
            if (!canObserverSeeEdge(query, currentEdge))
               break;
            double distanceSquared = currentEdge.distanceSquared(query);
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
            if (!canObserverSeeEdge(query, currentEdge))
               break;
            double distanceSquared = currentEdge.distanceSquared(query);
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

            if (canObserverSeeEdge(query, currentEdge))
            {
               double distanceSquared = currentEdge.distanceSquared(query);

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
    * Computes the minimum distance between a given point and the infinitely large plane supporting
    * this face.
    * <p>
    * The returned value is signed as follows: positive if the query is located above the face's
    * support plane, negative if it is located below.
    * </p>
    *
    * @param query the coordinates of the query. Not modified.
    * @return the signed minimum distance between the point and this face's support plane.
    */
   default double signedDistanceFromSupportPlane(Point3DReadOnly query)
   {
      return EuclidGeometryTools.signedDistanceFromPoint3DToPlane3D(query, getCentroid(), getNormal());
   }

   /**
    * Returns the minimum distance between this face and the given point.
    *
    * @param query the coordinates of the query. Not modified.
    * @return the minimum distance between the point and this face.
    */
   default double distance(Point3DReadOnly query)
   {
      HalfEdge3DReadOnly closestVisibleEdge = getClosestVisibleEdge(query);

      if (closestVisibleEdge == null)
         return distanceFromSupportPlane(query);
      else
         return closestVisibleEdge.distance(query);
   }

   /**
    * Computes the minimum distance between a given point and the infinitely large plane supporting
    * this face.
    *
    * @param query the coordinates of the query. Not modified.
    * @return the minimum distance between the point and this face's support plane.
    */
   default double distanceFromSupportPlane(Point3DReadOnly query)
   {
      return EuclidGeometryTools.distanceFromPoint3DToPlane3D(query, getCentroid(), getNormal());
   }

   /**
    * Computes the orthogonal projection of a point on this face.
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @return the projection of the point onto this face or {@code null} if the method failed.
    */
   default Point3DBasics orthogonalProjectionCopy(Point3DReadOnly pointToProject)
   {
      Point3D projection = new Point3D();
      if (orthogonalProjection(pointToProject, projection))
         return projection;
      else
         return null;
   }

   /**
    * Computes the orthogonal projection of a point on this face.
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto this face is stored.
    *                         Modified.
    * @return whether the method succeeded or not.
    */
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      HalfEdge3DReadOnly closestVisibleEdge = getClosestVisibleEdge(pointToProject);

      if (closestVisibleEdge == null)
         return EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, getCentroid(), getNormal(), projectionToPack);
      else
         return closestVisibleEdge.orthogonalProjection(pointToProject, projectionToPack);
   }

   /** {@inheritDoc} */
   @Override
   default Vertex3DReadOnly getSupportingVertex(Vector3DReadOnly supportDirection)
   {
      if (isEmpty())
         return null;

      HalfEdge3DReadOnly startEdge = getEdge(0);
      Vertex3DReadOnly supportingVertex = startEdge.getOrigin();
      double maxDot = supportingVertex.dot(supportDirection);
      HalfEdge3DReadOnly currentEdge = startEdge.getNext();

      while (currentEdge != startEdge)
      {
         Vertex3DReadOnly candidate = currentEdge.getOrigin();
         double currentDot = candidate.dot(supportDirection);

         if (currentDot > maxDot)
         {
            maxDot = currentDot;
            supportingVertex = candidate;
         }

         currentEdge = currentEdge.getNext();
      }

      return supportingVertex;
   }

   /** {@inheritDoc} */
   @Override
   default boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      if (isEmpty())
         return false;
      supportingVertexToPack.set(getSupportingVertex(supportDirection));
      return true;
   }

   /** {@inheritDoc} */
   @Override
   default boolean epsilonEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof Face3DReadOnly))
         return false;
      Face3DReadOnly other = (Face3DReadOnly) geometry;
      if (getNumberOfEdges() != other.getNumberOfEdges())
         return false;

      for (int edgeIndex = 0; edgeIndex < getNumberOfEdges(); edgeIndex++)
      {
         if (!getEdge(edgeIndex).epsilonEquals(other.getEdge(edgeIndex), epsilon))
            return false;
      }
      return true;
   }

   /** {@inheritDoc} */
   @Override
   default boolean geometricallyEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof Face3DReadOnly))
         return false;
      Face3DReadOnly other = (Face3DReadOnly) geometry;
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

   /** {@inheritDoc} */
   @Override
   default boolean equals(EuclidGeometry geometry)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof Face3DReadOnly))
         return false;

      Face3DReadOnly other = (Face3DReadOnly) geometry;

      if (getNumberOfEdges() != other.getNumberOfEdges())
         return false;

      for (int edgeIndex = 0; edgeIndex < getNumberOfEdges(); edgeIndex++)
      {
         if (!getEdge(edgeIndex).equals(other.getEdge(edgeIndex)))
            return false;
      }
      return true;
   }

   /**
    * Gets the representative {@code String} of this face 3D given a specific format to use.
    * <p>
    * Using the default format {@link EuclidCoreIOTools#DEFAULT_FORMAT}, this provides a {@code String}
    * as follows:
    *
    * <pre>
    * Face 3D: centroid: ( 2.621, -0.723, -1.355 ), normal: ( 0.903, -0.202,  0.378 ), area:  0.180, number of edges: 4
    *    [( 2.590, -0.496, -1.161 ); ( 2.746, -0.536, -1.554 )]
    *    [( 2.746, -0.536, -1.554 ); ( 2.651, -0.950, -1.549 )]
    *    [( 2.651, -0.950, -1.549 ); ( 2.496, -0.910, -1.157 )]
    *    [( 2.496, -0.910, -1.157 ); ( 2.590, -0.496, -1.161 )]
    * </pre>
    * </p>
    */
   @Override
   default String toString(String format)
   {
      return EuclidShapeIOTools.getFace3DString(format, this);
   }
}