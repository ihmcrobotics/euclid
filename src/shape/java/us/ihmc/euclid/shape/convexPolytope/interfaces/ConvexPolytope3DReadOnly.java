package us.ihmc.euclid.shape.convexPolytope.interfaces;

import java.util.ArrayDeque;
import java.util.Iterator;
import java.util.List;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read-only interface for a convex polytope 3D.
 * <p>
 * This is part of a Doubly Connected Edge List data structure
 * <a href="https://en.wikipedia.org/wiki/Doubly_connected_edge_list"> link</a>.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public interface ConvexPolytope3DReadOnly extends Shape3DReadOnly
{
   /**
    * Gets the number of vertices that compose this polytope.
    *
    * @return the number of vertices in this polytope.
    */
   default int getNumberOfVertices()
   {
      return getVertices().size();
   }

   /**
    * Gets the number of edges that compose this polytope.
    * <p>
    * Note: the number of edges is half the number of half-edges.
    * </p>
    *
    * @return the number of edges in this polytope.
    */
   default int getNumberOfEdges()
   {
      if (getNumberOfFaces() < 2)
         return getNumberOfHalfEdges();
      else
         return getNumberOfHalfEdges() / 2;
   }

   /**
    * Gets the number of half-edges that compose this polytope.
    * <p>
    * Note: the number of half-edges is twice the number of edges.
    * </p>
    * 
    * @return the number of half-edges in this polytope.
    */
   default int getNumberOfHalfEdges()
   {
      return getHalfEdges().size();
   }

   /**
    * Gets the number of faces that compose this polytope.
    *
    * @return the number of faces in this polytope.
    */
   default int getNumberOfFaces()
   {
      return getFaces().size();
   }

   /**
    * Gets this polytope's faces.
    *
    * @return this polytope's faces.
    */
   List<? extends Face3DReadOnly> getFaces();

   /**
    * Gets the i<sup>th</sup> face of this polytope.
    *
    * @param index the face index &in; [0; {@link #getNumberOfFaces()}[.
    * @return the read-only reference to the face.
    */
   default Face3DReadOnly getFace(int index)
   {
      return getFaces().get(index);
   }

   /**
    * Gets this polytope's half-edges.
    * <p>
    * Note that the number of half-edges is twice the number of edges in this polytope.
    * </p>
    *
    * @return this polytope's half-edges.
    */
   List<? extends HalfEdge3DReadOnly> getHalfEdges();

   /**
    * Gets the i<sup>th</sup> half-edge of this polytope.
    *
    * @param index the half-edge index &in; [0; {@link #getNumberOfHalfEdges()}[.
    * @return the read-only reference to the half-edge.
    */
   default HalfEdge3DReadOnly getHalfEdge(int index)
   {
      return getHalfEdges().get(index);
   }

   /**
    * Gets this polytope's vertices.
    *
    * @return this polytope's vertices.
    */
   List<? extends Vertex3DReadOnly> getVertices();

   /**
    * Gets the i<sup>th</sup> vertex of this polytope.
    *
    * @param index the vertex index &in; [0; {@link #getNumberOfVertices()}[.
    * @return the read-only reference to the vertex.
    */
   default Vertex3DReadOnly getVertex(int index)
   {
      return getVertices().get(index);
   }

   /** {@inheritDoc} */
   @Override
   BoundingBox3DReadOnly getBoundingBox();

   /** {@inheritDoc} */
   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      boundingBoxToPack.set(getBoundingBox());
   }

   /**
    * Gets the read-only reference to the centroid of this polytope.
    *
    * @return this polytope centroid location.
    */
   Point3DReadOnly getCentroid();

   /**
    * Gets this polytope volume.
    * 
    * @return this polytope volume.
    */
   double getVolume();

   /**
    * Gets the tolerance used for building this convex polytope.
    *
    * @return the construction tolerance.
    */
   double getConstructionEpsilon();

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      for (int vertexIndex = 0; vertexIndex < getNumberOfVertices(); vertexIndex++)
      {
         if (getVertex(vertexIndex).containsNaN())
            return true;
      }
      return false;
   }

   /** {@inheritDoc} */
   @Override
   default boolean isPointInside(Point3DReadOnly pointToCheck)
   {
      if (isEmpty())
         return false;

      if (!getBoundingBox().isInsideInclusive(pointToCheck))
         return false;

      if (getNumberOfFaces() <= 2)
         return getFace(0).isPointInside(pointToCheck, 0.0);

      for (int faceIndex = 0; faceIndex < getNumberOfFaces(); faceIndex++)
      {
         Face3DReadOnly face = getFace(faceIndex);

         if (face.canObserverSeeFace(pointToCheck))
            return false;
      }
      return true;
   }

   /** {@inheritDoc} */
   @Override
   default boolean isPointInside(Point3DReadOnly pointToCheck, double epsilon)
   {
      if (isEmpty())
         return false;

      if (!getBoundingBox().isInsideEpsilon(pointToCheck, epsilon))
         return false;

      if (getNumberOfFaces() <= 2)
         return getFace(0).isPointInside(pointToCheck, epsilon);

      for (int faceIndex = 0; faceIndex < getNumberOfFaces(); faceIndex++)
      {
         if (getFace(faceIndex).canObserverSeeFace(pointToCheck, epsilon))
            return false;
      }
      return true;
   }

   /** {@inheritDoc} */
   @Override
   default double signedDistance(Point3DReadOnly point)
   {
      if (getNumberOfFaces() == 0)
         return Double.NaN;
      if (getNumberOfFaces() == 1)
         return getFace(0).distance(point);

      boolean isOutside = false;
      double maxNegativeDistance = Double.NEGATIVE_INFINITY;
      int faceIndex;

      // First assume the query is inside and use the less expensive Face3DReadOnly.signedDistanceToPlane(Point3DReadOnly)
      for (faceIndex = 0; faceIndex < getNumberOfFaces(); faceIndex++)
      {
         Face3DReadOnly face = getFace(faceIndex);
         double signedDistanceToPlane = face.signedDistanceFromSupportPlane(point);

         if (signedDistanceToPlane < 0.0)
         {
            maxNegativeDistance = Math.max(maxNegativeDistance, signedDistanceToPlane);
         }
         else
         {
            isOutside = true;
            break;
         }
      }

      if (!isOutside)
      {
         return maxNegativeDistance;
      }
      else
      { // The query is outside.
         double closestFaceDistance = getFace(faceIndex).distance(point);

         faceIndex++;

         for (; faceIndex < getNumberOfFaces(); faceIndex++)
         {
            Face3DReadOnly face = getFace(faceIndex);
            if (!face.canObserverSeeFace(point))
               continue; // The query is below, cannot be the closest face.

            // Now use the more expensive Face3DReadOnly.distance(Point3DReadOnly)
            double candidateDistance = face.distance(point);
            closestFaceDistance = Math.min(closestFaceDistance, candidateDistance);
         }

         return closestFaceDistance;
      }
   }

   /** {@inheritDoc} */
   @Override
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      if (isEmpty() || isPointInside(pointToProject))
         return false;
      else
         return getClosestFace(pointToProject).orthogonalProjection(pointToProject, projectionToPack);
   }

   /** {@inheritDoc} */
   @Override
   default boolean evaluatePoint3DCollision(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack)
   {
      if (isEmpty())
         return false;

      if (getNumberOfFaces() == 1)
      {
         Face3DReadOnly face = getFace(0);

         if (face.getNumberOfEdges() == 1)
         {
            closestPointOnSurfaceToPack.set(face.getVertex(0));
            normalAtClosestPointToPack.sub(pointToCheck, closestPointOnSurfaceToPack);
            normalAtClosestPointToPack.normalize();
            return false;
         }
         else if (face.getNumberOfEdges() == 2)
         {
            face.getEdge(0).orthogonalProjection(pointToCheck, closestPointOnSurfaceToPack);
            normalAtClosestPointToPack.sub(pointToCheck, closestPointOnSurfaceToPack);
            normalAtClosestPointToPack.normalize();
            return false;
         }
         else
         {
            face.orthogonalProjection(pointToCheck, closestPointOnSurfaceToPack);

            if (face.isPointDirectlyAboveOrBelow(pointToCheck))
            {
               normalAtClosestPointToPack.set(face.getNormal());
            }
            else
            {
               normalAtClosestPointToPack.sub(pointToCheck, closestPointOnSurfaceToPack);
               normalAtClosestPointToPack.normalize();
            }
            return false;
         }
      }
      else
      {
         Face3DReadOnly closestFace = getClosestFace(pointToCheck);
         closestFace.orthogonalProjection(pointToCheck, closestPointOnSurfaceToPack);

         if (closestFace.isPointDirectlyAboveOrBelow(pointToCheck))
         {
            normalAtClosestPointToPack.set(closestFace.getNormal());
         }
         else
         {
            normalAtClosestPointToPack.sub(pointToCheck, closestPointOnSurfaceToPack);
            normalAtClosestPointToPack.normalize();
         }

         return !EuclidGeometryTools.isPoint3DAbovePlane3D(pointToCheck, closestFace.getCentroid(), closestFace.getNormal());
      }
   }

   /**
    * Finds and returns the closest face to the query.
    *
    * @param query the coordinates of the query. Not modified.
    * @return the closest face to the query.
    */
   default Face3DReadOnly getClosestFace(Point3DReadOnly query)
   {
      if (getNumberOfFaces() == 0)
         return null;
      if (getNumberOfFaces() == 1)
         return getFace(0);

      boolean isOutside = false;
      double maxNegativeDistance = Double.NEGATIVE_INFINITY;
      Face3DReadOnly closestFace = null;
      int faceIndex;

      // First assume the query is inside and use the less expensive Face3DReadOnly.signedDistanceToPlane(Point3DReadOnly)
      for (faceIndex = 0; faceIndex < getNumberOfFaces(); faceIndex++)
      {
         Face3DReadOnly face = getFace(faceIndex);
         double signedDistanceToPlane = face.signedDistanceFromSupportPlane(query);

         if (signedDistanceToPlane < 0.0)
         {
            if (signedDistanceToPlane > maxNegativeDistance)
            {
               closestFace = face;
               maxNegativeDistance = signedDistanceToPlane;
            }
         }
         else
         {
            isOutside = true;
            break;
         }
      }

      if (isOutside)
      { // The query is outside.
         closestFace = getFace(faceIndex);
         double closestFaceDistance = closestFace.distance(query);

         faceIndex++;

         for (; faceIndex < getNumberOfFaces(); faceIndex++)
         {
            Face3DReadOnly face = getFace(faceIndex);
            if (!face.canObserverSeeFace(query))
               continue; // The query is below, cannot be the closest face.

            // Now use the more expensive Face3DReadOnly.distance(Point3DReadOnly)
            double candidateDistance = face.distance(query);

            if (candidateDistance < closestFaceDistance)
            {
               closestFace = face;
               closestFaceDistance = candidateDistance;
            }
         }
      }

      return closestFace;
   }

   /** {@inheritDoc} */
   @Override
   default Vertex3DReadOnly getSupportingVertex(Vector3DReadOnly supportDirection)
   {
      if (isEmpty())
         return null;
      if (getNumberOfFaces() == 1)
         return getFace(0).getSupportingVertex(supportDirection);

      Vertex3DReadOnly bestVertex = getFace(0).getEdge(0).getOrigin();

      if (getNumberOfVertices() == 1)
         return bestVertex;

      double maxDotProduct = bestVertex.dot(supportDirection);
      Vertex3DReadOnly vertexCandidate = bestVertex;

      while (true)
      {
         for (HalfEdge3DReadOnly currentEdge : bestVertex.getAssociatedEdges())
         {
            Vertex3DReadOnly candidate = currentEdge.getDestination();

            double dotProduct = candidate.dot(supportDirection);

            if (dotProduct > maxDotProduct)
            {
               vertexCandidate = candidate;
               maxDotProduct = dotProduct;
            }
         }

         if (bestVertex == vertexCandidate)
            return bestVertex;
         else
            bestVertex = vertexCandidate;
      }
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

   /**
    * Tests if this polytope is empty, i.e. it has no vertices.
    *
    * @return {@code true} if the polytope has has at least one vertex, {@code false} otherwise.
    */
   default boolean isEmpty()
   {
      return getNumberOfVertices() == 0;
   }

   /** {@inheritDoc} */
   @Override
   default boolean isConvex()
   {
      return true;
   }

   /**
    * Tests on a per component basis if this convex polytope and {@code other} are equal to an
    * {@code epsilon}.
    * 
    * @param other   the other convex polytope to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two convex polytopes are equal component-wise, {@code false}
    *         otherwise.
    */
   default boolean epsilonEquals(ConvexPolytope3DReadOnly other, double epsilon)
   {
      if (other == null)
         return false;

      if (getNumberOfFaces() != other.getNumberOfFaces())
         return false;

      for (int faceIndex = 0; faceIndex < getNumberOfFaces(); faceIndex++)
      {
         if (!getFace(faceIndex).epsilonEquals(other.getFace(faceIndex), epsilon))
            return false;
      }

      return true;
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two convex polytopes are geometrically
    * similar.
    * 
    * @param other   the other convex polytope to compare against this. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two convex polytopes represent the same geometry, {@code false}
    *         otherwise.
    */
   default boolean geometricallyEquals(ConvexPolytope3DReadOnly other, double epsilon)
   {
      if (other == null)
         return false;

      if (getNumberOfFaces() != other.getNumberOfFaces())
         return false;

      ArrayDeque<Face3DReadOnly> thisFacesStack = new ArrayDeque<>(getFaces());

      for (int otherFaceIndex = 0; otherFaceIndex < getNumberOfFaces(); otherFaceIndex++)
      {
         Iterator<Face3DReadOnly> iterator = thisFacesStack.iterator();
         while (iterator.hasNext())
         {
            Face3DReadOnly thisFace = iterator.next();
            if (thisFace.geometricallyEquals(other.getFace(otherFaceIndex), epsilon))
            {
               iterator.remove();
               break;
            }
         }
      }

      return thisFacesStack.isEmpty();
   }

   /**
    * Tests on a per component basis, if this convex polytope 3D is exactly equal to {@code other}.
    *
    * @param other the other convex polytope 3D to compare against this. Not modified.
    * @return {@code true} if the two convex polytopes are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   default boolean equals(ConvexPolytope3DReadOnly other)
   {
      if (other == this)
         return true;

      if (other == null)
         return false;

      if (getNumberOfFaces() != other.getNumberOfFaces())
         return false;

      for (int faceIndex = 0; faceIndex < getNumberOfFaces(); faceIndex++)
      {
         if (!getFace(faceIndex).equals(other.getFace(faceIndex)))
            return false;
      }

      return true;
   }
}
