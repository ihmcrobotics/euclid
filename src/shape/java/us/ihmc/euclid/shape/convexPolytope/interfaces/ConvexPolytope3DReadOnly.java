package us.ihmc.euclid.shape.convexPolytope.interfaces;

import java.util.ArrayDeque;
import java.util.Iterator;
import java.util.List;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTools;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface ConvexPolytope3DReadOnly extends Shape3DReadOnly, ConvexPolytopeFeature3D
{
   /**
    * Returns the number of vertices that are in the polytope
    *
    * @return integer number of vertices in the polytope
    */
   default int getNumberOfVertices()
   {
      return getVertices().size();
   }

   /**
    * Returns the number of edge that are part of the polytope Note: number of edges is half the number
    * of half edges
    *
    * @return integer number of edges in the face
    */
   default int getNumberOfEdges()
   {
      if (getNumberOfFaces() < 2)
         return getNumberOfHalfEdges();
      else
         return getNumberOfHalfEdges() / 2;
   }

   default int getNumberOfHalfEdges()
   {
      return getHalfEdges().size();
   }

   /**
    * Returns the number of face that are constitute the polytope
    *
    * @return integer number of faces
    */
   default int getNumberOfFaces()
   {
      return getFaces().size();
   }

   /**
    * Gets a specified face of polytope
    *
    * @param index the index of the polytope. Should be smaller than {@code getNumberOfFaces()}
    * @return
    */
   default Face3DReadOnly getFace(int index)
   {
      return getFaces().get(index);
   }

   /**
    * Get a list of faces that constitute the polytope
    *
    * @return a list of read only references to the faces of the polytope
    */
   List<? extends Face3DReadOnly> getFaces();

   default HalfEdge3DReadOnly getHalfEdge(int index)
   {
      return getHalfEdges().get(index);
   }

   /**
    * Get a list of half edges that are part of this polytope. List will contain the half edge and its
    * twin Size of the list will be twice the number of edges returned by {@code getNumberOfEdges()}
    *
    * @return a list of read only references to the half edges that make up the faces of this polytope
    */
   List<? extends HalfEdge3DReadOnly> getHalfEdges();

   default Vertex3DReadOnly getVertex(int index)
   {
      return getVertices().get(index);
   }

   /**
    * Get a list of vertices that are part of this polytope. List does not contain any repetitions
    *
    * @return a list of read only reference to the vertices of the polytope
    */
   @Override
   List<? extends Vertex3DReadOnly> getVertices();

   @Override
   BoundingBox3DReadOnly getBoundingBox();

   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      boundingBoxToPack.set(getBoundingBox());
   }

   Point3DReadOnly getCentroid();

   double getVolume();

   /**
    * Gets the tolerance used for building this convex polytope.
    *
    * @return the construction tolerance.
    */
   double getConstructionEpsilon();

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

         if (EuclidGeometryTools.isPoint3DAbovePlane3D(pointToCheck, face.getCentroid(), face.getNormal()))
            return false;
      }
      return true;
   }

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
         if (EuclidPolytopeTools.canObserverSeeFace(pointToCheck, getFace(faceIndex), epsilon))
            return false;
      }
      return true;
   }

   @Override
   default double distance(Point3DReadOnly point)
   {
      return Math.min(0.0, signedDistance(point));
   }

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
         double signedDistanceToPlane = face.signedDistanceToPlane(point);

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

   @Override
   default Point3DBasics orthogonalProjectionCopy(Point3DReadOnly pointToProject)
   {
      if (isEmpty() || isPointInside(pointToProject))
         return null;
      else
         return getClosestFace(pointToProject).orthogonalProjectionCopy(pointToProject);
   }

   @Override
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      if (isEmpty() || isPointInside(pointToProject))
         return false;
      else
         return getClosestFace(pointToProject).orthogonalProjection(pointToProject, projectionToPack);
   }

   @Override
   default boolean doPoint3DCollisionTest(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack)
   {
      if (isEmpty())
         return false;

      Face3DReadOnly closestFace = getClosestFace(pointToCheck);
      closestFace.orthogonalProjection(pointToCheck, closestPointOnSurfaceToPack);
      closestFace.getSupportVectorDirectionTo(pointToCheck, normalAtClosestPointToPack);
      normalAtClosestPointToPack.normalize();

      return !EuclidGeometryTools.isPoint3DAbovePlane3D(pointToCheck, closestFace.getCentroid(), closestFace.getNormal());
   }

   default Face3DReadOnly getClosestFace(Point3DReadOnly point)
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
         double signedDistanceToPlane = face.signedDistanceToPlane(point);

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
         double closestFaceDistance = closestFace.distance(point);

         faceIndex++;

         for (; faceIndex < getNumberOfFaces(); faceIndex++)
         {
            Face3DReadOnly face = getFace(faceIndex);
            if (!face.canObserverSeeFace(point))
               continue; // The query is below, cannot be the closest face.

            // Now use the more expensive Face3DReadOnly.distance(Point3DReadOnly)
            double candidateDistance = face.distance(point);

            if (candidateDistance < closestFaceDistance)
            {
               closestFace = face;
               closestFaceDistance = candidateDistance;
            }
         }
      }

      return closestFace;
   }

   /**
    * Returns a reference to the polytope vertex that is further along the direction indicated by the
    * specified vector
    *
    * @param supportingVertexDirection the direction in which the search for said vertex is to be
    *           performed
    * @return a read only reference to the required vertex
    */
   @Override
   default Vertex3DReadOnly getSupportingVertex(Vector3DReadOnly supportDirection)
   {
      return getSupportingVertex(supportDirection.getX(), supportDirection.getY(), supportDirection.getZ());
   }

   default Vertex3DReadOnly getSupportingVertex(double supportDirectionX, double supportDirectionY, double supportDirectionZ)
   {
      if (isEmpty())
         return null;
      if (getNumberOfFaces() == 1)
         return getFace(0).getSupportingVertex(supportDirectionX, supportDirectionY, supportDirectionZ);

      Vertex3DReadOnly bestVertex = getFace(0).getEdge(0).getOrigin();

      if (getNumberOfVertices() == 1)
         return bestVertex;

      double maxDotProduct = TupleTools.dot(supportDirectionX, supportDirectionY, supportDirectionZ, bestVertex);
      Vertex3DReadOnly vertexCandidate = bestVertex;

      while (true)
      {
         for (HalfEdge3DReadOnly currentEdge : bestVertex.getAssociatedEdges())
         {
            Vertex3DReadOnly candidate = currentEdge.getDestination();

            double dotProduct = TupleTools.dot(supportDirectionX, supportDirectionY, supportDirectionZ, candidate);

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

   @Override
   default boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      if (isEmpty())
         return false;
      supportingVertexToPack.set(getSupportingVertex(supportDirection));
      return true;
   }

   @Override
   default boolean getSupportVectorDirectionTo(Point3DReadOnly point, Vector3DBasics supportVectorToPack)
   {
      if (isEmpty())
         return false;
      else
         return getClosestFace(point).getSupportVectorDirectionTo(point, supportVectorToPack);
   }

   @Override
   default ConvexPolytopeFeature3D getSmallestFeature(Point3DReadOnly point)
   {
      if (isEmpty())
         return null;
      else
         return getClosestFace(point).getSmallestFeature(point);
   }

   /**
    * Check is the polytope is empty (contains no vertices / edges)
    *
    * @return {@code true} if the polytope has faces that contain edges, otherwise {@code false}
    */
   default boolean isEmpty()
   {
      return getNumberOfVertices() == 0;
   }

   @Override
   default boolean isConvex()
   {
      return true;
   }

   default boolean equals(ConvexPolytope3DReadOnly other)
   {
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
}
