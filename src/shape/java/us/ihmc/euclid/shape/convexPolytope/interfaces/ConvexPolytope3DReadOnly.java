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

public interface ConvexPolytope3DReadOnly extends Shape3DReadOnly, Simplex3D
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
      return getClosestFace(point).distance(point);
   }

   @Override
   default double signedDistance(Point3DReadOnly point)
   {
      boolean isOutside = false;
      double maxNegativeDistance = Double.NEGATIVE_INFINITY;
      Face3DReadOnly lastFaceExplored = null;

      for (int faceIndex = 0; faceIndex < getNumberOfFaces(); faceIndex++)
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
            lastFaceExplored = face;
            break;
         }
      }

      if (!isOutside)
         return maxNegativeDistance;

      boolean goToNeighbor = false;
      boolean isQueryDirectlyAboveFace = true;
      Face3DReadOnly closestFace = lastFaceExplored;
      HalfEdge3DReadOnly closestEdge = null;
      double distanceSquaredToClosestEdge = Double.POSITIVE_INFINITY;

      while (true)
      {
         goToNeighbor = false;
         isQueryDirectlyAboveFace = true;
         distanceSquaredToClosestEdge = Double.POSITIVE_INFINITY;
         closestEdge = null;

         for (int edgeIndex = 0; edgeIndex < closestFace.getNumberOfEdges(); edgeIndex++)
         {
            HalfEdge3DReadOnly edge = closestFace.getEdge(edgeIndex);
            boolean isEdgeVisible = closestFace.canObserverSeeEdge(point, edge);

            if (isEdgeVisible)
            { // Visit neighbor.
               isQueryDirectlyAboveFace = false;
               HalfEdge3DReadOnly neighborEdge = edge.getTwin();
               Face3DReadOnly candidateFace = neighborEdge.getFace();
               boolean isNeighborEdgeVisible = candidateFace.canObserverSeeEdge(point, neighborEdge);

               if (!isNeighborEdgeVisible)
               {
                  closestFace = candidateFace;
                  goToNeighbor = true;
                  break;
               }
               else
               {
                  double distanceSquared = edge.distanceSquared(point);

                  if (distanceSquared < distanceSquaredToClosestEdge)
                  {
                     distanceSquaredToClosestEdge = distanceSquared;
                     closestEdge = edge;
                  }
               }
            }
         }

         if (!goToNeighbor)
         {
            Face3DReadOnly naiveClosestFace = getFaces().stream().sorted((f1, f2) -> Double.compare(f1.distance(point), f2.distance(point))).findFirst().get();
            if (isQueryDirectlyAboveFace)
            {
               assert closestFace == naiveClosestFace;
               return closestFace.distanceToPlane(point);
            }
            else
            {
               double percentage = closestEdge.percentageAlongLineSegment(point);

               Vertex3DReadOnly vertexToExplore = null;

               if (percentage < 0.0)
                  vertexToExplore = closestEdge.getOrigin();
               else if (percentage > 1.0)
                  vertexToExplore = closestEdge.getDestination();
               else
                  return Math.sqrt(distanceSquaredToClosestEdge);

               for (int edgeIndex = 0; edgeIndex < vertexToExplore.getNumberOfAssociatedEdges(); edgeIndex++)
               {
                  HalfEdge3DReadOnly candidateEdge = vertexToExplore.getAssociatedEdge(edgeIndex);
                  distanceSquaredToClosestEdge = Math.min(distanceSquaredToClosestEdge, candidateEdge.distanceSquared(point));
               }

               return Math.sqrt(distanceSquaredToClosestEdge);
            }
         }
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
      Face3DReadOnly lastFaceExplored = null;
      Face3DReadOnly closestFace = null;

      for (int faceIndex = 0; faceIndex < getNumberOfFaces(); faceIndex++)
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
            lastFaceExplored = face;
            break;
         }
      }

      if (!isOutside)
         return closestFace;

      boolean goToNeighbor = false;

      double supportDirectionX = point.getX() - getCentroid().getX();
      double supportDirectionY = point.getY() - getCentroid().getY();
      double supportDirectionZ = point.getZ() - getCentroid().getZ();
      Vertex3DReadOnly supportingVertex = lastFaceExplored.getVertex(0);
      double bestDot = TupleTools.dot(supportDirectionX, supportDirectionY, supportDirectionZ, supportingVertex);

      boolean hasClosestVertexBeenUpdated = true;

      while (hasClosestVertexBeenUpdated)
      {
         hasClosestVertexBeenUpdated = false;

         for (int connectedVertexIndex = 0; connectedVertexIndex < supportingVertex.getNumberOfAssociatedEdges(); connectedVertexIndex++)
         {
            Vertex3DReadOnly candidate = supportingVertex.getAssociatedEdge(connectedVertexIndex).getDestination();
            double dotCandidate = TupleTools.dot(supportDirectionX, supportDirectionY, supportDirectionZ, candidate);

            if (dotCandidate > bestDot)
            {
               hasClosestVertexBeenUpdated = true;
               supportingVertex = candidate;
               bestDot = dotCandidate;
               break;
            }
         }
      }

      double distanceSquaredToClosestEdge = Double.POSITIVE_INFINITY;

      for (int i = 0; i < supportingVertex.getNumberOfAssociatedEdges(); i++)
      {
         HalfEdge3DReadOnly edge = supportingVertex.getAssociatedEdge(i);
         Face3DReadOnly face = edge.getFace();

         if (face.canObserverSeeFace(point)) // && !face.canObserverSeeEdge(point, edge))
         {
            double distanceSquared = edge.distanceSquared(point);
            if (distanceSquared < distanceSquaredToClosestEdge)
            {
               distanceSquaredToClosestEdge = distanceSquared;
               closestFace = face;
            }
         }
      }

      if (closestFace == null)
      {
         closestFace = supportingVertex.getAssociatedEdge(0).getFace();
      }

      int count = 0;

      while (true)
      {
         if (count++ >= 10000)
            throw new RuntimeException("Did not converge");
         goToNeighbor = false;

         List<? extends HalfEdge3DReadOnly> lineOfSight = closestFace.lineOfSight(point);

         if (lineOfSight.isEmpty())
            return closestFace;

         HalfEdge3DReadOnly closestFaceEdge = null;

         if (lineOfSight.size() == 1)
         {
            closestFaceEdge = lineOfSight.get(0);
         }
         else
         {
            distanceSquaredToClosestEdge = Double.POSITIVE_INFINITY;
            
            for (int edgeIndex = 0; edgeIndex < lineOfSight.size(); edgeIndex++)
            {
               HalfEdge3DReadOnly edge = lineOfSight.get(edgeIndex);
               double distanceSquared = edge.distanceSquared(point);
               
               if (distanceSquared < distanceSquaredToClosestEdge)
               {
                  closestFaceEdge = edge;
                  distanceSquaredToClosestEdge = distanceSquared;
               }
            }
         }

         { // Visit neighbors.
           // 1- Explore neighbor through common edge:
            HalfEdge3DReadOnly neighborEdge = closestFaceEdge.getTwin();
            Face3DReadOnly candidateFace = neighborEdge.getFace();
            boolean isNeighborEdgeVisible = candidateFace.canObserverSeeEdge(point, neighborEdge);

            if (!isNeighborEdgeVisible)
            {
               closestFace = candidateFace;
               goToNeighbor = true;
               continue;
            }

            // 2- Test if the query is above the current edge:
            double percentageAlongEdge = closestFaceEdge.percentageAlongLineSegment(point);

            if (percentageAlongEdge >= 0.0 && percentageAlongEdge <= 1.0)
            {
               goToNeighbor = false;
               return closestFace;
            }

            // 3- Explore neighbors through common vertex:
            Vertex3DReadOnly vertex;

            if (percentageAlongEdge <= 0.5)
               vertex = closestFaceEdge.getOrigin();
            else
               vertex = closestFaceEdge.getDestination();

            // First test for scenario where the query is closest to the vertex.
            boolean isClosestToVertex = true;
            boolean isClosestToEdge = false;
            HalfEdge3DReadOnly closestEdge = null;

            for (int i = 0; i < vertex.getNumberOfAssociatedEdges(); i++)
            {
               HalfEdge3DReadOnly associatedEdge = vertex.getAssociatedEdge(i);
               percentageAlongEdge = associatedEdge.percentageAlongLineSegment(point);

               if (associatedEdge != closestFaceEdge)
               {
                  if (isClosestToVertex && percentageAlongEdge > 0.0)
                  {
                     isClosestToVertex = false;
                  }

                  if (!isClosestToEdge)
                  {
                     if (percentageAlongEdge >= 0.0 && percentageAlongEdge <= 1.0)
                     {
                        Face3DReadOnly associatedFace = associatedEdge.getFace();
                        HalfEdge3DReadOnly associatedTwinEdge = associatedEdge.getTwin();
                        Face3DReadOnly associatedTwinFace = associatedTwinEdge.getFace();

                        if (associatedFace.canObserverSeeEdge(point, associatedEdge) && associatedTwinFace.canObserverSeeEdge(point, associatedTwinEdge))
                        {
                           isClosestToEdge = true;
                           closestEdge = associatedEdge;
                           break;
                        }
                     }
                  }
               }
            }

            if (isClosestToVertex)
            {
               return closestFace;
            }
            else if (isClosestToEdge)
            {
               closestFace = closestEdge.getFace();
               return closestFace;
            }
            else
            {
               for (int i = 0; i < vertex.getNumberOfAssociatedEdges(); i++)
               {
                  HalfEdge3DReadOnly associatedEdge = vertex.getAssociatedEdge(i);
                  Face3DReadOnly associatedFace = associatedEdge.getFace();

                  if (associatedFace == closestFace)
                     continue;

                  if (!associatedFace.canObserverSeeEdge(point, associatedEdge))
                  { // Go to the associated face.
                     closestFace = associatedFace;
                     goToNeighbor = true;
                     break;
                  }
                  else if (associatedEdge.getTwin().getFace().canObserverSeeEdge(point, associatedEdge.getTwin()))
                  { // Test for scenario where the query is closest to an edge.
                     percentageAlongEdge = associatedEdge.percentageAlongLineSegment(point);
                     if (percentageAlongEdge >= 0.0 && percentageAlongEdge <= 1.0)
                     {
                        closestFace = associatedFace;
                        return closestFace;
                     }
                  }
               }
            }
         }

         if (!goToNeighbor)
         {
            return closestFace;
         }
      }

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
   default Simplex3D getSmallestSimplexMemberReference(Point3DReadOnly point)
   {
      if (isEmpty())
         return null;
      else
         return getClosestFace(point).getSmallestSimplexMemberReference(point);
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
