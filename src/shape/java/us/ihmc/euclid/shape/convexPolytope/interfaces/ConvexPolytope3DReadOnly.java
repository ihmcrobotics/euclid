package us.ihmc.euclid.shape.convexPolytope.interfaces;

import java.util.ArrayDeque;
import java.util.Iterator;
import java.util.List;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTools;
import us.ihmc.euclid.shape.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface ConvexPolytope3DReadOnly extends SupportingVertexHolder, Simplex3D
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

   BoundingBox3DReadOnly getBoundingBox();

   Point3DReadOnly getCentroid();

   /**
    * Gets the tolerance used for building this convex polytope.
    * 
    * @return the construction tolerance.
    */
   double getConstructionEpsilon();

   default boolean containsNaN()
   {
      for (int vertexIndex = 0; vertexIndex < getNumberOfVertices(); vertexIndex++)
      {
         if (getVertex(vertexIndex).containsNaN())
            return true;
      }
      return false;
   }

   default boolean isPointInside(Point3DReadOnly pointToCheck, double epsilon)
   {
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

   default Face3DReadOnly getClosestFace(Point3DReadOnly point)
   {
      if (getNumberOfFaces() == 0)
         return null;
      if (getNumberOfFaces() == 1)
         return getFace(0);

      Face3DReadOnly closestFace = null;
      double minDistance = Double.POSITIVE_INFINITY;

      for (int faceIndex = 0; faceIndex < getNumberOfFaces(); faceIndex++)
      {
         Face3DReadOnly face = getFace(faceIndex);
         double distance = face.distance(point);

         if (distance < minDistance)
         {
            closestFace = face;
            minDistance = distance;
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
   Vertex3DReadOnly getSupportingVertex(Vector3DReadOnly supportingVertexDirection);

   /**
    * Check is the polytope is empty (contains no vertices / edges)
    * 
    * @return {@code true} if the polytope has faces that contain edges, otherwise {@code false}
    */
   default boolean isEmpty()
   {
      return getNumberOfVertices() == 0;
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
