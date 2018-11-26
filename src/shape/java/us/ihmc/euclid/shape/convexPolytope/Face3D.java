package us.ihmc.euclid.shape.convexPolytope;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Simplex3D;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeIOTools;
import us.ihmc.euclid.shape.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * A template class for a DCEL face. A face is composed of
 * <li>{@code edges} list of half edges that make up this face
 * <li>{@code faceNormal} a vector normal to this face, can point in either direction
 *
 * @author Apoorv S
 *
 */
public class Face3D implements Simplex3D, SupportingVertexHolder, Face3DReadOnly, Clearable, Transformable
{
   /**
    * Unordered list of half edges that bound the face
    */
   private final List<HalfEdge3D> edges = new ArrayList<>();

   /**
    * A vector normal to the plane that this face lies on. Do not access directly since this is updated
    * only when the getter is called
    */
   private final Vector3D faceNormal = new Vector3D();
   /**
    * The variable used to store the centroid of the polytope whenever updated Do not access directly
    * since this is updated only when the getter is called
    */
   private final Point3D faceCentroid = new Point3D();

   // Temporary variables for calculations
   private final Vector3D tempVector = new Vector3D();
   private final Point3D tempPoint = new Point3D();
   private final List<HalfEdge3D> visibleEdgeList = new ArrayList<>();
   private boolean marked = false;

   /**
    * Default constructor. Does not initialize anything
    */
   public Face3D()
   {

   }

   /**
    * {@inheritDoc}
    */
   @Override
   public List<HalfEdge3D> getEdgeList()
   {
      return edges;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public HalfEdge3D getEdge(int index)
   {
      return edges.get(index);
   }

   /**
    * Adds a vertex to the face and updates all the associations accordingly
    *
    * @param vertexToAdd the vertex that must be added to the face
    * @param epsilon
    */
   public void addVertex(Vertex3D vertexToAdd, double epsilon)
   {
      if (epsilon > 0)
         vertexToAdd.round(epsilon);
      switch (edges.size())
      {
      case 0:
      {
         HalfEdge3D newEdge = new HalfEdge3D(vertexToAdd, vertexToAdd);
         newEdge.setFace(this);
         newEdge.setNextHalfEdge(newEdge);
         newEdge.setPreviousHalfEdge(newEdge);
         edges.add(newEdge);
         break;
      }
      case 1:
      {
         // Set the edge for the two points and then create its twin
         if (edges.get(0).getOriginVertex().epsilonEquals(vertexToAdd, epsilon))
            return;
         edges.get(0).setDestinationVertex(vertexToAdd);
         HalfEdge3D newEdge = new HalfEdge3D(vertexToAdd, edges.get(0).getOriginVertex());
         newEdge.setFace(this);
         newEdge.setNextHalfEdge(edges.get(0));
         newEdge.setPreviousHalfEdge(edges.get(0));
         edges.get(0).setNextHalfEdge(newEdge);
         edges.get(0).setPreviousHalfEdge(newEdge);
         edges.add(newEdge);
         break;
      }
      case 2:
      {
         if (edges.get(0).getOriginVertex().epsilonEquals(vertexToAdd, epsilon) || edges.get(0).getDestinationVertex().epsilonEquals(vertexToAdd, epsilon))
            return;
         // Create a new edge and assign an arbitrary configuration since there is no way to tell up and down in 3D space
         edges.get(1).setDestinationVertex(vertexToAdd);
         HalfEdge3D newEdge = new HalfEdge3D(vertexToAdd, edges.get(0).getOriginVertex());
         newEdge.setFace(this);
         edges.add(newEdge);
         newEdge.setNextHalfEdge(edges.get(0));
         edges.get(0).setPreviousHalfEdge(newEdge);
         newEdge.setPreviousHalfEdge(edges.get(1));
         edges.get(1).setNextHalfEdge(newEdge);
         break;
      }
      default:
      {
         // Now a ordering is available and all new vertices to add must be done accordingly. Also points must lie in the same plane
         if (!isPointInFacePlane(vertexToAdd, epsilon))
            return;

         getVisibleEdgeList(vertexToAdd, visibleEdgeList);
         HalfEdge3D firstVisibleEdge = visibleEdgeList.get(0);
         HalfEdge3D lastVisibleEdge = visibleEdgeList.get(visibleEdgeList.size() - 1);
         switch (visibleEdgeList.size())
         {
         case 0:
            return; // Case where the point is internal
         case 1:
            if (firstVisibleEdge.getOriginVertex().epsilonEquals(vertexToAdd, epsilon)
                  || firstVisibleEdge.getDestinationVertex().epsilonEquals(vertexToAdd, epsilon))
               return;
            HalfEdge3D additionalEdge = new HalfEdge3D(vertexToAdd, firstVisibleEdge.getDestinationVertex());
            additionalEdge.setFace(this);
            firstVisibleEdge.setDestinationVertex(vertexToAdd);
            additionalEdge.setNextHalfEdge(firstVisibleEdge.getNextHalfEdge());
            firstVisibleEdge.getNextHalfEdge().setPreviousHalfEdge(additionalEdge);
            firstVisibleEdge.setNextHalfEdge(additionalEdge);
            additionalEdge.setPreviousHalfEdge(firstVisibleEdge);
            edges.add(additionalEdge);
            break;
         default:
            firstVisibleEdge.setDestinationVertex(vertexToAdd);
            lastVisibleEdge.setOriginVertex(vertexToAdd);
            firstVisibleEdge.setNextHalfEdge(lastVisibleEdge);
            lastVisibleEdge.setPreviousHalfEdge(firstVisibleEdge);
            for (int i = 1; i < visibleEdgeList.size() - 1; i++)
               edges.remove(visibleEdgeList.get(i));
            break;
         }
         break;
      }
      }
   }

   public void getVisibleEdgeList(Point3DReadOnly vertex, List<HalfEdge3D> edgeList)
   {
      edgeList.clear();
      HalfEdge3D edgeUnderConsideration = getFirstVisibleEdge(vertex);
      for (int i = 0; edgeUnderConsideration != null && i < edges.size(); i++)
      {
         edgeList.add(edgeUnderConsideration);
         edgeUnderConsideration = edgeUnderConsideration.getNextHalfEdge();
         if (isPointOnInteriorSideOfEdgeInternal(vertex, edgeUnderConsideration))
            break;
      }
   }

   @Override
   public HalfEdge3D getFirstVisibleEdge(Point3DReadOnly vertex)
   {
      if (edges.size() == 0)
         return null;
      else if (edges.size() == 1 || edges.size() == 2)
         return edges.get(0);

      HalfEdge3D edgeUnderConsideration = edges.get(0);
      double previousDotProduct = getEdgeVisibilityProduct(vertex, edgeUnderConsideration);
      edgeUnderConsideration = edgeUnderConsideration.getNextHalfEdge();
      for (int i = 0; i < getNumberOfEdges(); i++)
      {
         double dotProduct = getEdgeVisibilityProduct(vertex, edgeUnderConsideration);
         if (dotProduct >= 0.0 && previousDotProduct < 0.0)
         {
            return edgeUnderConsideration;
         }
         else if (dotProduct >= 0.0 && previousDotProduct == 0.0)
         {
            return edgeUnderConsideration.getPreviousHalfEdge();
         }
         else
         {
            edgeUnderConsideration = edgeUnderConsideration.getNextHalfEdge();
            previousDotProduct = dotProduct;
         }
      }
      return null;
   }

   @Override
   public boolean isPointOnInteriorSideOfEdgeInternal(Point3DBasics point, int index)
   {
      updateFaceNormal();
      return isPointOnInteriorSideOfEdgeInternal(point, edges.get(index));
   }

   private boolean isPointOnInteriorSideOfEdgeInternal(Point3DReadOnly point, HalfEdge3D halfEdge)
   {
      return getEdgeVisibilityProduct(point, halfEdge) < 0;
   }

   @Override
   public double getFaceVisibilityProduct(Point3DReadOnly point)
   {
      tempVector.sub(point, getEdge(0).getOriginVertex());
      return dotFaceNormal(tempVector);
   }

   private double getEdgeVisibilityProduct(Point3DReadOnly point, HalfEdge3D halfEdge)
   {
      tempVector.sub(point, halfEdge.getOriginVertex());
      tempVector.cross(halfEdge.getEdgeVector());
      return tempVector.dot(getFaceNormal());
   }

   @Override
   public boolean isPointInFacePlane(Point3DReadOnly vertexToCheck, double epsilon)
   {
      boolean isInFacePlane;
      tempVector.sub(vertexToCheck, edges.get(0).getOriginVertex());
      if (edges.size() < 3)
      {
         isInFacePlane = !EuclidCoreTools.epsilonEquals(Math.abs(edges.get(0).getEdgeVector().dot(tempVector))
               / (edges.get(0).getEdgeVector().length() * tempVector.length()), 1.0, epsilon);
      }
      else
         isInFacePlane = EuclidCoreTools.epsilonEquals(tempVector.dot(getFaceNormal()), 0.0, epsilon);
      return isInFacePlane;
   }

   @Override
   public boolean isInteriorPoint(Point3DReadOnly vertexToCheck, double epsilon)
   {
      return isPointInFacePlane(vertexToCheck, epsilon) && isInteriorPointInternal(vertexToCheck);
   }

   private boolean isInteriorPointInternal(Point3DReadOnly vertexToCheck)
   {
      if (edges.size() < 3)
         return false;

      boolean result = true;
      HalfEdge3D halfEdge = edges.get(0);
      for (int i = 0; result && i < edges.size(); i++)
      {
         result &= isPointOnInteriorSideOfEdgeInternal(vertexToCheck, halfEdge);
         halfEdge = halfEdge.getNextHalfEdge();
      }
      return result;
   }

   @Override
   public Point3D getFaceCentroid()
   {
      updateFaceCentroid();
      return faceCentroid;
   }

   private void updateFaceCentroid()
   {
      faceCentroid.setToZero();
      for (int i = 0; i < edges.size(); i++)
         faceCentroid.add(edges.get(i).getOriginVertex());
      faceCentroid.scale(1.0 / edges.size());
   }

   @Override
   public Vector3D getFaceNormal()
   {
      updateFaceNormal();
      return faceNormal;
   }

   private void updateFaceNormal()
   {
      if (edges.size() < 3)
         faceNormal.setToZero();
      else
      {
         faceNormal.cross(edges.get(0).getEdgeVector(), edges.get(0).getNextHalfEdge().getEdgeVector());
         if (faceNormal.dot(faceNormal) > 1.0e-5)
            faceNormal.normalize();
      }
   }

   @Override
   public int getNumberOfEdges()
   {
      return edges.size();
   }

   @Override
   public void applyTransform(Transform transform)
   {
      for (int i = 0; i < getNumberOfEdges(); i++)
         edges.get(i).getOriginVertex().applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      for (int i = 0; i < getNumberOfEdges(); i++)
         edges.get(i).getOriginVertex().applyInverseTransform(transform);
   }

   public int findMatchingEdgeIndex(HalfEdge3DReadOnly edgeToSearch, double epsilon)
   {
      for (int i = 0; i < edges.size(); i++)
      {
         if (edges.get(i).epsilonEquals(edgeToSearch, epsilon))
            return i;
      }
      return -1;
   }

   public HalfEdge3DReadOnly findMatchingEdge(HalfEdge3DReadOnly edgeToSearch, double epsilon)
   {
      return edges.get(findMatchingEdgeIndex(edgeToSearch, epsilon));
   }

   public void reverseFaceNormal()
   {
      for (int i = 0; i < edges.size(); i++)
      {
         edges.get(i).reverseEdge();
      }
      updateFaceNormal();
   }

   public void clearEdgeList()
   {
      edges.clear();
   }

   @Override
   public boolean containsNaN()
   {
      boolean result = edges.size() > 0 && edges.get(0).containsNaN();
      for (int i = 1; !result && i < edges.size(); i++)
         result |= edges.get(i).getDestinationVertex().containsNaN();
      return result;
   }

   @Override
   public void setToNaN()
   {
      for (int i = 0; i < edges.size(); i++)
         edges.get(i).setToNaN();
   }

   @Override
   public double dotFaceNormal(Vector3DReadOnly direction)
   {
      updateFaceNormal();
      return direction.dot(faceNormal);
   }

   @Override
   public boolean isFaceVisible(Point3DReadOnly point, double epsilon)
   {
      return getFaceVisibilityProduct(point) > epsilon;
   }

   @Override
   public void setToZero()
   {
      for (int i = 0; i < edges.size(); i++)
         edges.get(i).setToZero();
   }

   @Override
   public double getMaxElement(int index)
   {
      HalfEdge3D edgeReference = edges.get(0);
      double maxElement = edgeReference.getOriginVertex().getElement(index);
      for (int i = 0; i < edges.size(); i++)
      {
         if (maxElement < edgeReference.getDestinationVertex().getElement(index))
            maxElement = edgeReference.getDestinationVertex().getElement(index);
         edgeReference = edgeReference.getNextHalfEdge();
      }
      return maxElement;
   }

   @Override
   public double getMinElement(int index)
   {
      HalfEdge3D edgeReference = edges.get(0);
      double minElement = edgeReference.getOriginVertex().getElement(index);
      for (int i = 0; i < edges.size(); i++)
      {
         if (minElement > edgeReference.getDestinationVertex().getElement(index))
            minElement = edgeReference.getDestinationVertex().getElement(index);
         edgeReference = edgeReference.getNextHalfEdge();
      }
      return minElement;
   }

   @Override
   public double getMaxX()
   {
      return getMaxElement(0);
   }

   @Override
   public double getMaxY()
   {
      return getMaxElement(1);
   }

   @Override
   public double getMaxZ()
   {
      return getMaxElement(2);
   }

   @Override
   public double getMinX()
   {
      return getMinElement(0);
   }

   @Override
   public double getMinY()
   {
      return getMinElement(1);
   }

   @Override
   public double getMinZ()
   {
      return getMinElement(2);
   }

   @Override
   public Face3D getNeighbouringFace(int index)
   {
      if (index > edges.size() || edges.get(index).getTwinHalfEdge() == null)
         return null;
      else
         return edges.get(index).getTwinHalfEdge().getFace();

   }

   @Override
   public Point3DReadOnly getSupportingVertex(Vector3DReadOnly supportVector)
   {
      Vertex3D bestVertex = edges.get(0).getOriginVertex();
      double maxDot = bestVertex.dot(supportVector);
      Vertex3D bestVertexCandidate = bestVertex;
      while (true)
      {
         bestVertexCandidate = bestVertex;
         for (int i = 0; i < bestVertex.getNumberOfAssociatedEdges(); i++)
         {
            double dotCandidate = bestVertex.getAssociatedEdges().get(i).getDestinationVertex().dot(supportVector);
            if (maxDot < dotCandidate)
            {
               maxDot = dotCandidate;
               bestVertexCandidate = bestVertex.getAssociatedEdge(i).getDestinationVertex();
            }
         }
         if (bestVertexCandidate == bestVertex)
            return bestVertex;
         else
            bestVertex = bestVertexCandidate;
      }
   }

   public void mark()
   {
      marked = true;
   }

   public void unmark()
   {
      marked = false;
   }

   @Override
   public boolean isMarked()
   {
      return marked;
   }

   @Override
   public boolean isNotMarked()
   {
      return !marked;
   }

   @Override
   public double distance(Point3DReadOnly point)
   {
      EuclidGeometryTools.orthogonalProjectionOnPlane3D(point, edges.get(0).getOriginVertex(), getFaceNormal(), tempPoint);
      if (isInteriorPointInternal(tempPoint))
         return point.distance(tempPoint);
      else
         return getEdgeClosestTo(tempPoint).distance(point);
   }

   @Override
   public HalfEdge3D getEdgeClosestTo(Point3DReadOnly point)
   {
      HalfEdge3D edge = getFirstVisibleEdge(tempPoint);
      double shortestDistance = edge.distance(tempPoint);
      double shortestDistanceCandidate = Double.NEGATIVE_INFINITY;
      while (shortestDistanceCandidate < shortestDistance)
      {
         edge = edge.getNextHalfEdge();
         shortestDistanceCandidate = edge.distance(tempPoint);
      }
      return edge.getPreviousHalfEdge();
   }

   @Override
   public void getSupportVectorDirectionTo(Point3DReadOnly point, Vector3DBasics supportVectorToPack)
   {
      EuclidGeometryTools.orthogonalProjectionOnPlane3D(point, edges.get(0).getOriginVertex(), getFaceNormal(), tempPoint);
      if (isInteriorPointInternal(tempPoint))
         supportVectorToPack.set(getFaceNormal());
      else
         getEdgeClosestTo(tempPoint).getSupportVectorDirectionTo(point, supportVectorToPack);
   }

   @Override
   public Simplex3D getSmallestSimplexMemberReference(Point3DReadOnly point)
   {
      EuclidGeometryTools.orthogonalProjectionOnPlane3D(point, edges.get(0).getOriginVertex(), getFaceNormal(), tempPoint);
      if (isInteriorPointInternal(tempPoint))
         return this;
      else
         return getEdgeClosestTo(tempPoint).getSmallestSimplexMemberReference(point);
   }

   @Override
   public String toString()
   {
      return EuclidPolytopeIOTools.getFace3DString(this);
   }
}
