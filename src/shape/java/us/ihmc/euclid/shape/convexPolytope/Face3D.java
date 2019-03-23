package us.ihmc.euclid.shape.convexPolytope;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeConstructionTools;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTools;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
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
public class Face3D implements Face3DReadOnly, Clearable, Transformable
{
   private final List<HalfEdge3D> edges = new ArrayList<>();
   private final List<Vertex3D> vertices = new ArrayList<>();

   /**
    * A vector normal to the plane that this face lies on. Do not access directly since this is updated
    * only when the getter is called
    */
   private final Vector3D normal = new Vector3D();
   /**
    * The variable used to store the centroid of the polytope whenever updated Do not access directly
    * since this is updated only when the getter is called
    */
   private final Point3D centroid = new Point3D();

   private double area;

   private final BoundingBox3D boundingBox = new BoundingBox3D();

   private final double constructionEpsilon;

   public Face3D(Vector3DReadOnly initialGuessNormal)
   {
      this(initialGuessNormal, EuclidPolytopeConstructionTools.DEFAULT_CONSTRUCTION_EPSILON);
   }

   public Face3D(Vector3DReadOnly initialGuessNormal, double constructionEpsilon)
   {
      normal.setAndNormalize(initialGuessNormal);
      boundingBox.setToNaN();
      this.constructionEpsilon = constructionEpsilon;
   }

   public Face3D(Collection<HalfEdge3D> faceEdges, Vector3DReadOnly normal, double constructionEpsilon)
   {
      boundingBox.setToNaN();
      this.constructionEpsilon = constructionEpsilon;

      set(faceEdges, normal);
   }

   public void set(Collection<HalfEdge3D> faceEdges, Vector3DReadOnly normal)
   {
      edges.clear();
      edges.addAll(faceEdges);
      edges.forEach(edge -> edge.setFace(this));
      this.normal.set(normal);
      updateVertices();
      updateNormal();
      updateCentroidAndArea();
      refreshBoundingBox();
   }

   /**
    * Adds a vertex to the face and updates all the associations accordingly
    *
    * @param vertexToAdd the vertex that must be added to the face
    * @param epsilon
    */
   public boolean addVertex(Vertex3D vertexToAdd)
   {
      return addVertex(vertexToAdd, null, false);
   }

   public boolean addVertex(Vertex3D vertexToAdd, Vertex3D faceVertexToLock, boolean lockEdgesWithTwin)
   {
      if (edges.isEmpty())
      {
         HalfEdge3D newEdge = new HalfEdge3D(vertexToAdd, vertexToAdd);
         newEdge.setFace(this);
         newEdge.setNext(newEdge);
         newEdge.setPrevious(newEdge);
         edges.add(newEdge);
      }
      else if (edges.size() == 1)
      {
         HalfEdge3D firstEdge = edges.get(0);
         if (firstEdge.getOrigin().geometricallyEquals(vertexToAdd, constructionEpsilon))
            return false;

         // Set the edge for the two points and then create its twin
         firstEdge.setDestination(vertexToAdd);
         HalfEdge3D newEdge = new HalfEdge3D(vertexToAdd, firstEdge.getOrigin());
         newEdge.setFace(this);
         newEdge.setNext(firstEdge);
         newEdge.setPrevious(firstEdge);
         firstEdge.setNext(newEdge);
         firstEdge.setPrevious(newEdge);
         edges.add(newEdge);
      }
      else if (edges.size() == 2)
      {
         HalfEdge3D firstEdge = edges.get(0);
         if (firstEdge.distance(vertexToAdd) < constructionEpsilon)
            return false;

         HalfEdge3D secondEdge = edges.get(1);

         // Ensuring clockwise ordering using the initial guess for the normal.
         Vector3D resultingNormal = EuclidPolytopeTools.crossProductOfLineSegment3Ds(firstEdge.getOrigin(), firstEdge.getDestination(),
                                                                                     firstEdge.getDestination(), vertexToAdd);
         if (resultingNormal.dot(normal) > 0.0)
         { // Counter-clockwise, need to reverse the ordering.
            firstEdge.flip();
            secondEdge.setOrigin(firstEdge.getDestination());
         }

         secondEdge.setDestination(vertexToAdd);
         HalfEdge3D newEdge = new HalfEdge3D(vertexToAdd, firstEdge.getOrigin(), secondEdge, firstEdge, this);
         firstEdge.setPrevious(newEdge);
         secondEdge.setNext(newEdge);
         edges.add(newEdge);
      }
      else
      {
         List<HalfEdge3D> lineOfSight = lineOfSight(vertexToAdd);

         if (lineOfSight.isEmpty())
            return false;

         if (lockEdgesWithTwin)
         {
            for (int i = 0; i < lineOfSight.size(); i++)
            {
               if (lineOfSight.get(i).getTwin() != null)
                  return false;
            }
         }

         if (lineOfSight.size() == 1 && lineOfSight.get(0).distance(vertexToAdd) < constructionEpsilon)
            return false;

         HalfEdge3D firstVisibleEdge = lineOfSight.get(0);
         HalfEdge3D lastVisibleEdge = lineOfSight.get(lineOfSight.size() - 1);

         HalfEdge3D edgeBeforeLineOfSight = firstVisibleEdge.getPrevious();
         HalfEdge3D edgeAfterLineOfSight = lastVisibleEdge.getNext();

         if (edgeBeforeLineOfSight.distanceFromSupportLine(vertexToAdd) < constructionEpsilon)
         {
            firstVisibleEdge = edgeBeforeLineOfSight;
            lineOfSight.add(0, firstVisibleEdge);
         }
         else if (EuclidGeometryTools.distanceFromPoint3DToLine3D(edgeBeforeLineOfSight.getOrigin(), vertexToAdd,
                                                                  edgeBeforeLineOfSight.getDestination()) < constructionEpsilon)
         { // Sometimes edgeBeforeLineOfSight is really small, in which case the previous test may not pass while the edge should be extended.
            firstVisibleEdge = edgeBeforeLineOfSight;
            lineOfSight.add(0, firstVisibleEdge);
         }

         if (edgeAfterLineOfSight.distanceFromSupportLine(vertexToAdd) < constructionEpsilon)
         {
            lastVisibleEdge = edgeAfterLineOfSight;
            lineOfSight.add(lastVisibleEdge);
         }
         else if (EuclidGeometryTools.distanceFromPoint3DToLine3D(edgeAfterLineOfSight.getDestination(), vertexToAdd,
                                                                  edgeAfterLineOfSight.getOrigin()) < constructionEpsilon)
         { // Sometimes edgeAfterLineOfSight is really small, in which case the previous test may not pass while the edge should be extended.
            lastVisibleEdge = edgeAfterLineOfSight;
            lineOfSight.add(lastVisibleEdge);
         }

         if (lineOfSight.size() == 1)
         {
            HalfEdge3D additionalEdge = new HalfEdge3D(vertexToAdd, firstVisibleEdge.getDestination());
            additionalEdge.setFace(this);
            firstVisibleEdge.setDestination(vertexToAdd);
            additionalEdge.setNext(firstVisibleEdge.getNext());
            firstVisibleEdge.getNext().setPrevious(additionalEdge);
            firstVisibleEdge.setNext(additionalEdge);
            additionalEdge.setPrevious(firstVisibleEdge);
            // Clear the twin information to force the update.
            firstVisibleEdge.setTwin(null);
            edges.add(additionalEdge);
         }
         else
         {
            if (faceVertexToLock != null)
            {
               for (int i = 1; i < lineOfSight.size(); i++)
               {
                  if (lineOfSight.get(i).getOrigin() == faceVertexToLock)
                     return false;
               }
            }

            firstVisibleEdge.setDestination(vertexToAdd);
            lastVisibleEdge.setOrigin(vertexToAdd);
            firstVisibleEdge.setNext(lastVisibleEdge);
            lastVisibleEdge.setPrevious(firstVisibleEdge);
            // Clear the twin information to force the update.
            firstVisibleEdge.setTwin(null);
            lastVisibleEdge.setTwin(null);

            for (int i = 1; i < lineOfSight.size() - 1; i++)
            {
               HalfEdge3D edgeToRemove = lineOfSight.get(i);
               edgeToRemove.destroy();
               edges.remove(edgeToRemove);
            }
         }
      }

      HalfEdge3D startEdge = edges.get(0);
      edges.clear();
      edges.add(startEdge);

      HalfEdge3D currentEdge = startEdge.getNext();
      while (currentEdge != startEdge)
      {
         edges.add(currentEdge);
         currentEdge = currentEdge.getNext();
      }

      updateVertices();
      updateNormal();
      updateCentroidAndArea();

      boundingBox.updateToIncludePoint(vertexToAdd);
      return true;
   }

   public void updateVertices()
   {
      vertices.clear();
      edges.forEach(edge -> vertices.add(edge.getOrigin()));
   }

   public void updateNormal()
   {
      if (vertices.size() > 3)
      {
         EuclidPolytopeConstructionTools.updateFace3DNormal(vertices, null, normal);
      }
      else if (vertices.size() == 3)
      {
         EuclidGeometryTools.normal3DFromThreePoint3Ds(vertices.get(0), vertices.get(2), vertices.get(1), normal);
      }
      else if (vertices.size() == 2)
      { // Redirect the normal so it is orthogonal to the edge.
         Vector3DBasics edgeDirection = getEdge(0).getDirection(false);
         normal.cross(edgeDirection, normal);
         normal.cross(normal, edgeDirection);
         normal.normalize();
      }
   }

   public void updateCentroidAndArea()
   {
      area = EuclidPolytopeConstructionTools.computeConvexPolygon3DArea(vertices, normal, vertices.size(), true, centroid);
   }

   public void refreshBoundingBox()
   {
      boundingBox.setToNaN();

      for (int i = 0; i < vertices.size(); i++)
      {
         boundingBox.updateToIncludePoint(vertices.get(i));
      }
   }

   public void flip()
   {
      for (int i = 0; i < edges.size(); i++)
      {
         edges.get(i).flip();
      }
      Collections.reverse(edges);
      updateVertices();
      normal.negate();
   }

   public void destroy()
   {
      for (int i = 0; i < edges.size(); i++)
      {
         edges.get(i).destroy();
      }
      edges.clear();
      normal.setToNaN();
      centroid.setToNaN();
      boundingBox.setToNaN();
      vertices.clear();
      area = Double.NaN;
   }

   @Override
   public List<HalfEdge3D> lineOfSight(Point3DReadOnly observer)
   {
      List<HalfEdge3D> lineOfSight = new ArrayList<>();

      HalfEdge3D edgeUnderConsideration = lineOfSightStart(observer);

      for (int i = 0; edgeUnderConsideration != null && i < edges.size(); i++)
      {
         lineOfSight.add(edgeUnderConsideration);
         edgeUnderConsideration = edgeUnderConsideration.getNext();
         if (!canObserverSeeEdge(observer, edgeUnderConsideration))
            break;
      }

      return lineOfSight;
   }

   @SuppressWarnings("unchecked")
   @Override
   public List<HalfEdge3D> lineOfSight(Point3DReadOnly observer, double epsilon)
   {
      return (List<HalfEdge3D>) Face3DReadOnly.super.lineOfSight(observer, epsilon);
   }

   @Override
   public HalfEdge3D lineOfSightStart(Point3DReadOnly observer)
   {
      return (HalfEdge3D) Face3DReadOnly.super.lineOfSightStart(observer);
   }

   @Override
   public HalfEdge3D lineOfSightEnd(Point3DReadOnly observer)
   {
      return (HalfEdge3D) Face3DReadOnly.super.lineOfSightEnd(observer);
   }

   @Override
   public Face3D getNeighbor(int index)
   {
      return (Face3D) Face3DReadOnly.super.getNeighbor(index);
   }

   @Override
   public HalfEdge3D getCommonEdgeWith(Face3DReadOnly neighbor)
   {
      return (HalfEdge3D) Face3DReadOnly.super.getCommonEdgeWith(neighbor);
   }

   @Override
   public HalfEdge3D getClosestEdge(Point3DReadOnly point)
   {
      return (HalfEdge3D) Face3DReadOnly.super.getClosestEdge(point);
   }

   @Override
   public HalfEdge3D getClosestVisibleEdge(Point3DReadOnly point)
   {
      return (HalfEdge3D) Face3DReadOnly.super.getClosestVisibleEdge(point);
   }

   @Override
   public List<Vertex3D> getVertices()
   {
      return vertices;
   }

   @Override
   public Vertex3D getVertex(int index)
   {
      return vertices.get(index);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public List<HalfEdge3D> getEdges()
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

   @Override
   public Point3D getCentroid()
   {
      return centroid;
   }

   @Override
   public Vector3D getNormal()
   {
      return normal;
   }

   @Override
   public double getArea()
   {
      return area;
   }

   @Override
   public BoundingBox3DReadOnly getBoundingBox()
   {
      return boundingBox;
   }

   @Override
   public boolean containsNaN()
   {
      return Face3DReadOnly.super.containsNaN();
   }

   @Override
   public void setToNaN()
   {
      for (int i = 0; i < edges.size(); i++)
         edges.get(i).setToNaN();
      centroid.setToNaN();
      normal.setToNaN();
      area = Double.NaN;
      boundingBox.setToNaN();
   }

   @Override
   public void setToZero()
   {
      for (int i = 0; i < edges.size(); i++)
         edges.get(i).setToZero();
      centroid.setToZero();
      normal.setToZero();
      area = 0.0;
      boundingBox.setToZero();
   }

   @Override
   public void applyTransform(Transform transform)
   {
      for (int i = 0; i < getNumberOfEdges(); i++)
         edges.get(i).getOrigin().applyTransform(transform);
      centroid.applyTransform(transform);
      normal.applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      for (int i = 0; i < getNumberOfEdges(); i++)
         edges.get(i).getOrigin().applyInverseTransform(transform);
      centroid.applyInverseTransform(transform);
      normal.applyInverseTransform(transform);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      else if (object instanceof Face3DReadOnly)
         return Face3DReadOnly.super.equals((Face3DReadOnly) object);
      else
         return false;
   }

   @Override
   public int hashCode()
   {
      // Using ArrayList.hashCode() to combine the hash-codes of the vertices defining this face.
      long hashCode = vertices.hashCode();
      hashCode = EuclidHashCodeTools.combineHashCode(hashCode, centroid.hashCode());
      hashCode = EuclidHashCodeTools.combineHashCode(hashCode, normal.hashCode());
      return EuclidHashCodeTools.toIntHashCode(hashCode);
   }

   @Override
   public String toString()
   {
      return EuclidShapeIOTools.getFace3DString(this);
   }
}
