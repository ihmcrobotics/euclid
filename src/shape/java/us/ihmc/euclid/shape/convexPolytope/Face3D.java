package us.ihmc.euclid.shape.convexPolytope;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeIOTools;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
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

   // Temporary variables for calculations
   private boolean marked = false;

   public Face3D(Vector3DReadOnly initialGuessNormal)
   {
      normal.setAndNormalize(initialGuessNormal);
   }

   /**
    * Adds a vertex to the face and updates all the associations accordingly
    *
    * @param vertexToAdd the vertex that must be added to the face
    * @param epsilon
    */
   public void addVertex(Vertex3D vertexToAdd, double epsilon)
   {
      if (edges.isEmpty())
      {
         HalfEdge3D newEdge = new HalfEdge3D(vertexToAdd, vertexToAdd);
         newEdge.setFace(this);
         newEdge.setNextEdge(newEdge);
         newEdge.setPreviousEdge(newEdge);
         edges.add(newEdge);
      }
      else if (edges.size() == 1)
      {
         HalfEdge3D firstEdge = edges.get(0);
         if (firstEdge.getOrigin().epsilonEquals(vertexToAdd, epsilon))
            return;

         // Set the edge for the two points and then create its twin
         firstEdge.setDestination(vertexToAdd);
         HalfEdge3D newEdge = new HalfEdge3D(vertexToAdd, firstEdge.getOrigin());
         newEdge.setFace(this);
         newEdge.setNextEdge(firstEdge);
         newEdge.setPreviousEdge(firstEdge);
         firstEdge.setNextEdge(newEdge);
         firstEdge.setPreviousEdge(newEdge);
         edges.add(newEdge);
      }
      else if (edges.size() == 2)
      {
         HalfEdge3D firstEdge = edges.get(0);
         if (firstEdge.getOrigin().epsilonEquals(vertexToAdd, epsilon) || firstEdge.getDestination().epsilonEquals(vertexToAdd, epsilon))
            return;

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
         HalfEdge3D newEdge = new HalfEdge3D(vertexToAdd, firstEdge.getOrigin());
         newEdge.setFace(this);
         newEdge.setNextEdge(firstEdge);
         firstEdge.setPreviousEdge(newEdge);
         newEdge.setPreviousEdge(secondEdge);
         secondEdge.setNextEdge(newEdge);
         edges.add(newEdge);
      }
      else
      {
         List<HalfEdge3D> lineOfSight = lineOfSight(vertexToAdd);

         if (lineOfSight.isEmpty())
            return;

         HalfEdge3D firstVisibleEdge = lineOfSight.get(0);
         HalfEdge3D lastVisibleEdge = lineOfSight.get(lineOfSight.size() - 1);

         if (lineOfSight.size() == 1)
         {
            if (firstVisibleEdge.getOrigin().epsilonEquals(vertexToAdd, epsilon))
               return;
            if (firstVisibleEdge.getDestination().epsilonEquals(vertexToAdd, epsilon))
               return;

            HalfEdge3D additionalEdge = new HalfEdge3D(vertexToAdd, firstVisibleEdge.getDestination());
            additionalEdge.setFace(this);
            firstVisibleEdge.setDestination(vertexToAdd);
            additionalEdge.setNextEdge(firstVisibleEdge.getNextEdge());
            firstVisibleEdge.getNextEdge().setPreviousEdge(additionalEdge);
            firstVisibleEdge.setNextEdge(additionalEdge);
            additionalEdge.setPreviousEdge(firstVisibleEdge);
            edges.add(additionalEdge);
         }
         else
         {
            firstVisibleEdge.setDestination(vertexToAdd);
            lastVisibleEdge.setOrigin(vertexToAdd);
            firstVisibleEdge.setNextEdge(lastVisibleEdge);
            lastVisibleEdge.setPreviousEdge(firstVisibleEdge);

            for (int i = 1; i < lineOfSight.size() - 1; i++)
               edges.remove(lineOfSight.get(i));
         }
      }

      HalfEdge3D startEdge = edges.get(0);
      edges.clear();
      edges.add(startEdge);

      HalfEdge3D currentEdge = startEdge.getNextEdge();
      while (currentEdge != startEdge)
      {
         edges.add(currentEdge);
         currentEdge = currentEdge.getNextEdge();
      }

      List<? extends Vertex3DReadOnly> vertices = getVertices();

      if (vertices.size() > 3)
      {
         EuclidPolytopeTools.updateFace3DNormal(vertices, null, normal);
      }
      else
      {
         if (vertices.size() == 3)
            EuclidGeometryTools.normal3DFromThreePoint3Ds(vertices.get(0), vertices.get(2), vertices.get(1), normal);
      }

      area = EuclidPolytopeTools.computeConvexPolygon3DArea(vertices, normal, vertices.size(), true, centroid);

      boundingBox.updateToIncludePoint(vertexToAdd);
   }

   public void flip()
   {
      for (int i = 0; i < edges.size(); i++)
      {
         edges.get(i).flip();
      }
      Collections.reverse(edges);
      normal.negate();
   }

   public void destroy()
   {
      for (int i = 0; i < edges.size(); i++)
      {
         edges.get(i).detroy();
      }
      edges.clear();
   }

   @Override
   public List<HalfEdge3D> lineOfSight(Point3DReadOnly observer)
   {
      List<HalfEdge3D> lineOfSight = new ArrayList<>();

      HalfEdge3D edgeUnderConsideration = lineOfSightStart(observer);

      for (int i = 0; edgeUnderConsideration != null && i < edges.size(); i++)
      {
         lineOfSight.add(edgeUnderConsideration);
         edgeUnderConsideration = edgeUnderConsideration.getNextEdge();
         if (!canObserverSeeEdge(observer, edgeUnderConsideration))
            break;
      }

      return lineOfSight;
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
   public Face3D getNeighbouringFace(int index)
   {
      return (Face3D) Face3DReadOnly.super.getNeighbouringFace(index);
   }

   @Override
   public HalfEdge3D getClosestEdge(Point3DReadOnly point)
   {
      return (HalfEdge3D) Face3DReadOnly.super.getClosestEdge(point);
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
   }

   @Override
   public void setToZero()
   {
      for (int i = 0; i < edges.size(); i++)
         edges.get(i).setToZero();
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
   public String toString()
   {
      return EuclidPolytopeIOTools.getFace3DString(this);
   }
}
