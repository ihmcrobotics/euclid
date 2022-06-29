package us.ihmc.euclid.shape.convexPolytope.impl;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DFactory;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeConstructionTools;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.SymmetricEigenDecomposition3D;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Base implementation of a face 3D that belongs to a convex polytope 3D.
 * <p>
 * This is part of a Doubly Connected Edge List data structure
 * <a href="https://en.wikipedia.org/wiki/Doubly_connected_edge_list"> link</a>.
 * </p>
 *
 * @author Apoorv Shrivastava
 * @author Sylvain Bertrand
 * @param <Vertex> the final type used for representing a vertex.
 * @param <Edge>   the final type used for representing a half-edge.
 * @param <Face>   the final type used for representing a face.
 */
public abstract class AbstractFace3D<Vertex extends AbstractVertex3D<Vertex, Edge, Face>, Edge extends AbstractHalfEdge3D<Vertex, Edge, Face>, Face extends AbstractFace3D<Vertex, Edge, Face>>
      implements Face3DReadOnly, Clearable, Transformable
{
   /** The list, in order, of the edges composing this face. */
   private final List<Edge> edges = new ArrayList<>();
   /** The list, in order, of the vertices composing this face. */
   private final List<Vertex> vertices = new ArrayList<>();
   /** The area of this face. */
   private double area;
   /**
    * Tolerance used when constructing a face. It is used to trigger a series of edge-cases including
    * for instance whether or not an edge should be extended.
    */
   private final double constructionEpsilon;
   /**
    * 3-by-3 covariance matrix computed from the vertices location and used to compute this face
    * normal.
    */
   private Matrix3D verticesCovariance;
   /** Eigen decomposition solver used to compute this face normal. */
   private SymmetricEigenDecomposition3D eigenDecomposition;
   /** Factory used to create half-edges of the proper type. */
   private final HalfEdge3DFactory<Vertex, Edge> edgeFactory;

   /**
    * Creates a new empty face.
    *
    * @param edgeFactory the factory to use for creating new edges when expanding this face.
    */
   public AbstractFace3D(HalfEdge3DFactory<Vertex, Edge> edgeFactory)
   {
      this(edgeFactory, EuclidPolytopeConstructionTools.DEFAULT_CONSTRUCTION_EPSILON);
   }

   /**
    * Creates a new empty face.
    *
    * @param edgeFactory         the factory to use for creating new edges when expanding this face.
    * @param constructionEpsilon tolerance used when adding vertices to a face to trigger a series of
    *                            edge-cases.
    */
   public AbstractFace3D(HalfEdge3DFactory<Vertex, Edge> edgeFactory, double constructionEpsilon)
   {
      this.constructionEpsilon = constructionEpsilon;
      this.edgeFactory = edgeFactory;
   }

   /**
    * Initializes internal memory for this empty face.
    * <p>
    * This is be invoked from the implementation's constructor.
    * </p>
    *
    * @param initialGuessNormal initial guess for what this face's normal should be. Not modified.
    */
   protected void initialize(Vector3DReadOnly initialGuessNormal)
   {
      getNormal().setAndNormalize(initialGuessNormal);
      getBoundingBox().setToNaN();
   }

   /**
    * Initializes this face with the given edges and normal.
    * <p>
    * This is be invoked from the implementation's constructor.
    * </p>
    *
    * @param faceEdges the edges that are to be used to setup this face.
    * @param normal    the pre-computed normal for this face.
    */
   protected void initialize(Collection<Edge> faceEdges, Vector3DReadOnly normal)
   {
      getBoundingBox().setToNaN();
      set(faceEdges, normal);
   }

   /**
    * Clears this face and sets its edges and normal.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param faceEdges the edges composing the new face. Not modified, reference to the edges saved.
    * @param normal    the face's normal. Not modified.
    */
   @SuppressWarnings("unchecked")
   public void set(Collection<Edge> faceEdges, Vector3DReadOnly normal)
   {
      edges.clear();
      edges.addAll(faceEdges);
      edges.forEach(edge -> edge.setFace((Face) this));
      getNormal().set(normal);
      updateVertices();
      updateNormal();
      updateCentroidAndArea();
      updateBoundingBox();

      for (Edge edge : faceEdges)
      {
         if (edge.getPrevious() == null)
            edge.setPrevious(faceEdges.stream().filter(e -> e.getDestination() == edge.getOrigin()).findFirst().get());
         if (edge.getNext() == null)
            edge.setNext(faceEdges.stream().filter(e -> e.getOrigin() == edge.getDestination()).findFirst().get());
      }
   }

   /**
    * Adds a vertex to the face and updates all the associations accordingly.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param vertexToAdd the vertex that is to be added to the face. Not modified, reference saved.
    * @return {@code true} if the vertex was added to this face, {@code false} if it was rejected.
    */
   public boolean addVertex(Vertex vertexToAdd)
   {
      return addVertex(vertexToAdd, null, false);
   }

   /**
    * Adds a vertex to the face and updates all the associations accordingly.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param vertexToAdd       the vertex that is to be added to the face. Not modified, reference
    *                          saved.
    * @param faceVertexToLock  (optional) vertex that already belongs to this face that must be
    *                          preserved when adding the new vertex. Not modified.
    * @param lockEdgesWithTwin enforces edges with a non {@code null} to be preserved when adding the
    *                          new vertex.
    * @return {@code true} if the vertex was added to this face, {@code false} if it was rejected.
    */
   public boolean addVertex(Vertex vertexToAdd, Vertex faceVertexToLock, boolean lockEdgesWithTwin)
   {
      boolean isFaceModified = false;

      if (edges.isEmpty())
      {
         isFaceModified |= handleNoEdgeCase(vertexToAdd);
      }
      else if (edges.size() == 1)
      {
         isFaceModified |= handleSingleEdgeCase(vertexToAdd);
      }
      else if (edges.size() == 2)
      {
         isFaceModified |= handleTwoEdgeCase(vertexToAdd, lockEdgesWithTwin);
      }
      else
      {
         isFaceModified |= handleMultipleEdgeCase(vertexToAdd, faceVertexToLock, lockEdgesWithTwin);
      }

      if (isFaceModified)
      {
         Edge startEdge = edges.get(0);
         edges.clear();
         edges.add(startEdge);

         Edge currentEdge = startEdge.getNext();
         while (currentEdge != startEdge)
         {
            edges.add(currentEdge);
            currentEdge = currentEdge.getNext();
         }

         updateVertices();
         updateNormal();
         updateCentroidAndArea();

         getBoundingBox().updateToIncludePoint(vertexToAdd);
      }

      return isFaceModified;
   }

   @SuppressWarnings("unchecked")
   private boolean handleNoEdgeCase(Vertex vertexToAdd)
   {
      Edge newEdge = edgeFactory.newInstance(vertexToAdd, vertexToAdd);
      newEdge.setFace((Face) this);
      newEdge.setNext(newEdge);
      newEdge.setPrevious(newEdge);
      edges.add(newEdge);
      return true;
   }

   @SuppressWarnings("unchecked")
   private boolean handleSingleEdgeCase(Vertex vertexToAdd)
   {
      Edge firstEdge = edges.get(0);
      if (firstEdge.getOrigin().geometricallyEquals(vertexToAdd, constructionEpsilon))
         return false;

      // Set the edge for the two points and then create its twin
      firstEdge.setDestination(vertexToAdd);
      Edge newEdge = edgeFactory.newInstance(vertexToAdd, firstEdge.getOrigin());
      newEdge.setFace((Face) this);
      newEdge.setNext(firstEdge);
      newEdge.setPrevious(firstEdge);
      firstEdge.setNext(newEdge);
      firstEdge.setPrevious(newEdge);
      edges.add(newEdge);
      return true;
   }

   @SuppressWarnings("unchecked")
   private boolean handleTwoEdgeCase(Vertex vertexToAdd, boolean lockEdgesWithTwin)
   {
      Edge firstEdge = edges.get(0);
      Edge secondEdge = edges.get(1);

      if (firstEdge.distance(vertexToAdd) < constructionEpsilon)
      {
         return false;
      }
      else if (!lockEdgesWithTwin || firstEdge.getTwin() == null)
      { // We need to check if either the origin or destination of the firstEdge would become unnecessary by adding the new vertex.
         if (firstEdge.getOrigin().distanceSquared(vertexToAdd) > firstEdge.getDestination().distanceSquared(vertexToAdd))
         {
            if (EuclidGeometryTools.distanceFromPoint3DToLineSegment3D(firstEdge.getDestination(), vertexToAdd, firstEdge.getOrigin()) < constructionEpsilon)
            {
               firstEdge.setDestination(vertexToAdd);
               secondEdge.setOrigin(vertexToAdd);
               return true;
            }
         }
         else
         {
            if (EuclidGeometryTools.distanceFromPoint3DToLineSegment3D(firstEdge.getOrigin(), vertexToAdd, firstEdge.getDestination()) < constructionEpsilon)
            {
               firstEdge.setOrigin(vertexToAdd);
               secondEdge.setDestination(vertexToAdd);
               return true;
            }
         }
      }

      // Ensuring clockwise ordering using the initial guess for the normal.
      Vector3D resultingNormal = EuclidPolytopeTools.crossProductOfLineSegment3Ds(firstEdge.getOrigin(),
                                                                                  firstEdge.getDestination(),
                                                                                  firstEdge.getDestination(),
                                                                                  vertexToAdd);
      if (resultingNormal.dot(getNormal()) > 0.0)
      { // Counter-clockwise, need to reverse the ordering.
         firstEdge.flip();
         secondEdge.setOrigin(firstEdge.getDestination());
      }

      secondEdge.setDestination(vertexToAdd);
      Edge newEdge = edgeFactory.newInstance(vertexToAdd, firstEdge.getOrigin());
      newEdge.setFace((Face) this);
      newEdge.setPrevious(secondEdge);
      newEdge.setNext(firstEdge);
      firstEdge.setPrevious(newEdge);
      secondEdge.setNext(newEdge);
      edges.add(newEdge);
      return true;
   }

   @SuppressWarnings("unchecked")
   private boolean handleMultipleEdgeCase(Vertex vertexToAdd, Vertex faceVertexToLock, boolean lockEdgesWithTwin)
   {
      List<Edge> lineOfSight = lineOfSight(vertexToAdd);

      if (lineOfSight.isEmpty())
         return false;

      Edge firstVisibleEdge = lineOfSight.get(0);

      if (lineOfSight.size() == 1 && firstVisibleEdge.distance(vertexToAdd) < constructionEpsilon)
         return false;

      Edge lastVisibleEdge = lineOfSight.get(lineOfSight.size() - 1);

      Edge edgeBeforeLineOfSight = firstVisibleEdge.getPrevious();
      Edge edgeAfterLineOfSight = lastVisibleEdge.getNext();

      if (edgeBeforeLineOfSight.distanceFromSupportLine(vertexToAdd) < constructionEpsilon)
      {
         firstVisibleEdge = edgeBeforeLineOfSight;
         lineOfSight.add(0, firstVisibleEdge);
      }
      else if (EuclidGeometryTools.distanceFromPoint3DToLine3D(edgeBeforeLineOfSight.getDestination(),
                                                               vertexToAdd,
                                                               edgeBeforeLineOfSight.getOrigin()) < constructionEpsilon)
      { // Sometimes edgeBeforeLineOfSight is really small, in which case the previous test may not pass while the edge should be extended.
         firstVisibleEdge = edgeBeforeLineOfSight;
         lineOfSight.add(0, firstVisibleEdge);
      }

      if (edgeAfterLineOfSight.distanceFromSupportLine(vertexToAdd) < constructionEpsilon)
      {
         lastVisibleEdge = edgeAfterLineOfSight;
         lineOfSight.add(lastVisibleEdge);
      }
      else if (EuclidGeometryTools.distanceFromPoint3DToLine3D(edgeAfterLineOfSight.getOrigin(),
                                                               vertexToAdd,
                                                               edgeAfterLineOfSight.getDestination()) < constructionEpsilon)
      { // Sometimes edgeAfterLineOfSight is really small, in which case the previous test may not pass while the edge should be extended.
         lastVisibleEdge = edgeAfterLineOfSight;
         lineOfSight.add(lastVisibleEdge);
      }

      if (lockEdgesWithTwin)
      {
         for (int i = 0; i < lineOfSight.size(); i++)
         {
            if (lineOfSight.get(i).getTwin() != null)
               return false;
         }
      }

      if (lineOfSight.size() == 1)
      {
         Edge additionalEdge = edgeFactory.newInstance(vertexToAdd, firstVisibleEdge.getDestination());
         additionalEdge.setFace((Face) this);
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
            Edge edgeToRemove = lineOfSight.get(i);
            edgeToRemove.destroy();
            edges.remove(edgeToRemove);
         }
      }

      return true;
   }

   private void updateVertices()
   {
      vertices.clear();
      edges.forEach(edge -> vertices.add(edge.getOrigin()));
   }

   /**
    * Computes and updates the normal of the supporting plane of this face.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * There is usually no need to refresh a face properties as they are automatically updated when
    * adding a new vertex.
    * </p>
    */
   public void updateNormal()
   {
      if (vertices.size() > 3)
      {
         if (verticesCovariance == null)
            verticesCovariance = new Matrix3D();
         EuclidPolytopeConstructionTools.computeCovariance3D(vertices, verticesCovariance);
         if (eigenDecomposition == null)
            eigenDecomposition = new SymmetricEigenDecomposition3D();
         EuclidPolytopeConstructionTools.updateFace3DNormal(eigenDecomposition, verticesCovariance, getNormal());
      }
      else if (vertices.size() == 3)
      {
         EuclidGeometryTools.normal3DFromThreePoint3Ds(vertices.get(0), vertices.get(2), vertices.get(1), getNormal());
      }
      else if (vertices.size() == 2)
      { // Redirect the normal so it is orthogonal to the edge.
         Vector3DBasics edgeDirection = getEdge(0).getDirection(false);
         getNormal().cross(edgeDirection, getNormal());
         getNormal().cross(getNormal(), edgeDirection);
         getNormal().normalize();
      }
   }

   /**
    * Computes and updates the centroid and area of this face.
    * <p>
    * There is usually no need to refresh a face properties as they are automatically updated when
    * adding a new vertex.
    * </p>
    */
   public void updateCentroidAndArea()
   {
      area = EuclidPolytopeConstructionTools.computeConvexPolygon3DArea(vertices, getNormal(), vertices.size(), true, getCentroid());
   }

   /**
    * Re-evaluates the bounding box of this face.
    * <p>
    * There is usually no need to refresh a face properties as they are automatically updated when
    * adding a new vertex.
    * </p>
    */
   public void updateBoundingBox()
   {
      getBoundingBox().setToNaN();

      for (int i = 0; i < vertices.size(); i++)
      {
         getBoundingBox().updateToIncludePoint(vertices.get(i));
      }
   }

   /**
    * Flips this face such that its normal points towards the opposite direction.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * The edges and vertices are updated to preserve clockwise ordering.
    * </p>
    */
   public void flip()
   {
      for (int i = 0; i < edges.size(); i++)
      {
         edges.get(i).flip();
      }
      Collections.reverse(edges);
      updateVertices();
      getNormal().negate();
   }

   /**
    * Destroys this face and its edges.
    * <p>
    * Besides invalidating internal data, this method remove the connection between this face's
    * features and the rest of the convex polytope that owned this face.
    * </p>
    */
   public void destroy()
   {
      for (int i = 0; i < edges.size(); i++)
      {
         edges.get(i).destroy();
      }
      edges.clear();
      getNormal().setToNaN();
      getCentroid().setToNaN();
      getBoundingBox().setToNaN();
      vertices.clear();
      area = Double.NaN;
   }

   /** {@inheritDoc} */
   @SuppressWarnings("unchecked")
   @Override
   public List<Edge> lineOfSight(Point3DReadOnly observer)
   {
      return (List<Edge>) Face3DReadOnly.super.lineOfSight(observer);
   }

   /** {@inheritDoc} */
   @SuppressWarnings("unchecked")
   @Override
   public List<Edge> lineOfSight(Point3DReadOnly observer, double epsilon)
   {
      return (List<Edge>) Face3DReadOnly.super.lineOfSight(observer, epsilon);
   }

   /** {@inheritDoc} */
   @SuppressWarnings("unchecked")
   @Override
   public Edge lineOfSightStart(Point3DReadOnly observer)
   {
      return (Edge) Face3DReadOnly.super.lineOfSightStart(observer);
   }

   /** {@inheritDoc} */
   @SuppressWarnings("unchecked")
   @Override
   public Edge lineOfSightEnd(Point3DReadOnly observer)
   {
      return (Edge) Face3DReadOnly.super.lineOfSightEnd(observer);
   }

   /** {@inheritDoc} */
   @SuppressWarnings("unchecked")
   @Override
   public Face getNeighbor(int index)
   {
      return (Face) Face3DReadOnly.super.getNeighbor(index);
   }

   /** {@inheritDoc} */
   @SuppressWarnings("unchecked")
   @Override
   public Edge getCommonEdgeWith(Face3DReadOnly neighbor)
   {
      return (Edge) Face3DReadOnly.super.getCommonEdgeWith(neighbor);
   }

   /** {@inheritDoc} */
   @SuppressWarnings("unchecked")
   @Override
   public Edge getClosestEdge(Point3DReadOnly point)
   {
      return (Edge) Face3DReadOnly.super.getClosestEdge(point);
   }

   /** {@inheritDoc} */
   @SuppressWarnings("unchecked")
   @Override
   public Edge getClosestVisibleEdge(Point3DReadOnly point)
   {
      return (Edge) Face3DReadOnly.super.getClosestVisibleEdge(point);
   }

   /** {@inheritDoc} */
   @Override
   public List<Vertex> getVertices()
   {
      return vertices;
   }

   /** {@inheritDoc} */
   @Override
   public Vertex getVertex(int index)
   {
      return vertices.get(index);
   }

   /** {@inheritDoc} */
   @Override
   public List<Edge> getEdges()
   {
      return edges;
   }

   /** {@inheritDoc} */
   @Override
   public Edge getEdge(int index)
   {
      return edges.get(index);
   }

   /** {@inheritDoc} */
   @Override
   public abstract Point3DBasics getCentroid();

   /** {@inheritDoc} */
   @Override
   public abstract Vector3DBasics getNormal();

   /** {@inheritDoc} */
   @Override
   public double getArea()
   {
      return area;
   }

   /** {@inheritDoc} */
   @Override
   public abstract BoundingBox3DBasics getBoundingBox();

   /** {@inheritDoc} */
   @Override
   public boolean containsNaN()
   {
      return Face3DReadOnly.super.containsNaN();
   }

   /** {@inheritDoc} */
   @Override
   public void setToNaN()
   {
      edges.clear();
      vertices.clear();
      getCentroid().setToNaN();
      getNormal().setToNaN();
      area = Double.NaN;
      getBoundingBox().setToNaN();
   }

   /** {@inheritDoc} */
   @Override
   public void setToZero()
   {
      edges.clear();
      vertices.clear();
      getCentroid().setToZero();
      getNormal().setToZero();
      area = 0.0;
      getBoundingBox().setToZero();
   }

   /** {@inheritDoc} */
   @Override
   public void applyTransform(Transform transform)
   {
      for (int i = 0; i < getNumberOfEdges(); i++)
         edges.get(i).getOrigin().applyTransform(transform);
      getCentroid().applyTransform(transform);
      getNormal().applyTransform(transform);
   }

   /** {@inheritDoc} */
   @Override
   public void applyInverseTransform(Transform transform)
   {
      for (int i = 0; i < getNumberOfEdges(); i++)
         edges.get(i).getOrigin().applyInverseTransform(transform);
      getCentroid().applyInverseTransform(transform);
      getNormal().applyInverseTransform(transform);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(Face3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Face3DReadOnly)
         return equals((Face3DReadOnly) object);
      else
         return false;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this face 3D.
    *
    * @return the hash code value for this face 3D.
    */
   @Override
   public int hashCode()
   {
      // Using ArrayList.hashCode() to combine the hash-codes of the vertices defining this face.
      return vertices.hashCode();
   }

   /**
    * Provides a {@code String} representation of this face 3D as follows:
    *
    * <pre>
    * Face 3D: centroid: ( 2.621, -0.723, -1.355 ), normal: ( 0.903, -0.202,  0.378 ), area:  0.180, number of edges: 4
    *    [( 2.590, -0.496, -1.161 ); ( 2.746, -0.536, -1.554 )]
    *    [( 2.746, -0.536, -1.554 ); ( 2.651, -0.950, -1.549 )]
    *    [( 2.651, -0.950, -1.549 ); ( 2.496, -0.910, -1.157 )]
    *    [( 2.496, -0.910, -1.157 ); ( 2.590, -0.496, -1.161 )]
    * </pre>
    *
    * @return the {@code String} representing this face 3D.
    */
   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
