package us.ihmc.euclid.referenceFrame.polytope;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.EigenDecomposition;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameFace3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeConstructionTools;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTools;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public class FrameFace3D implements FrameFace3DReadOnly, Clearable, Transformable
{
   private final ReferenceFrameHolder referenceFrameHolder;
   /** The list, in order, of the edges composing this face. */
   private final List<FrameHalfEdge3D> edges = new ArrayList<>();
   /** The list, in order, of the vertices composing this face. */
   private final List<FrameVertex3D> vertices = new ArrayList<>();
   /** The normal vector of the support plane of this face. */
   private final FixedFrameVector3DBasics normal = EuclidFrameFactories.newFixedFrameVector3DBasics(this);
   /** The centroid of this face. */
   private final FixedFramePoint3DBasics centroid = EuclidFrameFactories.newFixedFramePoint3DBasics(this);
   /** The area of this face. */
   private double area;
   /** The tightest bounding box entirely containing this face. */
   private final FixedFrameBoundingBox3DBasics boundingBox = EuclidFrameFactories.newFixedFrameBoundingBox3DBasics(this);
   /**
    * Tolerance used when constructing a face. It is used to trigger a series of edge-cases including
    * for instance whether or not an edge should be extended.
    */
   private final double constructionEpsilon;
   /**
    * 3-by-3 covariance matrix computed from the vertices location and used to compute this face
    * normal.
    */
   private final DenseMatrix64F verticesCovariance = new DenseMatrix64F(3, 3);
   /** Eigen decomposition solver used to compute this face normal. */
   private final EigenDecomposition<DenseMatrix64F> eigenDecomposition = DecompositionFactory.eig(3, true, true);

   /**
    * Creates a new empty face.
    *
    * @param initialGuessNormal initial guess for what this face's normal should be. Not modified.
    */
   public FrameFace3D(ReferenceFrameHolder referenceFrameHolder, FrameVector3DReadOnly initialGuessNormal)
   {
      this(referenceFrameHolder, initialGuessNormal, EuclidPolytopeConstructionTools.DEFAULT_CONSTRUCTION_EPSILON);
   }

   /**
    * Creates a new empty face.
    *
    * @param initialGuessNormal  initial guess for what this face's normal should be. Not modified.
    * @param constructionEpsilon tolerance used when adding vertices to a face to trigger a series of
    *                            edge-cases.
    */
   public FrameFace3D(ReferenceFrameHolder referenceFrameHolder, FrameVector3DReadOnly initialGuessNormal, double constructionEpsilon)
   {
      this.referenceFrameHolder = referenceFrameHolder;
      normal.setAndNormalize(initialGuessNormal);
      this.constructionEpsilon = constructionEpsilon;
      boundingBox.setToNaN();
   }

   /**
    * Creates a new face given its edges.
    *
    * @param faceEdges           the edges composing the new face. Not modified, reference to the edges
    *                            saved.
    * @param normal              the face's normal. Not modified.
    * @param constructionEpsilon tolerance used when adding vertices to a face to trigger a series of
    *                            edge-cases.
    */
   public FrameFace3D(ReferenceFrameHolder referenceFrameHolder, Collection<FrameHalfEdge3D> faceEdges, FrameVector3DReadOnly normal, double constructionEpsilon)
   {
      this.referenceFrameHolder = referenceFrameHolder;
      boundingBox.setToNaN();
      this.constructionEpsilon = constructionEpsilon;

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
   public void set(Collection<FrameHalfEdge3D> faceEdges, FrameVector3DReadOnly normal)
   {
      edges.clear();
      edges.addAll(faceEdges);
      edges.forEach(edge -> edge.setFace(this));
      this.normal.set(normal);
      updateVertices();
      updateNormal();
      updateCentroidAndArea();
      refreshBoundingBox();

      for (FrameHalfEdge3D edge : faceEdges)
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
   public boolean addVertex(FrameVertex3D vertexToAdd)
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
   public boolean addVertex(FrameVertex3D vertexToAdd, FrameVertex3D faceVertexToLock, boolean lockEdgesWithTwin)
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
         FrameHalfEdge3D startEdge = edges.get(0);
         edges.clear();
         edges.add(startEdge);

         FrameHalfEdge3D currentEdge = startEdge.getNext();
         while (currentEdge != startEdge)
         {
            edges.add(currentEdge);
            currentEdge = currentEdge.getNext();
         }

         updateVertices();
         updateNormal();
         updateCentroidAndArea();

         boundingBox.updateToIncludePoint(vertexToAdd);
      }

      return isFaceModified;
   }

   private boolean handleNoEdgeCase(FrameVertex3D vertexToAdd)
   {
      FrameHalfEdge3D newEdge = new FrameHalfEdge3D(referenceFrameHolder, vertexToAdd, vertexToAdd);
      newEdge.setFace(this);
      newEdge.setNext(newEdge);
      newEdge.setPrevious(newEdge);
      edges.add(newEdge);
      return true;
   }

   private boolean handleSingleEdgeCase(FrameVertex3D vertexToAdd)
   {
      FrameHalfEdge3D firstEdge = edges.get(0);
      if (firstEdge.getOrigin().geometricallyEquals(vertexToAdd, constructionEpsilon))
         return false;

      // Set the edge for the two points and then create its twin
      firstEdge.setDestination(vertexToAdd);
      FrameHalfEdge3D newEdge = new FrameHalfEdge3D(referenceFrameHolder, vertexToAdd, firstEdge.getOrigin());
      newEdge.setFace(this);
      newEdge.setNext(firstEdge);
      newEdge.setPrevious(firstEdge);
      firstEdge.setNext(newEdge);
      firstEdge.setPrevious(newEdge);
      edges.add(newEdge);
      return true;
   }

   private boolean handleTwoEdgeCase(FrameVertex3D vertexToAdd, boolean lockEdgesWithTwin)
   {
      FrameHalfEdge3D firstEdge = edges.get(0);
      FrameHalfEdge3D secondEdge = edges.get(1);

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
      if (resultingNormal.dot(normal) > 0.0)
      { // Counter-clockwise, need to reverse the ordering.
         firstEdge.flip();
         secondEdge.setOrigin(firstEdge.getDestination());
      }

      secondEdge.setDestination(vertexToAdd);
      FrameHalfEdge3D newEdge = new FrameHalfEdge3D(referenceFrameHolder, vertexToAdd, firstEdge.getOrigin());
      newEdge.setFace(this);
      newEdge.setPrevious(secondEdge);
      newEdge.setNext(firstEdge);
      firstEdge.setPrevious(newEdge);
      secondEdge.setNext(newEdge);
      edges.add(newEdge);
      return true;
   }

   private boolean handleMultipleEdgeCase(FrameVertex3D vertexToAdd, FrameVertex3D faceVertexToLock, boolean lockEdgesWithTwin)
   {
      List<FrameHalfEdge3D> lineOfSight = lineOfSight(vertexToAdd);

      if (lineOfSight.isEmpty())
         return false;

      FrameHalfEdge3D firstVisibleEdge = lineOfSight.get(0);

      if (lineOfSight.size() == 1 && firstVisibleEdge.distance(vertexToAdd) < constructionEpsilon)
         return false;

      FrameHalfEdge3D lastVisibleEdge = lineOfSight.get(lineOfSight.size() - 1);

      FrameHalfEdge3D edgeBeforeLineOfSight = firstVisibleEdge.getPrevious();
      FrameHalfEdge3D edgeAfterLineOfSight = lastVisibleEdge.getNext();

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
         FrameHalfEdge3D additionalEdge = new FrameHalfEdge3D(referenceFrameHolder, vertexToAdd, firstVisibleEdge.getDestination());
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
            FrameHalfEdge3D edgeToRemove = lineOfSight.get(i);
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
         EuclidPolytopeConstructionTools.computeCovariance3D(vertices, verticesCovariance);
         EuclidPolytopeConstructionTools.updateFace3DNormal(eigenDecomposition, verticesCovariance, normal);
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

   /**
    * Computes and updates the centroid and area of this face.
    * <p>
    * There is usually no need to refresh a face properties as they are automatically updated when
    * adding a new vertex.
    * </p>
    */
   public void updateCentroidAndArea()
   {
      area = EuclidPolytopeConstructionTools.computeConvexPolygon3DArea(vertices, normal, vertices.size(), true, centroid);
   }

   /**
    * Re-evaluates the bounding box of this face.
    * <p>
    * There is usually no need to refresh a face properties as they are automatically updated when
    * adding a new vertex.
    * </p>
    */
   public void refreshBoundingBox()
   {
      boundingBox.setToNaN();

      for (int i = 0; i < vertices.size(); i++)
      {
         boundingBox.updateToIncludePoint(vertices.get(i));
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
      normal.negate();
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
      normal.setToNaN();
      centroid.setToNaN();
      boundingBox.setToNaN();
      vertices.clear();
      area = Double.NaN;
   }

   /** {@inheritDoc} */
   @SuppressWarnings("unchecked")
   @Override
   public List<FrameHalfEdge3D> lineOfSight(Point3DReadOnly observer)
   {
      return (List<FrameHalfEdge3D>) FrameFace3DReadOnly.super.lineOfSight(observer);
   }

   @SuppressWarnings("unchecked")
   @Override
   public List<FrameHalfEdge3D> lineOfSight(FramePoint3DReadOnly observer)
   {
      return (List<FrameHalfEdge3D>) FrameFace3DReadOnly.super.lineOfSight(observer);
   }

   /** {@inheritDoc} */
   @SuppressWarnings("unchecked")
   @Override
   public List<FrameHalfEdge3D> lineOfSight(Point3DReadOnly observer, double epsilon)
   {
      return (List<FrameHalfEdge3D>) FrameFace3DReadOnly.super.lineOfSight(observer, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   public FrameHalfEdge3D lineOfSightStart(Point3DReadOnly observer)
   {
      return (FrameHalfEdge3D) FrameFace3DReadOnly.super.lineOfSightStart(observer);
   }

   /** {@inheritDoc} */
   @Override
   public FrameHalfEdge3D lineOfSightEnd(Point3DReadOnly observer)
   {
      return (FrameHalfEdge3D) FrameFace3DReadOnly.super.lineOfSightEnd(observer);
   }

   /** {@inheritDoc} */
   @Override
   public FrameFace3D getNeighbor(int index)
   {
      return (FrameFace3D) FrameFace3DReadOnly.super.getNeighbor(index);
   }

   /** {@inheritDoc} */
   @Override
   public FrameHalfEdge3D getCommonEdgeWith(Face3DReadOnly neighbor)
   {
      return (FrameHalfEdge3D) FrameFace3DReadOnly.super.getCommonEdgeWith(neighbor);
   }

   /** {@inheritDoc} */
   @Override
   public FrameHalfEdge3D getClosestEdge(Point3DReadOnly point)
   {
      return (FrameHalfEdge3D) FrameFace3DReadOnly.super.getClosestEdge(point);
   }

   /** {@inheritDoc} */
   @Override
   public FrameHalfEdge3D getClosestVisibleEdge(Point3DReadOnly point)
   {
      return (FrameHalfEdge3D) FrameFace3DReadOnly.super.getClosestVisibleEdge(point);
   }

   /** {@inheritDoc} */
   @Override
   public List<FrameVertex3D> getVertices()
   {
      return vertices;
   }

   /** {@inheritDoc} */
   @Override
   public FrameVertex3D getVertex(int index)
   {
      return vertices.get(index);
   }

   /** {@inheritDoc} */
   @Override
   public List<FrameHalfEdge3D> getEdges()
   {
      return edges;
   }

   /** {@inheritDoc} */
   @Override
   public FrameHalfEdge3D getEdge(int index)
   {
      return edges.get(index);
   }

   /** {@inheritDoc} */
   @Override
   public FixedFramePoint3DBasics getCentroid()
   {
      return centroid;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameVector3DBasics getNormal()
   {
      return normal;
   }

   /** {@inheritDoc} */
   @Override
   public double getArea()
   {
      return area;
   }

   /** {@inheritDoc} */
   @Override
   public FrameBoundingBox3DReadOnly getBoundingBox()
   {
      return boundingBox;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrameHolder.getReferenceFrame();
   }

   /** {@inheritDoc} */
   @Override
   public boolean containsNaN()
   {
      return FrameFace3DReadOnly.super.containsNaN();
   }

   /** {@inheritDoc} */
   @Override
   public void setToNaN()
   {
      edges.clear();
      vertices.clear();
      centroid.setToNaN();
      normal.setToNaN();
      area = Double.NaN;
      boundingBox.setToNaN();
   }

   /** {@inheritDoc} */
   @Override
   public void setToZero()
   {
      edges.clear();
      vertices.clear();
      centroid.setToZero();
      normal.setToZero();
      area = 0.0;
      boundingBox.setToZero();
   }

   /** {@inheritDoc} */
   @Override
   public void applyTransform(Transform transform)
   {
      for (int i = 0; i < getNumberOfEdges(); i++)
         edges.get(i).getOrigin().applyTransform(transform);
      centroid.applyTransform(transform);
      normal.applyTransform(transform);
   }

   /** {@inheritDoc} */
   @Override
   public void applyInverseTransform(Transform transform)
   {
      for (int i = 0; i < getNumberOfEdges(); i++)
         edges.get(i).getOrigin().applyInverseTransform(transform);
      centroid.applyInverseTransform(transform);
      normal.applyInverseTransform(transform);
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
      return EuclidShapeIOTools.getFace3DString(this);
   }
}
