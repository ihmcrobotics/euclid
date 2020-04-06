package us.ihmc.euclid.shape.convexPolytope.impl;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DFactory;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DFactory;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DFactory;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeConstructionTools;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTools;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DPoseBasics;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Implementation of a convex polytope 3D.
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
public abstract class AbstractConvexPolytope3D<Vertex extends AbstractVertex3D<Vertex, Edge, Face>, Edge extends AbstractHalfEdge3D<Vertex, Edge, Face>, Face extends AbstractFace3D<Vertex, Edge, Face>>
      implements ConvexPolytope3DReadOnly, Shape3DBasics, Transformable, Clearable
{
   /** The list of the vertices composing this convex polytope. */
   private final List<Vertex> vertices = new ArrayList<>();
   /** The list of the half-edges composing this convex polytope. */
   private final List<Edge> halfEdges = new ArrayList<>();
   /** The list of the faces composing this convex polytope. */
   private final List<Face> faces = new ArrayList<>();
   /** The volume of this convex polytope. */
   private double volume;
   /**
    * Tolerance used when constructing a convex polytope. It is used to trigger a series of edge-cases
    * including for instance whether or not a face should be extended.
    */
   private final double constructionEpsilon;
   /** Factory used to create vertices of the proper type. */
   private Vertex3DFactory<Vertex> vertexFactory;
   /** Factory used to create half-edges of the proper type. */
   private HalfEdge3DFactory<Vertex, Edge> edgeFactory;
   /** Factory used to create faces of the proper type. */
   private Face3DFactory<Face> faceFactory;
   /**
    * Last result from {@link #getSupportingVertex(Vertex3DReadOnly, Vector3DReadOnly)} that is
    * automatically reused for the next call to provide a slight speedup.
    */
   private Vertex3DReadOnly lastSupportingVertex = null;

   /**
    * Creates a new empty convex polytope.
    */
   public AbstractConvexPolytope3D()
   {
      this(EuclidPolytopeConstructionTools.DEFAULT_CONSTRUCTION_EPSILON);
   }

   /**
    * Creates a new empty convex polytope.
    *
    * @param constructionEpsilon tolerance used when adding vertices to a convex polytope to trigger a
    *                            series of edge-cases.
    */
   public AbstractConvexPolytope3D(double constructionEpsilon)
   {
      this.constructionEpsilon = constructionEpsilon;
   }

   /**
    * Sets the factories to use with the implementation.
    * <p>
    * Factories have to be provided before initializing this polytope.
    * </p>
    *
    * @param vertexFactory the factory to use for creating new vertices when expanding this polytope.
    * @param edgeFactory   the factory to use for creating new half-edges when expanding this polytope.
    * @param faceFactory   the factory to use for creating new faces when expanding this polytope.
    */
   protected void setFactories(Vertex3DFactory<Vertex> vertexFactory, HalfEdge3DFactory<Vertex, Edge> edgeFactory, Face3DFactory<Face> faceFactory)
   {
      this.vertexFactory = vertexFactory;
      this.edgeFactory = edgeFactory;
      this.faceFactory = faceFactory;
   }

   /**
    * Initializes this empty polytope.
    * <p>
    * The polytope must be initialized for the implementation's constructor.
    * </p>
    */
   protected void initialize()
   {
      getBoundingBox().setToNaN();
   }

   /**
    * Initializes this polytope with the given pre-computed faces.
    * <p>
    * The polytope must be initialized for the implementation's constructor.
    * </p>
    * <p>
    * This method updates twin of every half-edge if not done beforehand.
    * </p>
    *
    * @param faces this polytope faces.
    */
   protected void initialize(List<Face> faces)
   {
      this.faces.addAll(faces);

      updateEdges();
      updateVertices();
      updateBoundingBox();
      updateCentroidAndVolume();

      if (faces.size() > 1)
      {
         for (Edge halfEdge : halfEdges)
         {
            if (halfEdge.getTwin() == null)
            {
               Edge twin = halfEdge.getDestination().getEdgeTo(halfEdge.getOrigin());
               halfEdge.setTwin(twin);
               twin.setTwin(halfEdge);
            }
         }
      }
   }

   /**
    * Clears this convex polytope internal data, and invalidate its properties such as its bounding
    * box, centroid, and volume.
    */
   public void clear()
   {
      lastSupportingVertex = null;
      vertices.clear();
      halfEdges.clear();
      faces.clear();
      getBoundingBox().setToNaN();
      getCentroid().setToNaN();
      volume = Double.NaN;
   }

   /**
    * Clears this convex polytope internal data, and invalidate its properties such as its bounding
    * box, centroid, and volume.
    */
   @Override
   public void setToNaN()
   {
      clear();
   }

   /**
    * Clears this convex polytope internal data, and set to zero its properties such as its bounding
    * box, centroid, and volume.
    */
   @Override
   public void setToZero()
   {
      lastSupportingVertex = null;
      vertices.clear();
      halfEdges.clear();
      faces.clear();
      getBoundingBox().setToZero();
      getCentroid().setToZero();
      volume = 0.0;
   }

   /**
    * Sets this convex polytope to be identical to {@code other}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param other the other convex polytope to copy. Not modified.
    */
   public void set(ConvexPolytope3DReadOnly other)
   {
      clear();

      Map<Vertex3DReadOnly, Vertex> fromOtherToThisVertices = new HashMap<>(other.getNumberOfVertices());
      Map<HalfEdge3DReadOnly, Edge> fromOtherToThisEdges = new HashMap<>(other.getNumberOfEdges());

      List<? extends Vertex3DReadOnly> otherVertices = other.getVertices();

      for (Vertex3DReadOnly otherVertex : otherVertices)
      {
         Vertex vertex = vertexFactory.newInstance(otherVertex);
         vertices.add(vertex);
         fromOtherToThisVertices.put(otherVertex, vertex);
      }

      for (HalfEdge3DReadOnly otherEdge : other.getHalfEdges())
      {
         Vertex3DReadOnly otherOrigin = otherEdge.getOrigin();
         Vertex3DReadOnly otherDestination = otherEdge.getDestination();

         Vertex origin = fromOtherToThisVertices.get(otherOrigin);
         Vertex destination = fromOtherToThisVertices.get(otherDestination);

         Edge edge = edgeFactory.newInstance(origin, destination);
         halfEdges.add(edge);
         fromOtherToThisEdges.put(otherEdge, edge);
      }

      for (int edgeIndex = 0; edgeIndex < other.getHalfEdges().size(); edgeIndex++)
      {
         HalfEdge3DReadOnly otherEdge = other.getHalfEdge(edgeIndex);
         Edge edge = halfEdges.get(edgeIndex);

         Edge next = fromOtherToThisEdges.get(otherEdge.getNext());
         Edge previous = fromOtherToThisEdges.get(otherEdge.getPrevious());
         Edge twin = fromOtherToThisEdges.get(otherEdge.getTwin());

         edge.setNext(next);
         edge.setPrevious(previous);
         edge.setTwin(twin);
      }

      for (Face3DReadOnly otherFace : other.getFaces())
      {
         Vector3DReadOnly otherNormal = otherFace.getNormal();
         HalfEdge3DReadOnly otherFirstEdge = otherFace.getEdge(0);

         Edge firstEdge = fromOtherToThisEdges.get(otherFirstEdge);
         List<Edge> faceEdges = new ArrayList<>();
         Edge currentEdge = firstEdge;

         do
         {
            faceEdges.add(currentEdge);
            currentEdge = currentEdge.getNext();
         }
         while (currentEdge != firstEdge);

         Face face = faceFactory.newInstance(otherNormal, constructionEpsilon);
         face.initialize(faceEdges, otherNormal);
         faces.add(face);
      }

      getBoundingBox().set(other.getBoundingBox());
      getCentroid().set(other.getCentroid());
      volume = other.getVolume();
   }

   /**
    * Adds a new vertex to this convex polytope.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param vertexToAdd the vertex that is to be added to the convex polytope. Not modified.
    * @return {@code true} if the vertex was added to this convex polytope, {@code false} if it was
    *         rejected.
    */
   public boolean addVertex(Point3DReadOnly vertexToAdd)
   {
      return addVertices(Vertex3DSupplier.asVertex3DSupplier(vertexToAdd));
   }

   /**
    * Adds a new vertex to this convex polytope.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param vertex3DSupplier the vertex supplier to get the vertices to add to this convex polytope.
    * @return {@code true} if the vertex was added to this convex polytope, {@code false} if it was
    *         rejected.
    */
   public boolean addVertices(Vertex3DSupplier vertex3DSupplier)
   {
      boolean isPolytopeModified = false;

      for (int index = 0; index < vertex3DSupplier.getNumberOfVertices(); index++)
      {
         Vertex vertexToAdd = vertexFactory.newInstance(vertex3DSupplier.getVertex(index));

         if (faces.size() == 0)
            isPolytopeModified |= handleNoFaceCase(vertexToAdd);
         else if (faces.size() == 1)
            isPolytopeModified |= handleSingleFaceCase(vertexToAdd);
         else
            isPolytopeModified |= handleMultipleFaceCase(vertexToAdd);
      }

      if (faces.size() > 2)
      {
         for (int i = faces.size() - 1; i >= 0; i--)
         {
            Face face = faces.get(i);
            if (face.getNumberOfEdges() <= 2)
               removeFace(face);
         }
      }

      if (faces.size() == 2)
      {
         removeFace(faces.get(1));
         Face singleFace = faces.get(0);

         for (int i = 0; i < singleFace.getNumberOfEdges(); i++)
         {
            Edge edge = singleFace.getEdge(i);
            edge.getOrigin().clearAssociatedEdgeList();
            edge.getOrigin().addAssociatedEdge(edge);
         }
      }
      else if (faces.size() == 3)
      { // One of the faces holds onto all this polytope vertices. So we'll keep the face that has the most vertices.
         Face faceToKeep = faces.get(0);
         Face secondFace = faces.get(1);
         Face thirdFace = faces.get(2);

         if (secondFace.getNumberOfEdges() > faceToKeep.getNumberOfEdges())
         {
            removeFace(faceToKeep);
            faceToKeep = secondFace;
         }
         else
         {
            removeFace(secondFace);
         }

         if (thirdFace.getNumberOfEdges() > faceToKeep.getNumberOfEdges())
         {
            removeFace(faceToKeep);
            faceToKeep = thirdFace;
         }
         else
         {
            removeFace(thirdFace);
         }

         for (int i = 0; i < faceToKeep.getNumberOfEdges(); i++)
         {
            Edge edge = faceToKeep.getEdge(i);
            edge.getOrigin().clearAssociatedEdgeList();
            edge.getOrigin().addAssociatedEdge(edge);
         }
      }

      if (isPolytopeModified)
      {
         updateEdges();
         updateVertices();
         updateBoundingBox();
         updateCentroidAndVolume();
      }

      return isPolytopeModified;
   }

   private boolean handleNoFaceCase(Vertex vertexToAdd)
   {
      // Polytope is empty. Creating face and adding the vertex
      Face newFace = faceFactory.newInstance(Axis3D.Z, constructionEpsilon);
      newFace.addVertex(vertexToAdd);
      return faces.add(newFace);
   }

   private boolean handleSingleFaceCase(Vertex vertexToAdd)
   {
      Face firstFace = faces.get(0);

      if (firstFace.getNumberOfEdges() <= 2)
      { // The face is not an actual face yet, extend it.
         return firstFace.addVertex(vertexToAdd);
      }
      else if (!EuclidPolytopeTools.arePoint3DAndFace3DInPlane(vertexToAdd, firstFace, constructionEpsilon))
      { // Off the face plane => need to create new faces.
         if (firstFace.canObserverSeeFace(vertexToAdd))
            firstFace.flip();

         List<Face> newFaces = EuclidPolytopeConstructionTools.computeVertexNeighborFaces(faceFactory,
                                                                                          vertexToAdd,
                                                                                          firstFace.getEdges(),
                                                                                          Collections.emptyList(),
                                                                                          constructionEpsilon);
         if (newFaces != null)
         {
            faces.addAll(newFaces);
            return true;
         }
      }
      else if (!firstFace.isPointDirectlyAboveOrBelow(vertexToAdd))
      { // In face plane => need to extend the existing face.
         return firstFace.addVertex(vertexToAdd);
      }

      // The vertex already belongs to the face, nothing to do.
      return false;
   }

   private boolean handleMultipleFaceCase(Vertex vertexToAdd)
   {
      Set<Face> visibleFaces = new HashSet<>();
      List<Edge> silhouette = EuclidPolytopeTools.computeSilhouette(faces, vertexToAdd, constructionEpsilon, visibleFaces);

      if (silhouette != null)
      {
         List<Face> inPlaneFaces = EuclidPolytopeTools.computeInPlaneFacesAroundSilhouette(vertexToAdd, silhouette, constructionEpsilon);
         List<Face> newFaces = EuclidPolytopeConstructionTools.computeVertexNeighborFaces(faceFactory,
                                                                                          vertexToAdd,
                                                                                          silhouette,
                                                                                          inPlaneFaces,
                                                                                          constructionEpsilon);
         if (newFaces != null)
         {
            removeFaces(visibleFaces);
            faces.addAll(newFaces);
            return true;
         }
      }

      /*
       * The vertex is either: inside the polytope, inside a face, or got rejected in case adding it would
       * result in an inconsistent polytope (this is expected to occur only when dealing with numerical
       * inaccuracies).
       */
      return false;
   }

   private void updateVertices()
   {
      lastSupportingVertex = null;
      vertices.clear();
      faces.stream().flatMap(face -> face.getVertices().stream()).distinct().forEach(vertices::add);
   }

   private void updateEdges()
   {
      halfEdges.clear();
      for (Face face : faces)
         halfEdges.addAll(face.getEdges());
   }

   private void updateBoundingBox()
   {
      getBoundingBox().setToNaN();

      for (int faceIndex = 0; faceIndex < faces.size(); faceIndex++)
         getBoundingBox().combine(faces.get(faceIndex).getBoundingBox());
   }

   private void updateCentroidAndVolume()
   {
      volume = EuclidPolytopeConstructionTools.computeConvexPolytope3DVolume(this, getCentroid());
   }

   private void removeFaces(Collection<Face> facesToRemove)
   {
      for (Face face : facesToRemove)
         removeFace(face);
   }

   private void removeFace(Face faceToRemove)
   {
      if (faces.remove(faceToRemove))
         faceToRemove.destroy();
   }

   /** {@inheritDoc} */
   @Override
   public boolean containsNaN()
   {
      return ConvexPolytope3DReadOnly.super.containsNaN();
   }

   /**
    * Gets the i<sup>th</sup> face of this polytope.
    *
    * @param index the face index &in; [0; {@link #getNumberOfFaces()}[.
    * @return the reference to the face.
    */
   @Override
   public Face getFace(int index)
   {
      return faces.get(index);
   }

   /** {@inheritDoc} */
   @Override
   public List<Face> getFaces()
   {
      return faces;
   }

   /**
    * Gets the i<sup>th</sup> half-edge of this polytope.
    *
    * @param index the half-edge index &in; [0; {@link #getNumberOfHalfEdges()}[.
    * @return the reference to the half-edge.
    */
   @Override
   public Edge getHalfEdge(int index)
   {
      return halfEdges.get(index);
   }

   /** {@inheritDoc} */
   @Override
   public List<Edge> getHalfEdges()
   {
      return halfEdges;
   }

   /**
    * Gets the i<sup>th</sup> vertex of this polytope.
    *
    * @param index the vertex index &in; [0; {@link #getNumberOfVertices()}[.
    * @return the reference to the vertex.
    */
   @Override
   public Vertex getVertex(int index)
   {
      return vertices.get(index);
   }

   /** {@inheritDoc} */
   @Override
   public List<Vertex> getVertices()
   {
      return vertices;
   }

   /** {@inheritDoc} */
   @Override
   public abstract BoundingBox3DBasics getBoundingBox();

   /** {@inheritDoc} */
   @Override
   public double getConstructionEpsilon()
   {
      return constructionEpsilon;
   }

   /** {@inheritDoc} */
   @SuppressWarnings("unchecked")
   @Override
   public Face getClosestFace(Point3DReadOnly point)
   {
      return (Face) ConvexPolytope3DReadOnly.super.getClosestFace(point);
   }

   /** {@inheritDoc} */
   @Override
   public abstract Point3DBasics getCentroid();

   /** {@inheritDoc} */
   @Override
   public double getVolume()
   {
      return volume;
   }

   @Override
   public Vertex3DReadOnly getSupportingVertex(Vector3DReadOnly supportDirection)
   {
      lastSupportingVertex = getSupportingVertex(lastSupportingVertex, supportDirection);
      return lastSupportingVertex;
   }

   @Override
   public Shape3DPoseBasics getPose()
   {
      return null;
   }

   @Override
   public abstract AbstractConvexPolytope3D<Vertex, Edge, Face> copy();

   /** {@inheritDoc} */
   @Override
   public void applyTransform(Transform transform)
   {
      for (int i = 0; i < vertices.size(); i++)
      {
         vertices.get(i).applyTransform(transform);
      }

      for (int i = 0; i < faces.size(); i++)
      {
         Face face = faces.get(i);
         face.updateNormal();
         face.updateCentroidAndArea();
         face.refreshBoundingBox();
      }

      updateBoundingBox();
      updateCentroidAndVolume();
   }

   /** {@inheritDoc} */
   @Override
   public void applyInverseTransform(Transform transform)
   {
      for (int i = 0; i < vertices.size(); i++)
      {
         vertices.get(i).applyInverseTransform(transform);
      }

      for (int i = 0; i < faces.size(); i++)
      {
         Face face = faces.get(i);
         face.updateNormal();
         face.updateCentroidAndArea();
         face.refreshBoundingBox();
      }

      updateBoundingBox();
      updateCentroidAndVolume();
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(ConvexPolytope3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof ConvexPolytope3DReadOnly)
         return equals((ConvexPolytope3DReadOnly) object);
      else
         return false;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this convex polytope
    * 3D.
    *
    * @return the hash code value for this convex polytope 3D.
    */
   @Override
   public int hashCode()
   {
      // Using ArrayList.hashCode() to combine the hash-codes of the vertices defining this face.
      return vertices.hashCode();
   }

   /**
    * Provides a {@code String} representation of this convex polytope 3D as follows:
    *
    * <pre>
    * Convex polytope 3D: number of: [faces: 4, edges: 12, vertices: 4
    * Face list:
    *    centroid: ( 0.582, -0.023,  0.160 ), normal: ( 0.516, -0.673,  0.530 )
    *    centroid: ( 0.420,  0.176,  0.115 ), normal: (-0.038,  0.895, -0.444 )
    *    centroid: ( 0.264, -0.253, -0.276 ), normal: ( 0.506,  0.225, -0.833 )
    *    centroid: ( 0.198, -0.176, -0.115 ), normal: (-0.643, -0.374,  0.668 )
    * Edge list:
    *    [( 0.674,  0.482,  0.712 ); ( 0.870,  0.251,  0.229 )]
    *    [( 0.870,  0.251,  0.229 ); ( 0.204, -0.803, -0.461 )]
    *    [( 0.204, -0.803, -0.461 ); ( 0.674,  0.482,  0.712 )]
    *    [( 0.870,  0.251,  0.229 ); ( 0.674,  0.482,  0.712 )]
    *    [( 0.674,  0.482,  0.712 ); (-0.283, -0.207, -0.595 )]
    *    [(-0.283, -0.207, -0.595 ); ( 0.870,  0.251,  0.229 )]
    *    [( 0.204, -0.803, -0.461 ); ( 0.870,  0.251,  0.229 )]
    *    [( 0.870,  0.251,  0.229 ); (-0.283, -0.207, -0.595 )]
    *    [(-0.283, -0.207, -0.595 ); ( 0.204, -0.803, -0.461 )]
    *    [( 0.674,  0.482,  0.712 ); ( 0.204, -0.803, -0.461 )]
    *    [( 0.204, -0.803, -0.461 ); (-0.283, -0.207, -0.595 )]
    *    [(-0.283, -0.207, -0.595 ); ( 0.674,  0.482,  0.712 )]
    * Vertex list:
    *    ( 0.674,  0.482,  0.712 )
    *    ( 0.870,  0.251,  0.229 )
    *    ( 0.204, -0.803, -0.461 )
    *    (-0.283, -0.207, -0.595 )
    * </pre>
    *
    * @return the {@code String} representing this convex polytope 3D.
    */
   @Override
   public String toString()
   {
      return EuclidShapeIOTools.getConvexPolytope3DString(this);
   }
}
