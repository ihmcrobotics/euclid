package us.ihmc.euclid.shape.convexPolytope;

import java.util.*;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.ConvexPolytope3DTroublesomeDataset;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeConstructionTools;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTools;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DBasics;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
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
 */
public class ConvexPolytope3D implements ConvexPolytope3DReadOnly, Shape3DBasics, GeometryObject<ConvexPolytope3D>
{
   /** The list of the vertices composing this convex polytope. */
   private final List<Vertex3D> vertices = new ArrayList<>();
   /** The list of the half-edges composing this convex polytope. */
   private final List<HalfEdge3D> halfEdges = new ArrayList<>();
   /** The list of the faces composing this convex polytope. */
   private final List<Face3D> faces = new ArrayList<>();
   /** The centroid of this convex polytope. */
   private final Point3D centroid = new Point3D();
   /** The volume of this convex polytope. */
   private double volume;
   /** The tightest bounding box entirely containing this face. */
   private final BoundingBox3D boundingBox = new BoundingBox3D();
   /**
    * Tolerance used when constructing a convex polytope. It is used to trigger a series of edge-cases
    * including for instance whether or not a face should be extended.
    */
   private final double constructionEpsilon;

   /**
    * Creates a new empty convex polytope.
    */
   public ConvexPolytope3D()
   {
      this(EuclidPolytopeConstructionTools.DEFAULT_CONSTRUCTION_EPSILON);
   }

   /**
    * Creates a new empty convex polytope.
    *
    * @param constructionEpsilon tolerance used when adding vertices to a convex polytope to trigger a
    *                            series of edge-cases.
    */
   public ConvexPolytope3D(double constructionEpsilon)
   {
      boundingBox.setToNaN();
      this.constructionEpsilon = constructionEpsilon;
   }

   /**
    * Creates a new convex polytope and adds vertices provided by the given supplier.
    *
    * @param vertex3DSupplier the vertex supplier to get the vertices to add to this convex polytope.
    */
   public ConvexPolytope3D(Vertex3DSupplier vertex3DSupplier)
   {
      this(vertex3DSupplier, EuclidPolytopeConstructionTools.DEFAULT_CONSTRUCTION_EPSILON);
   }

   /**
    * Creates a new convex polytope and adds vertices provided by the given supplier.
    *
    * @param vertex3DSupplier    the vertex supplier to get the vertices to add to this convex
    *                            polytope.
    * @param constructionEpsilon tolerance used when adding vertices to a convex polytope to trigger a
    *                            series of edge-cases.
    */
   public ConvexPolytope3D(Vertex3DSupplier vertex3DSupplier, double constructionEpsilon)
   {
      this(constructionEpsilon);
      addVertices(vertex3DSupplier);
   }

   /**
    * Creates a new convex polytope identical to {@code other}.
    *
    * @param other the other convex polytope to copy. Not modified.
    */
   public ConvexPolytope3D(ConvexPolytope3DReadOnly other)
   {
      constructionEpsilon = other.getConstructionEpsilon();
      set(other);
   }

   /**
    * Creates a new convex polytope and initializes its faces.
    * <p>
    * This constructor should only be used with {@link ConvexPolytope3DTroublesomeDataset}.
    * </p>
    *
    * @param faces               the faces composing the new convex polytope. Not modified, faces
    *                            reference saved.
    * @param constructionEpsilon tolerance used when adding vertices to a convex polytope to trigger a
    *                            series of edge-cases.
    */
   public ConvexPolytope3D(List<Face3D> faces, double constructionEpsilon)
   {
      this(constructionEpsilon);

      this.faces.addAll(faces);

      updateEdges();
      updateVertices();
      updateBoundingBox();
      updateCentroidAndVolume();

      if (faces.size() > 1)
      {
         for (HalfEdge3D halfEdge : halfEdges)
         {
            if (halfEdge.getTwin() == null)
            {
               HalfEdge3D twin = halfEdge.getDestination().getEdgeTo(halfEdge.getOrigin());
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
      vertices.clear();
      halfEdges.clear();
      faces.clear();
      boundingBox.setToNaN();
      centroid.setToNaN();
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
      vertices.clear();
      halfEdges.clear();
      faces.clear();
      boundingBox.setToZero();
      centroid.setToZero();
      volume = 0.0;
   }

   /**
    * Sets this convex polytope to be identical to {@code other}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    */
   @Override
   public void set(ConvexPolytope3D other)
   {
      this.set((ConvexPolytope3DReadOnly) other);
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

      Map<Vertex3DReadOnly, Vertex3D> fromOtherToThisVertices = new HashMap<>(other.getNumberOfVertices());
      Map<HalfEdge3DReadOnly, HalfEdge3D> fromOtherToThisEdges = new HashMap<>(other.getNumberOfEdges());

      List<? extends Vertex3DReadOnly> otherVertices = other.getVertices();

      for (Vertex3DReadOnly otherVertex : otherVertices)
      {
         Vertex3D vertex = new Vertex3D(otherVertex);
         vertices.add(vertex);
         fromOtherToThisVertices.put(otherVertex, vertex);
      }

      for (HalfEdge3DReadOnly otherEdge : other.getHalfEdges())
      {
         Vertex3DReadOnly otherOrigin = otherEdge.getOrigin();
         Vertex3DReadOnly otherDestination = otherEdge.getDestination();

         Vertex3D origin = fromOtherToThisVertices.get(otherOrigin);
         Vertex3D destination = fromOtherToThisVertices.get(otherDestination);

         HalfEdge3D edge = new HalfEdge3D(origin, destination);
         halfEdges.add(edge);
         fromOtherToThisEdges.put(otherEdge, edge);
      }

      for (int edgeIndex = 0; edgeIndex < other.getHalfEdges().size(); edgeIndex++)
      {
         HalfEdge3DReadOnly otherEdge = other.getHalfEdge(edgeIndex);
         HalfEdge3D edge = halfEdges.get(edgeIndex);

         HalfEdge3D next = fromOtherToThisEdges.get(otherEdge.getNext());
         HalfEdge3D previous = fromOtherToThisEdges.get(otherEdge.getPrevious());
         HalfEdge3D twin = fromOtherToThisEdges.get(otherEdge.getTwin());

         edge.setNext(next);
         edge.setPrevious(previous);
         edge.setTwin(twin);
      }

      for (Face3DReadOnly otherFace : other.getFaces())
      {
         Vector3DReadOnly otherNormal = otherFace.getNormal();
         HalfEdge3DReadOnly otherFirstEdge = otherFace.getEdge(0);

         HalfEdge3D firstEdge = fromOtherToThisEdges.get(otherFirstEdge);
         List<HalfEdge3D> faceEdges = new ArrayList<>();
         HalfEdge3D currentEdge = firstEdge;

         do
         {
            faceEdges.add(currentEdge);
            currentEdge = currentEdge.getNext();
         }
         while (currentEdge != firstEdge);

         Face3D face = new Face3D(faceEdges, otherNormal, constructionEpsilon);
         faces.add(face);
      }

      boundingBox.set(other.getBoundingBox());
      centroid.set(other.getCentroid());
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
         Vertex3D vertexToAdd = new Vertex3D(vertex3DSupplier.getVertex(index));

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
            Face3D face = faces.get(i);
            if (face.getNumberOfEdges() <= 2)
               removeFace(face);
         }
      }

      if (faces.size() == 2)
      {
         removeFace(faces.get(1));
         Face3D singleFace = faces.get(0);

         for (int i = 0; i < singleFace.getNumberOfEdges(); i++)
         {
            HalfEdge3D edge = singleFace.getEdge(i);
            edge.getOrigin().clearAssociatedEdgeList();
            edge.getOrigin().addAssociatedEdge(edge);
         }
      }
      else if (faces.size() == 3)
      { // One of the faces holds onto all this polytope vertices. So we'll keep the face that has the most vertices.
         Face3D faceToKeep = faces.get(0);
         Face3D secondFace = faces.get(1);
         Face3D thirdFace = faces.get(2);

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
            HalfEdge3D edge = faceToKeep.getEdge(i);
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

   private boolean handleNoFaceCase(Vertex3D vertexToAdd)
   {
      // Polytope is empty. Creating face and adding the vertex
      Face3D newFace = new Face3D(Axis.Z, constructionEpsilon);
      newFace.addVertex(vertexToAdd);
      return faces.add(newFace);
   }

   private boolean handleSingleFaceCase(Vertex3D vertexToAdd)
   {
      Face3D firstFace = faces.get(0);

      if (firstFace.getNumberOfEdges() <= 2)
      { // The face is not an actual face yet, extend it.
         return firstFace.addVertex(vertexToAdd);
      }
      else if (!EuclidPolytopeTools.arePoint3DAndFace3DInPlane(vertexToAdd, firstFace, constructionEpsilon))
      { // Off the face plane => need to create new faces.
         if (firstFace.canObserverSeeFace(vertexToAdd))
            firstFace.flip();

         List<Face3D> newFaces = EuclidPolytopeConstructionTools.computeVertexNeighborFaces(vertexToAdd,
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

   private boolean handleMultipleFaceCase(Vertex3D vertexToAdd)
   {
      Set<Face3D> visibleFaces = new HashSet<>();
      List<HalfEdge3D> silhouette = EuclidPolytopeTools.computeSilhouette(faces, vertexToAdd, constructionEpsilon, visibleFaces);

      if (silhouette != null)
      {
         List<Face3D> inPlaneFaces = EuclidPolytopeTools.computeInPlaneFacesAroundSilhouette(vertexToAdd, silhouette, constructionEpsilon);
         List<Face3D> newFaces = EuclidPolytopeConstructionTools.computeVertexNeighborFaces(vertexToAdd, silhouette, inPlaneFaces, constructionEpsilon);
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
      vertices.clear();
      faces.stream().flatMap(face -> face.getVertices().stream()).distinct().forEach(vertices::add);
   }

   private void updateEdges()
   {
      halfEdges.clear();
      for (Face3D face : faces)
         halfEdges.addAll(face.getEdges());
   }

   private void updateBoundingBox()
   {
      boundingBox.setToNaN();

      for (int faceIndex = 0; faceIndex < faces.size(); faceIndex++)
         boundingBox.combine(faces.get(faceIndex).getBoundingBox());
   }

   private void updateCentroidAndVolume()
   {
      volume = EuclidPolytopeConstructionTools.computeConvexPolytope3DVolume(this, centroid);
   }

   private void removeFaces(Collection<Face3D> facesToRemove)
   {
      for (Face3D face : facesToRemove)
         removeFace(face);
   }

   private void removeFace(Face3D faceToRemove)
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
   public Face3D getFace(int index)
   {
      return faces.get(index);
   }

   /** {@inheritDoc} */
   @Override
   public List<Face3D> getFaces()
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
   public HalfEdge3D getHalfEdge(int index)
   {
      return halfEdges.get(index);
   }

   /** {@inheritDoc} */
   @Override
   public List<HalfEdge3D> getHalfEdges()
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
   public Vertex3D getVertex(int index)
   {
      return vertices.get(index);
   }

   /** {@inheritDoc} */
   @Override
   public List<Vertex3D> getVertices()
   {
      return vertices;
   }

   /** {@inheritDoc} */
   @Override
   public BoundingBox3DReadOnly getBoundingBox()
   {
      return boundingBox;
   }

   /** {@inheritDoc} */
   @Override
   public double getConstructionEpsilon()
   {
      return constructionEpsilon;
   }

   /** {@inheritDoc} */
   @Override
   public Face3D getClosestFace(Point3DReadOnly point)
   {
      return (Face3D) ConvexPolytope3DReadOnly.super.getClosestFace(point);
   }

   /** {@inheritDoc} */
   @Override
   public Point3DReadOnly getCentroid()
   {
      return centroid;
   }

   /** {@inheritDoc} */
   @Override
   public double getVolume()
   {
      return volume;
   }

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
         Face3D face = faces.get(i);
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
         Face3D face = faces.get(i);
         face.updateNormal();
         face.updateCentroidAndArea();
         face.refreshBoundingBox();
      }

      updateBoundingBox();
      updateCentroidAndVolume();
   }

   /**
    * Tests on a per component basis if {@code other} and {@code this} are equal to an {@code epsilon}.
    *
    * @param other   the other convex polytope to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two convex polytopes are equal component-wise, {@code false}
    *         otherwise.
    */
   @Override
   public boolean epsilonEquals(ConvexPolytope3D other, double epsilon)
   {
      return ConvexPolytope3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} and {@code other} to determine if the two convex polytopes are
    * geometrically similar.
    *
    * @param other   the convex polytope to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two convex polytope represent the same geometry, {@code false}
    *         otherwise.
    */
   @Override
   public boolean geometricallyEquals(ConvexPolytope3D other, double epsilon)
   {
      return ConvexPolytope3DReadOnly.super.geometricallyEquals(other, epsilon);
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
