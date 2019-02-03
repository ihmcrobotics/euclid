package us.ihmc.euclid.shape.convexPolytope;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Simplex3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeConstructionTools;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeIOTools;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 *
 * @author Apoorv S
 *
 */
public class ConvexPolytope3D implements ConvexPolytope3DReadOnly, Clearable, Transformable, Settable<ConvexPolytope3DReadOnly>
{
   private final ArrayList<Vertex3D> vertices = new ArrayList<>();
   private final ArrayList<HalfEdge3D> edges = new ArrayList<>();
   private final ArrayList<Face3D> faces = new ArrayList<>();
   /**
    * Bounding box for the polytope
    */
   private final BoundingBox3D boundingBox = new BoundingBox3D();

   private final Vector3D tempVector = new Vector3D();
   private final Point3D centroid = new Point3D();

   private final double constructionEpsilon;

   public ConvexPolytope3D()
   {
      this(EuclidPolytopeConstructionTools.DEFAULT_CONSTRUCTION_EPSILON);
   }

   public ConvexPolytope3D(double constructionEpsilon)
   {
      boundingBox.setToNaN();
      this.constructionEpsilon = constructionEpsilon;
   }

   public ConvexPolytope3D(Vertex3DSupplier vertex3DSupplier)
   {
      this(vertex3DSupplier, EuclidPolytopeConstructionTools.DEFAULT_CONSTRUCTION_EPSILON);
   }

   public ConvexPolytope3D(Vertex3DSupplier vertex3DSupplier, double constructionEpsilon)
   {
      this(constructionEpsilon);
      addVertices(vertex3DSupplier);
   }

   public ConvexPolytope3D(ConvexPolytope3DReadOnly polytope)
   {
      constructionEpsilon = polytope.getConstructionEpsilon();
      set(polytope);
   }

   public boolean addVertex(Point3DReadOnly vertexToAdd)
   {
      return addVertex(new Vertex3D(vertexToAdd));
   }

   /**
    * Adds a polytope vertex to the current polytope. In case needed faces are removed and recreated.
    * This will result in garbage. Fix if possible
    *
    * @param vertexToAdd
    * @param epsilon
    * @return
    */
   public boolean addVertex(Vertex3D vertexToAdd)
   {
      return addVertices(Collections.singleton(vertexToAdd));
   }

   public boolean addVertices(Vertex3DSupplier vertex3DSupplier)
   {
      List<Vertex3D> vertices = new ArrayList<>(vertex3DSupplier.getNumberOfVertices());
      for (int i = 0; i < vertex3DSupplier.getNumberOfVertices(); i++)
         vertices.add(new Vertex3D(vertex3DSupplier.getVertex(i)));
      return addVertices(vertices);
   }

   public boolean addVertices(Collection<Vertex3D> verticesToAdd)
   {
      boolean isPolytopeModified = false;

      for (Vertex3D vertexToAdd : verticesToAdd)
      {
         if (faces.size() == 0)
            isPolytopeModified |= handleNoFaceCase(vertexToAdd);
         else if (faces.size() == 1)
            isPolytopeModified |= handleSingleFaceCase(vertexToAdd);
         else
            isPolytopeModified |= handleMultipleFaceCase(vertexToAdd);
      }

      if (isPolytopeModified)
      {
         updateEdges();
         updateVertices();
         updateBoundingBox();
      }

      return isPolytopeModified;
   }

   private boolean handleNoFaceCase(Vertex3D vertexToAdd)
   {
      // Polytope is empty. Creating face and adding the vertex
      Face3D newFace = new Face3D(Axis.Z);
      newFace.addVertex(vertexToAdd, constructionEpsilon);
      faces.add(newFace);

      return true;
   }

   private boolean handleSingleFaceCase(Vertex3D vertexToAdd)
   {
      Face3D firstFace = faces.get(0);

      if (firstFace.getNumberOfEdges() <= 2)
      { // The face is not an actual face yet, extend it.
         firstFace.addVertex(vertexToAdd, constructionEpsilon);
         return true;
      }
      else if (!firstFace.isPointInFacePlane(vertexToAdd, constructionEpsilon))
      { // Off the face plane => need to create new faces.
         if (firstFace.canObserverSeeFace(vertexToAdd))
            firstFace.flip();

         faces.addAll(EuclidPolytopeConstructionTools.computeVertexNeighborFaces(vertexToAdd, firstFace.getEdges(), Collections.emptyList(),
                                                                                 constructionEpsilon));
         return true;
      }
      else if (!firstFace.isPointDirectlyAboveOrBelow(vertexToAdd))
      { // In face plane => need to extend the existing face.
         firstFace.addVertex(vertexToAdd, constructionEpsilon);
         return true;
      }
      else
      { // The vertex already belongs to the face, nothing to do.
         return false;
      }
   }

   private boolean handleMultipleFaceCase(Vertex3D vertexToAdd)
   {
      Set<Face3D> visibleFaces = new HashSet<>();
      List<Face3D> inPlaneFaces = new ArrayList<>();
      Collection<HalfEdge3D> silhouette = EuclidPolytopeTools.getSilhouette(faces, vertexToAdd, constructionEpsilon, visibleFaces, inPlaneFaces);

      if (silhouette != null)
      {
         removeFaces(visibleFaces);
         faces.addAll(EuclidPolytopeConstructionTools.computeVertexNeighborFaces(vertexToAdd, silhouette, inPlaneFaces, constructionEpsilon));
         return true;
      }
      else
      { // The vertex is either inside the polytope or inside a face.
         return false;
      }
   }

   @Override
   public int getNumberOfFaces()
   {
      return faces.size();
   }

   @Override
   public int getNumberOfEdges()
   {
      if (getNumberOfFaces() < 2)
         return edges.size();
      else
         return edges.size() / 2;
   }

   @Override
   public int getNumberOfVertices()
   {
      if (getNumberOfFaces() < 2)
      {
         return vertices.size();
      }
      else
      { // Polyhedron formula for quick calc
         return getNumberOfEdges() - getNumberOfFaces() + 2;
      }
   }

   @Override
   public Face3D getFace(int index)
   {
      return faces.get(index);
   }

   @Override
   public List<Face3D> getFaces()
   {
      return faces;
   }

   @Override
   public HalfEdge3D getEdge(int index)
   {
      return edges.get(index);
   }

   @Override
   public List<HalfEdge3D> getEdges()
   {
      return edges;
   }

   @Override
   public Vertex3D getVertex(int index)
   {
      return vertices.get(index);
   }

   @Override
   public List<Vertex3D> getVertices()
   {
      return vertices;
   }

   @Override
   public BoundingBox3DReadOnly getBoundingBox()
   {
      return boundingBox;
   }

   @Override
   public double getConstructionEpsilon()
   {
      return constructionEpsilon;
   }

   public Vector3DReadOnly getFaceNormalAt(Point3DReadOnly point)
   {
      return getClosestFace(point).getNormal();
   }

   @Override
   public void applyTransform(Transform transform)
   {
      // Applying the transform to the vertices is less expensive computationally but getting the vertices is hard
      for (int i = 0; i < vertices.size(); i++)
      {
         vertices.get(i).applyTransform(transform);
      }

      for (int i = 0; i < faces.size(); i++)
      {
         Face3D face = faces.get(i);
         face.updateNormal();
         face.updateCentroiAndArea();
         face.refreshBoundingBox();
      }

      updateBoundingBox();
      // FIXME this method introduces inconsistency in Face3D.
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      // Applying the transform to the vertices is less expensive computationally but getting the vertices is hard
      for (int i = 0; i < vertices.size(); i++)
      {
         vertices.get(i).applyInverseTransform(transform);
      }

      for (int i = 0; i < faces.size(); i++)
      {
         Face3D face = faces.get(i);
         face.updateNormal();
         face.updateCentroiAndArea();
         face.refreshBoundingBox();
      }

      updateBoundingBox();
      // FIXME this method introduces inconsistency in Face3D.
   }

   private void updateVertices()
   { // FIXME this is slow, maybe there's a way to keep track of the vertices as the polytope is being built.
      vertices.clear();
      faces.stream().flatMap(face -> face.getVertices().stream()).distinct().forEach(vertices::add);
   }

   private void updateEdges()
   {
      edges.clear();
      for (Face3D face : faces)
         edges.addAll(face.getEdges());
   }

   private void updateBoundingBox()
   {
      boundingBox.setToNaN();
      if (faces.isEmpty())
         return;
      boundingBox.set(faces.get(0).getBoundingBox());

      for (int faceIndex = 1; faceIndex < faces.size(); faceIndex++)
         boundingBox.combine(faces.get(faceIndex).getBoundingBox());
   }

   private void removeFaces(Collection<Face3D> facesToRemove)
   {
      for (Face3D face : facesToRemove)
         removeFace(face);
   }

   public void removeFace(Face3D faceToRemove)
   {
      if (faces.remove(faceToRemove))
         faceToRemove.destroy();
   }

   @Override
   public Vertex3DReadOnly getSupportingVertex(Vector3DReadOnly supportDirection)
   {
      Vertex3D bestVertex = faces.get(0).getEdge(0).getOrigin();
      tempVector.set(bestVertex);
      double maxDotProduct = supportDirection.dot(tempVector);
      Vertex3D vertexCandidate = bestVertex;

      while (true)
      {
         for (HalfEdge3D currentEdge : bestVertex.getAssociatedEdges())
         {
            tempVector.set(currentEdge.getDestination());
            double dotProduct = supportDirection.dot(tempVector);
            if (dotProduct > maxDotProduct)
            {
               vertexCandidate = currentEdge.getDestination();
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
   public boolean containsNaN()
   {
      return ConvexPolytope3DReadOnly.super.containsNaN();
   }

   @Override
   public void setToNaN()
   {
      // This should also set all the edges and vertices to NaN assuming all relationships are intact
      for (int i = 0; i < faces.size(); i++)
      {
         faces.get(i).setToNaN();
      }
   }

   @Override
   public void setToZero()
   {
      // This should also set all the edges and vertices to zero assuming all relationships are intact
      for (int i = 0; i < faces.size(); i++)
      {
         faces.get(i).setToZero();
      }
   }

   @Override
   public void set(ConvexPolytope3DReadOnly other)
   {
      throw new RuntimeException("Unimplemented feature");
   }

   public void clear()
   {
      vertices.clear();
      edges.clear();
      faces.clear();
   }

   @Override
   public double distance(Point3DReadOnly point)
   {
      return getClosestFace(point).distance(point);
   }

   @Override
   public Face3D getClosestFace(Point3DReadOnly point)
   {
      return (Face3D) ConvexPolytope3DReadOnly.super.getClosestFace(point);
   }

   private void updateCentroid()
   { // TODO This is not the centroid
      centroid.setToZero();
      for (int i = 0; i < vertices.size(); i++)
         centroid.add(vertices.get(i));
      centroid.scale(1.0 / vertices.size());
   }

   public Point3DReadOnly getCentroid()
   {
      updateCentroid();
      return centroid;
   }

   @Override
   public void getSupportVectorDirectionTo(Point3DReadOnly point, Vector3DBasics supportVectorToPack)
   {
      getClosestFace(point).getSupportVectorDirectionTo(point, supportVectorToPack);
   }

   @Override
   public Simplex3D getSmallestSimplexMemberReference(Point3DReadOnly point)
   {
      return getClosestFace(point).getSmallestSimplexMemberReference(point);
   }

   @Override
   public String toString()
   {
      return EuclidPolytopeIOTools.getConvexPolytope3DString(this);
   }
}
