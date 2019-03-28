package us.ihmc.euclid.shape.convexPolytope;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeConstructionTools;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTools;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DBasics;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 *
 * @author Apoorv S
 *
 */
public class ConvexPolytope3D implements ConvexPolytope3DReadOnly, Shape3DBasics, GeometryObject<ConvexPolytope3D>
{
   private final ArrayList<Vertex3D> vertices = new ArrayList<>();
   private final ArrayList<HalfEdge3D> edges = new ArrayList<>();
   private final ArrayList<Face3D> faces = new ArrayList<>();
   /**
    * Bounding box for the polytope
    */
   private final BoundingBox3D boundingBox = new BoundingBox3D();

   private final Point3D centroid = new Point3D();
   private double volume;

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
         for (HalfEdge3D halfEdge : edges)
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

   public void clear()
   {
      vertices.clear();
      edges.clear();
      faces.clear();
      boundingBox.setToNaN();
      centroid.setToNaN();
      volume = Double.NaN;
   }

   @Override
   public void setToNaN()
   {
      // This should also set all the edges and vertices to NaN assuming all relationships are intact
      for (int i = 0; i < faces.size(); i++)
      {
         faces.get(i).setToNaN();
      }
      boundingBox.setToNaN();
      centroid.setToNaN();
      volume = Double.NaN;
   }

   @Override
   public void setToZero()
   {
      // This should also set all the edges and vertices to zero assuming all relationships are intact
      for (int i = 0; i < faces.size(); i++)
      {
         faces.get(i).setToZero();
      }
      boundingBox.setToZero();
      centroid.setToZero();
      volume = 0.0;
   }

   @Override
   public void set(ConvexPolytope3D other)
   {
      this.set((ConvexPolytope3DReadOnly) other);
   }

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
         edges.add(edge);
         fromOtherToThisEdges.put(otherEdge, edge);
      }

      for (int edgeIndex = 0; edgeIndex < other.getHalfEdges().size(); edgeIndex++)
      {
         HalfEdge3DReadOnly otherEdge = other.getHalfEdge(edgeIndex);
         HalfEdge3D edge = edges.get(edgeIndex);

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

   public boolean addVertices(Collection<? extends Vertex3D> verticesToAdd)
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

         List<Face3D> newFaces = EuclidPolytopeConstructionTools.computeVertexNeighborFaces(vertexToAdd, firstFace.getEdges(), Collections.emptyList(),
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

   public void updateVertices()
   { // FIXME this is slow, maybe there's a way to keep track of the vertices as the polytope is being built.
      vertices.clear();
      faces.stream().flatMap(face -> face.getVertices().stream()).distinct().forEach(vertices::add);
   }

   public void updateEdges()
   {
      edges.clear();
      for (Face3D face : faces)
         edges.addAll(face.getEdges());
   }

   public void updateBoundingBox()
   {
      boundingBox.setToNaN();

      for (int faceIndex = 0; faceIndex < faces.size(); faceIndex++)
         boundingBox.combine(faces.get(faceIndex).getBoundingBox());
   }

   public void updateCentroidAndVolume()
   {
      volume = EuclidPolytopeConstructionTools.computeConvexPolytope3DVolume(this, centroid);
   }

   public void removeFaces(Collection<Face3D> facesToRemove)
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
   public boolean containsNaN()
   {
      return ConvexPolytope3DReadOnly.super.containsNaN();
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
   public HalfEdge3D getHalfEdge(int index)
   {
      return edges.get(index);
   }

   @Override
   public List<HalfEdge3D> getHalfEdges()
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

   @Override
   public Face3D getClosestFace(Point3DReadOnly point)
   {
      return (Face3D) ConvexPolytope3DReadOnly.super.getClosestFace(point);
   }

   @Override
   public Point3DReadOnly getCentroid()
   {
      return centroid;
   }

   @Override
   public double getVolume()
   {
      return volume;
   }

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

   @Override
   public boolean geometricallyEquals(ConvexPolytope3D other, double epsilon)
   {
      return ConvexPolytope3DReadOnly.super.geometricallyEquals(other, epsilon);
   }

   @Override
   public boolean epsilonEquals(ConvexPolytope3D other, double epsilon)
   {
      return ConvexPolytope3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   @Override
   public String toString()
   {
      return EuclidShapeIOTools.getConvexPolytope3DString(this);
   }
}
