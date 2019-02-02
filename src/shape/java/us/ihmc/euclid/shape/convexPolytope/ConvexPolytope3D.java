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
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Simplex3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
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
   private final ArrayList<Face3D> visibleFaces = new ArrayList<>();
   private final ArrayList<Face3D> silhouetteFaces = new ArrayList<>();
   private final ArrayList<Face3D> nonSilhouetteFaces = new ArrayList<>();
   private final ArrayList<Face3D> onFaceList = new ArrayList<>();
   private final ArrayList<HalfEdge3D> visibleSilhouetteList = new ArrayList<>();

   private Vector3D tempVector = new Vector3D();
   private Point3D centroid = new Point3D();

   public ConvexPolytope3D()
   {
   }

   public ConvexPolytope3D(ConvexPolytope3DReadOnly polytope)
   {
      set(polytope);
   }

   public void addVertex(double x, double y, double z, double epsilon)
   {
      addVertex(new Vertex3D(x, y, z), epsilon);
   }

   public void addVertex(Point3DReadOnly vertexToAdd, double epsilon)
   {
      Vertex3D vertex = new Vertex3D(vertexToAdd);
      addVertex(vertex, epsilon);
   }

   /**
    * Adds a polytope vertex to the current polytope. In case needed faces are removed and recreated.
    * This will result in garbage. Fix if possible
    *
    * @param vertexToAdd
    * @param epsilon
    * @return
    */
   public boolean addVertex(Vertex3D vertexToAdd, double epsilon)
   {
      boolean isPolytopeModified;

      if (faces.size() == 0)
         isPolytopeModified = handleNoFaceCase(vertexToAdd, epsilon);
      else if (faces.size() == 1)
         isPolytopeModified = handleSingleFaceCase(vertexToAdd, epsilon);
      else
         isPolytopeModified = handleMultipleFaceCase(vertexToAdd, epsilon);

      if (isPolytopeModified)
      {
         updateEdges();
         updateVertices();
         updateBoundingBox();
      }

      return isPolytopeModified;
   }

   private boolean handleNoFaceCase(Vertex3D vertexToAdd, double epsilon)
   {
      // Polytope is empty. Creating face and adding the vertex
      Face3D newFace = new Face3D(Axis.Z);
      newFace.addVertex(vertexToAdd, epsilon);
      faces.add(newFace);

      return true;
   }

   private boolean handleSingleFaceCase(Vertex3D vertexToAdd, double epsilon)
   {
      Face3D firstFace = faces.get(0);

      if (firstFace.getNumberOfEdges() <= 2)
      {
         firstFace.addVertex(vertexToAdd, epsilon);
      }
      else if (firstFace.isPointInFacePlane(vertexToAdd, epsilon))
      {
         if (!firstFace.isPointDirectlyAboveOrBelow(vertexToAdd))
            firstFace.addVertex(vertexToAdd, epsilon);
      }
      else
      {
         if (firstFace.canObserverSeeFace(vertexToAdd))
            firstFace.flip();

         visibleSilhouetteList.clear();
         HalfEdge3D halfEdge = firstFace.getEdge(0);

         for (int i = 0; i < firstFace.getNumberOfEdges(); i++)
         {
            visibleSilhouetteList.add(halfEdge);
            halfEdge = halfEdge.getPreviousEdge();
         }

         faces.addAll(EuclidPolytopeTools.computeVertexNeighborFaces(vertexToAdd, visibleSilhouetteList, Collections.emptyList(), epsilon));
      }

      return true;
   }

   private boolean handleMultipleFaceCase(Vertex3D vertexToAdd, double epsilon)
   {
      Set<Face3D> visibleFaces = new HashSet<>();
      List<HalfEdge3D> silhouette = EuclidPolytopeTools.getSilhouette(faces, vertexToAdd, epsilon, visibleFaces, onFaceList);

      if (silhouette != null)
      {
         removeFaces(visibleFaces);
         faces.addAll(EuclidPolytopeTools.computeVertexNeighborFaces(vertexToAdd, silhouette, onFaceList, epsilon));
         return true;
      }
      else
      {
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

   public Vector3DReadOnly getFaceNormalAt(Point3DReadOnly point)
   {
      return getFaceContainingPointClosestTo(point).getNormal();
   }

   @Override
   public void applyTransform(Transform transform)
   {
      // Applying the transform to the vertices is less expensive computationally but getting the vertices is hard
      for (int i = 0; i < vertices.size(); i++)
         vertices.get(i).applyTransform(transform);
      updateBoundingBox();
      // FIXME this method introduces inconsistency in Face3D.
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      // Applying the transform to the vertices is less expensive computationally but getting the vertices is hard
      for (int i = 0; i < vertices.size(); i++)
         vertices.get(i).applyInverseTransform(transform);
      updateBoundingBox();
      // FIXME this method introduces inconsistency in Face3D.
   }

   private void updateVertices()
   {
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

   /*
    * TODO: The visibility factor is based on the distance from the query to the face's support plane.
    * Maybe, a better metric is the consideration of the angle at which the face is seen:
    * atan(face.distanceToPlane(query) / face.distance(face.orthogonalProjectionToPlane(query)). As a
    * result, the farther the observer is away from the face, the greater the distance from the face's
    * support plane has to be conserve the same visibility factor. However, the way of computing the
    * least visible face may not even matter.
    */
   public Face3D getVisibleFaces(List<Face3D> faceReferencesToPack, Point3DReadOnly vertexUnderConsideration, double epsilon)
   {
      faceReferencesToPack.clear();
      faceReferencesToPack.addAll(EuclidPolytopeTools.getVisibleFaces(faces, vertexUnderConsideration, epsilon));
      return faceReferencesToPack.isEmpty() ? null : faceReferencesToPack.get(0);
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

   private Face3D isInteriorPointInternal(Point3DReadOnly pointToCheck, double epsilon)
   {
      if (faces.size() == 0)
         return null;
      else if (faces.size() == 1)
         return faces.get(0).isPointInside(pointToCheck, epsilon) ? null : faces.get(0);

      for (int i = 0; i < faces.size(); i++)
      {
         tempVector.sub(pointToCheck, faces.get(i).getEdge(0).getOrigin());
         double dotProduct = tempVector.dot(faces.get(i).getNormal());
         if (dotProduct >= epsilon || faces.get(i).getNumberOfEdges() < 3)
         {
            return faces.get(i);
         }
      }
      return null;
   }

   public boolean isInteriorPoint(Point3DReadOnly pointToCheck, double epsilon)
   {
      return isInteriorPointInternal(pointToCheck, epsilon) == null;
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
      for (int i = 0; i < faces.size(); i++)
      {
         if (faces.get(i).containsNaN())
            return true;
      }
      return false;
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
      visibleFaces.clear();
      silhouetteFaces.clear();
      nonSilhouetteFaces.clear();
      onFaceList.clear();
      visibleSilhouetteList.clear();
   }

   @Override
   public double distance(Point3DReadOnly point)
   {
      return getFaceContainingPointClosestTo(point).distance(point);
   }

   public Face3D getFaceContainingPointClosestTo(Point3DReadOnly point)
   {
      if (faces.size() == 0)
         return null;
      if (faces.size() == 1)
         return faces.get(0);

      Face3D closestFace = null;
      double minDistance = Double.POSITIVE_INFINITY;

      for (int faceIndex = 0; faceIndex < faces.size(); faceIndex++)
      {
         Face3D face = faces.get(faceIndex);
         double distance = face.distance(point);

         if (distance < minDistance)
         {
            closestFace = face;
            minDistance = distance;
         }
         // TODO Consider doing a
      }

      return closestFace;
   }

   private void updateCentroid()
   {
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
      getFaceContainingPointClosestTo(point).getSupportVectorDirectionTo(point, supportVectorToPack);
   }

   @Override
   public Simplex3D getSmallestSimplexMemberReference(Point3DReadOnly point)
   {
      return getFaceContainingPointClosestTo(point).getSmallestSimplexMemberReference(point);
   }

   @Override
   public String toString()
   {
      return EuclidPolytopeIOTools.getConvexPolytope3DString(this);
   }
}
