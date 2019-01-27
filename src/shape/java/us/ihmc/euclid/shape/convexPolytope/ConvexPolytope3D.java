package us.ihmc.euclid.shape.convexPolytope;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Simplex3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.ConvexPolytope3DFactories;
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

         onFaceList.clear();
         createFacesFromVisibleSilhouetteAndOnFaceList(visibleSilhouetteList, onFaceList, vertexToAdd, epsilon);
      }

      return true;
   }

   private boolean handleMultipleFaceCase(Vertex3D vertexToAdd, double epsilon)
   {
      getVisibleFaces(visibleFaces, vertexToAdd, epsilon);

      if (visibleFaces.isEmpty())
      {
         return false;
      }
      getFacesWhichPointIsOn(vertexToAdd, onFaceList, epsilon);

      getSilhouetteFaces(silhouetteFaces, nonSilhouetteFaces, visibleFaces);
      HalfEdge3D firstHalfEdgeForSilhouette = null;

      if (onFaceList.size() > 0)
      {
         if (EuclidPolytopeTools.isPointDirectlyAboveOrBelowAnyFace(onFaceList, vertexToAdd))
         {
            /*
             * @formatter:off
             * TODO I believe this test is unnecessary as it seems evident that the new point either belongs to none or all of onFaceList.
             * So checking only one is enough, which is done right after when testing if firstVisibleEdge == null.
             * For instance:
             *    1- onFaceList.size() == 1: trivial.
             *    2- onFaceList.size() == 2: then if the point belongs to 1 face and still be on the plane of the second
             *       then it has to belong to their common edge which the only set of coordinates where both planes intersect.
             *       So it has to belong to both faces.
             *    3- onFaceList.size() == 3: then if the point belongs to 1 face and still be on the planes of the two other faces,
             *       then it has to be on their vertex (onFaceList.size() == 3) which is the only point where all three planes intersect.
             *       in both cases, the point belongs to all faces. So it belongs to all faces.
             *    4- onFaceList.size() > 3: it is the same a the previous case, only one location exists where all planes intersect,
             *       so the new vertex either belongs to none or all faces.
             * @formatter:on
             */
            // The point actually belongs to one of the faces.
            return false;
         }

         HalfEdge3D firstVisibleEdge = onFaceList.get(0).lineOfSightStart(vertexToAdd);

         if (firstVisibleEdge == null)
            return false;

         firstHalfEdgeForSilhouette = firstVisibleEdge.getTwinEdge();
      }
      else
      {
         firstHalfEdgeForSilhouette = getSeedEdgeForSilhouetteCalculation(visibleFaces, silhouetteFaces.get(0));
      }

      if (firstHalfEdgeForSilhouette == null)
         return false;

      getVisibleSilhouetteUsingSeed(visibleSilhouetteList, firstHalfEdgeForSilhouette, visibleFaces);

      if (visibleSilhouetteList.isEmpty())
         return false;

      removeFaces(nonSilhouetteFaces);
      removeFaces(silhouetteFaces);
      createFacesFromVisibleSilhouetteAndOnFaceList(visibleSilhouetteList, onFaceList, vertexToAdd, epsilon);

      return true;
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
      faces.stream().flatMap(face -> face.getEdges().stream()).distinct().forEach(edges::add);
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

   public void getSilhouetteFaces(List<Face3D> silhouetteFacesToPack, List<Face3D> nonSilhouetteFacesToPack, List<Face3D> visibleFaceList)
   {
      if (silhouetteFacesToPack != null)
         silhouetteFacesToPack.clear();
      if (nonSilhouetteFacesToPack != null)
         nonSilhouetteFacesToPack.clear();
      for (int i = 0; i < visibleFaceList.size(); i++)
      {
         Face3D candidateFace = visibleFaceList.get(i);

         boolean allNeighbouringFacesVisible = true;
         for (int j = 0; j < candidateFace.getNumberOfEdges(); j++)
            allNeighbouringFacesVisible &= visibleFaceList.contains(candidateFace.getNeighbouringFace(j));

         if (allNeighbouringFacesVisible && nonSilhouetteFacesToPack != null)
            nonSilhouetteFacesToPack.add(candidateFace);
         else if (silhouetteFacesToPack != null)
            silhouetteFacesToPack.add(candidateFace);
      }
   }

   public void getVisibleSilhouette(Point3DReadOnly vertex, List<HalfEdge3D> visibleSilhouetteToPack, double epsilon)
   {
      Face3D leastVisibleFace = getVisibleFaces(visibleFaces, vertex, epsilon);

      if (visibleFaces.isEmpty())
      {
         return;
      }
      getFacesWhichPointIsOn(vertex, onFaceList, epsilon);
      getSilhouetteFaces(silhouetteFaces, nonSilhouetteFaces, visibleFaces);
      HalfEdge3D firstHalfEdgeForSilhouette;
      if (onFaceList.size() > 0)
         firstHalfEdgeForSilhouette = onFaceList.get(0).lineOfSightStart(vertex).getTwinEdge();
      else
         firstHalfEdgeForSilhouette = getSeedEdgeForSilhouetteCalculation(visibleFaces, leastVisibleFace);
      getVisibleSilhouetteUsingSeed(visibleSilhouetteToPack, firstHalfEdgeForSilhouette, visibleFaces);
   }

   public void getVisibleSilhouetteUsingSeed(List<HalfEdge3D> visibleSilhouetteToPack, HalfEdge3D seedHalfEdge, Collection<Face3D> silhouetteFaceList)
   {
      HalfEdge3D halfEdgeUnderConsideration = seedHalfEdge;
      visibleSilhouetteToPack.clear();
      int numberOfEdges = getNumberOfEdges();
      int count;
      for (count = 0; count < numberOfEdges; count++)
      {
         visibleSilhouetteToPack.add(halfEdgeUnderConsideration.getTwinEdge());
         Vertex3D destinationVertex = halfEdgeUnderConsideration.getDestination();
         for (int i = 0; i < destinationVertex.getNumberOfAssociatedEdges(); i++)
         {
            if (silhouetteFaceList.contains(destinationVertex.getAssociatedEdge(i).getFace())
                  && !silhouetteFaceList.contains(destinationVertex.getAssociatedEdge(i).getTwinEdge().getFace()))
            {
               halfEdgeUnderConsideration = destinationVertex.getAssociatedEdge(i);
               break;
            }
         }
         if (halfEdgeUnderConsideration == seedHalfEdge)
            break;
      }
      if (count == numberOfEdges && faces.size() > 1)
      {
         visibleSilhouetteToPack.clear();
      }
   }

   public HalfEdge3D getSeedEdgeForSilhouetteCalculation(List<Face3D> visibleFaceList, Face3D leastVisibleFace)
   {
      if (faces.size() == 1)
         return faces.get(0).getEdge(0);
      HalfEdge3D seedEdge = null;
      HalfEdge3D seedEdgeCandidate = leastVisibleFace.getEdge(0);
      for (int i = 0; seedEdge == null && i < leastVisibleFace.getNumberOfEdges(); i++)
      {
         if (!visibleFaceList.contains(seedEdgeCandidate.getTwinEdge().getFace()))
            seedEdge = seedEdgeCandidate;
         seedEdgeCandidate = seedEdgeCandidate.getNextEdge();
      }
      return seedEdge;
   }

   private void createFacesFromVisibleSilhouetteAndOnFaceList(List<HalfEdge3D> silhouetteEdges, List<Face3D> onFaceList, Vertex3D vertexToAdd, double epsilon)
   {
      HalfEdge3D previousLeadingEdge = null, trailingEdge = null;

      if (onFaceList.contains(silhouetteEdges.get(0).getFace()))
      {
         previousLeadingEdge = silhouetteEdges.get(0).getFace().lineOfSightStart(vertexToAdd);
         if (previousLeadingEdge == null)
         {
            System.out.println("vertex to add: " + vertexToAdd);
            System.out.println("Face: " + silhouetteEdges.get(0).getFace().toString());
            System.out.println("Polytope: " + toString());
         }
         silhouetteEdges.get(0).getFace().addVertex(vertexToAdd, epsilon);
         trailingEdge = previousLeadingEdge.getNextEdge();
      }
      else
      {
         Face3D firstFace = createFaceFromTwinEdgeAndVertex(vertexToAdd, silhouetteEdges.get(0), epsilon);
         previousLeadingEdge = firstFace.getEdge(0).getNextEdge();
         trailingEdge = firstFace.getEdge(0).getPreviousEdge();
      }

      for (int i = 1; i < silhouetteEdges.size(); i++)
      {
         if (onFaceList.contains(silhouetteEdges.get(i).getFace()))
         {
            Face3D faceToExtend = silhouetteEdges.get(i).getFace();
            HalfEdge3D tempEdge = faceToExtend.lineOfSightStart(vertexToAdd);
            faceToExtend.addVertex(vertexToAdd, epsilon);
            if (tempEdge == null)
               System.out.println("This again");
            if (tempEdge.getNextEdge() == null)
               System.out.println("WTF");
            twinEdges(previousLeadingEdge, tempEdge.getNextEdge());
            previousLeadingEdge = tempEdge;
         }
         else
         {
            Face3D newFace = createFaceFromTwinEdgeAndVertex(vertexToAdd, silhouetteEdges.get(i), epsilon);
            twinEdges(previousLeadingEdge, newFace.getEdge(0).getPreviousEdge());
            previousLeadingEdge = newFace.getEdge(0).getNextEdge();
         }
      }
      twinEdges(previousLeadingEdge, trailingEdge);
   }

   private void twinEdges(HalfEdge3D halfEdge1, HalfEdge3D halfEdge2)
   {
      assert halfEdge1.getOrigin() == halfEdge2.getDestination() && halfEdge1.getDestination() == halfEdge2.getOrigin(); // TODO
      halfEdge1.setTwinEdge(halfEdge2);
      halfEdge2.setTwinEdge(halfEdge1);
   }

   private Face3D createFaceFromTwinEdgeAndVertex(Vertex3D vertex, HalfEdge3D twinEdge, double epsilon)
   {
      Face3D newFace = ConvexPolytope3DFactories.newFace3DFromVertexAndTwinEdge(vertex, twinEdge, epsilon);
      faces.add(newFace);
      return newFace;
   }

   private void removeFaces(List<Face3D> facesToRemove)
   {
      for (int i = 0; i < facesToRemove.size(); i++)
      {
         removeFace(facesToRemove.get(i));
      }
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

   public void getFacesWhichPointIsOn(Point3DReadOnly vertexUnderConsideration, List<Face3D> faceReferenceToPack, double epsilon)
   {
      faceReferenceToPack.clear();
      faceReferenceToPack.addAll(EuclidPolytopeTools.getInPlaneFaces(faces, vertexUnderConsideration, epsilon));
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
         for (int i = 0; i < bestVertex.getNumberOfAssociatedEdges(); i++)
         {
            tempVector.set(bestVertex.getAssociatedEdge(i).getDestination());
            double dotProduct = supportDirection.dot(tempVector);
            if (dotProduct > maxDotProduct)
            {
               vertexCandidate = bestVertex.getAssociatedEdge(i).getDestination();
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
