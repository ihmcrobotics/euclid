package us.ihmc.euclid.shape.convexPolytope;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.PolytopeListener;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Simplex3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeIOTools;
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
public class ConvexPolytope3D implements ConvexPolytope3DReadOnly, Simplex3D, Clearable, Transformable, Settable<ConvexPolytope3DReadOnly>
{
   private final static boolean DEBUG = false;
   private final ArrayList<Vertex3D> vertices = new ArrayList<>();
   private final ArrayList<HalfEdge3D> edges = new ArrayList<>();
   private final ArrayList<Face3D> faces = new ArrayList<>();
   /**
    * Bounding box for the polytope
    */
   private boolean boundingBoxNeedsUpdating = false;
   private final BoundingBox3D boundingBox = new BoundingBox3D(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY,
                                                               Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
   private final ArrayList<Face3D> visibleFaces = new ArrayList<>();
   private final ArrayList<Face3D> silhouetteFaces = new ArrayList<>();
   private final ArrayList<Face3D> nonSilhouetteFaces = new ArrayList<>();
   private final ArrayList<Face3D> onFaceList = new ArrayList<>();
   private final ArrayList<HalfEdge3D> visibleSilhouetteList = new ArrayList<>();

   private Vector3D tempVector = new Vector3D();
   private Point3D centroid = new Point3D();
   private final PolytopeListener listener;

   public ConvexPolytope3D()
   {
      listener = null;
   }

   public ConvexPolytope3D(PolytopeListener listener)
   {
      this.listener = listener;
      this.listener.attachPolytope(this);
   }

   public ConvexPolytope3D(ConvexPolytope3DReadOnly polytope)
   {
      set(polytope);
      boundingBoxNeedsUpdating = true;
      listener = null;
   }

   public void getBoundingBox(BoundingBox3D boundingBoxToPack)
   {
      if (boundingBoxNeedsUpdating)
      {
         updateBoundingBox();
         boundingBoxNeedsUpdating = false;
      }
      boundingBoxToPack.set(boundingBox);
   }

   private void updateBoundingBox()
   {
      double xMin = Double.POSITIVE_INFINITY;
      double yMin = Double.POSITIVE_INFINITY;
      double zMin = Double.POSITIVE_INFINITY;

      double xMax = Double.NEGATIVE_INFINITY;
      double yMax = Double.NEGATIVE_INFINITY;
      double zMax = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < faces.size(); i++)
      {
         double x = faces.get(i).getMinX();
         double y = faces.get(i).getMinY();
         double z = faces.get(i).getMinZ();

         if (x < xMin)
            xMin = x;
         if (y < yMin)
            yMin = y;
         if (z < zMin)
            zMin = z;

         x = faces.get(i).getMaxX();
         y = faces.get(i).getMaxY();
         z = faces.get(i).getMaxZ();
         if (x > xMax)
            xMax = x;
         if (y > yMax)
            yMax = y;
         if (z > zMax)
            zMax = z;
      }
      boundingBox.set(xMin, yMin, zMin, xMax, yMax, zMax);
   }

   public int getNumberOfVertices()
   {
      if (getNumberOfFaces() < 2)
      {
         updateVertices();
         return vertices.size();
      }
      else
      { // Polyhedron formula for quick calc
         return getNumberOfEdges() - getNumberOfFaces() + 2;
      }
   }

   public List<Vertex3D> getVertices()
   {
      updateVertices();
      return vertices;
   }

   public void getVertices(List<Point3D> verticesToPack)
   {
      updateVertices();
      for (int i = 0; i < vertices.size(); i++)
         verticesToPack.get(i).set(vertices.get(i));
   }

   private void updateVertices()
   {
      unmarkAllFaces();
      vertices.clear();
      for (int i = 0; i < faces.size(); i++)
      {
         for (int j = 0; j < faces.get(i).getNumberOfEdges(); j++)
         {
            if (!faces.get(i).getEdge(j).getOrigin().isAnyFaceMarked())
            {
               vertices.add(faces.get(i).getEdge(j).getOrigin());
            }
         }
         faces.get(i).mark();
      }
   }

   public Vertex3D getVertex(int index)
   {
      updateVertices();
      return vertices.get(index);
   }

   public int getNumberOfEdges()
   {
      updateEdges();
      return edges.size() / 2;
   }

   public List<HalfEdge3D> getEdges()
   {
      updateEdges();
      return edges;
   }

   public HalfEdge3D getEdge(int index)
   {
      updateEdges();
      return edges.get(index);
   }

   private void updateEdges()
   {
      edges.clear();
      for (int i = 0; i < faces.size(); i++)
      {
         List<HalfEdge3D> faceEdgeList = faces.get(i).getEdges();
         for (int j = 0; j < faceEdgeList.size(); j++)
         {
            edges.add(faceEdgeList.get(j));
         }
      }
   }

   public int getNumberOfFaces()
   {
      return faces.size();
   }

   public List<Face3D> getFaces()
   {
      return faces;
   }

   public Face3D getFace(int index)
   {
      return faces.get(index);
   }

   public Vector3DReadOnly getFaceNormalAt(Point3DReadOnly point)
   {
      return getFaceContainingPointClosestTo(point).getNormal();
   }

   @Override
   public void applyTransform(Transform transform)
   {
      // Applying the transform to the vertices is less expensive computationally but getting the vertices is hard
      updateVertices();
      for (int i = 0; i < vertices.size(); i++)
         vertices.get(i).applyTransform(transform);
      boundingBoxNeedsUpdating = true;
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      // Applying the transform to the vertices is less expensive computationally but getting the vertices is hard
      updateVertices();
      for (int i = 0; i < vertices.size(); i++)
         vertices.get(i).applyInverseTransform(transform);
      boundingBoxNeedsUpdating = true;
   }

   private void unmarkAllFaces()
   {
      for (int i = 0; i < faces.size(); i++)
         faces.get(i).unmark();
   }

   public void addVertices(double epsilon, Point3D... vertices)
   {
      for (int i = 0; i < vertices.length; i++)
         addVertex(vertices[i], epsilon);
   }

   public void addVertices(double epsilon, List<Point3D> vertices)
   {
      for (int i = 0; i < vertices.size(); i++)
         addVertex(vertices.get(i), epsilon);
   }

   public void addVertices(List<Vertex3D> vertices, double epsilon)
   {
      for (int i = 0; i < vertices.size(); i++)
         addVertex(vertices.get(i), epsilon);
   }

   public void addVertex(double epsilon, double... coordinates)
   {
      addVertex(new Vertex3D(coordinates[0], coordinates[1], coordinates[2]), epsilon);
   }

   public void addVertex(double x, double y, double z, double epsilon)
   {
      addVertex(new Vertex3D(x, y, z), epsilon);
   }

   public void addVertex(Point3D vertexToAdd, double epsilon)
   {
      addVertex(new Vertex3D(vertexToAdd), epsilon);
   }

   /**
    * Adds a polytope vertex to the current polytope. In case needed faces are removed and recreated.
    * This will result in garbage. Fix if possible
    * 
    * @param vertexToAdd
    * @param epsilon
    * @return
    */
   public void addVertex(Vertex3D vertexToAdd, double epsilon)
   {
      if (faces.size() == 0)
      {
         // Polytope is empty. Creating face and adding the vertex
         Face3D newFace = new Face3D(Axis.Z);
         newFace.addVertex(vertexToAdd, epsilon);
         faces.add(newFace);
         boundingBoxNeedsUpdating = true;
         updateListener();
         return;
      }
      else if (faces.size() == 1)
      {
         if (faces.get(0).isPointInFacePlane(vertexToAdd, epsilon))
         {
            if (!faces.get(0).isPointInside(vertexToAdd, epsilon))
               faces.get(0).addVertex(vertexToAdd, epsilon);
            updateListener();
            return;
         }
         else
         {
            if (faces.get(0).canObserverSeeFace(vertexToAdd))
               faces.get(0).reverseFaceNormal();

            visibleSilhouetteList.clear();
            HalfEdge3D halfEdge = faces.get(0).getEdge(0);
            if (listener != null)
               listener.udpateVisibleEdgeSeed(halfEdge);
            for (int i = 0; i < faces.get(0).getNumberOfEdges(); i++)
            {
               visibleSilhouetteList.add(halfEdge);
               halfEdge = halfEdge.getPreviousEdge();
            }
            if (listener != null)
               listener.updateVisibleSilhouette(visibleSilhouetteList);
            onFaceList.clear();
            createFacesFromVisibleSilhouetteAndOnFaceList(visibleSilhouetteList, onFaceList, vertexToAdd, epsilon);
         }
         boundingBoxNeedsUpdating = true;
         updateListener();
         return;
      }

      Face3D visibleFaceSeed = getVisibleFaces(visibleFaces, vertexToAdd, epsilon);
      if (visibleFaces.isEmpty())
      {
         updateListener();
         return;
      }
      getFacesWhichPointIsOn(vertexToAdd, onFaceList, epsilon);
      if (DEBUG)
         System.out.println("Visible faces: " + visibleFaces.size() + ", On Faces: " + onFaceList.size());
      getSilhouetteFaces(silhouetteFaces, nonSilhouetteFaces, visibleFaces);
      HalfEdge3D firstHalfEdgeForSilhouette = null;
      if (listener != null)
      {
         listener.updateOnFaceList(onFaceList);
         listener.updateVisibleFaceList(Arrays.asList(visibleFaceSeed));
      }
      if (onFaceList.size() > 0)
      {
         if (checkIsInteriorPointOf(onFaceList, vertexToAdd, epsilon))
            return;
         HalfEdge3D firstVisibleEdge = getFirstVisibleEdgeFromOnFaceList(onFaceList, visibleFaces); //onFaceList.get(0).getFirstVisibleEdge(vertexToAdd);
         if (firstVisibleEdge == null)
            return;
         firstHalfEdgeForSilhouette = firstVisibleEdge.getTwinEdge();
      }
      else
      {
         firstHalfEdgeForSilhouette = getSeedEdgeForSilhouetteCalculation(visibleFaces, silhouetteFaces.get(0));
      }
      if (listener != null)
         listener.udpateVisibleEdgeSeed(firstHalfEdgeForSilhouette);
      if (firstHalfEdgeForSilhouette == null)
      {
         if (DEBUG)
            System.out.println("Seed edge was null, aborting. On faces: " + onFaceList.size() + ", visible: " + visibleFaces.size());
         return;
      }
      getVisibleSilhouetteUsingSeed(visibleSilhouetteList, firstHalfEdgeForSilhouette, visibleFaces);
      if (listener != null)
         listener.updateVisibleSilhouette(visibleSilhouetteList);
      if (visibleSilhouetteList.isEmpty())
      {
         if (DEBUG)
            System.out.println("Empty visible silhouette ");
         updateListener();
         return;
      }
      removeFaces(nonSilhouetteFaces);
      removeFaces(silhouetteFaces);
      createFacesFromVisibleSilhouetteAndOnFaceList(visibleSilhouetteList, onFaceList, vertexToAdd, epsilon);
      boundingBoxNeedsUpdating = true;
      updateListener();
   }

   private boolean checkIsInteriorPointOf(ArrayList<Face3D> onFaceList, Point3DReadOnly vertexToAdd, double epsilon)
   {
      for (int i = 0; i < onFaceList.size(); i++)
      {
         if (onFaceList.get(i).lineOfSightStart(vertexToAdd) == null)
            return true;
      }
      return false;
   }

   private HalfEdge3D getFirstVisibleEdgeFromOnFaceList(ArrayList<Face3D> onFaceList, ArrayList<Face3D> visibleFaces)
   {
      Face3D firstFace = onFaceList.get(0);
      HalfEdge3D edgeUnderConsideration = firstFace.getEdge(0);
      for (int i = 0; i < firstFace.getNumberOfEdges(); i++)
      {
         if (!visibleFaces.contains(edgeUnderConsideration.getNextEdge().getTwinEdge().getFace())
               && !onFaceList.contains(edgeUnderConsideration.getNextEdge().getTwinEdge().getFace())
               && visibleFaces.contains(edgeUnderConsideration.getTwinEdge().getFace()))
            return edgeUnderConsideration;
         else
            edgeUnderConsideration = edgeUnderConsideration.getNextEdge();
      }
      return null;
   }

   public void updateListener()
   {
      if (listener != null)
      {
         listener.updateAll();
      }
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
      HalfEdge3D firstHalfEdgeForSilhouette = onFaceList.size() > 0 ? onFaceList.get(0).lineOfSightStart(vertex).getTwinEdge()
            : getSeedEdgeForSilhouetteCalculation(visibleFaces, leastVisibleFace);
      getVisibleSilhouetteUsingSeed(visibleSilhouetteToPack, firstHalfEdgeForSilhouette, visibleFaces);
   }

   public void getVisibleSilhouetteUsingSeed(List<HalfEdge3D> visibleSilhouetteToPack, HalfEdge3D seedHalfEdge, List<Face3D> silhouetteFaceList)
   {
      HalfEdge3D halfEdgeUnderConsideration = seedHalfEdge;
      visibleSilhouetteToPack.clear();
      int numberOfEdges = getNumberOfEdges();
      int count;
      for (count = 0; count < numberOfEdges; count++)
      {
         if (DEBUG)
         {
            if (halfEdgeUnderConsideration == null)
               System.out.println("Half edge null " + faces.size());
            if (visibleSilhouetteToPack == null)
               System.out.println("visible list null");
            if (halfEdgeUnderConsideration.getTwinEdge() == null)
               System.out.println("Twing half edge null");
         }

         visibleSilhouetteToPack.add(halfEdgeUnderConsideration.getTwinEdge());
         Vertex3D destinationVertex = halfEdgeUnderConsideration.getDestination();
         for (int i = 0; i < destinationVertex.getNumberOfAssociatedEdges(); i++)
         {
            if (DEBUG)
            {
               if (destinationVertex.getAssociatedEdge(i) == null)
                  System.out.println("Associated edge is null");
               if (destinationVertex.getAssociatedEdge(i).getTwinEdge() == null)
                  System.out.println("Associated edge twin is null\n" + toString());
               if (destinationVertex.getAssociatedEdge(i).getTwinEdge().getFace() == null)
                  System.out.println("Associated edge twin face is null");
            }
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
         if (DEBUG)
         {
            System.out.println("Could not determine visible silhouette " + onFaceList.size() + ", " + silhouetteFaceList.size() + ", "
                  + visibleSilhouetteToPack.size());
            System.out.println("On face size: " + onFaceList.size());
            for (int i = 0; i < onFaceList.size(); i++)
            {
               System.out.println(onFaceList.get(i).toString());
            }
            System.out.println("Visible face size: " + visibleFaces.size());
            for (int i = 0; i < visibleFaces.size(); i++)
            {
               System.out.println(visibleFaces.get(i).toString());
            }
         }
         if (listener != null)
         {
            listener.updateOnFaceList(onFaceList);
            listener.updateVisibleFaceList(visibleFaces);
            listener.updateVisibleSilhouette(visibleSilhouetteToPack);
         }
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

   //   public S getSeedEdgeForSilhouetteCalculation(List<U> silhouetteFaceList)
   //   {
   //      U seedFaceCandidate = silhouetteFaceList.get(0);
   //      S seedEdgeCandidate = seedFaceCandidate.getEdge(0);
   //      for (int i = 0; i < seedFaceCandidate.getNumberOfEdges(); i++)
   //      {
   //         if (silhouetteFaceList.contains(seedEdgeCandidate.getTwinHalfEdge().getFace())
   //               && !silhouetteFaceList.contains(seedEdgeCandidate.getNextHalfEdge().getTwinHalfEdge().getFace()))
   //            break;
   //         seedEdgeCandidate = seedEdgeCandidate.getNextHalfEdge();
   //      }
   //      seedEdgeCandidate = seedEdgeCandidate.getNextHalfEdge();
   //      return seedEdgeCandidate;
   //   }

   //   private void createFacesFromVisibleSilhouette(PolytopeVertex vertexToAdd)
   //   {
   //      U firstNewFace = createFaceFromTwinEdgeAndVertex(vertexToAdd, visibleSilhouetteList.get(0));
   //      twinEdges(visibleSilhouetteList.get(0), firstNewFace.getEdge(0));
   //      for (int i = 1; i < visibleSilhouetteList.size(); i++)
   //      {
   //         U newFace = createFaceFromTwinEdgeAndVertex(vertexToAdd, visibleSilhouetteList.get(i));
   //         twinEdges(visibleSilhouetteList.get(i - 1).getTwinHalfEdge().getNextHalfEdge(), newFace.getEdge(0).getPreviousHalfEdge());
   //      }
   //      twinEdges(visibleSilhouetteList.get(visibleSilhouetteList.size() - 1).getTwinHalfEdge().getNextHalfEdge(), firstNewFace.getEdge(0).getPreviousHalfEdge());
   //   }

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
      if (halfEdge1.getOrigin() != halfEdge2.getDestination() && halfEdge1.getDestination() != halfEdge2.getOrigin())
         System.out.println("This should print \n\n\n\n");
      halfEdge1.setTwinEdge(halfEdge2);
      halfEdge2.setTwinEdge(halfEdge1);
   }

   private Face3D createFaceFromTwinEdgeAndVertex(Vertex3D vertex, HalfEdge3D twinEdge, double epsilon)
   {
      Vector3D initialNormal = new Vector3D();
      initialNormal.sub(vertex, centroid);
      Face3D newFace = new Face3D(initialNormal);
      faces.add(newFace);
      newFace.addVertex(twinEdge.getDestination(), epsilon);
      newFace.addVertex(twinEdge.getOrigin(), epsilon);
      newFace.addVertex(vertex, epsilon);
      twinEdges(newFace.getEdge(0), twinEdge);
      return newFace;
   }

   private void removeFaces(List<Face3D> facesToRemove)
   {
      for (int i = 0; i < facesToRemove.size(); i++)
      {
         removeFace(facesToRemove.get(i));
      }
   }

   public Face3D getVisibleFaces(List<Face3D> faceReferencesToPack, Point3DReadOnly vertexUnderConsideration, double epsilon)
   {
      Face3D leastVisibleFace = null;
      double minimumVisibilityProduct = Double.POSITIVE_INFINITY;
      faceReferencesToPack.clear();
      for (int i = 0; i < faces.size(); i++)
      {
         double visibilityProduct = faces.get(i).signedDistanceToPlane(vertexUnderConsideration);
         if (visibilityProduct > epsilon)
         {
            faceReferencesToPack.add(faces.get(i));
            if (visibilityProduct < minimumVisibilityProduct)
            {
               leastVisibleFace = faces.get(i);
               minimumVisibilityProduct = visibilityProduct;
            }
         }
      }
      return leastVisibleFace;
   }

   public void getFacesWhichPointIsOn(Point3DReadOnly vertexUnderConsideration, List<Face3D> faceReferenceToPack, double epsilon)
   {
      faceReferenceToPack.clear();

      for (int i = 0; i < faces.size(); i++)
      {
         if (faces.get(i).isPointInFacePlane(vertexUnderConsideration, epsilon))
         {
            faceReferenceToPack.add(faces.get(i));
         }
      }
   }

   public void removeFace(Face3D faceToRemove)
   {
      if (faces.remove(faceToRemove))
         faceToRemove.clear();
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
      boolean result = false;
      for (int i = 0; i < faces.size(); i++)
      {
         result |= faces.get(i).containsNaN();
      }
      return result;
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
      else if (faces.size() == 1)
      {
         return faces.get(0);
      }

      unmarkAllFaces();
      Face3D currentBestFace = faces.get(0);
      Face3D faceUnderConsideration = currentBestFace;
      double minDistance = faceUnderConsideration.distance(point);
      faceUnderConsideration.mark();

      for (int i = 0; i < faces.size(); i++)
      {
         for (int j = 0; j < currentBestFace.getNumberOfEdges(); j++)
         {
            if (currentBestFace.getNeighbouringFace(j) != null && currentBestFace.getNeighbouringFace(j).isNotMarked())
            {
               double distance = currentBestFace.getNeighbouringFace(j).distance(point);
               if (distance < minDistance)
               {
                  minDistance = distance;
                  faceUnderConsideration = currentBestFace.getNeighbouringFace(j);
               }
               currentBestFace.getNeighbouringFace(j).mark();
            }
         }
         if (faceUnderConsideration == currentBestFace)
            break;
         else
            currentBestFace = faceUnderConsideration;
      }
      return currentBestFace;
   }

   private void updateCentroid()
   {
      updateVertices();
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

   public String toString()
   {
      return EuclidPolytopeIOTools.getConvexPolytope3DString(this);
   }
}
