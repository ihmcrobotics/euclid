package us.ihmc.euclid.shape.convexPolytope.tools;

import static us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools.next;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools.previous;

import java.util.*;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.EigenDecomposition;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.convexPolytope.*;
import us.ihmc.euclid.shape.convexPolytope.impl.AbstractFace3D;
import us.ihmc.euclid.shape.convexPolytope.impl.AbstractHalfEdge3D;
import us.ihmc.euclid.shape.convexPolytope.impl.AbstractVertex3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.*;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.*;

/**
 * This class a set of tools for building a {@link ConvexPolytope3D}.
 *
 * @author Sylvain Bertrand
 */
public class EuclidPolytopeConstructionTools
{
   /**
    * Default value for the construction epsilon of a {@link ConvexPolytope3D}.
    */
   public static final double DEFAULT_CONSTRUCTION_EPSILON = 1.0e-10;

   private EuclidPolytopeConstructionTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Computes the faces containing the given {@code vertex} as follows:
    * <ul>
    * <li>if the vertex is in the plane of a silhouette edge's face, the face is expanded to include
    * the new vertex;
    * <li>otherwise, a new face is created from the vertex and the silhouette edge.
    * </ul>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param vertex          faces are modified and/or created to include this vertex.
    * @param silhouetteEdges the contour visible from the vertex. Each edge is expected to be
    *                        associated with either a hidden face or an in-plane face.
    * @param inPlaneFaces    the list of faces for which the vertex is considered to lie in the face's
    *                        support plane. These faces are expanded to include the new vertex.
    * @param epsilon         tolerance used for testing edge-cases such as equivalent vertices, vertex
    *                        lying on a line, etc.
    * @return the list of new faces that were created in the the process.
    */
   public static <Vertex extends AbstractVertex3D<Vertex, Edge, Face>, Edge extends AbstractHalfEdge3D<Vertex, Edge, Face>, Face extends AbstractFace3D<Vertex, Edge, Face>> List<Face> computeVertexNeighborFaces(Face3DFactory<Face> faceFactory,
                                                                                                                                                                                                                   Vertex vertex,
                                                                                                                                                                                                                   List<Edge> silhouetteEdges,
                                                                                                                                                                                                                   Collection<Face> inPlaneFaces,
                                                                                                                                                                                                                   double epsilon)
   {
      if (EuclidPolytopeTools.distanceToClosestHalfEdge3D(vertex, silhouetteEdges) <= epsilon)
         return null;

      // Last filter before actually modifying the polytope
      if (!filterInPlaneFaces(vertex, silhouetteEdges, inPlaneFaces, epsilon))
         return null;

      Vector3D towardOutside = new Vector3D();

      for (Edge silhouetteEdge : silhouetteEdges)
      {
         Face face = silhouetteEdge.getFace();

         if (!inPlaneFaces.contains(face))
         { // A new face will be created.
            towardOutside.cross(face.getNormal(), silhouetteEdge.getDirection(false));

            /*
             * Testing following edge-case: The new vertex is in between the face plane and the silhouetteEdge.
             * In such context, this would result in a new face wrong ordering of the vertices which would be
             * corrected by flipping the face normal which then points to the inside of the polytope.
             */
            if (EuclidGeometryTools.isPoint3DAbovePlane3D(vertex, face.getCentroid(), face.getNormal()))
            {
               if (EuclidPolytopeTools.isPoint3DOnLeftSideOfLine3D(vertex, silhouetteEdge.getOrigin(), silhouetteEdge.getDestination(), towardOutside))
                  return null;
            }
            else
            {
               if (EuclidPolytopeTools.isPoint3DOnRightSideOfLine3D(vertex, silhouetteEdge.getOrigin(), silhouetteEdge.getDestination(), towardOutside))
                  return null;
            }
         }
      }

      for (Edge silhouetteEdge : silhouetteEdges)
      { // We first destroy the faces that are to be removed so they remove their edges from the vertices associated edges.
         destroyTwinFace(silhouetteEdge);
      }

      if (!inPlaneFaces.isEmpty())
      {
         // Making copies of these lists so we don't modify the arguments.
         silhouetteEdges = new ArrayList<>(silhouetteEdges);
         inPlaneFaces = new ArrayList<>(inPlaneFaces);

         for (int i = 0; i < silhouetteEdges.size() && !inPlaneFaces.isEmpty(); i++)
         {
            Edge silhouetteEdge = silhouetteEdges.get(i);
            Face faceToExtend = silhouetteEdge.getFace();

            if (!inPlaneFaces.remove(faceToExtend))
               continue;

            // The faceToExtend may hold onto more than 1 silhouette edges.
            silhouetteEdges.removeAll(faceToExtend.lineOfSight(vertex, epsilon));
            i--; // The list's subsequent elements are being shifted, so we need to rewind the current index.
            boolean wasModified = faceToExtend.addVertex(vertex);
            assert wasModified;
         }
      }

      List<Face> newFaces = new ArrayList<>();
      // Markers used to skip edges that were already processed.
      int forloopStart = 0;
      int forloopEnd = silhouetteEdges.size() - 1;

      if (!silhouetteEdges.isEmpty())
      {
         /*
          * We first need to handle the first silhouetteEdge in a special way. Simple case: the associated
          * face is to be extended => we do nothing here and start the for-loop. Non-simple case: a new face
          * is to be created with that silhouetteEdge, we then need to search backward and forward along the
          * silhouette and see if the new face can be extended to include any of them, if so we need to make
          * sure the processed edges are skipped in the for-loop.
          */
         Edge firstSilhouetteEdge = silhouetteEdges.get(0);

         Face newFace = newFace3DFromVertexAndTwinEdge(faceFactory, vertex, firstSilhouetteEdge, epsilon);
         forloopStart = 1;

         Edge nextSilhouetteEdge;
         Edge previousSilhouetteEdge = firstSilhouetteEdge;

         for (int i = 1; i < silhouetteEdges.size(); i++)
         { // Perform forward search along the silhouette
            nextSilhouetteEdge = silhouetteEdges.get(i);

            if (previousSilhouetteEdge.getDestination() != nextSilhouetteEdge.getOrigin())
            { // The silhouette is discontinuous, meaning some edges were processed in between, end of search.
               break;
            }

            if (!EuclidPolytopeTools.arePoint3DAndFace3DInPlane(nextSilhouetteEdge.getDestination(), newFace, epsilon))
            { // We can't extend the newFace any further.
               break;
            }

            if (!newFace.canObserverSeeEdge(nextSilhouetteEdge.getDestination(), previousSilhouetteEdge.getTwin().getPrevious()))
            { // Extending the face would result in an unexpected face setup where the newEdge below would not exist.
               break;
            }

            // The new face can be extended to include the next silhouetteEdge.
            if (!newFace.addVertex(nextSilhouetteEdge.getDestination(), vertex, true))
            { // Extending the face to include would result in removing the vertex we're currently trying to add.
               break;
            }

            Edge newEdge = nextSilhouetteEdge.getDestination().getEdgeTo(nextSilhouetteEdge.getOrigin());
            nextSilhouetteEdge.setTwin(newEdge);
            newEdge.setTwin(nextSilhouetteEdge);
            forloopStart = i + 1;
            previousSilhouetteEdge = nextSilhouetteEdge;
         }

         nextSilhouetteEdge = firstSilhouetteEdge;

         for (int i = silhouetteEdges.size() - 1; i >= forloopStart; i--)
         { // Perform backward search along the silhouette
            previousSilhouetteEdge = silhouetteEdges.get(i);

            if (previousSilhouetteEdge.getDestination() != nextSilhouetteEdge.getOrigin())
            { // The silhouette is discontinuous, meaning some edges were processed in between, end of search.
               break;
            }

            if (!EuclidPolytopeTools.arePoint3DAndFace3DInPlane(previousSilhouetteEdge.getOrigin(), newFace, epsilon))
            { // We can't extend the newFace any further.
               break;
            }

            if (!newFace.canObserverSeeEdge(previousSilhouetteEdge.getOrigin(), nextSilhouetteEdge.getTwin().getNext()))
            { // Extending the face would result in an unexpected face setup where the newEdge below would not exist.
               break;
            }

            // The new face can be extended to include the next silhouetteEdge.
            if (!newFace.addVertex(previousSilhouetteEdge.getOrigin(), vertex, true))
            { // Extending the face to include would result in removing the vertex we're currently trying to add.
               break;
            }

            Edge newEdge = previousSilhouetteEdge.getDestination().getEdgeTo(previousSilhouetteEdge.getOrigin());
            previousSilhouetteEdge.setTwin(newEdge);
            newEdge.setTwin(previousSilhouetteEdge);
            forloopEnd = i - 1;
            nextSilhouetteEdge = previousSilhouetteEdge;
         }

         newFaces.add(newFace);
      }

      for (int i = forloopStart; i <= forloopEnd; i++)
      {
         Edge silhouetteEdge = silhouetteEdges.get(i);

         // Creating a new face.
         Face newFace = newFace3DFromVertexAndTwinEdge(faceFactory, vertex, silhouetteEdge, epsilon);

         Edge nextSilhouetteEdge;
         Edge previousSilhouetteEdge = silhouetteEdge;

         for (int j = i + 1; j < forloopEnd; j++)
         {
            nextSilhouetteEdge = silhouetteEdges.get(j);

            if (previousSilhouetteEdge.getDestination() != nextSilhouetteEdge.getOrigin())
            { // The silhouette is discontinuous, meaning some edges were processed in between, end of search.
               break;
            }

            if (!EuclidPolytopeTools.arePoint3DAndFace3DInPlane(nextSilhouetteEdge.getDestination(), newFace, epsilon))
            { // We can't extend the newFace any further, resume the main loop while skipping processed edges.
               break;
            }

            if (!newFace.canObserverSeeEdge(nextSilhouetteEdge.getDestination(), previousSilhouetteEdge.getTwin().getPrevious()))
            { // Extending the face would result in an unexpected face setup where the newEdge below would not exist.
               break;
            }

            // The new face can be extended to include the next silhouetteEdge.
            if (!newFace.addVertex(nextSilhouetteEdge.getDestination(), vertex, true))
            { // Extending the face to include would result in removing the vertex we're currently trying to add.
               break;
            }
            Edge newEdge = nextSilhouetteEdge.getDestination().getEdgeTo(nextSilhouetteEdge.getOrigin());
            nextSilhouetteEdge.setTwin(newEdge);
            newEdge.setTwin(nextSilhouetteEdge);
            i = j;
            previousSilhouetteEdge = nextSilhouetteEdge;
         }
         newFaces.add(newFace);
      }

      for (Edge startingFrom : vertex.getAssociatedEdges())
      { // Going through the new edges and associating the twins.
         Edge endingTo = startingFrom.getDestination().getEdgeTo(vertex);

         startingFrom.setTwin(endingTo);
         endingTo.setTwin(startingFrom);
      }

      return newFaces;
   }

   private static boolean filterInPlaneFaces(Vertex3DReadOnly vertex, List<? extends HalfEdge3DReadOnly> silhouetteEdges,
                                             Collection<? extends Face3DReadOnly> inPlaneFaces, double epsilon)
   {
      if (inPlaneFaces.isEmpty())
         return true;

      boolean hasInPlaneFacesBeenModified = false;

      Set<HalfEdge3DReadOnly> edgesToSkip = new HashSet<>();

      // Last filter before actually modifying the polytope
      for (HalfEdge3DReadOnly silhouetteEdge : silhouetteEdges)
      { // Modify/Create the faces that are to contain the new vertex. The faces will take care of updating the edges.

         if (edgesToSkip.contains(silhouetteEdge))
            continue; // We already tested it.

         Face3DReadOnly face = silhouetteEdge.getFace();

         if (inPlaneFaces.contains(face))
         { // The face has to be extended to include the new vertex
            /*
             * The following check is redundant with the internal logic of Face3D.addVertex(...) and is also
             * probably expensive, but it has to be done before we actually start modifying the polytope.
             */
            List<? extends HalfEdge3DReadOnly> lineOfSight = face.lineOfSight(vertex);

            if (lineOfSight.isEmpty())
            {
               inPlaneFaces.remove(face);
               hasInPlaneFacesBeenModified = true;
               continue;
            }

            boolean faceRemovedFromInPlaneList = false;

            if (!lineOfSight.contains(silhouetteEdge))
            { // The silhouette is very likely corrupted (super-concave) and there is no easy way to handle it. Let's abort.
               return false;
            }

            for (HalfEdge3DReadOnly lineOfSightEdge : lineOfSight)
            {
               if (lineOfSightEdge == silhouetteEdge)
                  continue; // The single visible edge is the silhouetteEdge, this is a safe context.

               if (silhouetteEdges.contains(lineOfSightEdge))
               {
                  if (silhouetteEdge != lineOfSightEdge)
                     edgesToSkip.add(lineOfSightEdge);
                  continue; // This is the expected scenario.
               }

               /*
                * It would be fine if the new vertex would result in extending an edge of the face. However, if the
                * vertex is not an extrapolation of the line, we should not be able to see it.
                */
               if (!EuclidPolytopeTools.arePoint3DAndHalfEdge3DInLine(vertex, lineOfSightEdge, epsilon))
               {
                  faceRemovedFromInPlaneList = true;
                  inPlaneFaces.remove(face);
                  hasInPlaneFacesBeenModified = true;
                  break;
               }
               /*
                * The edge will be extended, let's make sure that the neighbor would do the same. Just need to make
                * sure it is part of the in-plane faces.
                */
               if (!inPlaneFaces.contains(lineOfSightEdge.getTwin().getFace()))
               {
                  faceRemovedFromInPlaneList = true;
                  inPlaneFaces.remove(face);
                  hasInPlaneFacesBeenModified = true;
                  break;
               }
            }

            if (!faceRemovedFromInPlaneList)
            {
               /*
                * Edge-case: the vertex on the support line of either the previous or next edge to the
                * line-of-sight. Only case where it is fine is when the edge should be extended. The edge is not
                * part of the silhouette. In such scenario, the face if the twin of the previous/next should also
                * be part of the inPlaneFaces, if not we remove the face from the list which will force the
                * creation of a new face.
                */
               HalfEdge3DReadOnly edgeBeforeLineOfSight = lineOfSight.get(0).getPrevious();
               HalfEdge3DReadOnly edgeAfterLineOfSight = lineOfSight.get(lineOfSight.size() - 1).getNext();

               if (!silhouetteEdges.contains(edgeBeforeLineOfSight) && (edgeBeforeLineOfSight.distanceFromSupportLine(vertex) < epsilon
                     || EuclidGeometryTools.distanceFromPoint3DToLine3D(edgeBeforeLineOfSight.getDestination(),
                                                                        vertex,
                                                                        edgeBeforeLineOfSight.getOrigin()) < epsilon))
               {
                  if (!inPlaneFaces.contains(edgeBeforeLineOfSight.getTwin().getFace()))
                  {
                     inPlaneFaces.remove(face);
                     hasInPlaneFacesBeenModified = true;
                  }
               }

               if (!silhouetteEdges.contains(edgeAfterLineOfSight) && (edgeAfterLineOfSight.distanceFromSupportLine(vertex) < epsilon
                     || EuclidGeometryTools.distanceFromPoint3DToLine3D(edgeAfterLineOfSight.getOrigin(),
                                                                        vertex,
                                                                        edgeAfterLineOfSight.getDestination()) < epsilon))
               {
                  if (!inPlaneFaces.contains(edgeAfterLineOfSight.getTwin().getFace()))
                  {
                     inPlaneFaces.remove(face);
                     hasInPlaneFacesBeenModified = true;
                  }
               }
            }
         }
      }

      if (hasInPlaneFacesBeenModified)
         return filterInPlaneFaces(vertex, silhouetteEdges, inPlaneFaces, epsilon);
      else
         return true;
   }

   private static void destroyTwinFace(AbstractHalfEdge3D<?, ?, ?> edge)
   {
      AbstractHalfEdge3D<?, ?, ?> twin = edge.getTwin();
      if (twin == null)
         return;
      AbstractFace3D<?, ?, ?> face = twin.getFace();
      if (face == null)
         return;
      face.destroy();
   }

   /**
    * Creates a new face such that:
    * <ul>
    * <li>the given {@code vertex} belongs to the new face;
    * <li>the new face has an edge that is the twin of the given {@code twinEdge};
    * <li>the new face's normal can be computed using the direction of the given twin-edge.
    * </ul>
    *
    * @param vertex   a vertex of the new face.
    * @param twinEdge the edge which twin's associated face is the new face.
    * @param epsilon  tolerance used when testing if a vertex should be added or not.
    * @return the new face.
    */
   public static <Vertex extends AbstractVertex3D<Vertex, Edge, Face>, Edge extends AbstractHalfEdge3D<Vertex, Edge, Face>, Face extends AbstractFace3D<Vertex, Edge, Face>> Face newFace3DFromVertexAndTwinEdge(Face3DFactory<Face> faceFactory,
                                                                                                                                                                                                                 Vertex vertex,
                                                                                                                                                                                                                 Edge twinEdge,
                                                                                                                                                                                                                 double epsilon)
   {
      Vertex v1 = twinEdge.getDestination();
      Vertex v2 = twinEdge.getOrigin();
      Vertex v3 = vertex;

      // Estimate the face's normal based on its vertices and knowing the expecting ordering based on the twin-edge: v1, v2, then v3.
      Vector3D initialNormal = EuclidPolytopeTools.crossProductOfLineSegment3Ds(v1, v2, v2, v3);
      // As the vertices are clock-wise ordered the cross-product of 2 successive edges should be negated to obtain the face's normal.
      initialNormal.negate();

      Face face = faceFactory.newInstance(initialNormal, epsilon);

      boolean wasModified = face.addVertex(v1);
      assert wasModified;
      wasModified = face.addVertex(v2);
      assert wasModified;

      // Associate the first edge so the next operation on the face won't modify it.
      Edge faceFirstEdge = face.getEdge(0);
      twinEdge.setTwin(faceFirstEdge);
      faceFirstEdge.setTwin(twinEdge);

      wasModified = face.addVertex(v3, null, true);
      assert wasModified;

      Edge vertexToOrigin = vertex.getEdgeTo(twinEdge.getOrigin());
      Edge originToVertex = twinEdge.getOrigin().getEdgeTo(vertex);

      if (vertexToOrigin != null)
         vertexToOrigin.setTwin(originToVertex);
      if (originToVertex != null)
         originToVertex.setTwin(vertexToOrigin);

      Edge vertexToDestination = vertex.getEdgeTo(twinEdge.getDestination());
      Edge destinationToVertex = twinEdge.getDestination().getEdgeTo(vertex);

      if (vertexToDestination != null)
         vertexToDestination.setTwin(destinationToVertex);
      if (destinationToVertex != null)
         destinationToVertex.setTwin(vertexToDestination);

      return face;
   }

   /**
    * Computes the average and normal from the given {@code vertices} assuming they lying on the same
    * plane.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param vertices       the face vertices. Not modified.
    * @param averageToPack  point in which the average from the vertices is stored. Modified. Can be
    *                       {@code null}.
    * @param normalToUpdate the vector used to store the normal. The normal is updated such that
    *                       {@code oldNormal.dot(newNormal) > 0.0}. Modified.
    * @return whether the method succeeded or not.
    */
   public static boolean updateFace3DNormal(List<? extends Point3DReadOnly> vertices, Point3DBasics averageToPack, Vector3DBasics normalToUpdate)
   {
      DenseMatrix64F covarianceMatrix = new DenseMatrix64F(3, 3);
      computeCovariance3D(vertices, averageToPack, covarianceMatrix);
      return updateFace3DNormal(covarianceMatrix, normalToUpdate);
   }

   /**
    * Computes the face normal from its covariance matrix.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param covarianceMatrix the covariance matrix computed from the face vertices. Not modified.
    * @param normalToUpdate   the vector used to store the normal. The normal is updated such that
    *                         {@code oldNormal.dot(newNormal) > 0.0}. Modified.
    * @return whether the method succeeded or not.
    */
   public static boolean updateFace3DNormal(DenseMatrix64F covarianceMatrix, Vector3DBasics normalToUpdate)
   {
      return updateFace3DNormal(DecompositionFactory.eig(3, true, true), covarianceMatrix, normalToUpdate);
   }

   /**
    * Computes the face normal from its covariance matrix.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param eigenDecomposition the solver to use to computing the eigen vectors.
    * @param covarianceMatrix   the covariance matrix computed from the face vertices. Not modified.
    * @param normalToUpdate     the vector used to store the normal. The normal is updated such that
    *                           {@code oldNormal.dot(newNormal) > 0.0}. Modified.
    * @return whether the method succeeded or not.
    */
   public static boolean updateFace3DNormal(EigenDecomposition<DenseMatrix64F> eigenDecomposition, DenseMatrix64F covarianceMatrix,
                                            Vector3DBasics normalToUpdate)
   {
      if (!eigenDecomposition.decompose(covarianceMatrix))
         return false;

      double eigenValue0 = eigenDecomposition.getEigenvalue(0).getReal();
      double eigenValue1 = eigenDecomposition.getEigenvalue(1).getReal();
      double eigenValue2 = eigenDecomposition.getEigenvalue(2).getReal();

      int smallEigenValueIndex;

      if (eigenValue0 > eigenValue1)
      {
         if (eigenValue1 > eigenValue2)
         { // (eigenValue0 > eigenValue1 > eigenValue2)
            smallEigenValueIndex = 2;
         }
         else
         { // (// eigenValue0 > eigenValue2 > eigenValue1) or (eigenValue2 > eigenValue0 > eigenValue1)
            smallEigenValueIndex = 1;
         }
      }
      else
      {
         if (eigenValue0 > eigenValue2)
         { // (eigenValue1 > eigenValue0 > eigenValue2)
            smallEigenValueIndex = 2;
         }
         else
         { // (eigenValue1 > eigenValue2 > eigenValue0) or (eigenValue2 > eigenValue1 > eigenValue0)
            smallEigenValueIndex = 0;
         }
      }

      DenseMatrix64F eigenVector = eigenDecomposition.getEigenVector(smallEigenValueIndex);
      double newX = eigenVector.get(0, 0);
      double newY = eigenVector.get(1, 0);
      double newZ = eigenVector.get(2, 0);

      boolean negate = TupleTools.dot(newX, newY, newZ, normalToUpdate) < 0.0;

      normalToUpdate.set(eigenVector);
      if (negate)
         normalToUpdate.negate();

      return true;
   }

   /**
    * Computes the covariance matrix from a list of 3D tuples.
    *
    * @param input            the list of tuples to use for computing the covariance matrix. Not
    *                         modified.
    * @param covarianceToPack the matrix in which the 3-by-3 covariance matrix is stored. Modified.
    */
   public static void computeCovariance3D(List<? extends Tuple3DReadOnly> input, DenseMatrix64F covarianceToPack)
   {
      computeCovariance3D(input, null, covarianceToPack);
   }

   /**
    * Computes the covariance matrix from a list of 3D tuples.
    *
    * @param input            the list of tuples to use for computing the covariance matrix. Not
    *                         modified.
    * @param averageToPack    tuple in which the average from the input is stored. Modified. Can be
    *                         {@code null}.
    * @param covarianceToPack the matrix in which the 3-by-3 covariance matrix is stored. Modified.
    */
   public static void computeCovariance3D(List<? extends Tuple3DReadOnly> input, Tuple3DBasics averageToPack, DenseMatrix64F covarianceToPack)
   {
      double meanX = 0.0;
      double meanY = 0.0;
      double meanZ = 0.0;

      for (int i = 0; i < input.size(); i++)
      {
         Tuple3DReadOnly element = input.get(i);
         meanX += element.getX();
         meanY += element.getY();
         meanZ += element.getZ();
      }

      double inverseOfInputSize = 1.0 / input.size();

      meanX *= inverseOfInputSize;
      meanY *= inverseOfInputSize;
      meanZ *= inverseOfInputSize;

      if (averageToPack != null)
      {
         averageToPack.set(meanX, meanY, meanZ);
      }

      double covXX = 0.0, covYY = 0.0, covZZ = 0.0;
      double covXY = 0.0, covXZ = 0.0, covYZ = 0.0;

      for (int i = 0; i < input.size(); i++)
      {
         Tuple3DReadOnly element = input.get(i);
         double devX = element.getX() - meanX;
         double devY = element.getY() - meanY;
         double devZ = element.getZ() - meanZ;

         covXX += devX * devX * inverseOfInputSize;
         covYY += devY * devY * inverseOfInputSize;
         covZZ += devZ * devZ * inverseOfInputSize;
         covXY += devX * devY * inverseOfInputSize;
         covXZ += devX * devZ * inverseOfInputSize;
         covYZ += devY * devZ * inverseOfInputSize;

      }

      covarianceToPack.reshape(3, 3);
      covarianceToPack.set(0, covXX);
      covarianceToPack.set(1, covXY);
      covarianceToPack.set(2, covXZ);
      covarianceToPack.set(3, covXY);
      covarianceToPack.set(4, covYY);
      covarianceToPack.set(5, covYZ);
      covarianceToPack.set(6, covXZ);
      covarianceToPack.set(7, covYZ);
      covarianceToPack.set(8, covZZ);
   }

   /**
    * Computes the area and centroid of a 3D convex polygon defined by its vertices and normal.
    *
    * @param convexPolygon3D  the polygon vertices. Not modified.
    * @param normal           the normal of the plane on which the vertices are lying. Not modified.
    * @param numberOfVertices the number of vertices that belong to the convex polygon.
    * @param clockwiseOrdered whether the vertices are clockwise ordered.
    * @param centroidToPack   point used to store the coordinates of the polygon centroid. Modified.
    *                         Can be {@code null}.
    * @return the area of the polygon.
    */
   public static double computeConvexPolygon3DArea(List<? extends Point3DReadOnly> convexPolygon3D, Vector3DReadOnly normal, int numberOfVertices,
                                                   boolean clockwiseOrdered, Point3DBasics centroidToPack)
   {
      checkNumberOfVertices(convexPolygon3D, numberOfVertices);

      if (numberOfVertices == 0)
      {
         if (centroidToPack != null)
            centroidToPack.setToNaN();
         return Double.NaN;
      }
      else if (numberOfVertices < 3)
      {
         if (centroidToPack != null)
         {
            centroidToPack.setToZero();
            for (int i = 0; i < numberOfVertices; i++)
               centroidToPack.add(convexPolygon3D.get(i));
            centroidToPack.scale(1.0 / numberOfVertices);
         }
         return 0.0;
      }
      else
      {
         double area = 0.0;
         double Cx = 0.0;
         double Cy = 0.0;
         double Cz = 0.0;

         if (clockwiseOrdered)
         {
            for (int i = 0; i < numberOfVertices; i++)
            {
               Point3DReadOnly ci = convexPolygon3D.get(i);
               Point3DReadOnly ciMinus1 = convexPolygon3D.get(previous(i, numberOfVertices));

               double wx = ci.getY() * ciMinus1.getZ() - ci.getZ() * ciMinus1.getY();
               double wy = ci.getZ() * ciMinus1.getX() - ci.getX() * ciMinus1.getZ();
               double wz = ci.getX() * ciMinus1.getY() - ci.getY() * ciMinus1.getX();
               double weight = TupleTools.dot(wx, wy, wz, normal);

               Cx += (ci.getX() + ciMinus1.getX()) * weight;
               Cy += (ci.getY() + ciMinus1.getY()) * weight;
               Cz += (ci.getZ() + ciMinus1.getZ()) * weight;

               area += weight;
            }
         }
         else
         {
            for (int i = 0; i < numberOfVertices; i++)
            {
               Point3DReadOnly ci = convexPolygon3D.get(i);
               Point3DReadOnly ciPlus1 = convexPolygon3D.get(next(i, numberOfVertices));

               double wx = ci.getY() * ciPlus1.getZ() - ci.getZ() * ciPlus1.getY();
               double wy = ci.getZ() * ciPlus1.getX() - ci.getX() * ciPlus1.getZ();
               double wz = ci.getX() * ciPlus1.getY() - ci.getY() * ciPlus1.getX();
               double weight = TupleTools.dot(wx, wy, wz, normal);

               Cx += (ci.getX() + ciPlus1.getX()) * weight;
               Cy += (ci.getY() + ciPlus1.getY()) * weight;
               Cz += (ci.getZ() + ciPlus1.getZ()) * weight;

               area += weight;
            }
         }

         area *= 0.5;

         if (centroidToPack != null)
         {
            if (area < 1.0e-12)
            {
               centroidToPack.setToZero();
               for (int i = 0; i < numberOfVertices; i++)
                  centroidToPack.add(convexPolygon3D.get(i));
               centroidToPack.scale(1.0 / numberOfVertices);
            }
            else
            {
               double scale = 1.0 / (6.0 * area);
               Cx *= scale;
               Cy *= scale;
               Cz *= scale;

               centroidToPack.set(Cx, Cy, Cz);

               double dot = TupleTools.dot(Cx, Cy, Cz, normal);
               centroidToPack.scaleAdd(-dot, normal, centroidToPack);

               double average = 0.0;

               for (int i = 0; i < numberOfVertices; i++)
               {
                  Point3DReadOnly vertex = convexPolygon3D.get(i);
                  average += TupleTools.dot(vertex, normal) / numberOfVertices;
               }

               centroidToPack.scaleAdd(average, normal, centroidToPack);
            }
         }

         return area;
      }
   }

   /**
    * Computes the volume and centroid of a convex polytope.
    *
    * @param convexPolytope3D the convex polytope to evaluate. Not modified.
    * @param centroidToPack   the point used to store the coordinates of the centroid. Modified.
    * @return the volume of the convex polytope.
    */
   public static double computeConvexPolytope3DVolume(ConvexPolytope3DReadOnly convexPolytope3D, Point3DBasics centroidToPack)
   {
      centroidToPack.setToZero();
      double volume = 0.0;

      if (convexPolytope3D.getNumberOfVertices() <= 4)
      {
         for (int vertexIndex = 0; vertexIndex < convexPolytope3D.getNumberOfVertices(); vertexIndex++)
            centroidToPack.add(convexPolytope3D.getVertex(vertexIndex));
         centroidToPack.scale(1.0 / convexPolytope3D.getNumberOfVertices());

         if (convexPolytope3D.getNumberOfVertices() == 4)
            volume = EuclidShapeTools.tetrahedronVolume(convexPolytope3D.getVertex(0),
                                                        convexPolytope3D.getVertex(1),
                                                        convexPolytope3D.getVertex(2),
                                                        convexPolytope3D.getVertex(3));
         else
            volume = 0.0;
         return volume;
      }

      Point3DReadOnly a = convexPolytope3D.getFace(0).getVertex(0);

      // We can skip the first face as the vertex 'a' comes from it, so it does not participate in the centroid/volume calculation.
      for (int faceIndex = 1; faceIndex < convexPolytope3D.getNumberOfFaces(); faceIndex++)
      {
         /*
          * Each face is decomposed into triangles from which form tetrahedrons (a, b, c, d) using each
          * triangle vertices plus 'a'. We compute the centroid of these tetrahedrons which are then weighted
          * with their volume.
          */
         Face3DReadOnly face = convexPolytope3D.getFace(faceIndex);
         int numberOfTriangles = face.getNumberOfEdges() - 2;
         Vertex3DReadOnly b = face.getVertex(0);

         for (int triangleIndex = 0; triangleIndex < numberOfTriangles; triangleIndex++)
         {
            Vertex3DReadOnly c = face.getVertex(triangleIndex + 1);
            Vertex3DReadOnly d = face.getVertex(triangleIndex + 2);
            double tetrahedronVolume = EuclidShapeTools.tetrahedronVolume(a, b, c, d);
            // The centroid of the tetrahedron is: tetrahedronCentroid = (a + b + c + d) / 4.0
            // The centroid of the polytope is updated as: centroid += tetrahedronVolume * tetrahedronCentroid
            // which is equivalent to the following:
            double scale = 0.25 * tetrahedronVolume;
            centroidToPack.scaleAdd(scale, a, centroidToPack);
            centroidToPack.scaleAdd(scale, b, centroidToPack);
            centroidToPack.scaleAdd(scale, c, centroidToPack);
            centroidToPack.scaleAdd(scale, d, centroidToPack);
            volume += tetrahedronVolume;
         }
      }

      if (volume <= 1.0e-12)
      {
         for (Point3DReadOnly vertex : convexPolytope3D.getVertices())
         {
            centroidToPack.add(vertex);
         }
         centroidToPack.scale(1.0 / convexPolytope3D.getNumberOfVertices());
      }
      else
      {
         centroidToPack.scale(1.0 / volume);
      }

      return volume;
   }

   static void checkNumberOfVertices(List<? extends Point3DReadOnly> convexPolygon3D, int numberOfVertices)
   {
      if (numberOfVertices < 0 || numberOfVertices > convexPolygon3D.size())
         throw new IllegalArgumentException("Illegal numberOfVertices: " + numberOfVertices + ", expected a value in ] 0, " + convexPolygon3D.size() + "].");
   }

}
