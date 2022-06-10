package us.ihmc.euclid.shape.tools;

import static us.ihmc.euclid.shape.tools.EuclidShapeIOTools.getEuclidShape3DCollisionResultString;

import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.shape.collision.interfaces.EuclidShape3DCollisionResultReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * This class provides the tools to perform a variety of assertions on shape types.
 *
 * @author Sylvain Bertrand
 */
public class EuclidShapeTestTools
{
   private static final String DEFAULT_FORMAT = EuclidCoreTestTools.DEFAULT_FORMAT;

   private EuclidShapeTestTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Asserts on a per component basis that the two collision results are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected collision result. Not modified.
    * @param actual   the actual collision result. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two collision results are not equal. If only one of the arguments
    *                        is equal to {@code null}.
    */
   public static void assertEuclidShape3DCollisionResultEquals(EuclidShape3DCollisionResultReadOnly expected,
                                                               EuclidShape3DCollisionResultReadOnly actual,
                                                               double epsilon)
   {
      assertEuclidShape3DCollisionResultEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two collision results are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected collision result. Not modified.
    * @param actual        the actual collision result. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two collision results are not equal. If only one of the arguments
    *                        is equal to {@code null}.
    */
   public static void assertEuclidShape3DCollisionResultEquals(String messagePrefix,
                                                               EuclidShape3DCollisionResultReadOnly expected,
                                                               EuclidShape3DCollisionResultReadOnly actual,
                                                               double epsilon)
   {
      assertEuclidShape3DCollisionResultEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two collision results are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected collision result. Not modified.
    * @param actual        the actual collision result. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two collision results are not equal. If only one of the arguments
    *                        is equal to {@code null}.
    */
   public static void assertEuclidShape3DCollisionResultEquals(String messagePrefix,
                                                               EuclidShape3DCollisionResultReadOnly expected,
                                                               EuclidShape3DCollisionResultReadOnly actual,
                                                               double epsilon,
                                                               String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
      {
         if (expected.areShapesColliding() != actual.areShapesColliding())
         {
            throwNotEqualAssertionError(messagePrefix, expected, actual, format);
         }
         else
         {
            Vector3D differenceNormalOnA = new Vector3D();
            differenceNormalOnA.sub(expected.getNormalOnA(), actual.getNormalOnA());
            Vector3D differenceNormalOnB = new Vector3D();
            differenceNormalOnB.sub(expected.getNormalOnB(), actual.getNormalOnB());

            String difference = "[";
            difference += "distance: " + Math.abs(expected.getSignedDistance() - actual.getSignedDistance());
            difference += ", pointOnA: " + expected.getPointOnA().distance(actual.getPointOnA()) + ", normalOnA: " + differenceNormalOnA.length();
            difference += ", pointOnB: " + expected.getPointOnB().distance(actual.getPointOnB()) + ", normalOnB: " + differenceNormalOnB.length();
            difference += "]";
            throwNotEqualAssertionError(messagePrefix, expected, actual, format, difference);
         }
      }
   }

   /**
    * Asserts on a per component basis that the two collision results represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected collision result. Not modified.
    * @param actual   the actual collision result. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two collision results do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertEuclidShape3DCollisionResultGeometricallyEquals(EuclidShape3DCollisionResultReadOnly expected,
                                                                            EuclidShape3DCollisionResultReadOnly actual,
                                                                            double epsilon)
   {
      assertEuclidShape3DCollisionResultGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two collision results represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected               the expected collision result. Not modified.
    * @param actual                 the actual collision result. Not modified.
    * @param distanceEpsilon        the tolerance to use when comparing distance.
    * @param pointTangentialEpsilon tolerance to use when comparing {@code pointOnA} and
    *                               {@code pointOnB} in the plane perpendicular to the collision
    *                               vector, i.e. {@code collisionVector = pointOnA - pointOnB}. The
    *                               {@code distanceEpsilon} is used for comparing the points along the
    *                               collision vector.
    * @param normalEpsilon          the tolerance to use when comparing normals on shapes.
    * @throws AssertionError if the two collision results do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertEuclidShape3DCollisionResultGeometricallyEquals(EuclidShape3DCollisionResultReadOnly expected,
                                                                            EuclidShape3DCollisionResultReadOnly actual,
                                                                            double distanceEpsilon,
                                                                            double pointTangentialEpsilon,
                                                                            double normalEpsilon)
   {
      assertEuclidShape3DCollisionResultGeometricallyEquals(null, expected, actual, distanceEpsilon, pointTangentialEpsilon, normalEpsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two collision results represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected collision result. Not modified.
    * @param actual        the actual collision result. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two collision results do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertEuclidShape3DCollisionResultGeometricallyEquals(String messagePrefix,
                                                                            EuclidShape3DCollisionResultReadOnly expected,
                                                                            EuclidShape3DCollisionResultReadOnly actual,
                                                                            double epsilon)
   {
      assertEuclidShape3DCollisionResultGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two collision results represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix          prefix to add to the automated message.
    * @param expected               the expected collision result. Not modified.
    * @param actual                 the actual collision result. Not modified.
    * @param distanceEpsilon        the tolerance to use when comparing distance.
    * @param pointTangentialEpsilon tolerance to use when comparing {@code pointOnA} and
    *                               {@code pointOnB} in the plane perpendicular to the collision
    *                               vector, i.e. {@code collisionVector = pointOnA - pointOnB}. The
    *                               {@code distanceEpsilon} is used for comparing the points along the
    *                               collision vector.
    * @param normalEpsilon          the tolerance to use when comparing normals on shapes.
    * @throws AssertionError if the two collision results do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertEuclidShape3DCollisionResultGeometricallyEquals(String messagePrefix,
                                                                            EuclidShape3DCollisionResultReadOnly expected,
                                                                            EuclidShape3DCollisionResultReadOnly actual,
                                                                            double distanceEpsilon,
                                                                            double pointTangentialEpsilon,
                                                                            double normalEpsilon)
   {
      assertEuclidShape3DCollisionResultGeometricallyEquals(messagePrefix,
                                                            expected,
                                                            actual,
                                                            distanceEpsilon,
                                                            pointTangentialEpsilon,
                                                            normalEpsilon,
                                                            DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two collision results represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected collision result. Not modified.
    * @param actual        the actual collision result. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two collision results do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertEuclidShape3DCollisionResultGeometricallyEquals(String messagePrefix,
                                                                            EuclidShape3DCollisionResultReadOnly expected,
                                                                            EuclidShape3DCollisionResultReadOnly actual,
                                                                            double epsilon,
                                                                            String format)
   {
      assertEuclidShape3DCollisionResultGeometricallyEquals(messagePrefix, expected, actual, epsilon, epsilon, epsilon, format);
   }

   /**
    * Asserts on a per component basis that the two collision results represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix          prefix to add to the automated message.
    * @param expected               the expected collision result. Not modified.
    * @param actual                 the actual collision result. Not modified.
    * @param distanceEpsilon        the tolerance to use when comparing distance.
    * @param pointTangentialEpsilon tolerance to use when comparing {@code pointOnA} and
    *                               {@code pointOnB} in the plane perpendicular to the collision
    *                               vector, i.e. {@code collisionVector = pointOnA - pointOnB}. The
    *                               {@code distanceEpsilon} is used for comparing the points along the
    *                               collision vector.
    * @param normalEpsilon          the tolerance to use when comparing normals on shapes.
    * @param format                 the format to use for printing each component when an
    *                               {@code AssertionError} is thrown.
    * @throws AssertionError if the two collision results do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertEuclidShape3DCollisionResultGeometricallyEquals(String messagePrefix,
                                                                            EuclidShape3DCollisionResultReadOnly expected,
                                                                            EuclidShape3DCollisionResultReadOnly actual,
                                                                            double distanceEpsilon,
                                                                            double pointTangentialEpsilon,
                                                                            double normalEpsilon,
                                                                            String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, distanceEpsilon, pointTangentialEpsilon, normalEpsilon))
      {
         if (expected.areShapesColliding() != actual.areShapesColliding())
         {
            throwNotEqualAssertionError(messagePrefix, expected, actual, format);
         }
         else
         {
            Vector3D differenceNormalOnA = new Vector3D();
            differenceNormalOnA.sub(expected.getNormalOnA(), actual.getNormalOnA());
            Vector3D differenceNormalOnB = new Vector3D();
            differenceNormalOnB.sub(expected.getNormalOnB(), actual.getNormalOnB());

            String difference = "[";
            difference += "distance: " + Math.abs(expected.getSignedDistance() - actual.getSignedDistance());
            difference += ", pointOnA: " + expected.getPointOnA().distance(actual.getPointOnA()) + ", normalOnA: " + differenceNormalOnA.length();
            difference += ", pointOnB: " + expected.getPointOnB().distance(actual.getPointOnB()) + ", normalOnB: " + differenceNormalOnB.length();
            difference += "]";
            throwNotEqualAssertionError(messagePrefix, expected, actual, format, difference);
         }
      }
   }

   /**
    * Asserts the integrity of a convex polytope.
    * <p>
    * Properties asserted include notably:
    * <ul>
    * <li>vertices, edges, and faces are properly linked to each other.
    * <li>convexity between neighbor faces.
    * <li>convexity and clockwise winding for each face.
    * <li>each face's normal points outside.
    * </ul>
    * </p>
    *
    * @param convexPolytope3D the convex polytope to run assertions on. Not modified.
    * @throws AssertionError if the convex polytope is corrupted.
    */
   public static void assertConvexPolytope3DGeneralIntegrity(ConvexPolytope3DReadOnly convexPolytope3D)
   {
      assertConvexPolytope3DGeneralIntegrity(null, convexPolytope3D);
   }

   /**
    * Asserts the integrity of a convex polytope.
    * <p>
    * Properties asserted include notably:
    * <ul>
    * <li>vertices, edges, and faces are properly linked to each other.
    * <li>convexity between neighbor faces.
    * <li>convexity and clockwise winding for each face.
    * <li>each face's normal points outside.
    * </ul>
    * </p>
    *
    * @param messagePrefix    prefix to add to the automated message.
    * @param convexPolytope3D the convex polytope to run assertions on. Not modified.
    * @throws AssertionError if the convex polytope is corrupted.
    */
   public static void assertConvexPolytope3DGeneralIntegrity(String messagePrefix, ConvexPolytope3DReadOnly convexPolytope3D)
   {
      if (convexPolytope3D.getCentroid().containsNaN())
         EuclidCoreTestTools.throwAssertionError(messagePrefix, "The polytope's centroid contains NaN.");

      if (convexPolytope3D.getNumberOfFaces() > 1)
      {
         int verticesSize = convexPolytope3D.getVertices().size();
         int halfEdgesSize = convexPolytope3D.getHalfEdges().size();
         int facesSize = convexPolytope3D.getFaces().size();

         int expectedNumberOfVertices = EuclidPolytopeTools.computeConvexPolytopeNumberOfVertices(facesSize, halfEdgesSize / 2);
         if (verticesSize != expectedNumberOfVertices)
            EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                    "Inconsistent data size, expected " + expectedNumberOfVertices + " vertices but was " + verticesSize);
      }

      assertConvexPolytope3DFacesIntegrity(messagePrefix, convexPolytope3D);
      assertConvexPolytope3DHalfEdgesIntegrity(messagePrefix, convexPolytope3D);
      assertConvexPolytope3DVerticesIntegrity(messagePrefix, convexPolytope3D);
   }

   private static void assertConvexPolytope3DFacesIntegrity(String messagePrefix, ConvexPolytope3DReadOnly convexPolytope3D)
   {
      Set<Face3DReadOnly> faceSet = new HashSet<>();

      for (int faceIndex = 0; faceIndex < convexPolytope3D.getNumberOfFaces(); faceIndex++)
      { // We assert the uniqueness of the faces.
         Face3DReadOnly face = convexPolytope3D.getFace(faceIndex);
         if (!faceSet.add(face))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, faceIndex + "th face is a duplicate.");
      }

      Vector3D toOrigin = new Vector3D();
      Vector3D toDestination = new Vector3D();
      Vector3D toCentroid = new Vector3D();
      Vector3D cross = new Vector3D();

      if (convexPolytope3D.getNumberOfFaces() == 1)
      {
         for (int faceIndex = 0; faceIndex < convexPolytope3D.getNumberOfFaces(); faceIndex++)
         {
            Face3DReadOnly face = convexPolytope3D.getFace(faceIndex);
            Point3DReadOnly centroid = face.getCentroid();
            Vector3DReadOnly normal = face.getNormal();
            List<? extends HalfEdge3DReadOnly> edges = face.getEdges();

            for (int edgeIndex = 0; edgeIndex < edges.size(); edgeIndex++)
            {
               HalfEdge3DReadOnly edge = edges.get(edgeIndex);

               if (edge.getFace() != face)
                  EuclidCoreTestTools.throwAssertionError(messagePrefix, faceIndex + "th face: the " + edgeIndex + "th edge does not this face as its face.");
               if (!edges.contains(edge.getNext()))
                  EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                          faceIndex + "th face: the " + edgeIndex + "th edge's next does not belong to this face.");
               if (!edges.contains(edge.getPrevious()))
                  EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                          faceIndex + "th face: the " + edgeIndex + "th edge's previous does not belong to this face.");
               if (edges.indexOf(edge.getNext()) != (edgeIndex + 1) % edges.size())
                  EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                          faceIndex + "th face: the " + edgeIndex + "th edge's next is not at the next index in the list.");
            }

            if (edges.size() >= 3)
            {
               for (int edgeIndex = 0; edgeIndex < edges.size(); edgeIndex++)
               {
                  HalfEdge3DReadOnly edge = edges.get(edgeIndex);

                  if (edge.getFace() != face)
                     EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                             faceIndex + "th face: the " + edgeIndex + "th edge does not this face as its face.");
                  if (!edges.contains(edge.getNext()))
                     EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                             faceIndex + "th face: the " + edgeIndex + "th edge's next does not belong to this face.");
                  if (!edges.contains(edge.getPrevious()))
                     EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                             faceIndex + "th face: the " + edgeIndex + "th edge's previous does not belong to this face.");
                  if (edges.indexOf(edge.getNext()) != (edgeIndex + 1) % edges.size())
                     EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                             faceIndex + "th face: the " + edgeIndex + "th edge's next is not at the next index in the list.");

                  // Verifying the edges are all clockwise ordered.
                  toOrigin.sub(edge.getOrigin(), centroid);
                  toDestination.sub(edge.getDestination(), centroid);
                  cross.cross(toDestination, toOrigin);

                  if (cross.dot(normal) < 0.0)
                     EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                             faceIndex + "th face: the " + edgeIndex + "th edge is orientated counter-clockwise.");
               }
            }
         }
      }
      else
      {
         for (int faceIndex = 0; faceIndex < convexPolytope3D.getNumberOfFaces(); faceIndex++)
         {
            Face3DReadOnly face = convexPolytope3D.getFace(faceIndex);
            Point3DReadOnly centroid = face.getCentroid();
            Vector3DReadOnly normal = face.getNormal();
            List<? extends HalfEdge3DReadOnly> edges = face.getEdges();

            // Verifying the normal is pointing towards the outside of the polytope.
            toCentroid.sub(centroid, convexPolytope3D.getCentroid());

            if (toCentroid.dot(normal) < 0.0)
            {
               if (convexPolytope3D.getVolume() < convexPolytope3D.getConstructionEpsilon())
                  System.out.println("WARNING: "
                        + EuclidCoreTestTools.addPrefixToMessage(messagePrefix,
                                                                 faceIndex + "th face's normal might be pointing towards the inside of the polytope."));
               else
                  EuclidCoreTestTools.throwAssertionError(messagePrefix, faceIndex + "th face's normal is pointing towards the inside of the polytope.");
            }

            for (int edgeIndex = 0; edgeIndex < edges.size(); edgeIndex++)
            {
               HalfEdge3DReadOnly edge = edges.get(edgeIndex);
               Face3DReadOnly neighbor = edge.getTwin().getFace();

               if (edge.getFace() != face)
                  EuclidCoreTestTools.throwAssertionError(messagePrefix, faceIndex + "th face: the " + edgeIndex + "th edge does not this face as its face.");
               if (!edges.contains(edge.getNext()))
                  EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                          faceIndex + "th face: the " + edgeIndex + "th edge's next does not belong to this face.");
               if (!edges.contains(edge.getPrevious()))
                  EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                          faceIndex + "th face: the " + edgeIndex + "th edge's previous does not belong to this face.");
               if (edges.indexOf(edge.getNext()) != (edgeIndex + 1) % edges.size())
                  EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                          faceIndex + "th face: the " + edgeIndex + "th edge's next is not at the next index in the list.");

               // Verifying the edges are all clockwise ordered.
               toOrigin.sub(edge.getOrigin(), centroid);
               toDestination.sub(edge.getDestination(), centroid);
               cross.cross(toDestination, toOrigin);

               if (cross.dot(normal) < 0.0)
                  EuclidCoreTestTools.throwAssertionError(messagePrefix, faceIndex + "th face: the " + edgeIndex + "th edge is orientated counter-clockwise.");

               if (EuclidGeometryTools.isPoint3DAbovePlane3D(neighbor.getCentroid(), face.getCentroid(), face.getNormal()))
               {
                  if (face.signedDistanceFromSupportPlane(neighbor.getCentroid()) < convexPolytope3D.getConstructionEpsilon())
                     System.out.println("WARNING: "
                           + EuclidCoreTestTools.addPrefixToMessage(messagePrefix,
                                                                    faceIndex + "th face might be concave with respect to a neighor, the "
                                                                          + convexPolytope3D.getFaces().indexOf(neighbor) + "th face."));
                  else
                     EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                             faceIndex + "th face is concave with respect to a neighor, the "
                                                                   + convexPolytope3D.getFaces().indexOf(neighbor) + "th face.");
               }
            }
         }
      }
   }

   private static void assertConvexPolytope3DHalfEdgesIntegrity(String messagePrefix, ConvexPolytope3DReadOnly convexPolytope3D)
   {
      Set<HalfEdge3DReadOnly> halfEdgeSet = new HashSet<>();

      for (int halfEdgeIndex = 0; halfEdgeIndex < convexPolytope3D.getNumberOfHalfEdges(); halfEdgeIndex++)
      { // We assert the uniqueness of the half-edges.
         HalfEdge3DReadOnly halfEdge = convexPolytope3D.getHalfEdge(halfEdgeIndex);
         if (!halfEdgeSet.add(halfEdge))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge is a duplicate.");
      }

      for (int halfEdgeIndex = 0; halfEdgeIndex < convexPolytope3D.getNumberOfHalfEdges(); halfEdgeIndex++)
      {
         HalfEdge3DReadOnly halfEdge = convexPolytope3D.getHalfEdge(halfEdgeIndex);

         Vertex3DReadOnly origin = halfEdge.getOrigin();
         Vertex3DReadOnly destination = halfEdge.getDestination();
         HalfEdge3DReadOnly twin = halfEdge.getTwin();
         HalfEdge3DReadOnly next = halfEdge.getNext();
         HalfEdge3DReadOnly previous = halfEdge.getPrevious();
         Face3DReadOnly face = halfEdge.getFace();

         if (convexPolytope3D.getNumberOfFaces() > 1 && twin == null)
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's twin is null.");
         if (next == null)
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's next is null.");
         if (previous == null)
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's previous is null.");
         if (face == null)
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's face is null.");

         if (twin != null && (origin != twin.getDestination() || destination != twin.getOrigin()))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge is inconsistent with its twin.");
         if (origin != previous.getDestination())
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge is not attached to its previous.");
         if (destination != next.getOrigin())
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge is not attached to its next.");
         if (halfEdge.getFace() != previous.getFace())
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge does not share the same face as its previous.");
         if (halfEdge.getFace() != next.getFace())
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge does not share the same face as its next.");
         if (!origin.getAssociatedEdges().contains(halfEdge))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge is not associated to its origin.");
         if (origin != destination && destination.getAssociatedEdges().contains(halfEdge))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge should not be associated to its origin.");
         if (!halfEdge.getFace().getEdges().contains(halfEdge))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's face does not declare it as one of its edges.");

         if (!convexPolytope3D.getFaces().contains(face))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's face is not registered as a polytope face.");
         if (twin != null && !convexPolytope3D.getHalfEdges().contains(twin))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's twin is not registered as a polytope half-edge.");
         if (!convexPolytope3D.getHalfEdges().contains(next))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's next is not registered as a polytope half-edge.");
         if (!convexPolytope3D.getHalfEdges().contains(previous))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's previous is not registered as a polytope half-edge.");
         if (!convexPolytope3D.getVertices().contains(origin))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's origin is not registered as a polytope vertex.");
         if (!convexPolytope3D.getVertices().contains(destination))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's destination is not registered as a polytope vertex.");
      }
   }

   private static void assertConvexPolytope3DVerticesIntegrity(String messagePrefix, ConvexPolytope3DReadOnly convexPolytope3D)
   {
      Set<Vertex3DReadOnly> vertexSet = new HashSet<>();

      for (int vertexIndex = 0; vertexIndex < convexPolytope3D.getNumberOfVertices(); vertexIndex++)
      { // We assert the uniqueness of the vertices.
         Vertex3DReadOnly vertex = convexPolytope3D.getVertex(vertexIndex);
         if (!vertexSet.add(vertex))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, vertexIndex + "th vertex is a duplicate.");
      }

      for (int vertexIndex = 0; vertexIndex < convexPolytope3D.getNumberOfVertices(); vertexIndex++)
      {
         Vertex3DReadOnly vertex = convexPolytope3D.getVertex(vertexIndex);
         Collection<? extends HalfEdge3DReadOnly> associatedEdges = vertex.getAssociatedEdges();

         // Used to assert uniqueness of the faces obtained from the associated-edges.
         Set<Face3DReadOnly> connectedFaces = new HashSet<>();

         for (HalfEdge3DReadOnly associatedEdge : associatedEdges)
         {
            if (vertex != associatedEdge.getOrigin())
               EuclidCoreTestTools.throwAssertionError(messagePrefix, vertexIndex + "th vertex is not the origin of an associated edge.");

            if (!connectedFaces.add(associatedEdge.getFace()))
               EuclidCoreTestTools.throwAssertionError(messagePrefix, "The connected faces to " + vertexIndex + "th vertex are not unique.");

            if (convexPolytope3D.getNumberOfFaces() > 1)
            {
               if (associatedEdge.getDestination().getEdgeTo(vertex) == null)
                  EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                          vertexIndex + "th vertex has an edge which destination does not have an edge going back to it.");
               if (associatedEdge.getDestination().getEdgeTo(vertex) != associatedEdge.getTwin())
                  EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                          vertexIndex + "th vertex has an edge which destination's return edge is not the twin.");
            }
            if (!associatedEdge.getFace().getVertices().contains(vertex))
               EuclidCoreTestTools.throwAssertionError(messagePrefix, vertexIndex + "th vertex has an edge which face does not delare it.");

            if (!convexPolytope3D.getFaces().contains(associatedEdge.getFace()))
               EuclidCoreTestTools.throwAssertionError(messagePrefix, vertexIndex + "th vertex has an edge's face which is not registered as a polytope face.");
            if (!convexPolytope3D.getHalfEdges().contains(associatedEdge))
               EuclidCoreTestTools.throwAssertionError(messagePrefix, vertexIndex + "th vertex has an edge which is not registered as a polytope edge.");
            if (!convexPolytope3D.getVertices().contains(associatedEdge.getDestination()))
               EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                       vertexIndex + "th vertex has an edge's destination which is not registered as a polytope vertex.");
         }
      }
   }

   /**
    * Asserts on a per component basis that the two EuclidGeometries are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected EuclidGeomoetry. Not modified.
    * @param actual   the actual EuclidGeomoetry. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two EuclidGeomoetries are not equal. If only one of the arguments
    *                        is equal to {@code null}.
    */
   public static void assertEquals(EuclidGeometry expected, EuclidGeometry actual, double epsilon)
   {
      assertEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two EuclidGeometries are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected EuclidGeomoetry. Not modified.
    * @param actual        the actual EuclidGeomoetry. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two EuclidGeomoetries are not equal. If only one of the arguments
    *                        is equal to {@code null}.
    */
   public static void assertEquals(String messagePrefix, EuclidGeometry expected, EuclidGeometry actual, double epsilon)
   {
      assertEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two EuclidGeometries are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected EuclidGeomoetry. Not modified.
    * @param actual        the actual EuclidGeomoetry. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two EuclidGeomoetries are not equal. If only one of the arguments
    *                        is equal to {@code null}.
    */
   public static void assertEquals(String messagePrefix, EuclidGeometry expected, EuclidGeometry actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!(expected.epsilonEquals(actual, epsilon)))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts the two EuclidGeometries are geometrically equivalent to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected EuclidGeometry. Not modified.
    * @param actual   the actual EuclidGeometry. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two EuclidGeometries do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertGeometricallyEquals(EuclidGeometry expected, EuclidGeometry actual, double epsilon)
   {
      assertGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts the two EuclidGeometries are geometrically equivalent to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected EuclidGeometry. Not modified.
    * @param actual        the actual EuclidGeometry. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two EuclidGeometries do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertGeometricallyEquals(String messagePrefix, EuclidGeometry expected, EuclidGeometry actual, double epsilon)
   {
      assertGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts the two EuclidGeometries are geometrically equivalent to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected EuclidGeometry. Not modified.
    * @param actual        the actual EuclidGeometry. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two EuclidGeometries do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertGeometricallyEquals(String messagePrefix, EuclidGeometry expected, EuclidGeometry actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!(expected.geometricallyEquals(actual, epsilon)))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   private static void throwNotEqualAssertionError(String messagePrefix, EuclidGeometry expected, EuclidGeometry actual, String format)
   {
      String expectedAsString = expected.toString(format);
      String actualAsString = actual.toString(format);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   /**
    * Throws a new {@code AssertionError} as follows:
    *
    * <pre>
    * messagePrefix expected:
    * expectedAsString
    * but was:
    * actualAsString
    * </pre>
    *
    * @param messagePrefix    prefix to add to the error message.
    * @param expectedAsString the result that was expected in a {@code String} form.
    * @param actualAsString   the result that was obtained in a {@code String} form.
    */
   public static void throwNotEqualAssertionError(String messagePrefix, String expectedAsString, String actualAsString)
   {
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, null);
   }

   /**
    * Throws a new {@code AssertionError} as follows:
    *
    * <pre>
    * messagePrefix expected:
    * expectedAsString
    * but was:
    * actualAsString
    * Difference of: differenceAsString
    * </pre>
    *
    * @param messagePrefix      prefix to add to the error message.
    * @param expectedAsString   the result that was expected in a {@code String} form.
    * @param differenceAsString a short comprehensible summary of the difference between the expected
    *                           and obtained results.
    * @param actualAsString     the result that was obtained in a {@code String} form.
    */
   public static void throwNotEqualAssertionError(String messagePrefix, String expectedAsString, String actualAsString, String differenceAsString)
   {
      String errorMessage = addPrefixToMessage(messagePrefix, "expected:\n" + expectedAsString + "\n but was:\n" + actualAsString);
      if (differenceAsString != null)
         errorMessage += "\n" + differenceAsString;

      throw new AssertionError(errorMessage);
   }

   /**
    * Convenience method for prepending an optional prefix to a message.
    * <p>
    * In the case the given {@code prefix} is {@code null} or empty, the original {@code message} is
    * returned.
    * </p>
    *
    * @param prefix  the {@code String} to prepend to the message.
    * @param message the original message.
    * @return the message with the prefix.
    */
   public static String addPrefixToMessage(String prefix, String message)
   {
      if (prefix != null && !prefix.isEmpty())
         return prefix + " " + message;
      else
         return message;
   }

   private static void throwNotEqualAssertionError(String messagePrefix,
                                                   EuclidShape3DCollisionResultReadOnly expected,
                                                   EuclidShape3DCollisionResultReadOnly actual,
                                                   String format)
   {
      throwNotEqualAssertionError(messagePrefix, expected, actual, format, null);
   }

   private static void throwNotEqualAssertionError(String messagePrefix,
                                                   EuclidShape3DCollisionResultReadOnly expected,
                                                   EuclidShape3DCollisionResultReadOnly actual,
                                                   String format,
                                                   String differenceAsString)
   {
      String expectedAsString = getEuclidShape3DCollisionResultString(format, expected);
      String actualAsString = getEuclidShape3DCollisionResultString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, differenceAsString);
   }
}
