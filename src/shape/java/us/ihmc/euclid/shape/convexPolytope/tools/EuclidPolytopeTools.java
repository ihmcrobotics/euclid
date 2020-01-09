package us.ihmc.euclid.shape.convexPolytope.tools;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * This class provides a set of tools for performing operations with convex polytopes.
 *
 * @author Sylvain Bertrand
 */
public class EuclidPolytopeTools
{
   private EuclidPolytopeTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Calculates the of vertices for a convex polytope using Euler's formula.
    *
    * @param numberOfFaces the number of faces the convex polytope has.
    * @param numberOfEdges the number of edges the convex polytope has.
    * @return the number of vertices.
    */
   public static int computeConvexPolytopeNumberOfVertices(int numberOfFaces, int numberOfEdges)
   {
      return numberOfEdges - numberOfFaces + 2;
   }

   /**
    * Computes the cross product from two line segments as follows:
    *
    * <pre>
    * (lineSegmentEnd1 - lineSegmentStart1) x (lineSegmentEnd2 - lineSegmentStart2)
    * </pre>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param lineSegmentStart1 the start position of the first line segment. Not modified.
    * @param lineSegmentEnd1   the end position of the first line segment. Not modified.
    * @param lineSegmentStart2 the start position of the second line segment. Not modified.
    * @param lineSegmentEnd2   the end position of the second line segment. Not modified.
    * @return the result of the cross product.
    */
   public static Vector3D crossProductOfLineSegment3Ds(Point3DReadOnly lineSegmentStart1, Point3DReadOnly lineSegmentEnd1, Point3DReadOnly lineSegmentStart2,
                                                       Point3DReadOnly lineSegmentEnd2)
   {
      Vector3D crossProduct = new Vector3D();
      crossProductOfLineSegment3Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2, crossProduct);
      return crossProduct;
   }

   /**
    * Computes the cross product from two line segments as follows:
    *
    * <pre>
    * (lineSegmentEnd1 - lineSegmentStart1) x (lineSegmentEnd2 - lineSegmentStart2)
    * </pre>
    *
    * @param lineSegmentStart1  the start position of the first line segment. Not modified.
    * @param lineSegmentEnd1    the end position of the first line segment. Not modified.
    * @param lineSegmentStart2  the start position of the second line segment. Not modified.
    * @param lineSegmentEnd2    the end position of the second line segment. Not modified.
    * @param crossProductToPack the vector used to store the result of the cross product. Modified.
    */
   public static void crossProductOfLineSegment3Ds(Point3DReadOnly lineSegmentStart1, Point3DReadOnly lineSegmentEnd1, Point3DReadOnly lineSegmentStart2,
                                                   Point3DReadOnly lineSegmentEnd2, Vector3DBasics crossProductToPack)
   {
      double direction1X = lineSegmentEnd1.getX() - lineSegmentStart1.getX();
      double direction1Y = lineSegmentEnd1.getY() - lineSegmentStart1.getY();
      double direction1Z = lineSegmentEnd1.getZ() - lineSegmentStart1.getZ();

      double direction2X = lineSegmentEnd2.getX() - lineSegmentStart2.getX();
      double direction2Y = lineSegmentEnd2.getY() - lineSegmentStart2.getY();
      double direction2Z = lineSegmentEnd2.getZ() - lineSegmentStart2.getZ();

      double crossX = direction1Y * direction2Z - direction1Z * direction2Y;
      double crossY = direction1Z * direction2X - direction1X * direction2Z;
      double crossZ = direction1X * direction2Y - direction1Y * direction2X;
      crossProductToPack.set(crossX, crossY, crossZ);
   }

   /**
    * Tests whether the query is located on the left side of a line 3D.
    * <p>
    * The left side of the line is defined using a plane on which the line is lying. The left side of
    * the line can be defined as the region located above the plane define by:<br>
    * <tt>normal=(secondPointOnLine - firstPointOnLine) x planeNormal</tt> and<br>
    * <tt>point=firstPointOnLine</tt>
    * </p>
    *
    * @param point             the coordinates of the query. Not modified.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @param planeNormal       the normal of the plane the line is lying onto. Not modified.
    * @return {@code true} if the query is located on the left side of the line.
    */
   public static boolean isPoint3DOnLeftSideOfLine3D(Point3DReadOnly point, Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine,
                                                     Vector3DReadOnly planeNormal)
   {
      return isPoint3DOnSideOfLine3D(point, firstPointOnLine, secondPointOnLine, planeNormal, true);
   }

   /**
    * Tests whether the query is located on the left side of a line 3D.
    * <p>
    * The left side of the line is defined using a plane on which the line is lying. The left side of
    * the line can be defined as the region located above the plane define by:<br>
    * <tt>normal=lineDirection x planeNormal</tt> and<br>
    * <tt>point=pointOnLine</tt>
    * </p>
    *
    * @param point         the coordinates of the query. Not modified.
    * @param pointOnLine   a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @param planeNormal   the normal of the plane the line is lying onto. Not modified.
    * @return {@code true} if the query is located on the left side of the line.
    */
   public static boolean isPoint3DOnLeftSideOfLine3D(Point3DReadOnly point, Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection,
                                                     Vector3DReadOnly planeNormal)
   {
      return isPoint3DOnSideOfLine3D(point, pointOnLine, lineDirection, planeNormal, true);
   }

   /**
    * Tests whether the query is located on the right side of a line 3D.
    * <p>
    * The right side of the line is defined using a plane on which the line is lying. The right side of
    * the line can be defined as the region located below the plane define by:<br>
    * <tt>normal=(secondPointOnLine - firstPointOnLine) x planeNormal</tt> and<br>
    * <tt>point=firstPointOnLine</tt>
    * </p>
    *
    * @param point             the coordinates of the query. Not modified.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @param planeNormal       the normal of the plane the line is lying onto. Not modified.
    * @return {@code true} if the query is located on the right side of the line.
    */
   public static boolean isPoint3DOnRightSideOfLine3D(Point3DReadOnly point, Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine,
                                                      Vector3DReadOnly planeNormal)
   {
      return isPoint3DOnSideOfLine3D(point, firstPointOnLine, secondPointOnLine, planeNormal, false);
   }

   /**
    * Tests whether the query is located on the right side of a line 3D.
    * <p>
    * The right side of the line is defined using a plane on which the line is lying. The right side of
    * the line can be defined as the region located below the plane define by:<br>
    * <tt>normal=lineDirection x planeNormal</tt> and<br>
    * <tt>point=pointOnLine</tt>
    * </p>
    *
    * @param point         the coordinates of the query. Not modified.
    * @param pointOnLine   a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @param planeNormal   the normal of the plane the line is lying onto. Not modified.
    * @return {@code true} if the query is located on the right side of the line.
    */
   public static boolean isPoint3DOnRightSideOfLine3D(Point3DReadOnly point, Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection,
                                                      Vector3DReadOnly planeNormal)
   {
      return isPoint3DOnSideOfLine3D(point, pointOnLine, lineDirection, planeNormal, false);
   }

   /**
    * Tests whether the query is located on either the left or right of a line 3D.
    * <p>
    * The left side of the line is defined using a plane on which the line is lying. The left side of
    * the line can be defined as the region located above the plane define by:<br>
    * <tt>normal=(secondPointOnLine - firstPointOnLine) x planeNormal</tt> and<br>
    * <tt>point=firstPointOnLine</tt>
    * </p>
    *
    * @param pointX         the x-coordinate of the query.
    * @param pointY         the y-coordinate of the query.
    * @param pointZ         the z-coordinate of the query.
    * @param pointOnLineX   the x-coordinate of the a point located on the line.
    * @param pointOnLineY   the y-coordinate of the a point located on the line.
    * @param pointOnLineZ   the z-coordinate of the a point located on the line.
    * @param lineDirectionX the x-component of the line direction.
    * @param lineDirectionY the y-component of the line direction.
    * @param lineDirectionZ the z-component of the line direction.
    * @param planeNormalX   the x-component of the plane normal.
    * @param planeNormalY   the y-component of the plane normal.
    * @param planeNormalZ   the z-component of the plane normal.
    * @param testLeftSide   the query of the side, when equal to {@code true} this will test for the
    *                       left side, {@code false} this will test for the right side.
    * @return {@code true} if the point is on the query side of the line, {@code false} if the point is
    *         on the opposite side or exactly on the line.
    */
   public static boolean isPoint3DOnSideOfLine3D(double pointX, double pointY, double pointZ, double pointOnLineX, double pointOnLineY, double pointOnLineZ,
                                                 double lineDirectionX, double lineDirectionY, double lineDirectionZ, double planeNormalX, double planeNormalY,
                                                 double planeNormalZ, boolean testLeftSide)
   {
      double dx = pointX - pointOnLineX;
      double dy = pointY - pointOnLineY;
      double dz = pointZ - pointOnLineZ;
      // (lineDirection) X (dx, dy, dz)
      double crossX = lineDirectionY * dz - lineDirectionZ * dy;
      double crossY = lineDirectionZ * dx - lineDirectionX * dz;
      double crossZ = lineDirectionX * dy - lineDirectionY * dx;

      double dotProduct = crossX * planeNormalX + crossY * planeNormalY + crossZ * planeNormalZ;

      if (testLeftSide)
         return dotProduct > 0.0;
      else
         return dotProduct < 0.0;
   }

   /**
    * Tests whether the query is located on either the left or right of a line 3D.
    * <p>
    * The left side of the line is defined using a plane on which the line is lying. The left side of
    * the line can be defined as the region located above the plane define by:<br>
    * <tt>normal=(secondPointOnLine - firstPointOnLine) x planeNormal</tt> and<br>
    * <tt>point=firstPointOnLine</tt>
    * </p>
    *
    * @param point             the coordinates of the query. Not modified.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @param planeNormal       the normal of the plane the line is lying onto. Not modified.
    * @param testLeftSide      the query of the side, when equal to {@code true} this will test for the
    *                          left side, {@code false} this will test for the right side.
    * @return {@code true} if the point is on the query side of the line, {@code false} if the point is
    *         on the opposite side or exactly on the line.
    */
   public static boolean isPoint3DOnSideOfLine3D(Point3DReadOnly point, Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine,
                                                 Vector3DReadOnly planeNormal, boolean testLeftSide)
   {
      double pointOnLineX = firstPointOnLine.getX();
      double pointOnLineY = firstPointOnLine.getY();
      double pointOnLineZ = firstPointOnLine.getZ();
      double lineDirectionX = secondPointOnLine.getX() - firstPointOnLine.getX();
      double lineDirectionY = secondPointOnLine.getY() - firstPointOnLine.getY();
      double lineDirectionZ = secondPointOnLine.getZ() - firstPointOnLine.getZ();
      return isPoint3DOnSideOfLine3D(point.getX(),
                                     point.getY(),
                                     point.getZ(),
                                     pointOnLineX,
                                     pointOnLineY,
                                     pointOnLineZ,
                                     lineDirectionX,
                                     lineDirectionY,
                                     lineDirectionZ,
                                     planeNormal.getX(),
                                     planeNormal.getY(),
                                     planeNormal.getZ(),
                                     testLeftSide);
   }

   /**
    * Tests whether the query is located on either the left or right of a line 3D.
    * <p>
    * The left side of the line is defined using a plane on which the line is lying. The left side of
    * the line can be defined as the region located above the plane define by:<br>
    * <tt>normal=(secondPointOnLine - firstPointOnLine) x planeNormal</tt> and<br>
    * <tt>point=firstPointOnLine</tt>
    * </p>
    *
    * @param point         the coordinates of the query. Not modified.
    * @param pointOnLine   a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @param planeNormal   the normal of the plane the line is lying onto. Not modified.
    * @param testLeftSide  the query of the side, when equal to {@code true} this will test for the
    *                      left side, {@code false} this will test for the right side.
    * @return {@code true} if the point is on the query side of the line, {@code false} if the point is
    *         on the opposite side or exactly on the line.
    */
   public static boolean isPoint3DOnSideOfLine3D(Point3DReadOnly point, Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection,
                                                 Vector3DReadOnly planeNormal, boolean testLeftSide)
   {
      return isPoint3DOnSideOfLine3D(point.getX(),
                                     point.getY(),
                                     point.getZ(),
                                     pointOnLine.getX(),
                                     pointOnLine.getY(),
                                     pointOnLine.getZ(),
                                     lineDirection.getX(),
                                     lineDirection.getY(),
                                     lineDirection.getZ(),
                                     planeNormal.getX(),
                                     planeNormal.getY(),
                                     planeNormal.getZ(),
                                     testLeftSide);
   }

   /**
    * Computes the distance squared to the closest half-edge to the query.
    *
    * @param query     the coordinates of the query. Not modified.
    * @param halfEdges the half-edges to search for the closest to the query. Not modified.
    * @return the distance squared between the query and the closest half-edge to it.
    */
   public static double distanceSquaredToClosestHalfEdge3D(Point3DReadOnly query, List<? extends HalfEdge3DReadOnly> halfEdges)
   {
      if (halfEdges.isEmpty())
         return Double.NaN;

      double minDistanceSquared = halfEdges.get(0).distanceSquared(query);

      for (int edgeIndex = 1; edgeIndex < halfEdges.size(); edgeIndex++)
      {
         minDistanceSquared = Math.min(minDistanceSquared, halfEdges.get(edgeIndex).distanceSquared(query));
      }
      return minDistanceSquared;
   }

   /**
    * Computes the distance to the closest half-edge to the query.
    *
    * @param query     the coordinates of the query. Not modified.
    * @param halfEdges the half-edges to search for the closest to the query. Not modified.
    * @return the distance between the query and the closest half-edge to it.
    */
   public static double distanceToClosestHalfEdge3D(Point3DReadOnly query, List<? extends HalfEdge3DReadOnly> halfEdges)
   {
      return EuclidCoreTools.squareRoot(distanceSquaredToClosestHalfEdge3D(query, halfEdges));
   }

   /**
    * Finds the silhouette that is the continuous set of half-edges separating the faces visible from
    * the observer from the hidden faces.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param faces    the list of faces to search the silhouette from. Not modified.
    * @param observer the coordinates of the observer. Not modified.
    * @param epsilon  the tolerance used to determine whether a face is visible, the observer lies in a
    *                 face's support plane, or a face is not visible.
    * @param <F>      the type to use for the faces, it has to implement {@link Face3DReadOnly}.
    * @param <E>      the type of edges to return, it has to implement {@link HalfEdge3DReadOnly} and
    *                 has to be common to all the faces' edges.
    * @return the list of edges representing the silhouette, or {@code null} if the observer cannot see
    *         any face or if the observer is inside one of the faces.
    */
   public static <F extends Face3DReadOnly, E extends HalfEdge3DReadOnly> List<E> computeSilhouette(List<F> faces, Point3DReadOnly observer, double epsilon)
   {
      return computeSilhouette(faces, observer, epsilon, null);
   }

   /**
    * Finds the silhouette that is the continuous set of half-edges separating the faces visible from
    * the observer from the hidden faces.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param faces              the list of faces to search the silhouette from. Not modified.
    * @param observer           the coordinates of the observer. Not modified.
    * @param epsilon            the tolerance used to determine whether a face is visible, the observer
    *                           lies in a face's support plane, or a face is not visible.
    * @param visibleFacesToPack the collection used to store the visible faces. It is cleared before
    *                           starting the search. It preferable to provide an implementation that
    *                           supports fast queries for {@link Collection#contains(Object)}.
    *                           Modified. Can be {@code null}.
    * @param <F>                the type to use for the faces, it has to implement
    *                           {@link Face3DReadOnly}.
    * @param <E>                the type of edges to return, it has to implement
    *                           {@link HalfEdge3DReadOnly} and has to be common to all the faces'
    *                           edges.
    * @return the list of edges representing the silhouette, or {@code null} if the observer cannot see
    *         any face or if the observer is inside one of the faces.
    */
   @SuppressWarnings("unchecked")
   public static <F extends Face3DReadOnly, E extends HalfEdge3DReadOnly> List<E> computeSilhouette(List<F> faces, Point3DReadOnly observer, double epsilon,
                                                                                                    Collection<F> visibleFacesToPack)
   {
      if (faces.isEmpty())
         return null;
      if (faces.size() == 1)
         return (List<E>) faces.get(0).getEdges();

      Face3DReadOnly leastVisibleFace = null;
      double minimumDistance = Double.POSITIVE_INFINITY;

      if (visibleFacesToPack == null)
         visibleFacesToPack = new HashSet<>();

      visibleFacesToPack.clear();

      for (int faceIndex = 0; faceIndex < faces.size(); faceIndex++)
      { // First we go through the faces and sort them into 2 categories: visible faces, faces which support plane contains the observer.
         F face = faces.get(faceIndex);

         double signedDistance = face.signedDistanceFromSupportPlane(observer);

         if (signedDistance <= epsilon)
         {
            if (signedDistance >= -epsilon && face.isPointDirectlyAboveOrBelow(observer))
               return null; // The observer belongs to the face.
            else
               continue;
         }

         if (signedDistance < minimumDistance)
         {
            leastVisibleFace = face;
            minimumDistance = signedDistance;
         }

         visibleFacesToPack.add(face);
      }

      if (visibleFacesToPack.isEmpty())
         return null; // The observer cannot see any face => no silhouette.

      if (visibleFacesToPack.size() > 1)
      {
         for (F visibleFace : visibleFacesToPack)
         {
            boolean hasAtLeastOneVisibleNeighbor = false;

            for (int edgeIndex = 0; edgeIndex < visibleFace.getNumberOfEdges(); edgeIndex++)
            {
               if (visibleFacesToPack.contains(visibleFace.getNeighbor(edgeIndex)))
               {
                  hasAtLeastOneVisibleNeighbor = true;
                  break;
               }
            }

            if (!hasAtLeastOneVisibleNeighbor)
               return null;
         }
      }

      // Now we search for the silhouette and we start by looking for the first edge.
      E silhouetteStartEdge = null;

      for (int neighborIndex = 0; neighborIndex < leastVisibleFace.getNumberOfEdges(); neighborIndex++)
      {
         Face3DReadOnly neighbor = leastVisibleFace.getNeighbor(neighborIndex);

         if (!visibleFacesToPack.contains(neighbor))
         {
            silhouetteStartEdge = (E) leastVisibleFace.getEdge(neighborIndex).getTwin();
            break;
         }
      }

      assert silhouetteStartEdge != null;

      // Now we can finally build the silhouette
      List<E> silhouette = new ArrayList<>();
      silhouette.add(silhouetteStartEdge);

      Vertex3DReadOnly currentVertex = silhouetteStartEdge.getDestination();

      while (currentVertex != silhouetteStartEdge.getOrigin())
      {
         boolean foundNextEdge = false;

         for (HalfEdge3DReadOnly candidate : currentVertex.getAssociatedEdges())
         {
            if (visibleFacesToPack.contains(candidate.getFace()))
               continue; // The associated face should not be visible.
            if (!visibleFacesToPack.contains(candidate.getTwin().getFace()))
               continue; // The associated face to the twin edge should be visible.

            silhouette.add((E) candidate);
            currentVertex = candidate.getDestination();
            foundNextEdge = true;
            break;
         }

         if (!foundNextEdge)
         {
            throw new RuntimeException("Failed collecting the silhouette's edges");
         }
      }

      return silhouette;
   }

   /**
    * Navigates the given silhouette and collects the faces which support plane is close enough to the
    * query.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param query      the coordinates of the query. Not modified.
    * @param silhouette the list of the half-edges to navigate. Not modified.
    * @param epsilon    the tolerance used for determining whether the query is close to a face support
    *                   plane.
    * @return the list of faces for which the support plane is close to the query.
    */
   @SuppressWarnings("unchecked")
   public static <F extends Face3DReadOnly, E extends HalfEdge3DReadOnly> List<F> computeInPlaneFacesAroundSilhouette(Point3DReadOnly query,
                                                                                                                      Collection<E> silhouette, double epsilon)
   {
      List<F> inPlaneFacesToPack = new ArrayList<>();

      for (E edge : silhouette)
      {
         F face = (F) edge.getFace();

         if (inPlaneFacesToPack.contains(face))
            continue;

         if (arePoint3DAndFace3DInPlane(query, face, epsilon))
            inPlaneFacesToPack.add(face);
      }

      return inPlaneFacesToPack;
   }

   /**
    * Tests whether a point is close enough to a face support plane such that it could be extended to
    * include the query.
    *
    * @param query   the coordinates of the query. Not modified.
    * @param face    the face to evaluate. Not modified.
    * @param epsilon the tolerance used for determining whether the query is close to the face support
    *                plane.
    * @return {@code true} if the point is considered to be close enough to the face support plane,
    *         {@code false} otherwise.
    */
   public static boolean arePoint3DAndFace3DInPlane(Point3DReadOnly query, Face3DReadOnly face, double epsilon)
   {
      double distanceToPlane = face.distanceFromSupportPlane(query);

      if (distanceToPlane <= epsilon)
      {
         return true;
      }
      else if (distanceToPlane >= 4.0 * epsilon)
      {
         return false;
      }
      else
      {
         Point3D average = new Point3D();
         Vector3D normal = new Vector3D(face.getNormal());
         List<Point3DReadOnly> extendedFaceVertices = new ArrayList<>(face.getVertices());
         extendedFaceVertices.add(query);
         EuclidPolytopeConstructionTools.updateFace3DNormal(extendedFaceVertices, average, normal);

         for (Point3DReadOnly extendedFaceVertex : extendedFaceVertices)
         {
            if (EuclidGeometryTools.distanceFromPoint3DToPlane3D(extendedFaceVertex, average, normal) > epsilon)
            {
               return false;
            }
         }

         return true;
      }
   }

   /**
    * Tests whether a point is close enough to a half-edge support line such that it could be extended
    * to include the query.
    *
    * @param query    the coordinates of the query. Not modified.
    * @param halfEdge the half-edge to evaluate. Not modified.
    * @param epsilon  the tolerance used for determining whether the query is close to the half-edge
    *                 support line.
    * @return {@code true} if the point is considered to be close enough to the half-edge support
    *         plane, {@code false} otherwise.
    */
   public static boolean arePoint3DAndHalfEdge3DInLine(Point3DReadOnly query, HalfEdge3DReadOnly halfEdge, double epsilon)
   {
      if (halfEdge.distanceFromSupportLine(query) <= epsilon)
      {
         return true;
      }
      else
      {
         if (halfEdge.getOrigin().distanceSquared(query) > halfEdge.getDestination().distanceSquared(query))
         {
            if (EuclidGeometryTools.distanceFromPoint3DToLine3D(halfEdge.getDestination(), query, halfEdge.getOrigin()) <= epsilon)
               return true;
         }
         else
         {
            if (EuclidGeometryTools.distanceFromPoint3DToLine3D(halfEdge.getOrigin(), query, halfEdge.getDestination()) <= epsilon)
               return true;
         }

         return false;
      }
   }

   /**
    * Tests whether the tetrahedron defined by the given vertices contains the origin.
    *
    * @param p0 the first vertex of the tetrahedron. Not modified.
    * @param p1 the second vertex of the tetrahedron. Not modified.
    * @param p2 the third vertex of the tetrahedron. Not modified.
    * @param p3 the fourth vertex of the tetrahedron. Not modified.
    * @return {@code true} if the origin is located inside the tetrahedron, {@code false} otherwise.
    */
   public static boolean tetrahedronContainsOrigin(Point3DReadOnly p0, Point3DReadOnly p1, Point3DReadOnly p2, Point3DReadOnly p3)
   {
      Vector3D n;
      n = EuclidPolytopeTools.crossProductOfLineSegment3Ds(p0, p1, p0, p2); // (p1 - p0) x (p2 - p0)
      if (TupleTools.dot(n, p0) > 0.0 == TupleTools.dot(n, p3) > 0.0)
         return false;
      n = EuclidPolytopeTools.crossProductOfLineSegment3Ds(p1, p2, p1, p3); // (p2 - p1) x (p3 - p1)
      if (TupleTools.dot(n, p1) > 0.0 == TupleTools.dot(n, p0) > 0.0)
         return false;
      n = EuclidPolytopeTools.crossProductOfLineSegment3Ds(p1, p2, p1, p3); // (p3 - p2) x (p0 - p2)
      if (TupleTools.dot(n, p2) > 0.0 == TupleTools.dot(n, p1) > 0.0)
         return false;
      n = EuclidPolytopeTools.crossProductOfLineSegment3Ds(p3, p0, p3, p1); // (p0 - p3) x (p1 - p3)
      if (TupleTools.dot(n, p3) > 0.0 == TupleTools.dot(n, p2) > 0.0)
         return false;
      return true;
   }
}
