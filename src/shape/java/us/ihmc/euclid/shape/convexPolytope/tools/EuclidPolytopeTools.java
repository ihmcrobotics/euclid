package us.ihmc.euclid.shape.convexPolytope.tools;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

// TODO Remove unused/untested methods
public class EuclidPolytopeTools
{
   private EuclidPolytopeTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   // From nVertices - nEdges + nFaces = 2
   public static int computeConvexPolytopeNumberOfVertices(int numberOfFaces, int numberOfEdges)
   {
      return numberOfEdges - numberOfFaces + 2;
   }

   public static Vector3D crossProductOfLineSegment3Ds(Point3DReadOnly lineSegmentStart1, Point3DReadOnly lineSegmentEnd1, Point3DReadOnly lineSegmentStart2,
                                                       Point3DReadOnly lineSegmentEnd2)
   {
      Vector3D crossProduct = new Vector3D();
      crossProductOfLineSegment3Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2, crossProduct);
      return crossProduct;
   }

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

   public static boolean isPoint3DOnLeftSideOfLine3D(Point3DReadOnly point, Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine,
                                                     Vector3DReadOnly planeNormal, double epsilon)
   {
      return isPoint3DOnSideOfLine3D(point, firstPointOnLine, secondPointOnLine, planeNormal, true, epsilon);
   }

   public static boolean isPoint3DOnLeftSideOfLine3D(Point3DReadOnly point, Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection,
                                                     Vector3DReadOnly planeNormal, double epsilon)
   {
      return isPoint3DOnSideOfLine3D(point, pointOnLine, lineDirection, planeNormal, true, epsilon);
   }

   public static boolean isPoint3DOnRightSideOfLine3D(Point3DReadOnly point, Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine,
                                                      Vector3DReadOnly planeNormal, double epsilon)
   {
      return isPoint3DOnSideOfLine3D(point, firstPointOnLine, secondPointOnLine, planeNormal, false, epsilon);
   }

   public static boolean isPoint3DOnRightSideOfLine3D(Point3DReadOnly point, Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection,
                                                      Vector3DReadOnly planeNormal, double epsilon)
   {
      return isPoint3DOnSideOfLine3D(point, pointOnLine, lineDirection, planeNormal, false, epsilon);
   }

   public static boolean isPoint3DOnSideOfLine3D(double pointX, double pointY, double pointZ, double pointOnLineX, double pointOnLineY, double pointOnLineZ,
                                                 double lineDirectionX, double lineDirectionY, double lineDirectionZ, double planeNormalX, double planeNormalY,
                                                 double planeNormalZ, boolean testLeftSide, double epsilon)
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
         return dotProduct > epsilon;
      else
         return dotProduct < -epsilon;
   }

   public static boolean isPoint3DOnSideOfLine3D(Point3DReadOnly point, Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine,
                                                 Vector3DReadOnly planeNormal, boolean testLeftSide, double epsilon)
   {
      double pointOnLineX = firstPointOnLine.getX();
      double pointOnLineY = firstPointOnLine.getY();
      double pointOnLineZ = firstPointOnLine.getZ();
      double lineDirectionX = secondPointOnLine.getX() - firstPointOnLine.getX();
      double lineDirectionY = secondPointOnLine.getY() - firstPointOnLine.getY();
      double lineDirectionZ = secondPointOnLine.getZ() - firstPointOnLine.getZ();
      return isPoint3DOnSideOfLine3D(point.getX(), point.getY(), point.getZ(), pointOnLineX, pointOnLineY, pointOnLineZ, lineDirectionX, lineDirectionY,
                                     lineDirectionZ, planeNormal.getX(), planeNormal.getY(), planeNormal.getZ(), testLeftSide, epsilon);
   }

   public static boolean isPoint3DOnSideOfLine3D(Point3DReadOnly point, Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection,
                                                 Vector3DReadOnly planeNormal, boolean testLeftSide, double epsilon)
   {
      return isPoint3DOnSideOfLine3D(point.getX(), point.getY(), point.getZ(), pointOnLine.getX(), pointOnLine.getY(), pointOnLine.getZ(), lineDirection.getX(),
                                     lineDirection.getY(), lineDirection.getZ(), planeNormal.getX(), planeNormal.getY(), planeNormal.getZ(), testLeftSide,
                                     epsilon);
   }

   /**
    * Returns the minimum signed distance between the projection of a 3D point and an infinitely long
    * 3D line defined by a point and a direction onto a plane of given normal.
    * <p>
    * The calculated distance is negative if the query is located on the right side of the line.
    * </p>
    * <p>
    * The notion of left side is defined as the semi-open space that starts at {@code pointOnLine} and
    * extends to the direction given by the vector <tt>planeNormal &times; lineDirection</tt> and the
    * right side starts at {@code pointOnLine} and direction
    * <tt>-planeNormal &times; lineDirection</tt>. In an intuitive manner, the left side refers to the
    * side on the left of the line when looking at the line with the plane normal pointing toward you.
    * </p>
    * <p>
    * Note that the position of the plane does not affect the distance separating the query from the
    * line, so it not required.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code lineDirection.length() < }{@value #ONE_TRILLIONTH}, this method returns the
    * distance between {@code pointOnLine} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param pointX x-coordinate of the query.
    * @param pointY y-coordinate of the query.
    * @param pointZ z-coordinate of the query.
    * @param pointOnLineX x-coordinate of a point located on the line.
    * @param pointOnLineY y-coordinate of a point located on the line.
    * @param pointOnLineZ z-coordinate of a point located on the line.
    * @param lineDirectionX x-component of the line direction.
    * @param lineDirectionY y-component of the line direction.
    * @param lineDirectionZ y-component of the line direction.
    * @param planeNormalX x-component of the plane normal.
    * @param planeNormalY y-component of the plane normal.
    * @param planeNormalZ z-component of the plane normal.
    * @return the minimum distance between the projection of the 3D point and line onto the plane. The
    *         distance is negative if the query is located on the right side of the line.
    */
   public static double signedDistanceFromPoint3DToLine3D(double pointX, double pointY, double pointZ, double pointOnLineX, double pointOnLineY,
                                                          double pointOnLineZ, double lineDirectionX, double lineDirectionY, double lineDirectionZ,
                                                          double planeNormalX, double planeNormalY, double planeNormalZ)
   {
      return EuclidGeometryTools.signedDistanceFromPoint3DToPlane3D(pointX, pointY, pointZ, pointOnLineX, pointOnLineY, pointOnLineZ, planeNormalX,
                                                                    planeNormalY, planeNormalZ, lineDirectionX, lineDirectionY, lineDirectionZ);
   }

   public static double signedDistanceFromPoint3DToLine3D(Point3DReadOnly point, Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine,
                                                          Vector3DReadOnly planeNormal)
   {
      double pointOnLineX = firstPointOnLine.getX();
      double pointOnLineY = firstPointOnLine.getY();
      double pointOnLineZ = firstPointOnLine.getZ();
      double lineDirectionX = secondPointOnLine.getX() - firstPointOnLine.getX();
      double lineDirectionY = secondPointOnLine.getY() - firstPointOnLine.getY();
      double lineDirectionZ = secondPointOnLine.getZ() - firstPointOnLine.getZ();
      return signedDistanceFromPoint3DToLine3D(point.getX(), point.getY(), point.getZ(), pointOnLineX, pointOnLineY, pointOnLineZ, lineDirectionX,
                                               lineDirectionY, lineDirectionZ, planeNormal.getX(), planeNormal.getY(), planeNormal.getZ());
   }

   public static double signedDistanceFromPoint3DToLine3D(Point3DReadOnly point, Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection,
                                                          Vector3DReadOnly planeNormal)
   {
      return signedDistanceFromPoint3DToLine3D(point.getX(), point.getY(), point.getZ(), pointOnLine.getX(), pointOnLine.getY(), pointOnLine.getZ(),
                                               lineDirection.getX(), lineDirection.getY(), lineDirection.getZ(), planeNormal.getX(), planeNormal.getY(),
                                               planeNormal.getZ());
   }

   public static boolean canObserverSeeFace(Point3DReadOnly observer, Face3DReadOnly face, double epsilon)
   {
      return face.signedDistanceToPlane(observer) > epsilon;
   }

   public static boolean isOnFaceSupportPlane(Point3DReadOnly observer, Face3DReadOnly face, double epsilon)
   {
      return face.distanceToPlane(observer) <= epsilon;
   }

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

   public static double distanceToClosestHalfEdge3D(Point3DReadOnly query, List<? extends HalfEdge3DReadOnly> halfEdges)
   {
      return Math.sqrt(distanceSquaredToClosestHalfEdge3D(query, halfEdges));
   }

   /**
    * Filters the given {@code faces} to only return the ones that the given {@code observer} can see.
    * <p>
    * The least visible face is the first element of the returned list. Besides the latter, the
    * returned list follows no particular order.
    * </p>
    *
    * @param faces the list of faces to be tested. Not modified.
    * @param observer the location of the observer looking at the faces. Not modified.
    * @param visibilityThreshold the minimum distance between the observer and a face's plane before
    *           the face is consider visible. When negative, the observer can be below the face's plane
    *           and still be able to see the face.
    * @return the list of visible faces with in first position the least visible face.
    */
   public static <F extends Face3DReadOnly> List<F> extractVisibleFaces(List<F> faces, Point3DReadOnly observer, double visibilityThreshold)
   {
      List<F> visibleFaces = new ArrayList<>();

      int leastVisibleFaceIndex = -1;
      double minimumDistance = Double.POSITIVE_INFINITY;

      for (int faceIndex = 0; faceIndex < faces.size(); faceIndex++)
      {
         F face = faces.get(faceIndex);

         double signedDistance = face.signedDistanceToPlane(observer);

         if (signedDistance <= visibilityThreshold)
            continue;

         if (signedDistance < minimumDistance)
         {
            leastVisibleFaceIndex = visibleFaces.size();
            minimumDistance = signedDistance;
         }

         visibleFaces.add(face);
      }

      if (!visibleFaces.isEmpty())
      { // Moving the least visible to first position
         Collections.swap(visibleFaces, 0, leastVisibleFaceIndex);
      }

      return visibleFaces;
   }

   /**
    * Finds for the given {@code observer} coordinates the visible faces and the faces for which the
    * observer lies in their support plane.
    *
    * @param faces the list of faces to search through.
    * @param observer the coordinates from where we look at the faces.
    * @param epsilon the tolerance used to determine whether a face is visible, the observer lies in a
    *           face's support plane, or a face is not visible.
    * @param visibleFacesToPack the list used to store the visible faces. It is cleared before starting
    *           the search. The least visible face is packed as the first element of the list.
    * @param inPlaneFacesToPack the list used to store the faces for which the observer lies in their
    *           support plane. It is cleared before starting the search.
    * @return {@code true} if the observer cannot see any of the faces or if the observer is inside one
    *         of the faces, {@code false} otherwise.
    */
   public static <F extends Face3DReadOnly> boolean extractVisibleFaces(List<F> faces, Point3DReadOnly observer, double epsilon, List<F> visibleFacesToPack,
                                                                        List<F> inPlaneFacesToPack)
   {
      visibleFacesToPack.clear();
      inPlaneFacesToPack.clear();

      int leastVisibleFaceIndex = -1;
      double minimumDistance = Double.POSITIVE_INFINITY;

      for (int faceIndex = 0; faceIndex < faces.size(); faceIndex++)
      {
         F face = faces.get(faceIndex);

         double signedDistance = face.signedDistanceToPlane(observer);

         if (signedDistance <= epsilon)
         {
            if (signedDistance > -epsilon)
            {
               inPlaneFacesToPack.add(face);
               if (face.isPointDirectlyAboveOrBelow(observer))
                  return true;
            }
            continue;
         }

         if (signedDistance < minimumDistance)
         {
            leastVisibleFaceIndex = visibleFacesToPack.size();
            minimumDistance = signedDistance;
         }

         visibleFacesToPack.add(face);
      }

      if (!visibleFacesToPack.isEmpty())
      { // Moving the least visible to first position
         Collections.swap(visibleFacesToPack, 0, leastVisibleFaceIndex);
         return true;
      }
      else
      {
         return false;
      }
   }

   public static <F extends Face3DReadOnly> List<F> extractInPlaneFaces(List<F> faces, Point3DReadOnly query, double distanceThreshold)
   {
      return faces.stream().filter(face -> face.isPointInFacePlane(query, distanceThreshold)).collect(Collectors.toList());
   }

   /**
    * Finds the silhouette representing the border of the visible set of faces from the perspective of
    * an observer.
    *
    * @param faces the list of faces to search the silhouette from.
    * @param observer the coordinates of the observer.
    * @param observer the coordinates from where we look at the faces.
    * @param epsilon the tolerance used to determine whether a face is visible, the observer lies in a
    *           face's support plane, or a face is not visible.
    * @return the list of edges representing the silhouette, or {@code null} if the observer cannot see
    *         any face or if the observer is inside one of the faces.
    */
   public static <F extends Face3DReadOnly, E extends HalfEdge3DReadOnly> List<E> computeSilhouette(List<F> faces, Point3DReadOnly observer, double epsilon)
   {
      return computeSilhouette(faces, observer, epsilon, null);
   }

   /**
    * Finds the silhouette representing the border of the visible set of faces from the perspective of
    * an observer.
    *
    * @param faces the list of faces to search the silhouette from.
    * @param observer the coordinates of the observer.
    * @param observer the coordinates from where we look at the faces.
    * @param epsilon the tolerance used to determine whether a face is visible, the observer lies in a
    *           face's support plane, or a face is not visible.
    * @param visibleFacesToPack the collection used to store the visible faces. It is cleared before
    *           starting the search. It preferable to provide an implementation that supports fast
    *           queries for {@link Collection#contains(Object)}. Can be {@code null}.
    * @param <F> the type to use for the faces, it has to implement {@link Face3DReadOnly}.
    * @param <E> the type of edges to return, it has to implement {@link HalfEdge3DReadOnly} and has to
    *           be common to all the faces' edges.
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

         double signedDistance = face.signedDistanceToPlane(observer);

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

      int iteration = 0;

      while (currentVertex != silhouetteStartEdge.getOrigin())
      {
         if (iteration++ >= 10000000)
         {
            System.err.println(EuclidPolytopeTools.class.getSimpleName() + ": computeSilhouette got stuck in infinite loop.");
            return null;
         }

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

   @SuppressWarnings("unchecked")
   public static <F extends Face3DReadOnly, E extends HalfEdge3DReadOnly> List<F> computeInPlaneFacesAroundSilhouette(Point3DReadOnly observer,
                                                                                                                      Collection<E> silhouette, double epsilon)
   {
      List<F> inPlaneFacesToPack = new ArrayList<>();

      for (E edge : silhouette)
      {
         F face = (F) edge.getFace();

         if (inPlaneFacesToPack.contains(face))
            continue;

         if (arePoint3DAndFace3DInPlane(observer, face, epsilon))
            inPlaneFacesToPack.add(face);
      }

      return inPlaneFacesToPack;
   }

   public static boolean arePoint3DAndFace3DInPlane(Point3DReadOnly point, Face3DReadOnly face, double epsilon)
   {
      double distanceToPlane = face.distanceToPlane(point);

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
         extendedFaceVertices.add(point);
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

   public static boolean arePoint3DAndHalfEdge3DInLine(Point3DReadOnly point, HalfEdge3DReadOnly halfEdge, double epsilon)
   {
      if (halfEdge.distanceFromSupportLine(point) <= epsilon)
      {
         return true;
      }
      else
      {
         if (halfEdge.getOrigin().distanceSquared(point) > halfEdge.getDestination().distanceSquared(point))
         {
            if (EuclidGeometryTools.distanceFromPoint3DToLine3D(halfEdge.getDestination(), point, halfEdge.getOrigin()) <= epsilon)
               return true;
         }
         else
         {
            if (EuclidGeometryTools.distanceFromPoint3DToLine3D(halfEdge.getOrigin(), point, halfEdge.getDestination()) <= epsilon)
               return true;
         }

         return false;
      }
   }

   public static boolean isPointDirectlyAboveOrBelowAnyFace(List<? extends Face3DReadOnly> faces, Point3DReadOnly query)
   {
      for (int i = 0; i < faces.size(); i++)
      {
         if (faces.get(i).isPointDirectlyAboveOrBelow(query))
            return true;
      }
      return false;
   }

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
