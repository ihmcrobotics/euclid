package us.ihmc.euclid.referenceFrame.interfaces;

import java.util.List;

import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DBasics;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

/**
 * Read-only interface for a convex polygon defined in the XY-plane expressed in an immutable
 * reference frame.
 * <p>
 * This implementation of convex polygon is designed for garbage free operations.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FrameConvexPolygon2DReadOnly extends ConvexPolygon2DReadOnly, ReferenceFrameHolder, FrameVertex2DSupplier
{
   /** {@inheritDoc} */
   @Override
   List<? extends FramePoint2DReadOnly> getVertexBufferView();

   /** {@inheritDoc} */
   @Override
   default List<? extends FramePoint2DReadOnly> getPolygonVerticesView()
   {
      return getVertexBufferView().subList(0, getNumberOfVertices());
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint2DReadOnly getVertex(int index)
   {
      return (FramePoint2DReadOnly) ConvexPolygon2DReadOnly.super.getVertex(index);
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint2DReadOnly getNextVertex(int index)
   {
      return (FramePoint2DReadOnly) ConvexPolygon2DReadOnly.super.getNextVertex(index);
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint2DReadOnly getPreviousVertex(int index)
   {
      return (FramePoint2DReadOnly) ConvexPolygon2DReadOnly.super.getPreviousVertex(index);
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint2DReadOnly getVertexCCW(int index)
   {
      return (FramePoint2DReadOnly) ConvexPolygon2DReadOnly.super.getVertexCCW(index);
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint2DReadOnly getNextVertexCCW(int index)
   {
      return (FramePoint2DReadOnly) ConvexPolygon2DReadOnly.super.getNextVertexCCW(index);
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint2DReadOnly getPreviousVertexCCW(int index)
   {
      return (FramePoint2DReadOnly) ConvexPolygon2DReadOnly.super.getPreviousVertexCCW(index);
   }

   /** {@inheritDoc} */
   @Override
   FramePoint2DReadOnly getCentroid();

   /**
    * Adds a subset of this polygon's vertices into the given list.
    *
    * @param startIndexInclusive the index of the first vertex to add.
    * @param endIndexInclusive the index of the last vertex to add.
    * @param pointListToPack the list into which the vertices are to be added.
    * @see #getPointsInClockwiseOrder(int, int, List)
    */
   default void getFramePointsInClockwiseOrder(int startIndexInclusive, int endIndexInclusive, List<FramePoint2DReadOnly> pointListToPack)
   {
      checkIfUpToDate();
      checkIndexInBoundaries(startIndexInclusive);
      checkIndexInBoundaries(endIndexInclusive);
      int index = startIndexInclusive;

      while (true)
      {
         pointListToPack.add(getVertex(index));

         if (index == endIndexInclusive)
            break;
         index = getNextVertexIndex(index);
      }
   }

   /**
    * Adds a subset of this polygon's vertices into the given polygon.
    *
    * @param startIndexInclusive the index of the first vertex to add.
    * @param endIndexInclusive the index of the last vertex to add.
    * @param polygonToPack the polygon into which the vertices are to be added.
    * @throws ReferenceFrameMismatchException if {@code polygonToPack} and {@code this} are not
    *            expressed in the same reference frame.
    * @see #getVerticesInClockwiseOrder(int, int, ConvexPolygon2DBasics)
    */
   default void getVerticesInClockwiseOrder(int startIndexInclusive, int endIndexInclusive, FixedFrameConvexPolygon2DBasics polygonToPack)
   {
      checkReferenceFrameMatch(polygonToPack);
      ConvexPolygon2DReadOnly.super.getVerticesInClockwiseOrder(startIndexInclusive, endIndexInclusive, polygonToPack);
   }

   /**
    * Tests if the given point is inside this polygon or exactly on and edge/vertex of this polygon.
    *
    * @param point the query. Not modified.
    * @return {@code true} if the query is inside this polygon, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code point} and {@code this} are not expressed in
    *            the same reference frame.
    * @see #isPointInside(Point2DReadOnly)
    */
   default boolean isPointInside(FramePoint2DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return ConvexPolygon2DReadOnly.super.isPointInside(point);
   }

   /**
    * Determines if the point is inside this convex polygon given the tolerance {@code epsilon}.
    *
    * @param point the query. Not modified.
    * @param epsilon the tolerance to use during the test.
    * @return {@code true} if the query is considered to be inside the polygon, {@code false}
    *         otherwise.
    * @throws ReferenceFrameMismatchException if {@code point} and {@code this} are not expressed in
    *            the same reference frame.
    * @see #isPointInside(Point2DReadOnly, double)
    */
   default boolean isPointInside(FramePoint2DReadOnly point, double epsilon)
   {
      checkReferenceFrameMatch(point);
      return ConvexPolygon2DReadOnly.super.isPointInside(point, epsilon);
   }

   /**
    * Computes the coordinates of the closest point to the ray that belongs to this convex polygon.
    *
    * @param ray the ray to find the closest point to. Not modified.
    * @param closestPointToPack the point in which the coordinates of the closest point are stored.
    *           Modified.
    * @return {@code true} if the method succeeds, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code closestPointToPack} and {@code this} are not
    *            expressed in the same reference frame.
    * @see #getClosestPointWithRay(Line2DReadOnly, Point2DBasics)
    */
   default boolean getClosestPointWithRay(Line2DReadOnly ray, FixedFramePoint2DBasics closestPointToPack)
   {
      checkReferenceFrameMatch(closestPointToPack);
      return ConvexPolygon2DReadOnly.super.getClosestPointWithRay(ray, closestPointToPack);
   }

   /**
    * Computes the coordinates of the closest point to the ray that belongs to this convex polygon.
    *
    * @param ray the ray to find the closest point to. Not modified.
    * @param closestPointToPack the point in which the coordinates of the closest point are stored.
    *           Modified.
    * @return {@code true} if the method succeeds, {@code false} otherwise.
    * @see #getClosestPointWithRay(Line2DReadOnly, Point2DBasics)
    */
   default boolean getClosestPointWithRay(Line2DReadOnly ray, FramePoint2DBasics closestPointToPack)
   {
      closestPointToPack.setReferenceFrame(getReferenceFrame());
      return ConvexPolygon2DReadOnly.super.getClosestPointWithRay(ray, closestPointToPack);
   }

   /**
    * Computes the coordinates of the closest point to the ray that belongs to this convex polygon.
    *
    * @param ray the ray to find the closest point to. Not modified.
    * @param closestPointToPack the point in which the coordinates of the closest point are stored.
    *           Modified.
    * @return {@code true} if the method succeeds, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code ray} and {@code this} are not expressed in
    *            the same reference frame.
    * @see #getClosestPointWithRay(Line2DReadOnly, Point2DBasics)
    */
   default boolean getClosestPointWithRay(FrameLine2DReadOnly ray, Point2DBasics closestPointToPack)
   {
      checkReferenceFrameMatch(ray);
      return ConvexPolygon2DReadOnly.super.getClosestPointWithRay(ray, closestPointToPack);
   }

   /**
    * Computes the coordinates of the closest point to the ray that belongs to this convex polygon.
    *
    * @param ray the ray to find the closest point to. Not modified.
    * @param closestPointToPack the point in which the coordinates of the closest point are stored.
    *           Modified.
    * @return {@code true} if the method succeeds, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code ray}, {@code closestPointToPack}, and
    *            {@code this} are not expressed in the same reference frame.
    * @see #getClosestPointWithRay(Line2DReadOnly, Point2DBasics)
    */
   default boolean getClosestPointWithRay(FrameLine2DReadOnly ray, FixedFramePoint2DBasics closestPointToPack)
   {
      checkReferenceFrameMatch(ray);
      checkReferenceFrameMatch(closestPointToPack);
      return ConvexPolygon2DReadOnly.super.getClosestPointWithRay(ray, closestPointToPack);
   }

   /**
    * Computes the coordinates of the closest point to the ray that belongs to this convex polygon.
    *
    * @param ray the ray to find the closest point to. Not modified.
    * @param closestPointToPack the point in which the coordinates of the closest point are stored.
    *           Modified.
    * @return {@code true} if the method succeeds, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code ray} and {@code this} are not expressed in
    *            the same reference frame.
    * @see #getClosestPointWithRay(Line2DReadOnly, Point2DBasics)
    */
   default boolean getClosestPointWithRay(FrameLine2DReadOnly ray, FramePoint2DBasics closestPointToPack)
   {
      checkReferenceFrameMatch(ray);
      closestPointToPack.setReferenceFrame(getReferenceFrame());
      return ConvexPolygon2DReadOnly.super.getClosestPointWithRay(ray, closestPointToPack);
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint2DBasics getClosestPointWithRay(Line2DReadOnly ray)
   {
      Point2DBasics closestPointWithRay = ConvexPolygon2DReadOnly.super.getClosestPointWithRay(ray);
      if (closestPointWithRay == null)
         return null;
      else
         return new FramePoint2D(getReferenceFrame(), closestPointWithRay);
   }

   /**
    * Computes the coordinates of the closest point to the ray that belongs to this convex polygon.
    *
    * @param ray the ray to find the closest point to. Not modified.
    * @return the coordinates of the closest point if the method succeeds, {@code null} otherwise.
    * @throws ReferenceFrameMismatchException if {@code ray} and {@code this} are not expressed in
    *            the same reference frame.
    * @see #getClosestPointWithRay(Line2DReadOnly)
    */
   default FramePoint2DBasics getClosestPointWithRay(FrameLine2DReadOnly ray)
   {
      checkReferenceFrameMatch(ray);
      return getClosestPointWithRay((Line2DReadOnly) ray);
   }

   /**
    * Calculates the minimum distance between the point and this polygon.
    *
    * @param point the coordinates of the query. Not modified.
    * @return the value of the distance between the point and this polygon.
    * @throws ReferenceFrameMismatchException if {@code point} and {@code this} are not expressed in
    *            the same reference frame.
    * @see #distance(Point2DReadOnly)
    */
   default double distance(FramePoint2DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return ConvexPolygon2DReadOnly.super.distance(point);
   }

   /**
    * Returns minimum distance between the point and this polygon.
    *
    * @param point the coordinates of the query. Not modified.
    * @return the distance between the query and the polygon, it is negative if the point is inside
    *         the polygon.
    * @throws ReferenceFrameMismatchException if {@code point} and {@code this} are not expressed in
    *            the same reference frame.
    * @see #signedDistance(Point2DReadOnly)
    */
   default double signedDistance(FramePoint2DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return ConvexPolygon2DReadOnly.super.signedDistance(point);
   }

   /**
    * Computes the orthogonal projection of a 2D point on this 2D convex polygon.
    *
    * @param pointToProject the point to project on this polygon. Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code pointToProject} and {@code this} are not
    *            expressed in the same reference frame.
    * @see #orthogonalProjection(Point2DBasics)
    */
   default boolean orthogonalProjection(FixedFramePoint2DBasics pointToProject)
   {
      checkReferenceFrameMatch(pointToProject);
      return ConvexPolygon2DReadOnly.super.orthogonalProjection(pointToProject);
   }

   /**
    * Computes the orthogonal projection of a 2D point on this 2D convex polygon.
    *
    * @param pointToProject the point to project on this polygon. Modified.
    * @return whether the method succeeded or not.
    * @see #orthogonalProjection(Point2DBasics)
    */
   default boolean orthogonalProjection(FramePoint2DBasics pointToProject)
   {
      pointToProject.setReferenceFrame(getReferenceFrame());
      return ConvexPolygon2DReadOnly.super.orthogonalProjection(pointToProject);
   }

   /**
    * Computes the orthogonal projection of a 2D point this 2D convex polygon.
    *
    * @param pointToProject the coordinate of the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto the convex polygon is
    *           stored. Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code projectionToPack} and {@code this} are not
    *            expressed in the same reference frame.
    * @see #orthogonalProjection(Point2DReadOnly, Point2DBasics)
    */
   default boolean orthogonalProjection(Point2DReadOnly pointToProject, FixedFramePoint2DBasics projectionToPack)
   {
      checkReferenceFrameMatch(projectionToPack);
      return ConvexPolygon2DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 2D point this 2D convex polygon.
    *
    * @param pointToProject the coordinate of the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto the convex polygon is
    *           stored. Modified.
    * @return whether the method succeeded or not.
    * @see #orthogonalProjection(Point2DReadOnly, Point2DBasics)
    */
   default boolean orthogonalProjection(Point2DReadOnly pointToProject, FramePoint2DBasics projectionToPack)
   {
      projectionToPack.setReferenceFrame(getReferenceFrame());
      return ConvexPolygon2DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 2D point this 2D convex polygon.
    *
    * @param pointToProject the coordinate of the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto the convex polygon is
    *           stored. Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code pointToProject} and {@code this} are not
    *            expressed in the same reference frame.
    * @see #orthogonalProjection(Point2DReadOnly, Point2DBasics)
    */
   default boolean orthogonalProjection(FramePoint2DReadOnly pointToProject, Point2DBasics projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      return ConvexPolygon2DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 2D point this 2D convex polygon.
    *
    * @param pointToProject the coordinate of the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto the convex polygon is
    *           stored. Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code pointToProject}, {@code projectionToPack},
    *            and {@code this} are not expressed in the same reference frame.
    * @see #orthogonalProjection(Point2DReadOnly, Point2DBasics)
    */
   default boolean orthogonalProjection(FramePoint2DReadOnly pointToProject, FixedFramePoint2DBasics projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      checkReferenceFrameMatch(projectionToPack);
      return ConvexPolygon2DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 2D point this 2D convex polygon.
    *
    * @param pointToProject the coordinate of the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto the convex polygon is
    *           stored. Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code pointToProject} and {@code this} are not
    *            expressed in the same reference frame.
    * @see #orthogonalProjection(Point2DReadOnly, Point2DBasics)
    */
   default boolean orthogonalProjection(FramePoint2DReadOnly pointToProject, FramePoint2DBasics projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      projectionToPack.setReferenceFrame(getReferenceFrame());
      return ConvexPolygon2DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint2DBasics orthogonalProjectionCopy(Point2DReadOnly pointToProject)
   {
      Point2DBasics orthogonalProjection = ConvexPolygon2DReadOnly.super.orthogonalProjectionCopy(pointToProject);
      if (orthogonalProjection == null)
         return null;
      else
         return new FramePoint2D(getReferenceFrame(), orthogonalProjection);
   }

   /**
    * Computes the orthogonal projection of a 2D point on this 2D convex polygon.
    *
    * @param pointToProject the coordinate of the point to compute the projection of. Not modified.
    * @return the coordinates of the projection, or {@code null} if the method failed.
    * @throws ReferenceFrameMismatchException if {@code pointToProject} and {@code this} are not
    *            expressed in the same reference frame.
    * @see #orthogonalProjectionCopy(Point2DReadOnly)
    */
   default FramePoint2DBasics orthogonalProjectionCopy(FramePoint2DReadOnly pointToProject)
   {
      checkReferenceFrameMatch(pointToProject);
      return orthogonalProjectionCopy((Point2DReadOnly) pointToProject);
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the index of the
    * first vertex that is in the line-of-sight.
    *
    * @param observer the coordinates of the observer. Not modified.
    * @return the index of the first vertex that is in the line-of-sight, {@code -1} if this method
    *         fails.
    * @throws ReferenceFrameMismatchException if {@code observer} and {@code this} are not expressed
    *            in the same reference frame.
    * @see #lineOfSightStartIndex(Point2DReadOnly)
    */
   default int lineOfSightStartIndex(FramePoint2DReadOnly observer)
   {
      checkReferenceFrameMatch(observer);
      return ConvexPolygon2DReadOnly.super.lineOfSightStartIndex(observer);
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the index of the
    * last vertex that is in the line-of-sight.
    *
    * @param observer the coordinates of the observer. Not modified.
    * @return the index of the last vertex that is in the line-of-sight, {@code -1} if this method
    *         fails.
    * @throws ReferenceFrameMismatchException if {@code observer} and {@code this} are not expressed
    *            in the same reference frame.
    * @see #lineOfSightEndIndex(Point2DReadOnly)
    */
   default int lineOfSightEndIndex(FramePoint2DReadOnly observer)
   {
      checkReferenceFrameMatch(observer);
      return ConvexPolygon2DReadOnly.super.lineOfSightEndIndex(observer);
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the indices of the
    * first and last vertices that are in the line-of-sight.
    *
    * @param observer the coordinates of the observer. Not modified.
    * @return the indices in order of the first and last vertices that are in the line-of-sight,
    *         {@code null} if this method fails.
    * @throws ReferenceFrameMismatchException if {@code observer} and {@code this} are not expressed
    *            in the same reference frame.
    * @see #lineOfSightIndices(Point2DReadOnly)
    */
   default int[] lineOfSightIndices(FramePoint2DReadOnly observer)
   {
      checkReferenceFrameMatch(observer);
      return ConvexPolygon2DReadOnly.super.lineOfSightIndices(observer);
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the first vertex
    * that is in the line-of-sight.
    *
    * @param observer the coordinates of the observer. Not modified.
    * @param startVertexToPack point in which the coordinates of the first vertex in the
    *           line-of-sight are stored. Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code startVertexToPack} and {@code this} are not
    *            expressed in the same reference frame.
    * @see #lineOfSightStartVertex(Point2DReadOnly, Point2DBasics)
    */
   default boolean lineOfSightStartVertex(Point2DReadOnly observer, FixedFramePoint2DBasics startVertexToPack)
   {
      checkReferenceFrameMatch(startVertexToPack);
      return ConvexPolygon2DReadOnly.super.lineOfSightStartVertex(observer, startVertexToPack);
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the first vertex
    * that is in the line-of-sight.
    *
    * @param observer the coordinates of the observer. Not modified.
    * @param startVertexToPack point in which the coordinates of the first vertex in the
    *           line-of-sight are stored. Modified.
    * @return whether the method succeeded or not.
    * @see #lineOfSightStartVertex(Point2DReadOnly, Point2DBasics)
    */
   default boolean lineOfSightStartVertex(Point2DReadOnly observer, FramePoint2DBasics startVertexToPack)
   {
      startVertexToPack.setReferenceFrame(getReferenceFrame());
      return ConvexPolygon2DReadOnly.super.lineOfSightStartVertex(observer, startVertexToPack);
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the first vertex
    * that is in the line-of-sight.
    *
    * @param observer the coordinates of the observer. Not modified.
    * @param startVertexToPack point in which the coordinates of the first vertex in the
    *           line-of-sight are stored. Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code observer} and {@code this} are not expressed
    *            in the same reference frame.
    * @see #lineOfSightStartVertex(Point2DReadOnly, Point2DBasics)
    */
   default boolean lineOfSightStartVertex(FramePoint2DReadOnly observer, Point2DBasics startVertexToPack)
   {
      checkReferenceFrameMatch(observer);
      return ConvexPolygon2DReadOnly.super.lineOfSightStartVertex(observer, startVertexToPack);
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the first vertex
    * that is in the line-of-sight.
    *
    * @param observer the coordinates of the observer. Not modified.
    * @param startVertexToPack point in which the coordinates of the first vertex in the
    *           line-of-sight are stored. Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code observer}, {@code startVertexToPack}, and
    *            {@code this} are not expressed in the same reference frame.
    * @see #lineOfSightStartVertex(Point2DReadOnly, Point2DBasics)
    */
   default boolean lineOfSightStartVertex(FramePoint2DReadOnly observer, FixedFramePoint2DBasics startVertexToPack)
   {
      checkReferenceFrameMatch(observer);
      checkReferenceFrameMatch(startVertexToPack);
      return ConvexPolygon2DReadOnly.super.lineOfSightStartVertex(observer, startVertexToPack);
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the first vertex
    * that is in the line-of-sight.
    *
    * @param observer the coordinates of the observer. Not modified.
    * @param startVertexToPack point in which the coordinates of the first vertex in the
    *           line-of-sight are stored. Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code observer} and {@code this} are not expressed
    *            in the same reference frame.
    * @see #lineOfSightStartVertex(Point2DReadOnly, Point2DBasics)
    */
   default boolean lineOfSightStartVertex(FramePoint2DReadOnly observer, FramePoint2DBasics startVertexToPack)
   {
      checkReferenceFrameMatch(observer);
      startVertexToPack.setReferenceFrame(getReferenceFrame());
      return ConvexPolygon2DReadOnly.super.lineOfSightStartVertex(observer, startVertexToPack);
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the last vertex
    * that is in the line-of-sight.
    *
    * @param observer the coordinates of the observer. Not modified.
    * @param endVertexToPack point in which the coordinates of the last vertex in the line-of-sight
    *           are stored. Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code endVertexToPack} and {@code this} are not
    *            expressed in the same reference frame.
    * @see #lineOfSightEndVertex(Point2DReadOnly, Point2DBasics)
    */
   default boolean lineOfSightEndVertex(Point2DReadOnly observer, FixedFramePoint2DBasics endVertexToPack)
   {
      checkReferenceFrameMatch(endVertexToPack);
      return ConvexPolygon2DReadOnly.super.lineOfSightEndVertex(observer, endVertexToPack);
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the last vertex
    * that is in the line-of-sight.
    *
    * @param observer the coordinates of the observer. Not modified.
    * @param endVertexToPack point in which the coordinates of the last vertex in the line-of-sight
    *           are stored. Modified.
    * @return whether the method succeeded or not.
    * @see #lineOfSightEndVertex(Point2DReadOnly, Point2DBasics)
    */
   default boolean lineOfSightEndVertex(Point2DReadOnly observer, FramePoint2DBasics endVertexToPack)
   {
      endVertexToPack.setReferenceFrame(getReferenceFrame());
      return ConvexPolygon2DReadOnly.super.lineOfSightEndVertex(observer, endVertexToPack);
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the last vertex
    * that is in the line-of-sight.
    *
    * @param observer the coordinates of the observer. Not modified.
    * @param endVertexToPack point in which the coordinates of the last vertex in the line-of-sight
    *           are stored. Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code observer} and {@code this} are not expressed
    *            in the same reference frame.
    * @see #lineOfSightEndVertex(Point2DReadOnly, Point2DBasics)
    */
   default boolean lineOfSightEndVertex(FramePoint2DReadOnly observer, Point2DBasics endVertexToPack)
   {
      checkReferenceFrameMatch(observer);
      return ConvexPolygon2DReadOnly.super.lineOfSightEndVertex(observer, endVertexToPack);
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the last vertex
    * that is in the line-of-sight.
    *
    * @param observer the coordinates of the observer. Not modified.
    * @param endVertexToPack point in which the coordinates of the last vertex in the line-of-sight
    *           are stored. Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code observer}, {@code endVertexToPack}, and
    *            {@code this} are not expressed in the same reference frame.
    * @see #lineOfSightEndVertex(Point2DReadOnly, Point2DBasics)
    */
   default boolean lineOfSightEndVertex(FramePoint2DReadOnly observer, FixedFramePoint2DBasics endVertexToPack)
   {
      checkReferenceFrameMatch(observer);
      checkReferenceFrameMatch(endVertexToPack);
      return ConvexPolygon2DReadOnly.super.lineOfSightEndVertex(observer, endVertexToPack);
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the last vertex
    * that is in the line-of-sight.
    *
    * @param observer the coordinates of the observer. Not modified.
    * @param endVertexToPack point in which the coordinates of the last vertex in the line-of-sight
    *           are stored. Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code observer} and {@code this} are not expressed
    *            in the same reference frame.
    * @see #lineOfSightEndVertex(Point2DReadOnly, Point2DBasics)
    */
   default boolean lineOfSightEndVertex(FramePoint2DReadOnly observer, FramePoint2DBasics endVertexToPack)
   {
      checkReferenceFrameMatch(observer);
      endVertexToPack.setReferenceFrame(getReferenceFrame());
      return ConvexPolygon2DReadOnly.super.lineOfSightEndVertex(observer, endVertexToPack);
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint2DBasics lineOfSightStartVertexCopy(Point2DReadOnly observer)
   {
      Point2DBasics lineOfSightStartVertex = ConvexPolygon2DReadOnly.super.lineOfSightStartVertexCopy(observer);
      if (lineOfSightStartVertex == null)
         return null;
      else
         return new FramePoint2D(getReferenceFrame(), lineOfSightStartVertex);
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the first vertex
    * that is in the line-of-sight.
    *
    * @param observer the coordinates of the observer. Not modified.
    * @return the coordinates of the first vertex in the line-of-sight or {@code null} if this
    *         method failed. Modified.
    * @throws ReferenceFrameMismatchException if {@code observer} and {@code this} are not expressed
    *            in the same reference frame.
    * @see #lineOfSightStartVertexCopy(Point2DReadOnly)
    */
   default FramePoint2DBasics lineOfSightStartVertexCopy(FramePoint2DReadOnly observer)
   {
      checkReferenceFrameMatch(observer);
      return lineOfSightStartVertexCopy((Point2DReadOnly) observer);
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint2DBasics lineOfSightEndVertexCopy(Point2DReadOnly observer)
   {
      Point2DBasics lineOfSightEndVertex = ConvexPolygon2DReadOnly.super.lineOfSightEndVertexCopy(observer);
      if (lineOfSightEndVertex == null)
         return null;
      else
         return new FramePoint2D(getReferenceFrame(), lineOfSightEndVertex);
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the last vertex
    * that is in the line-of-sight.
    *
    * @param observer the coordinates of the observer. Not modified.
    * @return the coordinates of the last vertex in the line-of-sight or {@code null} if this method
    *         failed. Modified.
    * @throws ReferenceFrameMismatchException if {@code observer} and {@code this} are not expressed
    *            in the same reference frame.
    * @see #lineOfSightEndVertexCopy(Point2DReadOnly)
    */
   default FramePoint2DBasics lineOfSightEndVertexCopy(FramePoint2DReadOnly observer)
   {
      checkReferenceFrameMatch(observer);
      return lineOfSightEndVertexCopy((Point2DReadOnly) observer);
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint2DBasics[] lineOfSightVertices(Point2DReadOnly observer)
   {
      Point2DBasics[] lineOfSightVertices = ConvexPolygon2DReadOnly.super.lineOfSightVertices(observer);
      if (lineOfSightVertices == null)
         return null;
      else
         return new FramePoint2DBasics[] {new FramePoint2D(getReferenceFrame(), lineOfSightVertices[0]),
               new FramePoint2D(getReferenceFrame(), lineOfSightVertices[1])};
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the first and last
    * vertices that is in the line-of-sight.
    *
    * @param observer the coordinates of the observer. Not modified.
    * @return the coordinates in order of the first and last vertices in the line-of-sight or
    *         {@code null} if this method failed. Modified.
    * @throws ReferenceFrameMismatchException if {@code observer} and {@code this} are not expressed
    *            in the same reference frame.
    * @see #lineOfSightVertices(Point2DReadOnly)
    */
   default FramePoint2DBasics[] lineOfSightVertices(FramePoint2DReadOnly observer)
   {
      checkReferenceFrameMatch(observer);
      return lineOfSightVertices((Point2DReadOnly) observer);
   }

   /**
    * Determines whether an observer can see the outside of the given edge of this convex polygon.
    *
    * @param edgeIndex the vertex index of the start of the edge.
    * @param observer the coordinates of the observer. Not modified.
    * @return {@code true} if the observer can see the outside of the edge, {@code false} if the
    *         observer cannot see the outside or is lying on the edge.
    * @throws ReferenceFrameMismatchException if {@code observer} and {@code this} are not expressed
    *            in the same reference frame.
    * @see #canObserverSeeEdge(int, Point2DReadOnly)
    */
   default boolean canObserverSeeEdge(int edgeIndex, FramePoint2DReadOnly observer)
   {
      checkReferenceFrameMatch(observer);
      return ConvexPolygon2DReadOnly.super.canObserverSeeEdge(edgeIndex, observer);
   }

   /**
    * Tests if the given point lies on an edge of this convex polygon.
    *
    * @param point the coordinates of the query. Not modified.
    * @return {@code true} if the point is considered to be on an edge of this polygon,
    *         {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code point} and {@code this} are not expressed in
    *            the same reference frame.
    * @see #pointIsOnPerimeter(Point2DReadOnly)
    */
   default boolean pointIsOnPerimeter(FramePoint2DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return ConvexPolygon2DReadOnly.super.pointIsOnPerimeter(point);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line 2D and this
    * convex polygon 2D.
    *
    * @param line the line that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the line and the polygon.
    * @throws ReferenceFrameMismatchException if {@code code secondIntersectionToPack} and
    *            {@code this} are not expressed in the same reference frame.
    * @see #intersectionWith(Line2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(Line2DReadOnly line, Point2DBasics firstIntersectionToPack, FixedFramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(secondIntersectionToPack);
      return ConvexPolygon2DReadOnly.super.intersectionWith(line, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line 2D and this
    * convex polygon 2D.
    *
    * @param line the line that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the line and the polygon.
    * @throws ReferenceFrameMismatchException {@code firstIntersectionToPack} and {@code this} are
    *            not expressed in the same reference frame.
    * @see #intersectionWith(Line2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(Line2DReadOnly line, FixedFramePoint2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(firstIntersectionToPack);
      return ConvexPolygon2DReadOnly.super.intersectionWith(line, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line 2D and this
    * convex polygon 2D.
    *
    * @param line the line that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the line and the polygon.
    * @throws ReferenceFrameMismatchException if {@code firstIntersectionToPack},
    *            {@code code secondIntersectionToPack}, and {@code this} are not expressed in the
    *            same reference frame.
    * @see #intersectionWith(Line2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(Line2DReadOnly line, FixedFramePoint2DBasics firstIntersectionToPack, FixedFramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(firstIntersectionToPack);
      checkReferenceFrameMatch(secondIntersectionToPack);
      return ConvexPolygon2DReadOnly.super.intersectionWith(line, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line 2D and this
    * convex polygon 2D.
    *
    * @param line the line that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the line and the polygon.
    * @see #intersectionWith(Line2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(Line2DReadOnly line, FramePoint2DBasics firstIntersectionToPack, FramePoint2DBasics secondIntersectionToPack)
   {
      firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return ConvexPolygon2DReadOnly.super.intersectionWith(line, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line 2D and this
    * convex polygon 2D.
    *
    * @param line the line that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the line and the polygon.
    * @throws ReferenceFrameMismatchException if {@code line} and {@code this} are not expressed in
    *            the same reference frame.
    * @see #intersectionWith(Line2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(FrameLine2DReadOnly line, Point2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(line);
      return ConvexPolygon2DReadOnly.super.intersectionWith(line, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line 2D and this
    * convex polygon 2D.
    *
    * @param line the line that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the line and the polygon.
    * @throws ReferenceFrameMismatchException if {@code line},
    *            {@code code secondIntersectionToPack}, and {@code this} are not expressed in the
    *            same reference frame.
    * @see #intersectionWith(Line2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(FrameLine2DReadOnly line, Point2DBasics firstIntersectionToPack, FixedFramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(line);
      checkReferenceFrameMatch(secondIntersectionToPack);
      return ConvexPolygon2DReadOnly.super.intersectionWith(line, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line 2D and this
    * convex polygon 2D.
    *
    * @param line the line that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the line and the polygon.
    * @throws ReferenceFrameMismatchException if {@code line}, {@code firstIntersectionToPack}, and
    *            {@code this} are not expressed in the same reference frame.
    * @see #intersectionWith(Line2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(FrameLine2DReadOnly line, FixedFramePoint2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(line);
      checkReferenceFrameMatch(firstIntersectionToPack);
      return ConvexPolygon2DReadOnly.super.intersectionWith(line, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line 2D and this
    * convex polygon 2D.
    *
    * @param line the line that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the line and the polygon.
    * @throws ReferenceFrameMismatchException if {@code line}, {@code firstIntersectionToPack},
    *            {@code code secondIntersectionToPack}, and {@code this} are not expressed in the
    *            same reference frame.
    * @see #intersectionWith(Line2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(FrameLine2DReadOnly line, FixedFramePoint2DBasics firstIntersectionToPack, FixedFramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(line);
      checkReferenceFrameMatch(firstIntersectionToPack);
      checkReferenceFrameMatch(secondIntersectionToPack);
      return ConvexPolygon2DReadOnly.super.intersectionWith(line, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line 2D and this
    * convex polygon 2D.
    *
    * @param line the line that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the line and the polygon.
    * @throws ReferenceFrameMismatchException if {@code line} and {@code this} are not expressed in
    *            the same reference frame.
    * @see #intersectionWith(Line2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(FrameLine2DReadOnly line, FramePoint2DBasics firstIntersectionToPack, FramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(line);
      firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return ConvexPolygon2DReadOnly.super.intersectionWith(line, firstIntersectionToPack, secondIntersectionToPack);
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint2DBasics[] intersectionWith(Line2DReadOnly line)
   {
      Point2DBasics[] intersections = ConvexPolygon2DReadOnly.super.intersectionWith(line);
      if (intersections == null)
      {
         return null;
      }
      else
      {
         FramePoint2DBasics[] result = new FramePoint2DBasics[intersections.length];
         for (int i = 0; i < intersections.length; i++)
            result[i] = new FramePoint2D(getReferenceFrame(), intersections[i]);
         return result;
      }
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line 2D and this
    * convex polygon 2D.
    *
    * @param line the line that may intersect this polygon. Not modified.
    * @return the coordinates of the intersections between the line and the polygon, or {@code null}
    *         if they do not intersect.
    * @throws ReferenceFrameMismatchException if {@code line} and {@code this} are not expressed in
    *            the same reference frame.
    * @see #intersectionWith(Line2DReadOnly)
    */
   default FramePoint2DBasics[] intersectionWith(FrameLine2DReadOnly line)
   {
      checkReferenceFrameMatch(line);
      return intersectionWith((Line2DReadOnly) line);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given ray 2D and this
    * convex polygon 2D.
    *
    * @param ray the ray that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the ray and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the ray and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the ray and the polygon.
    * @throws ReferenceFrameMismatchException if {@code secondIntersectionToPack}, and {@code this}
    *            are not expressed in the same reference frame.
    * @see #intersectionWithRay(Line2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWithRay(Line2DReadOnly ray, Point2DBasics firstIntersectionToPack, FixedFramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(secondIntersectionToPack);
      return ConvexPolygon2DReadOnly.super.intersectionWithRay(ray, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given ray 2D and this
    * convex polygon 2D.
    *
    * @param ray the ray that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the ray and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the ray and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the ray and the polygon.
    * @throws ReferenceFrameMismatchException if {@code firstIntersectionToPack}, and {@code this}
    *            are not expressed in the same reference frame.
    * @see #intersectionWithRay(Line2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWithRay(Line2DReadOnly ray, FixedFramePoint2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(firstIntersectionToPack);
      return ConvexPolygon2DReadOnly.super.intersectionWithRay(ray, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given ray 2D and this
    * convex polygon 2D.
    *
    * @param ray the ray that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the ray and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the ray and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the ray and the polygon.
    * @throws ReferenceFrameMismatchException if {@code firstIntersectionToPack},
    *            {@code secondIntersectionToPack}, and {@code this} are not expressed in the same
    *            reference frame.
    * @see #intersectionWithRay(Line2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWithRay(Line2DReadOnly ray, FixedFramePoint2DBasics firstIntersectionToPack, FixedFramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(firstIntersectionToPack);
      checkReferenceFrameMatch(secondIntersectionToPack);
      return ConvexPolygon2DReadOnly.super.intersectionWithRay(ray, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given ray 2D and this
    * convex polygon 2D.
    *
    * @param ray the ray that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the ray and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the ray and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the ray and the polygon.
    * @throws ReferenceFrameMismatchException if {@code firstIntersectionToPack},
    *            {@code secondIntersectionToPack}, and {@code this} are not expressed in the same
    *            reference frame.
    * @see #intersectionWithRay(Line2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWithRay(Line2DReadOnly ray, FramePoint2DBasics firstIntersectionToPack, FramePoint2DBasics secondIntersectionToPack)
   {
      firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return ConvexPolygon2DReadOnly.super.intersectionWithRay(ray, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given ray 2D and this
    * convex polygon 2D.
    *
    * @param ray the ray that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the ray and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the ray and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the ray and the polygon.
    * @throws ReferenceFrameMismatchException if {@code ray} and {@code this} are not expressed in
    *            the same reference frame.
    * @see #intersectionWithRay(Line2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWithRay(FrameLine2DReadOnly ray, Point2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(ray);
      return ConvexPolygon2DReadOnly.super.intersectionWithRay(ray, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given ray 2D and this
    * convex polygon 2D.
    *
    * @param ray the ray that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the ray and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the ray and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the ray and the polygon.
    * @throws ReferenceFrameMismatchException if {@code ray}, {@code secondIntersectionToPack}, and
    *            {@code this} are not expressed in the same reference frame.
    * @see #intersectionWithRay(Line2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWithRay(FrameLine2DReadOnly ray, Point2DBasics firstIntersectionToPack, FixedFramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(ray);
      checkReferenceFrameMatch(secondIntersectionToPack);
      return ConvexPolygon2DReadOnly.super.intersectionWithRay(ray, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given ray 2D and this
    * convex polygon 2D.
    *
    * @param ray the ray that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the ray and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the ray and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the ray and the polygon.
    * @throws ReferenceFrameMismatchException if {@code ray}, {@code firstIntersectionToPack}, and
    *            {@code this} are not expressed in the same reference frame.
    * @see #intersectionWithRay(Line2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWithRay(FrameLine2DReadOnly ray, FixedFramePoint2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(ray);
      checkReferenceFrameMatch(firstIntersectionToPack);
      return ConvexPolygon2DReadOnly.super.intersectionWithRay(ray, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given ray 2D and this
    * convex polygon 2D.
    *
    * @param ray the ray that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the ray and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the ray and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the ray and the polygon.
    * @throws ReferenceFrameMismatchException if {@code ray}, {@code firstIntersectionToPack},
    *            {@code secondIntersectionToPack}, and {@code this} are not expressed in the same
    *            reference frame.
    * @see #intersectionWithRay(Line2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWithRay(FrameLine2DReadOnly ray, FixedFramePoint2DBasics firstIntersectionToPack, FixedFramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(ray);
      checkReferenceFrameMatch(firstIntersectionToPack);
      checkReferenceFrameMatch(secondIntersectionToPack);
      return ConvexPolygon2DReadOnly.super.intersectionWithRay(ray, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given ray 2D and this
    * convex polygon 2D.
    *
    * @param ray the ray that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the ray and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the ray and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the ray and the polygon.
    * @throws ReferenceFrameMismatchException if {@code ray}, {@code firstIntersectionToPack},
    *            {@code secondIntersectionToPack}, and {@code this} are not expressed in the same
    *            reference frame.
    * @see #intersectionWithRay(Line2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWithRay(FrameLine2DReadOnly ray, FramePoint2DBasics firstIntersectionToPack, FramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(ray);
      firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return ConvexPolygon2DReadOnly.super.intersectionWithRay(ray, firstIntersectionToPack, secondIntersectionToPack);
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint2DBasics[] intersectionWithRay(Line2DReadOnly ray)
   {
      Point2DBasics[] intersections = ConvexPolygon2DReadOnly.super.intersectionWithRay(ray);
      if (intersections == null)
      {
         return null;
      }
      else
      {
         FramePoint2DBasics[] result = new FramePoint2DBasics[intersections.length];
         for (int i = 0; i < intersections.length; i++)
            result[i] = new FramePoint2D(getReferenceFrame(), intersections[i]);
         return result;
      }
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given ray 2D and this
    * convex polygon 2D.
    *
    * @param ray the ray that may intersect this polygon. Not modified.
    * @return the intersections between the ray and the polygon.
    * @throws ReferenceFrameMismatchException if {@code ray} and {@code this} are not expressed in
    *            the same reference frame.
    * @see #intersectionWithRay(Line2DReadOnly)
    */
   default FramePoint2DBasics[] intersectionWithRay(FrameLine2DReadOnly ray)
   {
      checkReferenceFrameMatch(ray);
      return intersectionWithRay((Line2DReadOnly) ray);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line segment 2D and
    * this convex polygon 2D.
    *
    * @param lineSegment2D the line segment that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the line and the polygon.
    * @throws ReferenceFrameMismatchException if {@code secondIntersectionToPack} and {@code this}
    *            are not expressed in the same reference frame.
    * @see #intersectionWith(LineSegment2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(LineSegment2DReadOnly lineSegment2D, Point2DBasics firstIntersectionToPack, FixedFramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(secondIntersectionToPack);
      return ConvexPolygon2DReadOnly.super.intersectionWith(lineSegment2D, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line segment 2D and
    * this convex polygon 2D.
    *
    * @param lineSegment2D the line segment that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the line and the polygon.
    * @throws ReferenceFrameMismatchException if {@code firstIntersectionToPack} and {@code this}
    *            are not expressed in the same reference frame.
    * @see #intersectionWith(LineSegment2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(LineSegment2DReadOnly lineSegment2D, FixedFramePoint2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(firstIntersectionToPack);
      return ConvexPolygon2DReadOnly.super.intersectionWith(lineSegment2D, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line segment 2D and
    * this convex polygon 2D.
    *
    * @param lineSegment2D the line segment that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the line and the polygon.
    * @throws ReferenceFrameMismatchException if {@code firstIntersectionToPack},
    *            {@code secondIntersectionToPack}, and {@code this} are not expressed in the same
    *            reference frame.
    * @see #intersectionWith(LineSegment2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(LineSegment2DReadOnly lineSegment2D, FixedFramePoint2DBasics firstIntersectionToPack,
                                FixedFramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(firstIntersectionToPack);
      checkReferenceFrameMatch(secondIntersectionToPack);
      return ConvexPolygon2DReadOnly.super.intersectionWith(lineSegment2D, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line segment 2D and
    * this convex polygon 2D.
    *
    * @param lineSegment2D the line segment that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the line and the polygon.
    * @see #intersectionWith(LineSegment2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(LineSegment2DReadOnly lineSegment2D, FramePoint2DBasics firstIntersectionToPack, FramePoint2DBasics secondIntersectionToPack)
   {
      firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return ConvexPolygon2DReadOnly.super.intersectionWith(lineSegment2D, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line segment 2D and
    * this convex polygon 2D.
    *
    * @param lineSegment2D the line segment that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the line and the polygon.
    * @throws ReferenceFrameMismatchException if {@code lineSegment2D} and {@code this} are not
    *            expressed in the same reference frame.
    * @see #intersectionWith(LineSegment2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(FrameLineSegment2DReadOnly lineSegment2D, Point2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(lineSegment2D);
      return ConvexPolygon2DReadOnly.super.intersectionWith(lineSegment2D, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line segment 2D and
    * this convex polygon 2D.
    *
    * @param lineSegment2D the line segment that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the line and the polygon.
    * @throws ReferenceFrameMismatchException if {@code lineSegment2D},
    *            {@code secondIntersectionToPack}, and {@code this} are not expressed in the same
    *            reference frame.
    * @see #intersectionWith(LineSegment2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(FrameLineSegment2DReadOnly lineSegment2D, Point2DBasics firstIntersectionToPack,
                                FixedFramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(lineSegment2D);
      checkReferenceFrameMatch(secondIntersectionToPack);
      return ConvexPolygon2DReadOnly.super.intersectionWith(lineSegment2D, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line segment 2D and
    * this convex polygon 2D.
    *
    * @param lineSegment2D the line segment that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the line and the polygon.
    * @throws ReferenceFrameMismatchException if {@code lineSegment2D},
    *            {@code firstIntersectionToPack}, and {@code this} are not expressed in the same
    *            reference frame.
    * @see #intersectionWith(LineSegment2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(FrameLineSegment2DReadOnly lineSegment2D, FixedFramePoint2DBasics firstIntersectionToPack,
                                Point2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(lineSegment2D);
      checkReferenceFrameMatch(firstIntersectionToPack);
      return ConvexPolygon2DReadOnly.super.intersectionWith(lineSegment2D, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line segment 2D and
    * this convex polygon 2D.
    *
    * @param lineSegment2D the line segment that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the line and the polygon.
    * @throws ReferenceFrameMismatchException if {@code lineSegment2D},
    *            {@code firstIntersectionToPack}, {@code secondIntersectionToPack}, and {@code this}
    *            are not expressed in the same reference frame.
    * @see #intersectionWith(LineSegment2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(FrameLineSegment2DReadOnly lineSegment2D, FixedFramePoint2DBasics firstIntersectionToPack,
                                FixedFramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(lineSegment2D);
      checkReferenceFrameMatch(firstIntersectionToPack);
      checkReferenceFrameMatch(secondIntersectionToPack);
      return ConvexPolygon2DReadOnly.super.intersectionWith(lineSegment2D, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line segment 2D and
    * this convex polygon 2D.
    *
    * @param lineSegment2D the line segment that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the line and the polygon.
    * @throws ReferenceFrameMismatchException if {@code lineSegment2D} and {@code this} are not
    *            expressed in the same reference frame.
    * @see #intersectionWith(LineSegment2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(FrameLineSegment2DReadOnly lineSegment2D, FramePoint2DBasics firstIntersectionToPack,
                                FramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(lineSegment2D);
      firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return ConvexPolygon2DReadOnly.super.intersectionWith(lineSegment2D, firstIntersectionToPack, secondIntersectionToPack);
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint2DBasics[] intersectionWith(LineSegment2DReadOnly lineSegment2D)
   {
      Point2DBasics[] intersections = ConvexPolygon2DReadOnly.super.intersectionWith(lineSegment2D);
      if (intersections == null)
      {
         return null;
      }
      else
      {
         FramePoint2DBasics[] result = new FramePoint2DBasics[intersections.length];
         for (int i = 0; i < intersections.length; i++)
            result[i] = new FramePoint2D(getReferenceFrame(), intersections[i]);
         return result;
      }
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line segment 2D and
    * this convex polygon 2D.
    *
    * @param lineSegment2D the line segment that may intersect this polygon. Not modified.
    * @return the intersections between the line segment and the polygon.
    * @throws ReferenceFrameMismatchException if {@code lineSegment2D} and {@code this} are not
    *            expressed in the same reference frame.
    * @see #intersectionWith(LineSegment2DReadOnly)
    */
   default FramePoint2DBasics[] intersectionWith(FrameLineSegment2DReadOnly lineSegment2D)
   {
      checkReferenceFrameMatch(lineSegment2D);
      return intersectionWith((LineSegment2DReadOnly) lineSegment2D);
   }

   /**
    * Finds the index of the closest edge to the query.
    *
    * @param point the coordinates of the query. Not modified.
    * @return the index of the closest edge to the query.
    * @throws ReferenceFrameMismatchException if {@code point} and {@code this} are not expressed in
    *            the same reference frame.
    * @see #getClosestEdgeIndex(Point2DReadOnly)
    */
   default int getClosestEdgeIndex(FramePoint2DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return ConvexPolygon2DReadOnly.super.getClosestEdgeIndex(point);
   }

   /**
    * Finds the index of the closest edge to the query.
    *
    * @param point the coordinates of the query. Not modified.
    * @param closestEdgeToPack the line segment used to store the result. Not modified.
    * @return whether this method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code point} and {@code this} are not expressed in
    *            the same reference frame.
    * @see #getClosestEdge(Point2DReadOnly, LineSegment2DBasics)
    */
   default boolean getClosestEdge(FramePoint2DReadOnly point, LineSegment2DBasics closestEdgeToPack)
   {
      checkReferenceFrameMatch(point);
      return ConvexPolygon2DReadOnly.super.getClosestEdge(point, closestEdgeToPack);
   }

   /**
    * Finds the index of the closest edge to the query.
    *
    * @param point the coordinates of the query. Not modified.
    * @param closestEdgeToPack the line segment used to store the result. Not modified.
    * @return whether this method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code point}, {@code closestEdgetoPack}, and
    *            {@code this} are not expressed in the same reference frame.
    * @see #getClosestEdge(Point2DReadOnly, LineSegment2DBasics)
    */
   default boolean getClosestEdge(FramePoint2DReadOnly point, FixedFrameLineSegment2DBasics closestEdgeToPack)
   {
      checkReferenceFrameMatch(point);
      checkReferenceFrameMatch(closestEdgeToPack);
      return ConvexPolygon2DReadOnly.super.getClosestEdge(point, closestEdgeToPack);
   }

   /**
    * Finds the index of the closest edge to the query.
    *
    * @param point the coordinates of the query. Not modified.
    * @param closestEdgeToPack the line segment used to store the result. Not modified.
    * @return whether this method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code point} and {@code this} are not expressed in
    *            the same reference frame.
    * @see #getClosestEdge(Point2DReadOnly, LineSegment2DBasics)
    */
   default boolean getClosestEdge(FramePoint2DReadOnly point, FrameLineSegment2DBasics closestEdgeToPack)
   {
      checkReferenceFrameMatch(point);
      closestEdgeToPack.setReferenceFrame(getReferenceFrame());
      return ConvexPolygon2DReadOnly.super.getClosestEdge(point, closestEdgeToPack);
   }

   /**
    * Finds the index of the closest edge to the query.
    *
    * @param point the coordinates of the query. Not modified.
    * @param closestEdgeToPack the line segment used to store the result. Not modified.
    * @return whether this method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code closestEdgetoPack} and {@code this} are not
    *            expressed in the same reference frame.
    * @see #getClosestEdge(Point2DReadOnly, LineSegment2DBasics)
    */
   default boolean getClosestEdge(Point2DReadOnly point, FixedFrameLineSegment2DBasics closestEdgeToPack)
   {
      checkReferenceFrameMatch(closestEdgeToPack);
      return ConvexPolygon2DReadOnly.super.getClosestEdge(point, closestEdgeToPack);
   }

   /**
    * Finds the index of the closest edge to the query.
    *
    * @param point the coordinates of the query. Not modified.
    * @param closestEdgeToPack the line segment used to store the result. Not modified.
    * @return whether this method succeeded or not.
    * @see #getClosestEdge(Point2DReadOnly, LineSegment2DBasics)
    */
   default boolean getClosestEdge(Point2DReadOnly point, FrameLineSegment2DBasics closestEdgeToPack)
   {
      closestEdgeToPack.setReferenceFrame(getReferenceFrame());
      return ConvexPolygon2DReadOnly.super.getClosestEdge(point, closestEdgeToPack);
   }

   /** {@inheritDoc} */
   @Override
   default FrameLineSegment2DBasics getClosestEdgeCopy(Point2DReadOnly point)
   {
      LineSegment2DBasics closestEdge = ConvexPolygon2DReadOnly.super.getClosestEdgeCopy(point);
      if (closestEdge == null)
         return null;
      else
         return new FrameLineSegment2D(getReferenceFrame(), closestEdge);
   }

   /**
    * Finds the index of the closest edge to the query.
    *
    * @param point the coordinates of the query. Not modified.
    * @return the line segment representing the closest edge or {@code null} if this method failed.
    * @throws ReferenceFrameMismatchException if {@code point} and {@code this} are not expressed in
    *            the same reference frame.
    * @see #getClosestEdgeCopy(Point2DReadOnly)
    */
   default FrameLineSegment2DBasics getClosestEdgeCopy(FramePoint2DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return getClosestEdgeCopy((Point2DReadOnly) point);
   }

   /**
    * Finds the index of the closest vertex to the query.
    *
    * @param point the coordinates of the query. Not modified.
    * @return the index of the closest vertex to the query.
    * @throws ReferenceFrameMismatchException if {@code point} and {@code this} are not expressed in
    *            the same reference frame.
    * @see #getClosestVertexIndex(Point2DReadOnly)
    */
   default int getClosestVertexIndex(FramePoint2DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return ConvexPolygon2DReadOnly.super.getClosestVertexIndex(point);
   }

   /**
    * Finds the index of the closest vertex to the query.
    *
    * @param point the coordinates of the query. Not modified.
    * @param vertexToPack point used to store the result. Modified.
    * @return whether this method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code vertexToPack} and {@code this} are not
    *            expressed in the same reference frame.
    * @see #getClosestVertex(Point2DReadOnly, Point2DBasics)
    */
   default boolean getClosestVertex(Point2DReadOnly point, FixedFramePoint2DBasics vertexToPack)
   {
      checkReferenceFrameMatch(vertexToPack);
      return ConvexPolygon2DReadOnly.super.getClosestVertex(point, vertexToPack);
   }

   /**
    * Finds the index of the closest vertex to the query.
    *
    * @param point the coordinates of the query. Not modified.
    * @param vertexToPack point used to store the result. Modified.
    * @return whether this method succeeded or not.
    * @see #getClosestVertex(Point2DReadOnly, Point2DBasics)
    */
   default boolean getClosestVertex(Point2DReadOnly point, FramePoint2DBasics vertexToPack)
   {
      vertexToPack.setReferenceFrame(getReferenceFrame());
      return ConvexPolygon2DReadOnly.super.getClosestVertex(point, vertexToPack);
   }

   /**
    * Finds the index of the closest vertex to the query.
    *
    * @param point the coordinates of the query. Not modified.
    * @param vertexToPack point used to store the result. Modified.
    * @return whether this method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code point} and {@code this} are not expressed in
    *            the same reference frame.
    * @see #getClosestVertex(Point2DReadOnly, Point2DBasics)
    */
   default boolean getClosestVertex(FramePoint2DReadOnly point, Point2DBasics vertexToPack)
   {
      checkReferenceFrameMatch(point);
      return ConvexPolygon2DReadOnly.super.getClosestVertex(point, vertexToPack);
   }

   /**
    * Finds the index of the closest vertex to the query.
    *
    * @param point the coordinates of the query. Not modified.
    * @param vertexToPack point used to store the result. Modified.
    * @return whether this method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code point}, {@code vertexToPack}, and
    *            {@code this} are not expressed in the same reference frame.
    * @see #getClosestVertex(Point2DReadOnly, Point2DBasics)
    */
   default boolean getClosestVertex(FramePoint2DReadOnly point, FixedFramePoint2DBasics vertexToPack)
   {
      checkReferenceFrameMatch(point);
      checkReferenceFrameMatch(vertexToPack);
      return ConvexPolygon2DReadOnly.super.getClosestVertex(point, vertexToPack);
   }

   /**
    * Finds the index of the closest vertex to the query.
    *
    * @param point the coordinates of the query. Not modified.
    * @param vertexToPack point used to store the result. Modified.
    * @return whether this method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code point} and {@code this} are not expressed in
    *            the same reference frame.
    * @see #getClosestVertex(Point2DReadOnly, Point2DBasics)
    */
   default boolean getClosestVertex(FramePoint2DReadOnly point, FramePoint2DBasics vertexToPack)
   {
      checkReferenceFrameMatch(point);
      vertexToPack.setReferenceFrame(getReferenceFrame());
      return ConvexPolygon2DReadOnly.super.getClosestVertex(point, vertexToPack);
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint2DBasics getClosestVertexCopy(Point2DReadOnly point)
   {
      Point2DBasics closestVertex = ConvexPolygon2DReadOnly.super.getClosestVertexCopy(point);
      if (closestVertex == null)
         return null;
      else
         return new FramePoint2D(getReferenceFrame(), closestVertex);
   }

   /**
    * Finds the index of the closest vertex to the query.
    *
    * @param point the coordinates of the query. Not modified.
    * @return the coordinates of the closest vertex, or {@code null} if this method failed.
    * @throws ReferenceFrameMismatchException if {@code point} and {@code this} are not expressed in
    *            the same reference frame.
    * @see #getClosestVertexCopy(Point2DReadOnly)
    */
   default FramePoint2DBasics getClosestVertexCopy(FramePoint2DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return getClosestVertexCopy((Point2DReadOnly) point);
   }

   /**
    * Finds the index of the closest vertex to the given line.
    *
    * @param line the query. Not modified.
    * @return the index of the closest vertex to the query.
    * @throws ReferenceFrameMismatchException if {@code line} and {@code this} are not expressed in
    *            the same reference frame.
    * @see #getClosestVertexIndex(Line2DReadOnly)
    */
   default int getClosestVertexIndex(FrameLine2DReadOnly line)
   {
      checkReferenceFrameMatch(line);
      return ConvexPolygon2DReadOnly.super.getClosestVertexIndex(line);
   }

   /**
    * Finds the index of the closest vertex to the given line.
    *
    * @param line the query. Not modified.
    * @param vertexToPack point used to store the result. Modified.
    * @return whether this method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code vertexToPack} and {@code this} are not
    *            expressed in the same reference frame.
    * @see #getClosestVertex(Line2DReadOnly, Point2DBasics)
    */
   default boolean getClosestVertex(Line2DReadOnly line, FixedFramePoint2DBasics vertexToPack)
   {
      checkReferenceFrameMatch(vertexToPack);
      return ConvexPolygon2DReadOnly.super.getClosestVertex(line, vertexToPack);
   }

   /**
    * Finds the index of the closest vertex to the given line.
    *
    * @param line the query. Not modified.
    * @param vertexToPack point used to store the result. Modified.
    * @return whether this method succeeded or not.
    * @see #getClosestVertex(Line2DReadOnly, Point2DBasics)
    */
   default boolean getClosestVertex(Line2DReadOnly line, FramePoint2DBasics vertexToPack)
   {
      vertexToPack.setReferenceFrame(getReferenceFrame());
      return ConvexPolygon2DReadOnly.super.getClosestVertex(line, vertexToPack);
   }

   /**
    * Finds the index of the closest vertex to the given line.
    *
    * @param line the query. Not modified.
    * @param vertexToPack point used to store the result. Modified.
    * @return whether this method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code line} and {@code this} are not expressed in
    *            the same reference frame.
    * @see #getClosestVertex(Line2DReadOnly, Point2DBasics)
    */
   default boolean getClosestVertex(FrameLine2DReadOnly line, Point2DBasics vertexToPack)
   {
      checkReferenceFrameMatch(line);
      return ConvexPolygon2DReadOnly.super.getClosestVertex(line, vertexToPack);
   }

   /**
    * Finds the index of the closest vertex to the given line.
    *
    * @param line the query. Not modified.
    * @param vertexToPack point used to store the result. Modified.
    * @return whether this method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code line}, {@code vertexToPack}, and
    *            {@code this} are not expressed in the same reference frame.
    * @see #getClosestVertex(Line2DReadOnly, Point2DBasics)
    */
   default boolean getClosestVertex(FrameLine2DReadOnly line, FixedFramePoint2DBasics vertexToPack)
   {
      checkReferenceFrameMatch(line);
      checkReferenceFrameMatch(vertexToPack);
      return ConvexPolygon2DReadOnly.super.getClosestVertex(line, vertexToPack);
   }

   /**
    * Finds the index of the closest vertex to the given line.
    *
    * @param line the query. Not modified.
    * @param vertexToPack point used to store the result. Modified.
    * @return whether this method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code line} and {@code this} are not expressed in
    *            the same reference frame.
    * @see #getClosestVertex(Line2DReadOnly, Point2DBasics)
    */
   default boolean getClosestVertex(FrameLine2DReadOnly line, FramePoint2DBasics vertexToPack)
   {
      checkReferenceFrameMatch(line);
      vertexToPack.setReferenceFrame(getReferenceFrame());
      return ConvexPolygon2DReadOnly.super.getClosestVertex(line, vertexToPack);
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint2DBasics getClosestVertexCopy(Line2DReadOnly line)
   {
      Point2DBasics closestVertex = ConvexPolygon2DReadOnly.super.getClosestVertexCopy(line);
      if (closestVertex == null)
         return null;
      else
         return new FramePoint2D(getReferenceFrame(), closestVertex);
   }

   /**
    * Finds the index of the closest vertex to the given line.
    *
    * @param line the query. Not modified.
    * @return the coordinates of the closest vertex or {@code null} if this method failed.
    * @throws ReferenceFrameMismatchException if {@code line} and {@code this} are not expressed in
    *            the same reference frame.
    * @see #getClosestVertexCopy(Line2DReadOnly)
    */
   default FramePoint2DBasics getClosestVertexCopy(FrameLine2DReadOnly line)
   {
      checkReferenceFrameMatch(line);
      return getClosestVertexCopy((Line2DReadOnly) line);
   }

   /**
    * Packs the endpoints of an edge of this polygon into {@code edgeToPack}.
    *
    * @param edgeIndex index of the vertex that starts the edge.
    * @param edgeToPack line segment used to store the edge endpoints. Modified.
    * @throws ReferenceFrameMismatchException if {@code edge} and {@code this} are not expressed in
    *            the same reference frame.
    * @see #getEdge(int, LineSegment2DBasics)
    */
   default void getEdge(int edgeIndex, FixedFrameLineSegment2DBasics edgeToPack)
   {
      checkReferenceFrameMatch(edgeToPack);
      ConvexPolygon2DReadOnly.super.getEdge(edgeIndex, edgeToPack);
   }

   /**
    * Packs the endpoints of an edge of this polygon into {@code edgeToPack}.
    *
    * @param edgeIndex index of the vertex that starts the edge.
    * @param edgeToPack line segment used to store the edge endpoints. Modified.
    * @see #getEdge(int, LineSegment2DBasics)
    */
   default void getEdge(int edgeIndex, FrameLineSegment2DBasics edgeToPack)
   {
      edgeToPack.setReferenceFrame(getReferenceFrame());
      ConvexPolygon2DReadOnly.super.getEdge(edgeIndex, edgeToPack);
   }

   /** {@inheritDoc} */
   @Override
   default FrameConvexPolygon2DBasics translateCopy(Tuple2DReadOnly translation)
   {
      FrameConvexPolygon2D copy = new FrameConvexPolygon2D(this);
      copy.translate(translation);
      return copy;
   }

   /**
    * Copies this polygon, translates the copy, and returns it.
    *
    * @param translation the translation to apply to the copy of this polygon. Not modified.
    * @return the copy of this polygon translated.
    * @throws ReferenceFrameMismatchException if {@code translation} and {@code this} are not
    *            expressed in the same reference frame.
    * @see #translateCopy(Tuple2DReadOnly)
    */
   default FrameConvexPolygon2DBasics translateCopy(FrameTuple2DReadOnly translation)
   {
      checkReferenceFrameMatch(translation);
      return translateCopy((Tuple2DReadOnly) translation);
   }

   /**
    * Tests on a per vertex and per component basis, if this polygon is exactly equal to
    * {@code other}.
    *
    * @param other the other polygon to compare against this. Not modified.
    * @return {@code true} if the two polygons are exactly equal component-wise and expressed in the
    *         same frame reference frame, {@code false} otherwise.
    * @see #equals(ConvexPolygon2DReadOnly)
    */
   default boolean equals(FrameConvexPolygon2DReadOnly other)
   {
      if (other == null || getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return ConvexPolygon2DReadOnly.super.equals(other);
   }

   /**
    * Tests on a per-component basis on every vertices if this convex polygon is equal to
    * {@code other} with the tolerance {@code epsilon}.
    * <p>
    * The method returns {@code false} if the two polygons have different size.
    * </p>
    * <p>
    * The method returns {@code false} if the two polygons are expressed in different frames.
    * </p>
    *
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two line segments are equal, {@code false} otherwise.
    * @see #epsilonEquals(ConvexPolygon2DReadOnly, double)
    */
   default boolean epsilonEquals(FrameConvexPolygon2DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return ConvexPolygon2DReadOnly.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} and {@code other} to determine if the two convex polygons are
    * geometrically similar.
    * <p>
    * This method performs the comparison on a per vertex basis while accounting for a possible
    * shift in the polygon indexing. For instance, two polygons that have the same vertices in
    * clockwise or counter-clockwise order, are considered geometrically equal even if they do not
    * start with the same vertex.
    * </p>
    *
    * @param other the convex polygon to compare to.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the convex polygons represent the same geometry, {@code false}
    *         otherwise.
    * @throws ReferenceFrameMismatchException if {@code other} and {@code this} are not expressed in
    *            the same reference frame.
    * @see #geometricallyEquals(ConvexPolygon2DReadOnly, double)
    */
   default boolean geometricallyEquals(FrameConvexPolygon2DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return ConvexPolygon2DReadOnly.super.geometricallyEquals(other, epsilon);
   }
}
